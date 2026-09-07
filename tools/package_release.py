#!/usr/bin/env python3
"""Packages a release candidate per RELEASE_DELIVERABLE_CONTRACT.md: merges
the bootloader/partition-table/application images produced by an ESP-IDF
build into one flat binary at offset 0x000000, verifies the merged image's
own application segment against the audited pre-merge application binary
(build/mtkcore.bin) byte-for-byte and by SHA-256, generates the MD5 sidecar
(32 uppercase hex bytes, no whitespace/newline) ONLY after every check
passes, and writes a mechanically-generated PACKAGING_MANIFEST.md
(deliberately separate from this project's own hand-curated
RELEASE_NOTES.md -- see write_release_notes's own doc comment), a copy of
partitions.csv, and a machine-readable merged-image map -- none of them
containing an absolute personal filesystem path.

This tool only reads build output already produced by `idf.py build` and
the installed esptool.py; it performs no network access, no git operation,
and never copies anything to the Desktop.

The validation logic (`validate_merged_image`) is a plain, importable
function so a standalone negative-test suite (tools/validate_release_
negative.py) can exercise the SAME checks against synthetic/corrupted
inputs, rather than reimplementing them a second time under drift risk.

Usage: python3 tools/package_release.py --build-dir build --out <staging-dir>
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys


def sh(cmd, **kw):
    print("+ " + " ".join(cmd))
    return subprocess.run(cmd, check=True, **kw)


def sha256_hex(data):
    return hashlib.sha256(data).hexdigest().upper()


def md5_hex(data):
    return hashlib.md5(data).hexdigest().upper()


def validate_merged_image(data, audited_app_bytes, app_offset,
                           partition_table_offset=0x8000, bootloader_magic_offset=0x0,
                           audited_bootloader_bytes=None, audited_partition_table_bytes=None):
    """Returns a list of validation errors (empty list = a valid, audited-
    bound merged image). Every check here is a HARD failure -- there is no
    soft/NOTE-only outcome; a caller with no audited_app_bytes available
    yet must not call this at all rather than pass an empty placeholder
    (an empty audited_app_bytes would make the byte-for-byte check always
    fail, correctly, but the caller should make that unavailability
    explicit instead of relying on this function to do so).

    `audited_bootloader_bytes`/`audited_partition_table_bytes` are
    optional so this function still works as a pure structural/app-
    binding check when a caller has no build-input bytes to compare
    against (e.g. a synthetic negative test); when given, the bootloader
    and partition-table REGIONS embedded in `data` are compared byte-for-
    byte against them -- RC11 verification correction (Codex independent
    audit) "byte-compare the bootloader and partition-table regions
    embedded in MtkCore.bin against the exact build inputs, just as the
    application is already bound". Magic-byte checks alone only prove
    "this looks like the right KIND of image at this offset", never "this
    is the exact audited image" -- the same reasoning that already governs
    the application segment.
    """
    errors = []
    if len(data) <= app_offset:
        errors.append(f"merged image too small ({len(data)} bytes) to contain an application at "
                       f"0x{app_offset:X}")
        return errors  # nothing further is safe to index below
    if data[bootloader_magic_offset] != 0xE9:
        errors.append(f"bootloader image magic at 0x{bootloader_magic_offset:X} is "
                       f"0x{data[bootloader_magic_offset]:02X}, expected 0xE9")
    if len(data) <= partition_table_offset + 1:
        errors.append(f"merged image too small ({len(data)} bytes) to contain a partition table at "
                       f"0x{partition_table_offset:X}")
    elif not (data[partition_table_offset] == 0xAA and data[partition_table_offset + 1] == 0x50):
        errors.append(f"partition-table magic at 0x{partition_table_offset:X} is "
                       f"{data[partition_table_offset]:02X}{data[partition_table_offset + 1]:02X}, expected AA50")
    if data[app_offset] != 0xE9:
        errors.append(f"application image magic at 0x{app_offset:X} is 0x{data[app_offset]:02X}, expected 0xE9")

    # Requirement: compare the application segment byte-for-byte AND by
    # SHA-256 against the audited pre-merge build/mtkcore.bin -- either
    # check alone would already catch a mismatch, but both are computed
    # and reported so a caller sees exactly which property failed.
    merged_app_bytes = data[app_offset:]
    if merged_app_bytes != audited_app_bytes:
        errors.append(f"merged image's application segment (offset 0x{app_offset:X}, "
                       f"{len(merged_app_bytes)} bytes) does not byte-for-byte match the audited "
                       f"application binary ({len(audited_app_bytes)} bytes)")
    elif sha256_hex(merged_app_bytes) != sha256_hex(audited_app_bytes):
        # Unreachable if the byte-for-byte check above already passed
        # (identical bytes always hash identically) -- kept as an
        # independent, explicit check anyway per the stated requirement
        # ("byte-for-byte AND by SHA-256"), not merely implied by the
        # first one, and as defense in depth against a future refactor
        # that might compare a hash without also comparing raw bytes.
        errors.append("merged image's application segment SHA-256 does not match the audited "
                       "application binary's SHA-256 (despite matching byte-for-byte -- should be "
                       "unreachable; report this as a bug in this check itself)")

    if audited_bootloader_bytes is not None:
        merged_bootloader_bytes = data[bootloader_magic_offset:bootloader_magic_offset + len(audited_bootloader_bytes)]
        if merged_bootloader_bytes != audited_bootloader_bytes:
            errors.append(f"merged image's bootloader region (offset 0x{bootloader_magic_offset:X}, "
                           f"{len(audited_bootloader_bytes)} bytes) does not byte-for-byte match the "
                           f"audited bootloader.bin")
        elif sha256_hex(merged_bootloader_bytes) != sha256_hex(audited_bootloader_bytes):
            errors.append("merged image's bootloader region SHA-256 does not match the audited "
                           "bootloader.bin (despite matching byte-for-byte -- should be unreachable)")

    if audited_partition_table_bytes is not None:
        merged_pt_bytes = data[partition_table_offset:partition_table_offset + len(audited_partition_table_bytes)]
        if merged_pt_bytes != audited_partition_table_bytes:
            errors.append(f"merged image's partition-table region (offset 0x{partition_table_offset:X}, "
                           f"{len(audited_partition_table_bytes)} bytes) does not byte-for-byte match the "
                           f"audited partition-table.bin")
        elif sha256_hex(merged_pt_bytes) != sha256_hex(audited_partition_table_bytes):
            errors.append("merged image's partition-table region SHA-256 does not match the audited "
                           "partition-table.bin (despite matching byte-for-byte -- should be unreachable)")

    return errors


def validate_esp_image(image_path, chip):
    """Runs esptool's OWN image-info parser against a real, standalone ESP
    image file (a bootloader or application image -- never a slice inside
    a bigger merged blob; esptool needs the whole image file itself) and
    parses ITS OWN reported checksum/validation-hash verdict.

    RC11 release-finalization correction (Codex independent audit):
    "Strengthen package validation so it hard-fails unless esptool
    validates both the extracted bootloader image and application image,
    including chip type, image checksum, and validation hash. Magic bytes
    alone are insufficient." A real, confirmed finding while implementing
    this: esptool's own `image-info` subcommand exits 0 EVEN WHEN it
    reports a checksum/hash as invalid (verified directly: a single
    corrupted non-magic byte inside a segment still returns exit code 0,
    with "Checksum: 0xXX (invalid - calculated 0xYY)" and "Validation
    hash: ... (invalid)" printed to stdout) -- so THIS function's own
    return value, not esptool's process exit code, is the actual gate;
    a caller must never treat "esptool ran without raising" as "esptool
    says this image is valid".

    Returns a list of errors (empty = valid: chip matches, checksum
    valid, validation hash valid)."""
    errors = []
    try:
        result = subprocess.run(
            [sys.executable, "-m", "esptool", "--chip", chip, "image-info", image_path],
            capture_output=True, text=True, timeout=60)
    except Exception as e:
        return [f"esptool image-info could not be run against {image_path}: {e}"]
    output = result.stdout + result.stderr
    if result.returncode != 0:
        errors.append(f"esptool image-info exited {result.returncode} for {image_path}:\n{output}")
        return errors

    lines = output.splitlines()
    chip_line = next((l for l in lines if l.strip().startswith("Chip ID:")), None)
    if not chip_line:
        errors.append(f"esptool image-info produced no 'Chip ID:' line for {image_path} -- cannot confirm "
                       f"chip type:\n{output}")
    else:
        # "Chip ID: 13 (ESP32-C6)" -- compare normalized alnum-only forms
        # so "esp32c6" (this tool's own chip argument) matches "ESP32-C6"
        # (esptool's own display form) regardless of hyphenation/case.
        norm_chip = re.sub(r"[^a-z0-9]", "", chip.lower())
        norm_line = re.sub(r"[^a-z0-9]", "", chip_line.lower())
        if norm_chip not in norm_line:
            errors.append(f"esptool image-info chip mismatch for {image_path}: '{chip_line.strip()}' "
                           f"does not name the expected chip '{chip}'")

    checksum_line = next((l for l in lines if l.strip().startswith("Checksum:")), None)
    if not checksum_line:
        errors.append(f"esptool image-info produced no 'Checksum:' line for {image_path}")
    elif "(valid)" not in checksum_line:
        errors.append(f"esptool image-info reports an INVALID image checksum for {image_path}: "
                       f"'{checksum_line.strip()}'")

    hash_line = next((l for l in lines if l.strip().startswith("Validation hash:")), None)
    if not hash_line:
        errors.append(f"esptool image-info produced no 'Validation hash:' line for {image_path} -- this "
                       f"ESP-IDF/esptool combination may not embed one; a missing hash on an image that "
                       f"should have one is itself treated as a hard failure, never silently accepted")
    elif "(valid)" not in hash_line:
        errors.append(f"esptool image-info reports an INVALID validation hash for {image_path}: "
                       f"'{hash_line.strip()}'")

    return errors


def validate_sidecar(merged_bytes, sidecar_bytes):
    """Returns a list of errors (empty = valid). Sidecar syntax exactly as
    the real M1 SD updater expects: exactly 32 uppercase hex ASCII bytes,
    no whitespace, no trailing newline, matching the merged binary's own
    real MD5."""
    errors = []
    if len(sidecar_bytes) != 32:
        errors.append(f"MD5 sidecar is not exactly 32 bytes (got {len(sidecar_bytes)})")
        return errors
    if not all((48 <= b <= 57) or (65 <= b <= 70) for b in sidecar_bytes):
        errors.append("MD5 sidecar is not 32 uppercase hex ASCII bytes (lowercase, whitespace, or "
                       "non-hex byte present)")
        return errors
    expected = md5_hex(merged_bytes).encode("ascii")
    if sidecar_bytes != expected:
        errors.append(f"MD5 sidecar does not match the merged binary: sidecar={sidecar_bytes.decode()} "
                       f"computed={expected.decode()}")
    return errors


def validate_map_against_bin(map_obj, merged_bytes, app_offset, chip=None,
                              expected_offsets=None, audited_app_bytes=None):
    """Returns a list of errors (empty = valid). Catches a STALE map (one
    that does not describe the ACTUAL current merged binary) even when the
    .bin+.md5 pair is itself internally self-consistent -- a third,
    independently-generated artifact an attacker or mistake would also
    have to tamper with to go undetected.

    RC11 release-finalization correction (Codex independent audit):
    "Strengthen merged-image-map validation. Require and verify: chip and
    offsets; merged binary size/MD5/SHA-256; application size/MD5/SHA-256;
    nested audited_application_input size/MD5/SHA-256; expected segment
    entries." Every field named there is now a REQUIRED key (missing any
    one of them is itself a hard failure, not merely skipped), and every
    one with a computable expected value is cross-checked against the
    actual bytes/build inputs, not merely checked for presence.
    `chip`/`expected_offsets`/`audited_app_bytes` are optional so this
    function still works as a pure "is this map internally well-formed and
    bin-consistent" check when a caller (e.g. a synthetic negative test)
    has no real chip/build context to compare against."""
    errors = []
    required = (
        "chip", "flash_offset", "partition_table_offset", "application_offset",
        "merged_binary", "merged_binary_size", "md5_uppercase_hex", "sha256_uppercase_hex",
        "app_size_bytes", "app_md5_uppercase_hex", "app_sha256_uppercase_hex",
        "audited_application_input", "segments",
    )
    for key in required:
        if key not in map_obj:
            errors.append(f"merged_image_map.json is missing required field '{key}'")
    if errors:
        return errors

    audited = map_obj["audited_application_input"]
    for key in ("file", "size", "md5_uppercase_hex", "sha256_uppercase_hex"):
        if key not in audited:
            errors.append(f"merged_image_map.json's audited_application_input is missing required field '{key}'")
    if errors:
        return errors

    # Release-tooling correction (this round): "audited_application_input.file
    # must equal 'mtkcore.bin'" -- a FIXED, non-negotiable expected value.
    # Previously the application segment's expected filename (below, in
    # expected_file_map) was derived FROM this same mutable field, so a
    # caller who corrupted audited_application_input.file AND the matching
    # segment's file to the SAME wrong value (e.g. both set to
    # "renamed-app.bin") passed undetected: the derived expected value moved
    # in lockstep with the corruption instead of staying anchored to the one
    # real audited build artifact this project actually produces.
    if audited["file"] != "mtkcore.bin":
        errors.append(f"merged_image_map.json audited_application_input.file '{audited['file']}' does not "
                       f"equal the required 'mtkcore.bin'")

    # RC11 verification correction (Codex independent audit): "require
    # merged_binary == MtkCore.bin" -- the approved release filename
    # (requirement #10 of the prior release-artifact-binding round),
    # cross-checked here so a map that describes some OTHER filename
    # (e.g. a leftover candidate name) is rejected outright.
    if map_obj["merged_binary"] != "MtkCore.bin":
        errors.append(f"merged_image_map.json merged_binary '{map_obj['merged_binary']}' does not equal "
                       f"the required 'MtkCore.bin'")

    if map_obj["merged_binary_size"] != len(merged_bytes):
        errors.append(f"merged_image_map.json size {map_obj['merged_binary_size']} does not match the "
                       f"actual merged binary size {len(merged_bytes)}")
    if map_obj["md5_uppercase_hex"] != md5_hex(merged_bytes):
        errors.append("merged_image_map.json md5_uppercase_hex does not match the actual merged binary")
    if map_obj["sha256_uppercase_hex"] != sha256_hex(merged_bytes):
        errors.append("merged_image_map.json sha256_uppercase_hex does not match the actual merged binary")

    if len(merged_bytes) > app_offset:
        real_app_bytes = merged_bytes[app_offset:]
        if map_obj["app_size_bytes"] != len(real_app_bytes):
            errors.append(f"merged_image_map.json app_size_bytes {map_obj['app_size_bytes']} does not "
                           f"match the actual merged binary's own application segment size "
                           f"{len(real_app_bytes)}")
        if map_obj["app_md5_uppercase_hex"] != md5_hex(real_app_bytes):
            errors.append("merged_image_map.json app_md5_uppercase_hex does not match the actual "
                           "merged binary's own application segment (stale map)")
        if map_obj["app_sha256_uppercase_hex"] != sha256_hex(real_app_bytes):
            errors.append("merged_image_map.json app_sha256_uppercase_hex does not match the actual "
                           "merged binary's own application segment (stale map)")

    if chip is not None and map_obj["chip"] != chip:
        errors.append(f"merged_image_map.json chip '{map_obj['chip']}' does not match the expected chip "
                       f"'{chip}'")

    if expected_offsets is not None:
        for key, expected in expected_offsets.items():
            if map_obj.get(key) != expected:
                errors.append(f"merged_image_map.json {key} '{map_obj.get(key)}' does not match the "
                               f"expected value '{expected}'")

    if audited_app_bytes is not None:
        if audited["size"] != len(audited_app_bytes):
            errors.append(f"merged_image_map.json audited_application_input.size {audited['size']} does "
                           f"not match the actual audited application binary size {len(audited_app_bytes)}")
        if audited["md5_uppercase_hex"] != md5_hex(audited_app_bytes):
            errors.append("merged_image_map.json audited_application_input.md5_uppercase_hex does not "
                           "match the actual audited application binary")
        if audited["sha256_uppercase_hex"] != sha256_hex(audited_app_bytes):
            errors.append("merged_image_map.json audited_application_input.sha256_uppercase_hex does not "
                           "match the actual audited application binary")

    # RC11 verification correction (Codex independent audit): "require the
    # exact mapping 0x000000 -> bootloader/bootloader.bin, 0x008000 ->
    # partition_table/partition-table.bin, 0x010000 -> mtkcore.bin; reject
    # wrong relative filenames, swapped filenames, duplicate offsets, or
    # missing entries." A set-of-offsets comparison (the previous check)
    # cannot catch two entries at the CORRECT offsets with their FILES
    # swapped, nor does it name a duplicate-offset failure specifically --
    # both are now checked directly against an exact offset->file mapping.
    segments = map_obj["segments"]
    expected_segment_count = 3  # bootloader, partition-table, application -- this project's own fixed layout
    if len(segments) != expected_segment_count:
        errors.append(f"merged_image_map.json has {len(segments)} segment entries, expected "
                       f"{expected_segment_count} (bootloader, partition-table, application)")
    else:
        seg_offset_list = [s.get("offset") for s in segments]
        if len(set(seg_offset_list)) != len(seg_offset_list):
            errors.append(f"merged_image_map.json has duplicate segment offsets: {seg_offset_list}")
        else:
            # Release-tooling correction (this round): the expected
            # application-segment filename here is now a FIXED literal
            # ("mtkcore.bin"), never derived from audited["file"] -- that
            # field is itself validated against the same fixed literal
            # above, but deriving the segment's own expectation from it
            # meant a coordinated corruption of BOTH fields to the same
            # wrong value (e.g. both "renamed-app.bin") passed this check
            # undetected, since the two mutable values always agreed with
            # each other regardless of what either one actually said.
            expected_file_map = {
                "0x000000": "bootloader/bootloader.bin",
                map_obj.get("partition_table_offset"): "partition_table/partition-table.bin",
                map_obj.get("application_offset"): "mtkcore.bin",
            }
            actual_file_map = {s.get("offset"): s.get("file") for s in segments}
            if actual_file_map != expected_file_map:
                errors.append(f"merged_image_map.json segments {actual_file_map} do not match the exact "
                               f"expected offset->file mapping {expected_file_map} (missing entry, wrong "
                               f"relative filename, or filenames swapped between offsets)")
        for s in segments:
            if "file" not in s or not s["file"]:
                errors.append(f"merged_image_map.json segment entry at offset {s.get('offset')} has no "
                               f"'file' field")
            elif os.path.isabs(s["file"]):
                errors.append(f"merged_image_map.json segment entry '{s['file']}' is an absolute path -- "
                               f"segment file paths must always be relative")

    return errors


def gather_build_identity(build_dir):
    """Reads ESP-IDF version, target, and this project's own build-
    candidate identity (MTK_BUILD_ID/MTK_BUILD_EPOCH_S) from build output
    already on disk -- never invents/derives a value this build did not
    actually record, and never copies an absolute filesystem path (only
    the specific safe fields below are read out of project_description.json,
    which also carries several absolute-path fields this function
    deliberately does not touch)."""
    identity = {
        "esp_idf_version": None,
        "target_chip": None,
        "build_candidate_id": None,
        "build_epoch_s": None,
        "build_dirty": 0,  # no VCS in this clean-room tree (docs/DECISION_LOG.md's own binding constraint) -- the only honest, disclosed answer
    }
    proj_desc_path = os.path.join(build_dir, "project_description.json")
    if os.path.isfile(proj_desc_path):
        with open(proj_desc_path) as f:
            proj_desc = json.load(f)
        identity["esp_idf_version"] = proj_desc.get("git_revision")  # ESP-IDF's own git revision, e.g. "v6.0.1" -- not this project's (no VCS here)
        identity["target_chip"] = proj_desc.get("target")

    cache_path = os.path.join(build_dir, "CMakeCache.txt")
    if os.path.isfile(cache_path):
        with open(cache_path) as f:
            cache_text = f.read()
        m = re.search(r"^MTK_RELEASE_CANDIDATE:STRING=(.*)$", cache_text, re.MULTILINE)
        if m:
            identity["build_candidate_id"] = m.group(1).strip()
        m = re.search(r"^MTK_RELEASE_EPOCH_S:STRING=(.*)$", cache_text, re.MULTILINE)
        if m:
            identity["build_epoch_s"] = m.group(1).strip()
    return identity


def write_release_notes(out_dir, image_map, identity, hardware_tested):
    """Writes PACKAGING_MANIFEST.md -- a mechanically-generated record of
    every field this batch's own requirement #11 asks for (ESP-IDF
    version, target, binary size, MD5, SHA-256, flash/partition offsets,
    build identity/dirty status, hardware-test status), and nothing that
    could leak an absolute personal filesystem path (identity/image_map
    are both already scrubbed of those by their own producing functions
    above). Deliberately a SEPARATE file from release/RELEASE_NOTES.md --
    that file is this project's own hand-curated, per-candidate narrative
    ("What changed" tables, capability-manifest references, etc.); this
    tool only ever appends the mechanical facts a computer can derive from
    the build output itself, and must never silently overwrite/replace
    that hand-authored history."""
    lines = []
    lines.append(f"# Packaging manifest: {identity.get('build_candidate_id') or '(unknown)'}")
    lines.append("")
    lines.append("Generated by `tools/package_release.py`. Computer-side packaging/validation")
    lines.append("only -- see \"Hardware test status\" below. This is a MECHANICALLY-GENERATED")
    lines.append("companion to `RELEASE_NOTES.md` (this project's own hand-curated per-candidate")
    lines.append("narrative) -- it never replaces or duplicates that file's own content.")
    lines.append("")
    lines.append("## Build identity")
    lines.append("")
    lines.append(f"- Build candidate ID: `{identity.get('build_candidate_id') or '(unknown)'}`")
    lines.append(f"- Build epoch (Unix seconds UTC, SOURCE_DATE_EPOCH-style, fixed per candidate): "
                  f"`{identity.get('build_epoch_s') or '(unknown)'}`")
    # RC11 release-finalization correction (Codex independent audit):
    # "Change no-VCS build provenance from dirty=0 to an explicit state...
    # Do not imply a clean git tree." The firmware's own wire GET_VERSION
    # response still carries a fixed uint8_t build_dirty field (0, matched
    # exactly by this generated text below) -- that wire value/interface
    # is unchanged by this correction (a real behavior/interface freeze,
    # not this batch's to touch); only THIS prose description of it is
    # rewritten so a reader never mistakes "0" for "a clean git tree
    # exists" when there is no git tree here at all to be clean or dirty.
    lines.append(f"- Build commit: `UNAVAILABLE (pre-repository clean-room workspace)`")
    lines.append(f"- Dirty status: `NOT_APPLICABLE` (no VCS in this clean-room tree -- the firmware's own "
                  f"wire build_dirty field is fixed at 0 for this same reason, which must never be read as "
                  f"a claim of a clean git tree)")
    lines.append(f"- ESP-IDF version: `{identity.get('esp_idf_version') or '(unknown)'}`")
    lines.append(f"- Target chip: `{identity.get('target_chip') or '(unknown)'}`")
    lines.append("")
    lines.append("## Merged image")
    lines.append("")
    lines.append(f"- File: `{image_map['merged_binary']}`")
    lines.append(f"- Size: {image_map['merged_binary_size']} bytes")
    lines.append(f"- Flash offset: `{image_map['flash_offset']}`")
    lines.append(f"- MD5 (uppercase hex): `{image_map['md5_uppercase_hex']}`")
    lines.append(f"- SHA-256 (uppercase hex): `{image_map['sha256_uppercase_hex']}`")
    lines.append(f"- Application-segment MD5 (uppercase hex, offset {image_map['application_offset']} "
                  f"onward): `{image_map['app_md5_uppercase_hex']}`")
    lines.append(f"- Application-segment SHA-256 (uppercase hex): `{image_map['app_sha256_uppercase_hex']}`")
    lines.append("")
    lines.append("## Partition offsets")
    lines.append("")
    lines.append(f"- Bootloader: `0x000000`")
    lines.append(f"- Partition table: `{image_map['partition_table_offset']}`")
    for p in image_map.get("partitions", []):
        lines.append(f"- {p['name']} ({p['type']}/{p['subtype']}): offset `{p['offset']}`, size `{p['size']}`")
    lines.append("")
    lines.append("## Hardware test status")
    lines.append("")
    if hardware_tested:
        lines.append("Hardware-validated.")
    else:
        lines.append("NOT hardware-validated. This candidate has passed computer-side packaging and "
                      "artifact validation only (merge structure, application-byte/hash binding, sidecar "
                      "syntax, host test suites, and a clean ESP-IDF target build) -- see "
                      "docs/PROVENANCE.md for the full list of disclosed hardware-only gaps.")
    lines.append("")
    with open(os.path.join(out_dir, "PACKAGING_MANIFEST.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


def parse_partitions_csv(path):
    """Minimal partitions.csv parser -- Name,Type,SubType,Offset,Size,Flags,
    comments (#) and blank lines skipped. Returns a list of dicts."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = [c.strip() for c in line.split(",")]
            if len(fields) < 5:
                continue
            out.append({"name": fields[0], "type": fields[1], "subtype": fields[2],
                        "offset": fields[3], "size": fields[4]})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", default="build")
    ap.add_argument("--out", default="release", help="Output/staging directory")
    ap.add_argument("--project-root", default=".")
    ap.add_argument("--hardware-tested", action="store_true",
                     help="Only pass this if this exact candidate has genuinely been hardware-validated.")
    args = ap.parse_args()

    root = os.path.abspath(args.project_root)
    build_dir = os.path.join(root, args.build_dir)
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    flasher_args_path = os.path.join(build_dir, "flasher_args.json")
    with open(flasher_args_path) as f:
        flasher_args = json.load(f)

    # flasher_args.json's "flash_files" maps offset(str) -> relative bin path.
    flash_files = flasher_args["flash_files"]
    entries = sorted(((int(off, 0), path) for off, path in flash_files.items()), key=lambda x: x[0])
    print("Merge inputs (offset, file):")
    for off, path in entries:
        print(f"  0x{off:06X}  {path}")

    # Requirement #1: build/mtkcore.bin (flasher_args.json's own "app"
    # entry -- never hardcoded) IS the audited application input. Hashed
    # BEFORE the merge step runs, from the same bytes idf.py build itself
    # just produced.
    app_offset = int(flasher_args["app"]["offset"], 0)
    audited_app_path = os.path.join(build_dir, flasher_args["app"]["file"])
    with open(audited_app_path, "rb") as f:
        audited_app_bytes = f.read()
    audited_app_sha256 = sha256_hex(audited_app_bytes)
    audited_app_md5 = md5_hex(audited_app_bytes)
    print(f"Audited application input: {os.path.relpath(audited_app_path, root)} "
          f"({len(audited_app_bytes)} bytes, SHA-256 {audited_app_sha256})")

    # RC11 verification correction (Codex independent audit): "byte-
    # compare the bootloader and partition-table regions embedded in
    # MtkCore.bin against the exact build inputs, just as the application
    # is already bound" -- the same audited-input treatment app_offset's
    # own bytes already got, extended to the other two segments.
    audited_bootloader_path = os.path.join(build_dir, flasher_args["bootloader"]["file"])
    with open(audited_bootloader_path, "rb") as f:
        audited_bootloader_bytes = f.read()
    audited_partition_table_path = os.path.join(build_dir, flasher_args["partition-table"]["file"])
    with open(audited_partition_table_path, "rb") as f:
        audited_partition_table_bytes = f.read()

    merged_bin = os.path.join(out_dir, "MtkCore.bin")
    flash_size = flasher_args.get("flash_settings", {}).get("flash_size", "4MB")
    chip = flasher_args.get("extra_esptool_args", {}).get("chip", "esp32c6")

    merge_cmd = [
        sys.executable, "-m", "esptool",
        "--chip", chip,
        "merge_bin",
        "-o", merged_bin,
        "--flash_size", flash_size,
    ]
    for off, path in entries:
        merge_cmd += [f"0x{off:X}", os.path.join(build_dir, path)]
    sh(merge_cmd, cwd=root)

    with open(merged_bin, "rb") as f:
        data = f.read()
    size = len(data)

    # ---- Hard validation FIRST -- the MD5 sidecar is written only after
    # every one of these passes (requirement #5: "generate the sidecar
    # only after the final merged binary exists" -- read as "exists AND
    # is confirmed valid/final", not merely "the merge step returned"). ----
    errors = validate_merged_image(data, audited_app_bytes, app_offset,
                                    partition_table_offset=int(flasher_args["partition-table"]["offset"], 0),
                                    bootloader_magic_offset=int(flasher_args["bootloader"]["offset"], 0),
                                    audited_bootloader_bytes=audited_bootloader_bytes,
                                    audited_partition_table_bytes=audited_partition_table_bytes)

    # RC11 release-finalization correction (Codex independent audit):
    # "Magic bytes alone are insufficient" -- runs esptool's OWN image
    # parser (not this script's own hand-rolled magic-byte check) against
    # the real, standalone bootloader and application image FILES (not a
    # slice of the merged blob -- image-info needs a whole image file),
    # confirming chip type, image checksum, AND the embedded validation
    # hash, exactly as the requirement names them. See validate_esp_
    # image's own doc comment for why esptool's process exit code alone is
    # NOT trusted here (a confirmed, real gap: it exits 0 even when it
    # reports an invalid checksum/hash).
    errors += [f"bootloader image: {e}" for e in validate_esp_image(audited_bootloader_path, chip)]
    errors += [f"application image: {e}" for e in validate_esp_image(audited_app_path, chip)]

    if errors:
        print("PACKAGING FAILED -- no sidecar/map/release-notes written:")
        for e in errors:
            print("  - " + e)
        sys.exit(1)

    md5_hex_val = md5_hex(data)
    sha256_hex_val = sha256_hex(data)
    app_bytes = data[app_offset:]
    app_md5_hex_val = md5_hex(app_bytes)
    app_sha256_hex_val = sha256_hex(app_bytes)
    assert app_md5_hex_val == audited_app_md5  # already proven by validate_merged_image above; re-asserted for defense in depth

    # ---- partitions.csv copy (requirement #7) -- copied verbatim from
    # this project's own source tree, no absolute path substitution
    # needed since the file itself contains none. ----
    src_partitions_csv = os.path.join(root, "partitions.csv")
    partitions = []
    if os.path.isfile(src_partitions_csv):
        with open(src_partitions_csv, "rb") as f:
            partitions_csv_bytes = f.read()
        with open(os.path.join(out_dir, "partitions.csv"), "wb") as f:
            f.write(partitions_csv_bytes)
        partitions = parse_partitions_csv(src_partitions_csv)

    # ---- merged-image map (machine-readable) -- every path recorded is
    # relative (flasher_args.json's own convention), never absolute. ----
    image_map = {
        "chip": chip,
        "flash_size": flash_size,
        "flash_offset": "0x000000",
        "partition_table_offset": f"0x{int(flasher_args['partition-table']['offset'], 0):06X}",
        "application_offset": f"0x{app_offset:06X}",
        "merged_binary": os.path.basename(merged_bin),
        "merged_binary_size": size,
        "md5_uppercase_hex": md5_hex_val,
        "sha256_uppercase_hex": sha256_hex_val,
        "app_size_bytes": len(app_bytes),
        "app_md5_uppercase_hex": app_md5_hex_val,
        "app_sha256_uppercase_hex": app_sha256_hex_val,
        "audited_application_input": {
            "file": flasher_args["app"]["file"],
            "size": len(audited_app_bytes),
            "md5_uppercase_hex": audited_app_md5,
            "sha256_uppercase_hex": audited_app_sha256,
        },
        "segments": [{"offset": f"0x{off:06X}", "file": path} for off, path in entries],
        "partitions": partitions,
    }

    # Self-check: the map this tool is ABOUT to write must itself pass the
    # SAME hard-fail validation (chip/offsets/sizes/hashes/segments) any
    # later consumer (test_release_artifact.c, validate_release_negative.py)
    # will run against it -- catches a bug in this script's own map
    # construction immediately, before ever writing a file that would
    # otherwise look authoritative.
    map_self_errors = validate_map_against_bin(
        image_map, data, app_offset, chip=chip,
        expected_offsets={
            "flash_offset": "0x000000",
            "partition_table_offset": image_map["partition_table_offset"],
            "application_offset": image_map["application_offset"],
        },
        audited_app_bytes=audited_app_bytes)
    if map_self_errors:
        print("PACKAGING FAILED (merged_image_map.json self-check, should be unreachable):")
        for e in map_self_errors:
            print("  - " + e)
        sys.exit(1)

    with open(os.path.join(out_dir, "merged_image_map.json"), "w") as f:
        json.dump(image_map, f, indent=2)
        f.write("\n")

    # ---- release metadata (requirement #11) ----
    identity = gather_build_identity(build_dir)
    write_release_notes(out_dir, image_map, identity, args.hardware_tested)

    # ---- MD5 sidecar LAST, only now that everything above has passed and
    # been written (requirement #5/#6). ----
    md5_path = os.path.join(out_dir, "MtkCore.md5")
    with open(md5_path, "wb") as f:
        f.write(md5_hex_val.encode("ascii"))  # exactly 32 uppercase hex bytes, no newline
    sidecar_errors = validate_sidecar(data, md5_hex_val.encode("ascii"))
    if sidecar_errors:
        print("PACKAGING FAILED (sidecar self-check, should be unreachable):")
        for e in sidecar_errors:
            print("  - " + e)
        sys.exit(1)

    print(f"OK: {merged_bin} ({size} bytes)")
    print(f"MD5: {md5_hex_val}")
    print(f"SHA-256: {sha256_hex_val}")
    print(f"Application segment MD5: {app_md5_hex_val}")
    print(f"Application segment SHA-256: {app_sha256_hex_val}")
    return image_map


if __name__ == "__main__":
    main()
