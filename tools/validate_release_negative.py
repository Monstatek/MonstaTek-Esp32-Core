#!/usr/bin/env python3
"""Computer-side negative tests for the release artifact/sidecar pair
(RELEASE_CANDIDATE_CHECKLIST.md "Computer-side negative tests") and for
tools/package_release.py's own merge/map validation logic: missing binary,
missing sidecar, malformed sidecar, one-byte binary mutation, a
substituted/renamed pair, a missing merged-image map, a STALE map (present
but no longer describing the actual binary), application bytes differing
from the audited pre-merge build, and an invalid bootloader/partition-
table/application image magic byte must ALL be rejected. Run via ctest
(host_tests/CMakeLists.txt registers this as test_release_negative) or
directly: python3 tools/validate_release_negative.py

The merge/map/sidecar checks below call directly into tools/package_
release.py's own validate_merged_image/validate_map_against_bin/
validate_sidecar functions -- the SAME logic packaging itself hard-fails
on, not a second, independently-drifting reimplementation of it.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import package_release as pr  # noqa: E402


def validate_pair(bin_path, md5_path):
    """Returns a list of validation errors (empty list = valid pair)."""
    errors = []
    if not os.path.isfile(bin_path):
        return ["missing binary"]
    if not os.path.isfile(md5_path):
        return ["missing sidecar"]
    with open(bin_path, "rb") as f:
        data = f.read()
    with open(md5_path, "rb") as f:
        sidecar = f.read()
    if len(sidecar) != 32:
        errors.append(f"sidecar is not exactly 32 bytes (got {len(sidecar)})")
        return errors
    if not all((48 <= b <= 57) or (65 <= b <= 70) for b in sidecar):
        errors.append("sidecar is not uppercase hex")
        return errors
    expected = hashlib.md5(data).hexdigest().upper().encode("ascii")
    if sidecar != expected:
        errors.append(f"sidecar mismatch: file={sidecar.decode()} computed={expected.decode()}")
    return errors


BOOTLOADER_REGION_LEN = 0x100  # arbitrary but fixed synthetic "audited bootloader.bin" length
PARTITION_TABLE_REGION_LEN = 0x100  # arbitrary but fixed synthetic "audited partition-table.bin" length


def make_synthetic_merged_image(app_bytes, corrupt_bootloader=False, corrupt_partition_table=False,
                                 corrupt_app=False, corrupt_bootloader_region=False,
                                 corrupt_partition_table_region=False):
    """Builds a minimal-but-structurally-real synthetic merged image: a
    bootloader region (magic 0xE9 at 0x0, BOOTLOADER_REGION_LEN bytes),
    zero-padding up to the partition table (magic AA50 at 0x8000,
    PARTITION_TABLE_REGION_LEN bytes), zero-padding up to the application
    offset (0x10000), then `app_bytes` (its own magic 0xE9 at byte 0, as a
    real ESP-IDF app image always has).

    `corrupt_bootloader_region`/`corrupt_partition_table_region` flip a
    byte WELL INSIDE the respective region but never at its own magic-byte
    position -- RC11 verification correction (independent audit)
    "byte-compare the bootloader and partition-table regions ... just as
    the application is already bound": these prove that NEW check
    specifically, distinct from `corrupt_bootloader`/`corrupt_partition_
    table` above (which corrupt the magic byte itself, already caught by
    the pre-existing magic-byte check alone)."""
    app_offset = 0x10000
    buf = bytearray(app_offset)
    buf[0] = 0x00 if corrupt_bootloader else 0xE9
    buf[0x8000:0x8002] = bytes([0x00, 0x00]) if corrupt_partition_table else bytes([0xAA, 0x50])
    if corrupt_bootloader_region:
        buf[0x50] ^= 0xFF  # inside [0, BOOTLOADER_REGION_LEN), never byte 0
    if corrupt_partition_table_region:
        buf[0x8050] ^= 0xFF  # inside [0x8000, 0x8000+PARTITION_TABLE_REGION_LEN), never the AA50 magic
    data = bytes(buf) + (bytes([0x00]) + app_bytes[1:] if corrupt_app and app_bytes else app_bytes)
    return data, app_offset


def audited_regions_from(good_merged_bytes):
    """Extracts the (bootloader_bytes, partition_table_bytes) pair a real
    `package_release.py` caller would have read from the actual build
    inputs BEFORE merging -- here, sliced from a known-good synthetic
    image built by make_synthetic_merged_image with no corruption flags,
    so a test can corrupt the MERGED copy independently and prove the
    mismatch is caught."""
    bootloader_bytes = good_merged_bytes[0:BOOTLOADER_REGION_LEN]
    pt_bytes = good_merged_bytes[0x8000:0x8000 + PARTITION_TABLE_REGION_LEN]
    return bootloader_bytes, pt_bytes


def make_valid_map(merged_bytes, app_offset, audited_app_bytes=None, chip="esp32c6"):
    """A syntactically- and value-complete merged_image_map.json object --
    every field RC11's own "strengthen merged-image-map validation"
    requirement names, all internally consistent with `merged_bytes`. A
    test that wants to prove one specific field's own corruption is
    rejected should take this valid baseline and mutate exactly that one
    field, never construct a partial map by hand (which would trigger the
    unrelated "missing field" check instead of the one actually under
    test)."""
    app_bytes = merged_bytes[app_offset:]
    if audited_app_bytes is None:
        audited_app_bytes = app_bytes
    return {
        "chip": chip,
        "flash_offset": "0x000000",
        "partition_table_offset": "0x008000",
        "application_offset": f"0x{app_offset:06X}",
        "merged_binary": "MtkCore.bin",
        "merged_binary_size": len(merged_bytes),
        "md5_uppercase_hex": pr.md5_hex(merged_bytes),
        "sha256_uppercase_hex": pr.sha256_hex(merged_bytes),
        "app_size_bytes": len(app_bytes),
        "app_md5_uppercase_hex": pr.md5_hex(app_bytes),
        "app_sha256_uppercase_hex": pr.sha256_hex(app_bytes),
        "audited_application_input": {
            "file": "mtkcore.bin",
            "size": len(audited_app_bytes),
            "md5_uppercase_hex": pr.md5_hex(audited_app_bytes),
            "sha256_uppercase_hex": pr.sha256_hex(audited_app_bytes),
        },
        "segments": [
            {"offset": "0x000000", "file": "bootloader/bootloader.bin"},
            {"offset": "0x008000", "file": "partition_table/partition-table.bin"},
            {"offset": f"0x{app_offset:06X}", "file": "mtkcore.bin"},
        ],
    }


def esptool_available():
    """True only if `python -m esptool` is genuinely importable/runnable
    by THIS interpreter -- host_tests/ (this script's own normal ctest
    invocation, via the plain system Python3, not an ESP-IDF-sourced one)
    must stay runnable with no target toolchain installed at all, matching
    this whole suite's own established "no target toolchain needed"
    property. The esptool-dependent negative test below is skipped (with
    a clear, explicit NOTE, never silently) rather than failing or
    fabricating a pass when this is false."""
    try:
        r = subprocess.run([sys.executable, "-m", "esptool", "version"],
                            capture_output=True, text=True, timeout=30)
        return r.returncode == 0
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=None,
                     help="Project root (so --root/release can be cross-checked with a second, "
                          "independent MD5 implementation against test_release_artifact.c's own "
                          "hand-rolled one). Optional: if omitted, or release/ doesn't exist there "
                          "yet, only the synthetic negative-test battery below runs.")
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="mtek_release_negative_")
    try:
        bin_a = os.path.join(tmp, "a.bin")
        bin_b = os.path.join(tmp, "b.bin")
        md5_a = os.path.join(tmp, "a.md5")
        md5_b = os.path.join(tmp, "b.md5")

        with open(bin_a, "wb") as f:
            f.write(b"\xe9" + b"\x00" * 4095)
        with open(bin_b, "wb") as f:
            f.write(b"\xe9" + b"\x11" * 4095)
        with open(md5_a, "wb") as f:
            f.write(hashlib.md5(open(bin_a, "rb").read()).hexdigest().upper().encode("ascii"))
        with open(md5_b, "wb") as f:
            f.write(hashlib.md5(open(bin_b, "rb").read()).hexdigest().upper().encode("ascii"))

        failures = []

        # A genuinely valid pair must pass.
        if validate_pair(bin_a, md5_a):
            failures.append("valid pair was rejected")

        # Missing binary.
        if not validate_pair(os.path.join(tmp, "does_not_exist.bin"), md5_a):
            failures.append("missing binary was accepted")

        # Missing sidecar.
        if not validate_pair(bin_a, os.path.join(tmp, "does_not_exist.md5")):
            failures.append("missing sidecar was accepted")

        # Malformed sidecar: lowercase.
        md5_lower = os.path.join(tmp, "lower.md5")
        with open(md5_lower, "wb") as f:
            f.write(hashlib.md5(open(bin_a, "rb").read()).hexdigest().lower().encode("ascii"))
        if not validate_pair(bin_a, md5_lower):
            failures.append("lowercase sidecar was accepted")

        # Malformed sidecar: trailing newline.
        md5_nl = os.path.join(tmp, "nl.md5")
        with open(md5_nl, "wb") as f:
            f.write(hashlib.md5(open(bin_a, "rb").read()).hexdigest().upper().encode("ascii") + b"\n")
        if not validate_pair(bin_a, md5_nl):
            failures.append("sidecar with trailing newline was accepted")

        # One-byte binary mutation invalidates the sidecar.
        bin_mut = os.path.join(tmp, "mut.bin")
        data = bytearray(open(bin_a, "rb").read())
        data[100] ^= 0xFF
        with open(bin_mut, "wb") as f:
            f.write(bytes(data))
        if not validate_pair(bin_mut, md5_a):
            failures.append("one-byte-mutated binary was accepted against the original sidecar")

        # Substituted/renamed mismatched pair.
        if not validate_pair(bin_a, md5_b):
            failures.append("mismatched pair (a.bin against b.md5) was accepted")

        # ---- tools/package_release.py's own merge/map validation logic
        # (requirements: invalid bootloader/partition-table/application
        # image, application bytes differing from the audited build,
        # missing map, stale map). ----
        audited_app = b"\xe9" + bytes(range(256)) * 8  # a deterministic, non-trivial "application" payload

        # A genuinely valid synthetic merged image must pass.
        good_merged, app_offset = make_synthetic_merged_image(audited_app)
        if pr.validate_merged_image(good_merged, audited_app, app_offset):
            failures.append("a valid synthetic merged image was rejected")

        # Invalid bootloader magic.
        bad_bootloader, _ = make_synthetic_merged_image(audited_app, corrupt_bootloader=True)
        if not pr.validate_merged_image(bad_bootloader, audited_app, app_offset):
            failures.append("an invalid bootloader magic byte was accepted")

        # Invalid partition-table magic.
        bad_pt, _ = make_synthetic_merged_image(audited_app, corrupt_partition_table=True)
        if not pr.validate_merged_image(bad_pt, audited_app, app_offset):
            failures.append("an invalid partition-table magic was accepted")

        # Invalid application image magic.
        bad_app_magic, _ = make_synthetic_merged_image(audited_app, corrupt_app=True)
        if not pr.validate_merged_image(bad_app_magic, audited_app, app_offset):
            failures.append("an invalid application image magic byte was accepted")

        # Application bytes differing from the audited build (a merged
        # image whose own app segment does NOT match what was actually
        # audited pre-merge -- e.g. a stale substituted application image).
        different_audited_app = b"\xe9" + bytes(range(256))[::-1] * 8
        if not pr.validate_merged_image(good_merged, different_audited_app, app_offset):
            failures.append("a merged image whose application segment differs from the audited "
                             "build was accepted")

        # ---- RC11 verification correction (independent audit):
        # "byte-compare the bootloader and partition-table regions
        # embedded in MtkCore.bin against the exact build inputs, just as
        # the application is already bound." A valid pair must pass; a
        # non-magic byte corrupted inside either region (never at its own
        # magic-byte position, so this proves the NEW byte-region check
        # specifically, not the pre-existing magic-byte check) must be
        # rejected. ----
        audited_bootloader, audited_pt = audited_regions_from(good_merged)
        if pr.validate_merged_image(good_merged, audited_app, app_offset,
                                     audited_bootloader_bytes=audited_bootloader,
                                     audited_partition_table_bytes=audited_pt):
            failures.append("a valid merged image was rejected when bootloader/partition-table regions "
                             "were also checked against the audited build inputs")

        bad_bootloader_region, _ = make_synthetic_merged_image(audited_app, corrupt_bootloader_region=True)
        if not pr.validate_merged_image(bad_bootloader_region, audited_app, app_offset,
                                         audited_bootloader_bytes=audited_bootloader,
                                         audited_partition_table_bytes=audited_pt):
            failures.append("a merged image with a corrupted non-magic bootloader-region byte (still "
                             "valid magic bytes) was accepted")

        bad_pt_region, _ = make_synthetic_merged_image(audited_app, corrupt_partition_table_region=True)
        if not pr.validate_merged_image(bad_pt_region, audited_app, app_offset,
                                         audited_bootloader_bytes=audited_bootloader,
                                         audited_partition_table_bytes=audited_pt):
            failures.append("a merged image with a corrupted non-magic partition-table-region byte "
                             "(still valid magic bytes) was accepted")

        # Sidecar syntax/match against the synthetic merged image.
        if pr.validate_sidecar(good_merged, pr.md5_hex(good_merged).encode("ascii")):
            failures.append("a valid sidecar for the synthetic merged image was rejected")
        if not pr.validate_sidecar(good_merged, pr.md5_hex(good_merged).lower().encode("ascii")):
            failures.append("a lowercase sidecar for the synthetic merged image was accepted")
        if not pr.validate_sidecar(good_merged, pr.md5_hex(bad_bootloader).encode("ascii")):
            failures.append("a mismatched sidecar for the synthetic merged image was accepted")

        # Missing map.
        if not pr.validate_map_against_bin({}, good_merged, app_offset):
            failures.append("a completely empty (effectively missing) map was accepted")

        # Stale map: valid syntactically, but describes a DIFFERENT binary
        # than the one actually being validated (e.g. left over from an
        # earlier candidate, never regenerated).
        stale_map = make_valid_map(bad_bootloader, app_offset)  # describes the WRONG (corrupted) image
        if not pr.validate_map_against_bin(stale_map, good_merged, app_offset):
            failures.append("a stale map (describing a different binary) was accepted")

        # A genuinely current, valid map must pass.
        current_map = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        if pr.validate_map_against_bin(current_map, good_merged, app_offset, chip="esp32c6",
                                        expected_offsets={"flash_offset": "0x000000",
                                                           "partition_table_offset": "0x008000",
                                                           "application_offset": f"0x{app_offset:06X}"},
                                        audited_app_bytes=audited_app):
            failures.append("a genuinely current, valid map was rejected")

        # ---- RC11 release-finalization correction (independent audit): "Strengthen merged-image-map validation... Add negative
        # tests for each stale or corrupted field." One targeted mutation
        # per required field, each against an otherwise-valid baseline map
        # (never a hand-built partial map, which would trip the unrelated
        # "missing field" check instead of the field actually under test). ----
        field_corruption_cases = [
            ("chip", "esp32s3"),
            ("flash_offset", "0x001000"),
            ("partition_table_offset", "0x009000"),
            ("application_offset", "0x020000"),
            ("merged_binary", "SomeOtherName.bin"),  # RC11 verification correction: "require merged_binary == MtkCore.bin"
            ("merged_binary_size", current_map["merged_binary_size"] + 1),
            ("md5_uppercase_hex", "0" * 32),
            ("sha256_uppercase_hex", "0" * 64),
            ("app_size_bytes", current_map["app_size_bytes"] + 1),
            ("app_md5_uppercase_hex", "1" * 32),
            ("app_sha256_uppercase_hex", "1" * 64),
        ]
        for field, bad_value in field_corruption_cases:
            corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
            corrupted[field] = bad_value
            if not pr.validate_map_against_bin(corrupted, good_merged, app_offset, chip="esp32c6",
                                                expected_offsets={"flash_offset": "0x000000",
                                                                   "partition_table_offset": "0x008000",
                                                                   "application_offset": f"0x{app_offset:06X}"},
                                                audited_app_bytes=audited_app):
                failures.append(f"a map with a corrupted '{field}' field was accepted")

        # Nested audited_application_input field corruption.
        for field, bad_value in [("size", 999999), ("md5_uppercase_hex", "2" * 32), ("sha256_uppercase_hex", "2" * 64)]:
            corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
            corrupted["audited_application_input"][field] = bad_value
            if not pr.validate_map_against_bin(corrupted, good_merged, app_offset, audited_app_bytes=audited_app):
                failures.append(f"a map with a corrupted audited_application_input.{field} field was accepted")

        # Missing nested audited_application_input entirely.
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        del corrupted["audited_application_input"]
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map missing audited_application_input entirely was accepted")

        # Wrong segment count (a segment silently dropped).
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"] = corrupted["segments"][:2]
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with a missing segment entry was accepted")

        # Wrong segment offset (one entry silently pointing at the wrong offset).
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"][0]["offset"] = "0x000100"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with a wrong segment offset was accepted")

        # Absolute personal filesystem path in a segment entry.
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"][2]["file"] = "/Users/someone/build/mtkcore.bin"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with an absolute-path segment entry was accepted")

        # ---- RC11 verification correction (independent audit):
        # "require the exact mapping 0x000000 -> bootloader/bootloader.bin,
        # 0x008000 -> partition_table/partition-table.bin, 0x010000 ->
        # mtkcore.bin; reject wrong relative filenames, swapped filenames,
        # duplicate offsets, or missing entries." A genuinely valid map
        # (current_map, already proven above) covers "correct mapping
        # accepted"; each mutation below proves exactly one named
        # rejection. ----

        # Duplicate offsets (two entries at the SAME offset; a third,
        # required offset silently never appears at all as a result --
        # still exactly 3 entries, so this is NOT the same case as "wrong
        # segment count" above).
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"][1]["offset"] = corrupted["segments"][0]["offset"]  # both now "0x000000"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with duplicate segment offsets was accepted")

        # Swapped filenames: the correct SET of offsets and the correct SET
        # of filenames are both still present, just paired wrongly -- a
        # plain "is this the expected set of offsets" check (the pre-RC11
        # behavior) would miss this entirely.
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"][0]["file"], corrupted["segments"][1]["file"] = (
            corrupted["segments"][1]["file"], corrupted["segments"][0]["file"])
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with swapped bootloader/partition-table filenames was accepted")

        # Wrong relative filename (right offset, plausible-looking but
        # incorrect path).
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["segments"][0]["file"] = "bootloader/boot.bin"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with a wrong relative bootloader filename was accepted")

        # ---- Release-tooling correction (this round): "audited_application_
        # input.file must equal 'mtkcore.bin', and the application segment
        # mapping must remain exactly 0x010000 -> mtkcore.bin -- even when
        # BOTH fields are changed together to the same wrong value." Three
        # cases, per the requirement: (1) only the nested audited-input
        # field is changed, (2) both the nested field AND the matching
        # segment's file are changed to the SAME incorrect filename (the
        # exact coordinated-corruption case this round's fix closes -- prior
        # to the fix, the segment's own expected filename was DERIVED from
        # this same mutable field, so the two moved in lockstep and this
        # case passed undetected), (3) the genuinely valid current map still
        # passes (already proven above by the `current_map` check, restated
        # here as an explicit control immediately alongside these two so a
        # reader sees all three cases from this requirement together). ----

        # Case 1: only audited_application_input.file changed.
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["audited_application_input"]["file"] = "renamed-app.bin"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with only audited_application_input.file changed to an incorrect "
                             "filename was accepted")

        # Case 2: audited_application_input.file AND the 0x010000 segment's
        # file both changed to the SAME incorrect filename.
        corrupted = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        corrupted["audited_application_input"]["file"] = "renamed-app.bin"
        for seg in corrupted["segments"]:
            if seg["offset"] == f"0x{app_offset:06X}":
                seg["file"] = "renamed-app.bin"
        if not pr.validate_map_against_bin(corrupted, good_merged, app_offset):
            failures.append("a map with audited_application_input.file AND the application segment's file "
                             "BOTH changed to the same incorrect filename ('renamed-app.bin') was accepted "
                             "-- the application-segment expectation must be a fixed literal, never derived "
                             "from the mutable audited_application_input.file field")

        # Case 3 (control): the genuinely current, valid map -- both fields
        # correctly "mtkcore.bin" -- must still pass.
        current_map_control = make_valid_map(good_merged, app_offset, audited_app_bytes=audited_app)
        if pr.validate_map_against_bin(current_map_control, good_merged, app_offset):
            failures.append("a genuinely current, valid map (audited_application_input.file == "
                             "application segment file == 'mtkcore.bin') was rejected")

        # ---- RC11 release-finalization correction (independent audit): "Strengthen package validation so it hard-fails unless
        # esptool validates both the extracted bootloader image and
        # application image, including chip type, image checksum, and
        # validation hash. Magic bytes alone are insufficient. Add a
        # negative test that corrupts a non-magic byte covered by an ESP
        # image checksum/hash and proves rejection." Gated on esptool
        # actually being importable/runnable by THIS interpreter (see
        # esptool_available's own doc comment) so this script stays
        # runnable with no target toolchain installed, matching this
        # whole suite's own established property; skipped with an
        # explicit NOTE, never silently. ----
        if esptool_available():
            real_bootloader = None
            for candidate_dir in ("build_verify_clean", "build"):
                p = os.path.join(args.root or ".", candidate_dir, "bootloader", "bootloader.bin")
                if os.path.isfile(p):
                    real_bootloader = p
                    break
            if real_bootloader:
                good_esp_errors = pr.validate_esp_image(real_bootloader, "esp32c6")
                if good_esp_errors:
                    failures.append(f"a real, valid bootloader image was rejected by validate_esp_image: {good_esp_errors}")

                corrupt_path = os.path.join(tmp, "corrupt_bootloader.bin")
                data = bytearray(open(real_bootloader, "rb").read())
                # Byte 500: well inside a real segment's own body, never
                # the image header/magic byte at offset 0 -- proves the
                # checksum/validation-hash check catches a corruption
                # magic-byte inspection alone would miss entirely.
                data[500] ^= 0xFF
                with open(corrupt_path, "wb") as f:
                    f.write(bytes(data))
                corrupt_esp_errors = pr.validate_esp_image(corrupt_path, "esp32c6")
                if not corrupt_esp_errors:
                    failures.append("a bootloader image with a corrupted non-magic byte (still valid magic "
                                     "bytes) was accepted -- checksum/validation-hash check did not catch it")
            else:
                print("NOTE: no real build_verify_clean/build bootloader.bin found -- skipping the real-"
                      "image esptool corruption test (expected before any idf.py build has run).")
        else:
            print("NOTE: esptool is not importable/runnable by this interpreter -- skipping the esptool "
                  "image-checksum/validation-hash negative test (expected when this script runs via the "
                  "plain host_tests Python3, with no ESP-IDF environment sourced; tools/package_release.py "
                  "itself always runs from an ESP-IDF-sourced environment, where this check is real).")

        # RC9 independent correction order P1 "the synthetic Python negative
        # tests do not validate the actual release pair either": cross-check
        # the REAL release/MtkCore.bin + MtkCore.md5, when present, with this
        # independent MD5 implementation (Python hashlib) -- agreeing with
        # test_release_artifact.c's own hand-rolled RFC 1321 implementation
        # is a real, separate piece of evidence, not a duplicate of it. Not a
        # failure if the release hasn't been packaged yet (this script also
        # runs as part of the pre-packaging host-test pass).
        if args.root:
            real_bin = os.path.join(args.root, "release", "MtkCore.bin")
            real_md5 = os.path.join(args.root, "release", "MtkCore.md5")
            if os.path.isfile(real_bin) or os.path.isfile(real_md5):
                real_errors = validate_pair(real_bin, real_md5)
                if real_errors:
                    failures.append(f"real release/MtkCore.bin + MtkCore.md5 pair failed validation: {real_errors}")
                else:
                    print(f"cross-checked real pair OK: {real_bin}")
            else:
                print("NOTE: real release/MtkCore.bin + MtkCore.md5 not present yet -- "
                      "skipping the real-pair cross-check (expected before packaging).")

        if failures:
            print("FAILED:")
            for f in failures:
                print("  - " + f)
            sys.exit(1)
        print("OK: all release-artifact negative tests passed")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
