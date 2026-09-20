#!/usr/bin/env python3
"""Host-side negative tests for the release artifact/sidecar pair, and for
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
import errno
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading

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
    position. They exercise the byte-comparison of the bootloader and
    partition-table regions -- binding them just as the application is
    already bound -- distinct from `corrupt_bootloader`/`corrupt_partition_
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


def run_atomic_noclobber_rename_toctou_tests(tmp):
    """Deterministic proof that tools/package_release.py's atomic_
    noclobber_rename genuinely closes the finalization-boundary TOCTOU
    race a plain `os.path.lexists(dst)` preflight check followed by a
    later os.replace/os.rename could lose: a destination that comes into
    existence AFTER that early check ran -- whether it was already there
    by the time the real publish step executes, or is created by a
    concurrent actor racing directly against the publish call itself --
    must never be silently replaced, and the losing side (whichever one
    loses) must be left completely untouched: never partially moved,
    merged, or corrupted, and never silently consumed."""
    failures = []

    # Case A: destination already exists before atomic_noclobber_rename is
    # ever called -- the simplest instance of the race, exactly what the
    # early os.path.lexists() convenience check in main() is meant to
    # catch, but exercised here directly against the real safety
    # guarantee (the atomic primitive itself), independent of that
    # earlier, merely-cosmetic check. Independent-review addendum P1
    # "direct no-clobber proof covers only directories": extended to all
    # four destination kinds a real --out path could already be --
    # directory, plain file, a non-dangling symlink (must not be followed
    # into, and its target's own content must survive untouched), and a
    # DANGLING symlink (the exact case a naive os.path.exists() check
    # would miss entirely, since it follows symlinks and would report
    # "absent" for one whose target does not exist -- os.path.lexists()/
    # the real rename primitives operate on the directory entry itself,
    # regardless of what it points to, so this proves the fix actually
    # rejects that case too).
    def make_destination(case_dir, kind):
        """Creates a pre-existing destination of `kind` at case_dir/out.
        Returns (dst_path, verify_fn); verify_fn() returns a list of
        failure strings (empty if the destination's own exact content/
        link target survived a rejected rename untouched)."""
        dst = os.path.join(case_dir, "out")
        if kind == "directory":
            os.makedirs(dst)
            with open(os.path.join(dst, "marker.txt"), "w") as f:
                f.write("pre-existing, must survive\n")

            def verify():
                if not os.path.isdir(dst) or os.path.islink(dst):
                    return ["destination directory was replaced or is no longer a plain directory"]
                if not os.path.isfile(os.path.join(dst, "marker.txt")) or \
                        open(os.path.join(dst, "marker.txt")).read() != "pre-existing, must survive\n":
                    return ["destination directory's own content was modified"]
                return []
            return dst, verify
        if kind == "file":
            with open(dst, "w") as f:
                f.write("pre-existing file content, must survive\n")

            def verify():
                if not os.path.isfile(dst) or os.path.islink(dst):
                    return ["destination file was replaced or is no longer a plain file"]
                if open(dst).read() != "pre-existing file content, must survive\n":
                    return ["destination file's own content was modified"]
                return []
            return dst, verify
        if kind == "symlink_valid":
            target_dir = os.path.join(case_dir, "symlink_target")
            os.makedirs(target_dir)
            with open(os.path.join(target_dir, "must_not_change.txt"), "w") as f:
                f.write("target content, must survive\n")
            os.symlink(target_dir, dst)

            def verify():
                if not os.path.islink(dst) or os.readlink(dst) != target_dir:
                    return ["non-dangling symlink destination was replaced or repointed"]
                if not os.path.isfile(os.path.join(target_dir, "must_not_change.txt")) or \
                        open(os.path.join(target_dir, "must_not_change.txt")).read() != "target content, must survive\n":
                    return ["non-dangling symlink's own target content was modified -- it must never be followed into"]
                return []
            return dst, verify
        if kind == "symlink_dangling":
            dangling_target = os.path.join(case_dir, "does_not_exist_target")
            os.symlink(dangling_target, dst)

            def verify():
                errs = []
                if not os.path.islink(dst) or os.readlink(dst) != dangling_target:
                    errs.append("dangling symlink destination was replaced or repointed")
                if os.path.exists(dangling_target):
                    errs.append("dangling symlink's own (nonexistent) target unexpectedly now exists")
                return errs
            return dst, verify
        raise ValueError(kind)

    for kind in ("directory", "file", "symlink_valid", "symlink_dangling"):
        for trial in range(5):
            case_dir = os.path.join(tmp, f"toctou_pre_{kind}_{trial}")
            os.makedirs(case_dir)
            dst, verify = make_destination(case_dir, kind)
            src = os.path.join(case_dir, "staging")
            os.makedirs(src)
            with open(os.path.join(src, "payload.txt"), "w") as f:
                f.write("staged package content\n")

            try:
                pr.atomic_noclobber_rename(src, dst)
                failures.append(f"toctou pre-existing case ({kind}, trial {trial}): "
                                 f"atomic_noclobber_rename clobbered a pre-existing "
                                 f"{kind} destination instead of raising")
                continue
            except OSError as e:
                if e.errno != errno.EEXIST:
                    failures.append(f"toctou pre-existing case ({kind}, trial {trial}): "
                                     f"unexpected errno {e.errno} (expected EEXIST)")
            failures.extend(f"toctou pre-existing case ({kind}, trial {trial}): {msg}" for msg in verify())
            if not os.path.isdir(src) or not os.path.isfile(os.path.join(src, "payload.txt")):
                failures.append(f"toctou pre-existing case ({kind}, trial {trial}): the staging "
                                 f"directory was consumed/removed despite the rename being "
                                 f"rejected -- it must be left in place for the caller to clean up")

    # Case B: destination is created by a CONCURRENT actor racing directly
    # against atomic_noclobber_rename itself -- the actual finalization-
    # boundary window a check-then-os.replace pair cannot close (the
    # earlier check can genuinely pass, then this exact race lands before
    # the replace runs). A barrier lines the two threads up as tightly as
    # the scheduler allows; both do real blocking OS calls (os.makedirs /
    # the renamex_np-or-renameat2-backed atomic_noclobber_rename), which
    # release the GIL, so this is a genuine concurrent race, not one
    # serialized by the interpreter. Repeated across many trials so the
    # race is actually exercised rather than incidentally won by
    # whichever thread the scheduler happens to run first every time --
    # though the OS-level atomicity guarantee being proved holds
    # regardless of ordering, which is exactly the point.
    for trial in range(50):
        case_dir = os.path.join(tmp, f"toctou_race_{trial}")
        os.makedirs(case_dir)
        dst = os.path.join(case_dir, "out")
        src = os.path.join(case_dir, "staging")
        os.makedirs(src)
        with open(os.path.join(src, "payload.txt"), "w") as f:
            f.write("staged package content\n")

        if os.path.lexists(dst):
            failures.append(f"toctou race case {trial}: test setup invalid -- dst already "
                             f"exists before the race even starts")
            continue

        barrier = threading.Barrier(2)
        injector_result = {}
        rename_result = {}

        def injector():
            barrier.wait()
            try:
                os.makedirs(dst)
                with open(os.path.join(dst, "marker.txt"), "w") as f:
                    f.write("concurrently-created destination\n")
                injector_result["ok"] = True
            except OSError as e:
                injector_result["ok"] = False
                injector_result["errno"] = e.errno

        def renamer():
            barrier.wait()
            try:
                pr.atomic_noclobber_rename(src, dst)
                rename_result["ok"] = True
            except OSError as e:
                rename_result["ok"] = False
                rename_result["errno"] = e.errno

        t1 = threading.Thread(target=injector)
        t2 = threading.Thread(target=renamer)
        t1.start(); t2.start()
        t1.join(); t2.join()

        injector_won = injector_result.get("ok") is True
        rename_won = rename_result.get("ok") is True

        if injector_won == rename_won:
            failures.append(f"toctou race case {trial}: exactly one side must win the race -- "
                             f"got injector_ok={injector_won} rename_ok={rename_won}")
            continue

        if rename_won:
            if injector_result.get("errno") != errno.EEXIST:
                failures.append(f"toctou race case {trial}: rename won but the injector's own "
                                 f"os.makedirs did not fail with EEXIST ({injector_result})")
            if os.path.isdir(src):
                failures.append(f"toctou race case {trial}: rename reported success but its "
                                 f"own staging directory was not actually consumed")
            if not os.path.isfile(os.path.join(dst, "payload.txt")):
                failures.append(f"toctou race case {trial}: rename reported success but dst "
                                 f"does not contain the staged payload")
        else:
            if rename_result.get("errno") != errno.EEXIST:
                failures.append(f"toctou race case {trial}: injector won but "
                                 f"atomic_noclobber_rename did not fail with EEXIST "
                                 f"({rename_result})")
            if not os.path.isdir(dst) or not os.path.isfile(os.path.join(dst, "marker.txt")) or \
                    open(os.path.join(dst, "marker.txt")).read() != "concurrently-created destination\n":
                failures.append(f"toctou race case {trial}: the concurrently-created "
                                 f"destination was clobbered or corrupted by the losing rename")
            if not os.path.isdir(src) or not os.path.isfile(os.path.join(src, "payload.txt")):
                failures.append(f"toctou race case {trial}: the losing rename's own staging "
                                 f"directory was consumed/removed despite losing the race -- it "
                                 f"must be left in place for the caller to clean up")

    return failures


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

        # ----
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

        # ---- "Strengthen merged-image-map validation... Add negative
        # tests for each stale or corrupted field." One targeted mutation
        # per required field, each against an otherwise-valid baseline map
        # (never a hand-built partial map, which would trip the unrelated
        # "missing field" check instead of the field actually under test). ----
        field_corruption_cases = [
            ("chip", "esp32s3"),
            ("flash_offset", "0x001000"),
            ("partition_table_offset", "0x009000"),
            ("application_offset", "0x020000"),
            ("merged_binary", "SomeOtherName.bin"),  # "require merged_binary == MtkCore.bin"
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

        # ----
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

        # ---- Release-tooling correction (): "audited_application_
        # input.file must equal 'mtkcore.bin', and the application segment
        # mapping must remain exactly 0x010000 -> mtkcore.bin -- even when
        # BOTH fields are changed together to the same wrong value." Three
        # cases, per the requirement: (1) only the nested audited-input
        # field is changed, (2) both the nested field AND the matching
        # segment's file are changed to the SAME incorrect filename (the
        # exact coordinated-corruption case's fix closes -- prior
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

        # ---- "Strengthen package validation so it hard-fails unless
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

        # "the synthetic Python negative
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

        # ---- package_release.py's own source-VCS-identity detection
        # (gather_build_identity/detect_source_vcs_identity): a correction
        # found this manifest field previously hardcoded "no VCS" regardless
        # of the real --project-root, which is false whenever packaging
        # actually runs against a real Git worktree/clone. These checks
        # exercise the real function against a real, disposable git repo
        # (not a second reimplementation of it). ----
        vcs_repo = os.path.join(tmp, "vcs_test_repo")
        os.makedirs(vcs_repo)
        git_env = dict(os.environ)
        git_env.update({
            "GIT_AUTHOR_NAME": "test", "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_NAME": "test", "GIT_COMMITTER_EMAIL": "test@example.invalid",
        })

        def git(*cmd_args):
            return subprocess.run(["git", "-C", vcs_repo] + list(cmd_args),
                                   capture_output=True, text=True, env=git_env)

        git("init", "-q")
        with open(os.path.join(vcs_repo, "f.txt"), "w") as f:
            f.write("v1\n")
        git("add", "f.txt")
        git("commit", "-q", "-m", "initial")
        real_commit = git("rev-parse", "HEAD").stdout.strip()

        # exact clean commit recorded
        identity = pr.gather_build_identity(tmp, vcs_repo)
        if identity.get("vcs") != "git" or identity.get("source_commit") != real_commit:
            failures.append(f"a real clean git worktree was not reported as vcs=git with the exact "
                             f"commit ({identity})")
        if identity.get("source_dirty") is not False:
            failures.append(f"a genuinely clean git worktree was not reported as source_dirty=False "
                             f"({identity})")

        # dirty source clearly recorded (release policy: package_release.py's
        # own main() refuses to package when this is True -- see the
        # "PACKAGING FAILED (dirty source tree)" branch)
        with open(os.path.join(vcs_repo, "f.txt"), "a") as f:
            f.write("uncommitted change\n")
        dirty_identity = pr.gather_build_identity(tmp, vcs_repo)
        if dirty_identity.get("vcs") != "git" or dirty_identity.get("source_dirty") is not True:
            failures.append(f"an uncommitted working-tree change was not reported as source_dirty=True "
                             f"({dirty_identity})")
        if dirty_identity.get("source_commit") != real_commit:
            failures.append("a dirty working tree changed the reported commit hash, but only the "
                             "index/worktree changed, not HEAD")
        git("checkout", "-q", "--", "f.txt")  # restore clean for the determinism check below

        # no-VCS source represented honestly
        novcs_dir = os.path.join(tmp, "no_vcs_dir")
        os.makedirs(novcs_dir)
        novcs_identity = pr.gather_build_identity(tmp, novcs_dir)
        if novcs_identity.get("vcs") != "none" or novcs_identity.get("source_commit") is not None \
                or novcs_identity.get("source_dirty") is not None:
            failures.append(f"a directory with no .git was not honestly reported as vcs=none/"
                             f"commit=None/dirty=None ({novcs_identity})")

        # malformed/forged metadata rejected: the same 40-hex-char guard
        # gather_build_identity relies on must reject anything else, rather
        # than trusting arbitrary git output verbatim.
        for bad in ("", "not-a-commit", "a" * 39, "a" * 41, "g" * 40, real_commit.upper()):
            if pr._COMMIT_HASH_RE.match(bad):
                failures.append(f"the commit-hash validator accepted a malformed/forged value: {bad!r}")

        # generated manifest remains deterministic: same clean commit twice
        # in a row must produce byte-identical PACKAGING_MANIFEST.md text.
        fake_image_map = {"merged_binary": "MtkCore.bin", "merged_binary_size": 1,
                           "flash_offset": "0x000000", "partition_table_offset": "0x008000",
                           "application_offset": "0x010000",
                           "md5_uppercase_hex": "0" * 32, "sha256_uppercase_hex": "0" * 64,
                           "app_size_bytes": 1, "app_md5_uppercase_hex": "0" * 32,
                           "app_sha256_uppercase_hex": "0" * 64}

        det_identity = pr.gather_build_identity(tmp, vcs_repo)
        out1 = os.path.join(tmp, "manifest_det_1")
        out2 = os.path.join(tmp, "manifest_det_2")
        os.makedirs(out1)
        os.makedirs(out2)
        pr.write_release_notes(out1, fake_image_map, det_identity, False)
        pr.write_release_notes(out2, fake_image_map, det_identity, False)
        text1 = open(os.path.join(out1, "PACKAGING_MANIFEST.md")).read()
        text2 = open(os.path.join(out2, "PACKAGING_MANIFEST.md")).read()
        if text1 != text2:
            failures.append("PACKAGING_MANIFEST.md was not deterministic across two runs against the "
                             "identical clean commit")
        if vcs_repo.replace(os.sep, "/") in text1 or tmp.replace(os.sep, "/") in text1:
            failures.append("PACKAGING_MANIFEST.md leaked an absolute filesystem path from the "
                             "source-VCS-identity fields")

        # ---- build/source-root binding (verify_build_source_root) and the
        # three end-to-end fail-closed/no-partial-output invariants in
        # package_release.py's own main(): dirty source, unusable/malformed
        # Git metadata, and an unrelated project root labeling someone
        # else's build. All three are exercised against the REAL CLI
        # entry point (subprocess, not a reimplementation of main()'s own
        # decision), and each must leave the requested --out path exactly
        # as it was found: absent if it never existed, or with a
        # pre-existing sentinel file untouched if it did. ----
        unrelated_repo = os.path.join(tmp, "unrelated_repo")
        os.makedirs(unrelated_repo)
        subprocess.run(["git", "-C", unrelated_repo, "init", "-q"], env=git_env)
        with open(os.path.join(unrelated_repo, "g.txt"), "w") as f:
            f.write("v1\n")
        subprocess.run(["git", "-C", unrelated_repo, "add", "g.txt"], env=git_env)
        subprocess.run(["git", "-C", unrelated_repo, "commit", "-q", "-m", "initial"], env=git_env)

        fake_build_dir = os.path.join(tmp, "fake_build_dir")
        os.makedirs(fake_build_dir)
        with open(os.path.join(fake_build_dir, "project_description.json"), "w") as f:
            json.dump({"project_path": vcs_repo, "git_revision": "v6.0.1", "target": "esp32c6"}, f)
        with open(os.path.join(fake_build_dir, "CMakeCache.txt"), "w") as f:
            f.write(f"CMAKE_HOME_DIRECTORY:INTERNAL={vcs_repo}\n")

        # direct function-level checks (the exact function main() calls)
        ok, err = pr.verify_build_source_root(fake_build_dir, vcs_repo)
        if not ok:
            failures.append(f"verify_build_source_root rejected a genuinely matching build/source "
                             f"root pair: {err}")
        ok2, err2 = pr.verify_build_source_root(fake_build_dir, unrelated_repo)
        if ok2:
            failures.append("verify_build_source_root accepted a build directory paired with an "
                             "unrelated, unconnected project root")

        def run_packaging_cli(project_root, out_dir, build_dir=fake_build_dir):
            return subprocess.run(
                [sys.executable, pr.__file__, "--build-dir", os.path.relpath(build_dir, project_root),
                 "--project-root", project_root, "--out", out_dir],
                capture_output=True, text=True, env=git_env)

        def assert_output_absent_or_untouched(case_name, out_dir, sentinel_path, proc):
            if proc.returncode == 0:
                failures.append(f"{case_name}: packaging CLI exited 0 (expected a non-zero rejection)")
            if os.path.isdir(out_dir):
                if os.path.isfile(sentinel_path):
                    with open(sentinel_path) as f:
                        if f.read() != "sentinel\n":
                            failures.append(f"{case_name}: pre-existing sentinel file was modified")
                else:
                    failures.append(f"{case_name}: pre-existing sentinel file was removed")
                unexpected = set(os.listdir(out_dir)) - {"sentinel.txt"}
                if unexpected:
                    failures.append(f"{case_name}: packaging wrote unexpected files into --out "
                                     f"despite rejecting the package: {sorted(unexpected)}")
            # if out_dir does not exist at all, that's also an acceptable
            # "untouched" outcome for a case where --out never pre-existed.

        # Case: build/source-root mismatch (unrelated clean repo)
        out_mismatch = os.path.join(tmp, "out_root_mismatch")
        os.makedirs(out_mismatch)
        with open(os.path.join(out_mismatch, "sentinel.txt"), "w") as f:
            f.write("sentinel\n")
        proc = run_packaging_cli(unrelated_repo, out_mismatch)
        assert_output_absent_or_untouched("build-root mismatch", out_mismatch,
                                           os.path.join(out_mismatch, "sentinel.txt"), proc)
        if "binding" not in (proc.stdout + proc.stderr).lower():
            failures.append(f"build-root mismatch: rejection message did not mention binding "
                             f"verification (stdout/stderr: {(proc.stdout + proc.stderr)[:400]!r})")

        # Case: dirty source tree
        with open(os.path.join(vcs_repo, "f.txt"), "a") as f:
            f.write("dirty again\n")
        out_dirty = os.path.join(tmp, "out_dirty")
        os.makedirs(out_dirty)
        with open(os.path.join(out_dirty, "sentinel.txt"), "w") as f:
            f.write("sentinel\n")
        proc = run_packaging_cli(vcs_repo, out_dirty)
        assert_output_absent_or_untouched("dirty source", out_dirty,
                                           os.path.join(out_dirty, "sentinel.txt"), proc)
        subprocess.run(["git", "-C", vcs_repo, "checkout", "-q", "--", "f.txt"], env=git_env)

        # Case: unusable/malformed Git metadata (.git marker present but
        # not a real repository -- `git rev-parse HEAD` must fail against
        # it, driving the vcs_error hard-fail path, not a silent "no VCS"
        # fallback).
        broken_git_repo = os.path.join(tmp, "broken_git_repo")
        os.makedirs(os.path.join(broken_git_repo, ".git"))  # present but empty/invalid
        broken_check = subprocess.run(["git", "-C", broken_git_repo, "rev-parse", "HEAD"],
                                       capture_output=True, text=True)
        if broken_check.returncode == 0:
            failures.append("test setup invalid: an empty .git directory was accepted by "
                             "`git rev-parse HEAD` -- cannot exercise the malformed-Git-metadata path")
        else:
            broken_identity = pr.gather_build_identity(tmp, broken_git_repo)
            if not broken_identity.get("vcs_error"):
                failures.append(f"gather_build_identity did not report vcs_error for a broken .git "
                                 f"directory ({broken_identity})")
            out_broken = os.path.join(tmp, "out_broken_git")
            os.makedirs(out_broken)
            with open(os.path.join(out_broken, "sentinel.txt"), "w") as f:
                f.write("sentinel\n")
            proc = run_packaging_cli(broken_git_repo, out_broken)
            assert_output_absent_or_untouched("malformed Git metadata", out_broken,
                                               os.path.join(out_broken, "sentinel.txt"), proc)
            if "unusable git metadata" not in (proc.stdout + proc.stderr).lower():
                failures.append(f"malformed Git metadata: rejection message did not mention it "
                                 f"(stdout/stderr: {(proc.stdout + proc.stderr)[:400]!r})")

        # ---- atomic, fail-closed --out publish (main()'s single
        # atomic_noclobber_rename call -- an OS-level atomic no-clobber
        # rename, replacing both the original per-entry shutil.move loop
        # AND a later, still-TOCTOU-vulnerable lexists-then-os.replace
        # pair). A source-level check always runs (no toolchain needed);
        # the end-to-end file/directory/symlink refusal and stale-extras
        # checks additionally need a REAL build directory (flasher_args.json
        # + real ESP images) to reach the actual publish step, so they run
        # only when one can be found -- consistent with this file's own
        # existing esptool_available() gating convention. The dedicated
        # TOCTOU-race tests below (Case A/B) exercise the real primitive
        # directly and need no build directory at all. ----
        pr_source = open(pr.__file__).read()
        if "shutil.move(os.path.join(staging_dir" in pr_source:
            failures.append("package_release.py's main() still contains a per-entry "
                             "shutil.move finalization loop; expected a single "
                             "atomic_noclobber_rename call")
        if "os.replace(staging_dir, requested_out_dir)" in pr_source:
            failures.append("package_release.py's main() still finalizes via a bare "
                             "os.replace(staging_dir, requested_out_dir) -- a check-then-replace "
                             "TOCTOU; expected atomic_noclobber_rename")
        if pr_source.count("atomic_noclobber_rename(staging_dir, requested_out_dir)") != 1:
            failures.append("package_release.py's main() does not install the staged package "
                             "via exactly one atomic_noclobber_rename(staging_dir, "
                             "requested_out_dir) call")
        if "os.path.lexists(requested_out_dir)" not in pr_source:
            failures.append("package_release.py's main() does not guard --out with "
                             "os.path.lexists (required to refuse a symlink without following it)")

        # Deterministic proof (owner-approved sanitizer-fix round: "add a
        # deterministic test that injects creation of the destination at
        # the finalization boundary and proves the pre-existing file/
        # directory/symlink is not replaced") that atomic_noclobber_rename
        # itself -- the real safety guarantee, not merely the early
        # convenience check above -- actually closes the race.
        failures.extend(run_atomic_noclobber_rename_toctou_tests(tmp))

        real_build_dir = os.environ.get("MTK_TEST_REAL_BUILD_DIR")
        if not real_build_dir:
            for candidate in ("build_verify_clean", "build"):
                p = os.path.join(args.root or ".", candidate)
                if os.path.isfile(os.path.join(p, "flasher_args.json")):
                    real_build_dir = p
                    break

        if esptool_available() and real_build_dir and os.path.isfile(os.path.join(real_build_dir, "flasher_args.json")):
            real_root = args.root or "."

            # a fresh path: exactly the expected file set, no stale extras
            out_fresh = os.path.join(tmp, "out_fresh_valid")
            proc = run_packaging_cli(real_root, out_fresh, build_dir=real_build_dir)
            if proc.returncode != 0:
                failures.append(f"a valid invocation to a brand-new --out path failed unexpectedly: "
                                 f"{(proc.stdout + proc.stderr)[-800:]!r}")
            elif not os.path.isdir(out_fresh):
                failures.append("a valid invocation to a brand-new --out path did not create it")
            else:
                got = set(os.listdir(out_fresh))
                expected = {"MtkCore.bin", "MtkCore.md5", "merged_image_map.json",
                            "PACKAGING_MANIFEST.md", "partitions.csv"}
                if got != expected:
                    failures.append(f"a valid invocation produced an unexpected file set: got {sorted(got)}, "
                                     f"expected {sorted(expected)}")

            # pre-existing FILE at --out: refused, file content untouched
            out_file = os.path.join(tmp, "out_is_a_file")
            with open(out_file, "w") as f:
                f.write("pre-existing file, not a package\n")
            proc = run_packaging_cli(real_root, out_file, build_dir=real_build_dir)
            if proc.returncode == 0:
                failures.append("a valid invocation was accepted when --out was a pre-existing FILE")
            if not os.path.isfile(out_file) or open(out_file).read() != "pre-existing file, not a package\n":
                failures.append("--out being a pre-existing file was modified despite rejection")

            # pre-existing NON-EMPTY DIRECTORY at --out: refused, untouched
            out_nonempty = os.path.join(tmp, "out_is_nonempty_dir")
            os.makedirs(out_nonempty)
            with open(os.path.join(out_nonempty, "keep.txt"), "w") as f:
                f.write("must survive\n")
            proc = run_packaging_cli(real_root, out_nonempty, build_dir=real_build_dir)
            if proc.returncode == 0:
                failures.append("a valid invocation was accepted when --out was a pre-existing "
                                 "non-empty directory")
            if sorted(os.listdir(out_nonempty)) != ["keep.txt"] or \
                    open(os.path.join(out_nonempty, "keep.txt")).read() != "must survive\n":
                failures.append("--out being a pre-existing non-empty directory was modified "
                                 "despite rejection")

            # pre-existing SYMLINK at --out: refused, neither the symlink
            # nor its target's contents touched, and the target is never
            # deleted or followed-into.
            symlink_target = os.path.join(tmp, "symlink_target_dir")
            os.makedirs(symlink_target)
            with open(os.path.join(symlink_target, "must_not_change.txt"), "w") as f:
                f.write("target content\n")
            out_symlink = os.path.join(tmp, "out_is_a_symlink")
            os.symlink(symlink_target, out_symlink)
            proc = run_packaging_cli(real_root, out_symlink, build_dir=real_build_dir)
            if proc.returncode == 0:
                failures.append("a valid invocation was accepted when --out was a pre-existing symlink")
            if not os.path.islink(out_symlink) or os.readlink(out_symlink) != symlink_target:
                failures.append("--out being a pre-existing symlink was deleted or repointed "
                                 "despite rejection")
            if sorted(os.listdir(symlink_target)) != ["must_not_change.txt"] or \
                    open(os.path.join(symlink_target, "must_not_change.txt")).read() != "target content\n":
                failures.append("the symlink target directory's contents were modified despite "
                                 "packaging being rejected (the symlink must never be followed into)")
        else:
            print("NOTE: no real build directory with flasher_args.json found (and/or esptool "
                  "unavailable) -- skipping the real end-to-end --out file/directory/symlink "
                  "refusal and fresh-path file-set tests (expected when this script runs via the "
                  "plain host_tests Python3 with no prior idf.py build present; set "
                  "MTK_TEST_REAL_BUILD_DIR to a real build directory, or run from an ESP-IDF-"
                  "sourced environment with build_verify_clean/ or build/ present, to exercise them).")

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
