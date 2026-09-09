# Development and verification

## Prerequisites

The reviewed target build used ESP-IDF v6.0.1 and ESP32-C6. Host tests declare CMake 3.16 or later and C11; they require a compatible compiler, threading support, and sanitizer runtimes for sanitizer configurations. Python 3 is used by engineering tools.

Record the actual compiler, SDK, Python, CMake, and Ninja versions used by a published build. Do not treat a previous build directory or developer-specific SDK path as an onboarding prerequisite. Keep the SDK external to the source repository unless a separate vendoring decision is made.

## Source tests versus release validation

The current host test configuration enables ASan and UBSan by default through `MTK_SANITIZE`. TSan verification requires a separate configuration; the sanitizer configurations must not be mixed.

The release-artifact test defaults to requiring the binary, MD5 sidecar, merged-image map, partition CSV, and packaging manifest. A source-only checkout intentionally excludes local release artifacts. The existing `MTK_RELEASE_PRE_PACKAGING=1` mode permits the release-artifact test to skip deep validation when required package files are absent. Such a run must be labeled pre-packaging and must never be reported as validating a release package.

From the repository root, configure and run the source-only host suite outside the checkout:

```sh
cmake -S host_tests -B /tmp/mtek-host-build
cmake --build /tmp/mtek-host-build
MTK_RELEASE_PRE_PACKAGING=1 ctest --test-dir /tmp/mtek-host-build --output-on-failure
```

The environment variable skips deep release-package validation only when a
package is absent. Do not use it to claim that a release package passed.

For the ESP32-C6 target, activate ESP-IDF v6.0.1, then run:

```sh
python3 tools/gen_schema.py --schema tools/schemas.json --out components/mtek_schema
python3 tools/gen_compat_map.py --schema tools/schemas.json --out components/mtek_transport_spi_compat
idf.py -B /tmp/mtek-idf-build set-target esp32c6
idf.py -B /tmp/mtek-idf-build build
python3 tools/check_resource_budget.py /tmp/mtek-idf-build/mtkcore.map
```

Use a fresh build directory. The resulting `mtkcore.bin` is an application
image, not the merged updater artifact.

## Generated files

Include the reviewed schema and required generated source in the selected source set. Establish generator idempotence by retaining before/after hashes or a byte comparison across all generated outputs. Run generation before the final target build.

## Stack-budget tool changes build outputs

The current stack-budget tool reuses compiler commands with stack-usage reporting enabled and overwrites ordinary object files. Running it against a candidate's final build directory makes Ninja's stored dependency timestamps stale and schedules recompilation and relinking.

Use an isolated measurement build with the same reviewed source and compiler configuration. Keep the candidate build pristine. A final dry-run may contain documented ESP-IDF bootloader wrapper and size-check steps, but should schedule no source compilation, archive relink, ELF relink, or application regeneration.

## Clean-checkout acceptance

Before publication, verify that the selected source set contains every build input, uses no personal filesystem paths, supports the documented pre-packaging test workflow, and builds with the recorded toolchain. Save sanitized results separately from raw machine logs.
