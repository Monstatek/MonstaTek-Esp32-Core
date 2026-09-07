#!/usr/bin/env python3
"""RC6 independent audit gate #2 "The exact target stack/RAM/heap budget is
published and mechanically checked in CI/build scripts": parses the real
ESP-IDF linker map for this build (`idf.py size`'s own underlying
`esp_idf_size` tool, `--format json2`) and fails if the DIRAM (the shared
internal SRAM region backing every static .bss/.data allocation AND every
FreeRTOS heap allocation at runtime -- task stacks, esp_wifi_init, the
NimBLE stack, lwIP, NVS) usage leaves less than MIN_FREE_DIRAM_BYTES free
for runtime heap.

This is a *static*-allocation gate, not a hardware-measured free-heap
gate: it catches a build whose own .bss/.data footprint alone leaves an
implausibly small (or negative) remainder for anything FreeRTOS/Wi-Fi/BLE
allocates at runtime -- exactly the RC6 audit's own finding (441,154 of
452,112 DIRAM bytes, 97.6%, consumed by static allocations before this
round's fixes, leaving 10,958 bytes for every runtime heap need). It
cannot and does not claim the chip will actually boot -- that needs real
hardware (docs/PROVENANCE.md's own disclosed hardware-only gaps) -- only
that the static-allocation half of the budget is measured and bounded by
an explicit, documented threshold, mechanically enforced here rather than
asserted in prose alone.

Usage (after a normal `idf.py build`, from the project root):
    python3 tools/check_resource_budget.py [path/to/build/mtkcore.map]
Defaults to build/mtkcore.map. Requires the `esp_idf_size` package that
ships with the same ESP-IDF `export.sh` environment `idf.py build` itself
needs (not a plain host-only dependency -- this check is only meaningful
against a real ESP-IDF target build's own linker map, unlike every
host_tests/ ctest, which needs no target toolchain at all).
"""
import json
import subprocess
import sys

# See docs/RESOURCE_BUDGET.md's "RC6 measured SRAM conflict" section for
# the full before/after accounting this threshold is derived from. Chosen
# with real margin under the post-fix measured free figure (182,990 bytes
# on this sdkconfig as of this check's own last real measurement) so a
# modest future static-allocation growth does not immediately fail this
# gate, while still catching a regression back toward the pre-fix crisis
# long before it reaches that extreme.
MIN_FREE_DIRAM_BYTES = 100_000


def main():
    map_path = sys.argv[1] if len(sys.argv) > 1 else "build/mtkcore.map"
    try:
        raw = subprocess.check_output(
            [sys.executable, "-m", "esp_idf_size", "--format", "json2", map_path],
            stderr=subprocess.STDOUT,
        )
    except FileNotFoundError:
        print("check_resource_budget: esp_idf_size not importable -- run this from an "
              "ESP-IDF `export.sh`-sourced environment after `idf.py build` (see this "
              "script's own module docstring), not a plain host shell.", file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as e:
        print(f"check_resource_budget: esp_idf_size failed:\n{e.output.decode(errors='replace')}", file=sys.stderr)
        return 2

    data = json.loads(raw)
    diram = next((l for l in data["layout"] if l.get("name") == "DIRAM"), None)
    if diram is None:
        print("check_resource_budget: no DIRAM region in the linker map -- unexpected "
              "target/sdkconfig, update this script's region name before trusting it.", file=sys.stderr)
        return 2

    used, total, free = diram["used"], diram["total"], diram["free"]
    pct = 100.0 * used / total if total else 0.0
    print(f"DIRAM: used={used} total={total} free={free} ({pct:.1f}% used)")

    if free < MIN_FREE_DIRAM_BYTES:
        print(f"check_resource_budget: FAIL -- only {free} bytes of DIRAM free for runtime "
              f"heap (FreeRTOS task stacks, esp_wifi_init, NimBLE, lwIP, NVS), below the "
              f"documented {MIN_FREE_DIRAM_BYTES}-byte minimum (docs/RESOURCE_BUDGET.md). "
              f"This is the exact class of defect RC6's independent audit found (P0 "
              f"'Target stack usage is catastrophically larger than the configured "
              f"stacks').", file=sys.stderr)
        return 1

    print(f"check_resource_budget: OK -- {free} bytes of DIRAM free for runtime heap "
          f"(>= documented {MIN_FREE_DIRAM_BYTES}-byte minimum).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
