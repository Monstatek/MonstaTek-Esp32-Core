#!/usr/bin/env python3
"""Build and verify every supported ESP32-C6 image variant.

Each variant gets its OWN sdkconfig output file. This is not a stylistic
choice. `SDKCONFIG_DEFAULTS` only seeds a config file that does not already
exist, so invoking `idf.py -D SDKCONFIG_DEFAULTS=...` repeatedly against the
default root `sdkconfig` silently reuses whichever variant was configured
first: every later build then compiles that same configuration and the
variants come out byte-identical while appearing to succeed. Passing an
explicit per-variant `-D SDKCONFIG=` is what keeps them independent.

`--check` re-runs the structural assertions against already-built directories
without rebuilding, so the release packaging path can gate on them cheaply.

The assertions below are deliberately about link-level and config-level
facts -- which objects the linker actually pulled in, which symbols survived
into the ELF -- because the failures this guards against (a Kconfig-gated
transport that is requested at runtime but never compiled, a second SPI slave
contending for the one peripheral) all build and link cleanly while being
wrong on hardware.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# name -> (sdkconfig defaults chain, structural expectations)
VARIANTS = {
    "universal": {
        "defaults": ["sdkconfig.defaults"],
        "config": {
            "CONFIG_BT_ENABLED": True,
            "CONFIG_MTEK_IEEE802154_ENABLED": False,
            "CONFIG_OPENTHREAD_RADIO": False,
        },
        # Core's own SPI transport owns the STM32 link in this image.
        "symbols_present": ["spi_runtime_task"],
        "symbols_absent": ["esp_openthread_host_rcp_spi_init"],
        "objects_absent": ["esp_openthread_spi_slave.c.obj"],
    },
    "154": {
        "defaults": ["sdkconfig.defaults", "sdkconfig.154"],
        "config": {
            "CONFIG_BT_ENABLED": False,
            "CONFIG_MTEK_IEEE802154_ENABLED": True,
            "CONFIG_OPENTHREAD_RADIO": False,
        },
        "symbols_present": ["spi_runtime_task"],
        "symbols_absent": ["esp_openthread_host_rcp_spi_init"],
        "objects_absent": ["esp_openthread_spi_slave.c.obj"],
    },
    "154-rcp": {
        "defaults": ["sdkconfig.defaults", "sdkconfig.154", "sdkconfig.154-rcp"],
        "config": {
            "CONFIG_BT_ENABLED": False,
            "CONFIG_MTEK_IEEE802154_ENABLED": True,
            "CONFIG_OPENTHREAD_RADIO": True,
            # The transport is #if-gated in ESP-IDF's port layer and defaults
            # to UART, which the M1 cannot carry: setting only the runtime
            # host_connection_mode links the UART host path instead.
            "CONFIG_OPENTHREAD_RCP_SPI": True,
            "CONFIG_OPENTHREAD_RCP_UART": False,
        },
        # Spinel owns the one general-purpose SPI slave here, so Core's own
        # SPI runtime must not also be linked in.
        "symbols_present": ["esp_openthread_host_rcp_spi_init"],
        "symbols_absent": ["spi_runtime_task"],
        "objects_present": ["esp_openthread_spi_slave.c.obj"],
        "objects_absent": ["esp_openthread_uart.c.obj"],
    },
}


def build_dir(name):
    return os.path.join(REPO, f"build.{name}")


def sdkconfig_path(name):
    return os.path.join(build_dir(name), "sdkconfig")


def run(cmd, **kw):
    return subprocess.run(cmd, cwd=REPO, text=True, capture_output=True, **kw)


def build(name, spec, clean):
    bd = build_dir(name)
    if clean and os.path.isdir(bd):
        shutil.rmtree(bd)
    os.makedirs(bd, exist_ok=True)
    cmd = [
        "idf.py", "-B", bd,
        "-D", f"SDKCONFIG={sdkconfig_path(name)}",
        "-D", "SDKCONFIG_DEFAULTS=" + ";".join(spec["defaults"]),
        "build",
    ]
    r = run(cmd)
    if r.returncode != 0:
        sys.stderr.write(r.stdout[-4000:] + r.stderr[-4000:])
        return False
    return True


def read_config(name):
    p = sdkconfig_path(name)
    if not os.path.exists(p):
        return None
    out = {}
    for line in open(p):
        line = line.strip()
        m = re.match(r"^(CONFIG_[A-Za-z0-9_]+)=(.*)$", line)
        if m:
            out[m.group(1)] = m.group(2)
        else:
            m = re.match(r"^# (CONFIG_[A-Za-z0-9_]+) is not set$", line)
            if m:
                out[m.group(1)] = None
    return out


def nm_symbols(name):
    elf = os.path.join(build_dir(name), "mtkcore.elf")
    for tool in ("riscv32-esp-elf-nm", "nm"):
        r = subprocess.run([tool, elf], text=True, capture_output=True)
        if r.returncode == 0:
            return r.stdout
    return None


def check(name, spec, failures):
    def bad(msg):
        failures.append(f"{name}: {msg}")

    cfg = read_config(name)
    if cfg is None:
        bad("no sdkconfig -- variant was never configured")
        return
    for key, want in spec["config"].items():
        got = cfg.get(key)
        is_on = got == "y"
        if is_on != want:
            bad(f"{key} is {'on' if is_on else 'off'}, expected {'on' if want else 'off'}")

    syms = nm_symbols(name)
    if syms is None:
        bad("could not read symbols from mtkcore.elf")
    else:
        for s in spec.get("symbols_present", []):
            if s not in syms:
                bad(f"symbol '{s}' missing from the image but required")
        for s in spec.get("symbols_absent", []):
            if s in syms:
                bad(f"symbol '{s}' present but must not be linked into this image")

    mapf = os.path.join(build_dir(name), "mtkcore.map")
    if not os.path.exists(mapf):
        bad("no linker map")
    else:
        mp = open(mapf, errors="replace").read()
        for o in spec.get("objects_present", []):
            if o not in mp:
                bad(f"object '{o}' was never linked but is required")
        for o in spec.get("objects_absent", []):
            if o in mp:
                bad(f"object '{o}' was linked but must not be")


def check_distinct(failures):
    """The regression this file exists for: three variants collapsing onto one
    configuration and producing identical binaries."""
    sigs, bins = {}, {}
    for name in VARIANTS:
        cfg = read_config(name)
        if cfg is not None:
            sigs[name] = json.dumps(sorted(cfg.items()))
        b = os.path.join(build_dir(name), "mtkcore.bin")
        if os.path.exists(b):
            bins[name] = open(b, "rb").read()
    names = sorted(sigs)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if sigs[a] == sigs[b]:
                failures.append(
                    f"{a} and {b} resolved to an IDENTICAL configuration -- "
                    "they are sharing one sdkconfig instead of using their own")
    names = sorted(bins)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if bins[a] == bins[b]:
                failures.append(f"{a} and {b} produced byte-identical binaries")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify already-built variants without rebuilding")
    ap.add_argument("--no-clean", action="store_true")
    ap.add_argument("--variant", action="append",
                    help="limit to one variant (repeatable)")
    args = ap.parse_args()

    names = args.variant or list(VARIANTS)
    for n in names:
        if n not in VARIANTS:
            sys.exit(f"unknown variant '{n}'; known: {', '.join(VARIANTS)}")

    if not args.check:
        for n in names:
            print(f"== building {n} ==", flush=True)
            if not build(n, VARIANTS[n], clean=not args.no_clean):
                sys.exit(f"build_variants: '{n}' failed to build")

    failures = []
    for n in names:
        check(n, VARIANTS[n], failures)
    if len(names) == len(VARIANTS):
        check_distinct(failures)

    for n in names:
        b = os.path.join(build_dir(n), "mtkcore.bin")
        if os.path.exists(b):
            import hashlib
            h = hashlib.sha256(open(b, "rb").read()).hexdigest()
            print(f"{n:<10} {os.path.getsize(b):>9} bytes  sha256={h}")

    if failures:
        print("\nbuild_variants: FAILED")
        for f in failures:
            print("  -", f)
        sys.exit(1)
    print("\nbuild_variants: OK -- every variant is independently configured "
          "and carries the transport its image is supposed to carry.")


if __name__ == "__main__":
    main()
