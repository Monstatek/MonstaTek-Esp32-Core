#!/usr/bin/env python3
"""/
Required verification #4 "Target-ELF stack report shows safe margins for
every task/callback path; no nested path exceeds its allocated stack":
measures REAL per-function stack frame sizes (GCC's own `-fstack-usage`,
not an estimate) for a documented set of known task/callback entry points
and their hand-verified worst-case reachable call chains, and fails if any
chain's summed frame size leaves less than the documented safety margin of
its host stack.

Disclosed scope: this is NOT a general, automatic call-graph analyzer --
`.su` files carry no call-graph information, only each function's own
frame size, so the call chains below are hand-traced from the real source
(cross-checked against each function's own callees) and hard-coded, not
discovered. A chain that silently changes shape (a call removed, a new one
added, a function renamed) will not update itself -- CHAIN_FAILURE below
fails loud specifically to catch a chain member disappearing from the
build (renamed/removed) so a stale, no-longer-meaningful check cannot pass
silently forever. This mirrors check_resource_budget.py's own disclosed
"static-allocation gate, not a hardware-measured" scope note: real,
useful, mechanically enforced -- and honestly bounded, not oversold.

Usage (after a normal `idf.py build`, from an ESP-IDF `export.sh`-sourced
environment, from the project root):
    python3 tools/check_stack_budget.py [path/to/build]
Defaults to `build`. Recompiles each source file named below with
`-fstack-usage` appended to its real compile command (from
compile_commands.json) to produce a fresh `.su` file -- the project's own
normal build does not carry this flag, so this script does not require a
special build configuration, only a normal `idf.py build` having already
run once (to produce compile_commands.json).
"""
import json
import os
import re
import subprocess
import sys

# (task/callback name, configured stack bytes, safety margin fraction --
# the chain's summed frame size must stay under stack_bytes * margin,
# leaving the rest for whatever this script cannot itself measure: the
# canonical router/service layer beneath the last chain member, ESP-IDF's
# own HAL/driver call frames, and interrupt/register-save overhead).
CHAINS = [
    {
        "task": "ble_tick_task (blocking signal sample)",
        "requires_sdkconfig": "CONFIG_BT_ENABLED",
        "stack_bytes": 4096,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "ble_tick_task"),
            ("components/mtek_ble_service/mtek_ble_logic.c", "mtek_ble_signal_meter_tick"),
            ("components/mtek_ble_service/mtek_ble_hal_esp32.c", "esp32_signal_sample"),
        ],
    },
    {
        "task": "periodic_delivery_task (GATT delivery)",
        "requires_sdkconfig": "CONFIG_BT_ENABLED",
        "stack_bytes": 4096,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "periodic_delivery_task"),
            ("components/mtek_ble_service/mtek_ble_logic.c", "mtek_ble_gatt_tick"),
            ("components/mtek_core/mtek_async_queue.c", "mtk_async_sink_event"),
        ],
    },
    {
        "task": "periodic_delivery_task (capture deadlines)",
        "stack_bytes": 4096,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "periodic_delivery_task"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "mtek_capture_channel_hop_tick"),
        ],
    },
    {
        "task": "spi_runtime_task (native SPI v1 worst path)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "mtek_spi_native_dispatch_feed_cell"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "dispatch_complete_message"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "try_deliver_frame"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "stage_cell"),
        ],
    },
    {
        "task": "spi_runtime_task (Mtek Compatibility/C3 worst path)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_compat/mtek_compat_dispatch.c", "mtek_compat_dispatch_request"),
            ("components/mtek_transport_spi_compat/mtek_compat_dispatch.c", "handle_raw_tx"),
        ],
    },
    {
        # Step 1 raw-TX/monitor-mode foundation audit (2026-09-19): every
        # existing native-SPI/Mtek-Compatibility chain above stops at this
        # transport's own request-decode wrapper (dispatch_complete_message)
        # and never follows its own, real, unconditional
        # `mtk_router_dispatch(...)` call (mtek_spi_native_dispatch.c's own
        # synchronous-lifecycle branch) on into the canonical service
        # dispatch/handler/HAL frames every opcode actually reaches --
        # previously an unmeasured gap, not a documented exclusion. Audited
        # here for RAW_TX_SEND specifically: its own decoded
        # mtk_raw_tx_send_req_t local (1,489-byte frame buffer) makes
        # handle_raw_tx_send's real frame (1,536 bytes) the single largest
        # canonical opcode handler reachable from native SPI, so this is
        # the binding case for "does the canonical dispatch tail, once
        # actually included, still fit" -- it does (native SPI's own
        # dispatch_complete_message synchronous branch never also holds
        # try_deliver_frame/stage_cell's own frames at the same time; those
        # run strictly after mtk_router_dispatch already returned, on the
        # ACCEPTED_ASYNC branch only -- confirmed by reading
        # dispatch_complete_message's own source, not assumed).
        "task": "spi_runtime_task (native SPI v1: RAW_TX_SEND canonical dispatch tail)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "mtek_spi_native_dispatch_feed_cell"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "dispatch_complete_message"),
            ("components/mtek_router/mtek_router.c", "mtk_router_dispatch"),
            ("components/mtek_wifi_service/mtek_wifi_logic.c", "mtek_wifi_dispatch"),
            ("components/mtek_wifi_service/mtek_wifi_logic.c", "handle_raw_tx_send"),
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "esp32_raw_tx"),
        ],
    },
    {
        # Same gap as above, Mtek Compatibility/C3 transport: the existing
        # "Mtek Compatibility/C3 worst path" chain measures only
        # mtek_compat_dispatch_request's own translation wrapper
        # (handle_raw_tx, mtek_compat_dispatch.c) in isolation -- that
        # wrapper's real, unconditional `router_call(...)` ->
        # `mtk_router_dispatch(...)` call (confirmed by reading its own
        # source) reaches this exact same canonical handle_raw_tx_send tail,
        # never previously included.
        "task": "spi_runtime_task (Mtek Compatibility/C3: RAW_TX_SEND canonical dispatch tail)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_compat/mtek_compat_dispatch.c", "mtek_compat_dispatch_request"),
            ("components/mtek_transport_spi_compat/mtek_compat_dispatch.c", "handle_raw_tx"),
            ("components/mtek_router/mtek_router.c", "mtk_router_dispatch"),
            ("components/mtek_wifi_service/mtek_wifi_logic.c", "mtek_wifi_dispatch"),
            ("components/mtek_wifi_service/mtek_wifi_logic.c", "handle_raw_tx_send"),
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "esp32_raw_tx"),
        ],
    },
    {
        # Same class of gap, the general-purpose monitor-mode/capture
        # service's own largest handler (handle_capture_poll_read's own
        # 1,026-byte `out[]` wire-encode buffer, service 0x0004 opcode
        # 0x0006): reachable from native SPI via the exact same
        # dispatch_complete_message synchronous-lifecycle tail as above.
        "task": "spi_runtime_task (native SPI v1: CAPTURE_POLL_READ canonical dispatch tail)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "mtek_spi_native_dispatch_feed_cell"),
            ("components/mtek_transport_spi_native/mtek_spi_native_dispatch.c", "dispatch_complete_message"),
            ("components/mtek_router/mtek_router.c", "mtk_router_dispatch"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "mtek_capture_dispatch"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "handle_capture_poll_read"),
        ],
    },
    {
        # Mtek Compatibility/C3's own CAPTURE_POLL_READ case (0x0315) is
        # handled inline inside mtek_compat_dispatch_request itself (no
        # separate wrapper function the way RAW_TX has one) -- its own
        # locals are already folded into that function's own measured
        # frame below, so only the canonical tail past mtk_router_dispatch
        # is new here.
        "task": "spi_runtime_task (Mtek Compatibility/C3: CAPTURE_POLL_READ canonical dispatch tail)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/mtek_spi_runtime.c", "spi_runtime_task"),
            ("components/mtek_transport_spi_compat/mtek_compat_dispatch.c", "mtek_compat_dispatch_request"),
            ("components/mtek_router/mtek_router.c", "mtk_router_dispatch"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "mtek_capture_dispatch"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "handle_capture_poll_read"),
        ],
    },
    {
        "task": "uart_repl_task (worst UART command-formatting path)",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "uart_repl_task"),
            ("components/mtek_transport_uart/mtek_uart_adapter.c", "handle_scan_a"),
        ],
    },
    {
        #
        # format_ble_device_detail's own ble_ad_info_t local (Service
        # UUID16/128 lists, Service Data, unknown-AD-type text buffer) is
        # a genuinely new, non-trivial stack frame added --
        # audited here rather than left as an unmeasured new UART path
        # (RC8's own "audit every task/callback entry path... not just
        # the 3 named functions" instruction).
        "task": "uart_repl_task (BLE list-detail worst path)",
        "requires_sdkconfig": "CONFIG_BT_ENABLED",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "uart_repl_task"),
            ("components/mtek_transport_uart/mtek_uart_adapter.c", "handle_ble_list_all_detail"),
            ("components/mtek_transport_uart/mtek_uart_adapter.c", "format_ble_device_detail"),
        ],
    },
    {
        # "BLE connection discovery lacks
        # nested service/char/descriptor counts/listing": gatt_discover_
        # tree's own decoded-response locals (services/chars/descs pages)
        # are a new stack frame added -- audited here for the
        # same reason as the entry above.
        "task": "uart_repl_task (BLE nested GATT discovery worst path)",
        "requires_sdkconfig": "CONFIG_BT_ENABLED",
        "stack_bytes": 12288,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "uart_repl_task"),
            ("components/mtek_transport_uart/mtek_uart_adapter.c", "handle_ble_services"),
            ("components/mtek_transport_uart/mtek_uart_adapter.c", "gatt_discover_tree"),
        ],
    },
    {
        # RC11 promiscuous-mode audit follow-up #1/#8 "never call Wi-Fi
        # control APIs from the Wi-Fi promiscuous callback" / "measure the
        # new queue RAM and mtek_wifi_rx stack usage": frame_cb/hs_frame_cb
        # no longer run in the Wi-Fi driver task context at all -- only
        # promisc_trampoline does now (a bounded copy + zero-timeout queue
        # send, see its own doc comment in mtek_wifi_hal_esp32.c). This
        # entry replaces the OLD "frame_cb in driver context" assumption
        # this chain previously modeled; frame_cb/hs_frame_cb themselves
        # now run on wifi_promisc_tick_task's own KNOWN 4096-byte stack
        # instead (the two new chains below), a real, project-owned budget
        # rather than an assumed, unconfirmed Wi-Fi-driver-internal one.
        "task": "Wi-Fi driver promiscuous RX callback (promisc_trampoline)",
        # This context's real stack budget is still owned by the Wi-Fi
        # driver, not this project's own task creation -- not independently
        # confirmed (disclosed hardware/ESP-IDF-internals
        # gap, docs/PROVENANCE.md). Conservatively assumed no larger than
        # CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE (2304 bytes on this
        # sdkconfig), the smallest real task stack this project's own
        # sdkconfig configures anywhere, as a deliberately pessimistic
        # stand-in until a real measurement is available.
        "stack_bytes": 2304,
        "margin": 0.60,
        "members": [
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "promisc_trampoline"),
        ],
    },
    {
        # "STA scan cancel and
        # callback state are target data races": sta_scan_promisc_cb is a
        # SEPARATE, still-direct Wi-Fi driver RX callback registration
        # (STA_SCAN's own, via esp32_sta_scan -- unrelated to promisc_
        # trampoline/esp32_promisc_service's deferred-delivery mechanism
        # above, which backs only deauth/handshake/MonstaShark capture's
        # shared promisc_start/stop). Same disclosed stack-budget-
        # ownership caveat and same conservative stand-in figure as
        # promisc_trampoline's own entry above.
        "task": "Wi-Fi driver promiscuous RX callback (sta_scan_promisc_cb)",
        "stack_bytes": 2304,
        "margin": 0.60,
        "members": [
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "sta_scan_promisc_cb"),
        ],
    },
    {
        # RC11 promiscuous-mode audit follow-up #8 "measure the new queue
        # RAM and mtek_wifi_rx stack usage in the target build": frame_cb
        # now runs here -- main/app_main.c's own wifi_promisc_tick_task
        # (created with a real, project-configured 4096-byte stack, "mtek_
        # wifi_rx"), via esp32_promisc_service's own drain loop -- a real,
        # known budget instead of the Wi-Fi driver's own unconfirmed one.
        "task": "wifi_promisc_tick_task (MonstaShark capture worst path)",
        "stack_bytes": 4096,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "wifi_promisc_tick_task"),
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "esp32_promisc_service"),
            ("components/mtek_capture_service/mtek_capture_logic.c", "frame_cb"),
        ],
    },
    {
        # RC11 promiscuous-mode audit follow-up #8: hs_frame_cb's own
        # matching entry -- same task/drain path as frame_cb above, the
        # OTHER real consumer of esp32_promisc_start/promisc_trampoline's
        # shared deferred-delivery mechanism.
        "task": "wifi_promisc_tick_task (WPA handshake capture worst path)",
        "stack_bytes": 4096,
        "margin": 0.60,
        "members": [
            ("main/app_main.c", "wifi_promisc_tick_task"),
            ("components/mtek_wifi_service/mtek_wifi_hal_esp32.c", "esp32_promisc_service"),
            ("components/mtek_wifi_service/mtek_wifi_logic.c", "hs_frame_cb"),
        ],
    },
]



def sdkconfig_has(build_dir, option):
    """True if `option` is set to y in the build's own generated sdkconfig.
    A chain may declare requires_sdkconfig so that a build variant which
    deliberately compiles a subsystem out skips that chain instead of
    reporting CHAIN_FAILURE. A symbol missing while its option IS enabled
    still fails loud, which is the behaviour that catches a real rename."""
    # The build directory's own generated header is authoritative per build.
    # The project-level sdkconfig reflects whichever variant was configured
    # last, so it must not be used to judge a specific build directory.
    header = os.path.join(build_dir, "config", "sdkconfig.h")
    if os.path.isfile(header):
        with open(header) as f:
            return f"#define {option} 1" in f.read()
    return True   # cannot tell: assume present so nothing is silently skipped

def find_compile_command(compile_commands, rel_path):
    needle = rel_path.replace("\\", "/")
    for entry in compile_commands:
        if entry["file"].replace("\\", "/").endswith(needle):
            return entry
    return None


def stack_usage_for(entry, function_name):
    """Recompiles this one translation unit with -fstack-usage appended
    and returns function_name's own frame size in bytes, or None if that
    function does not appear in the resulting .su file (renamed/removed/
    inlined away -- inlining is suppressed by -O0 fallback below only if
    needed; real optimized builds may inline small helpers, which this
    script surfaces as CHAIN_FAILURE rather than silently skipping)."""
    cmd = entry["command"] + " -fstack-usage"
    r = subprocess.run(cmd, shell=True, cwd=entry["directory"], capture_output=True, text=True)
    if r.returncode != 0:
        print(f"check_stack_budget: recompiling {entry['file']} with -fstack-usage failed:\n{r.stderr[-2000:]}", file=sys.stderr)
        return None
    output = entry.get("output", "")
    su_path = re.sub(r"\.(o|obj)$", ".su", output) if output else ""
    if not su_path or not os.path.isfile(su_path):
        # Fall back to searching the build directory for the .su file
        # matching this source's basename, in case `output` was absent.
        base = os.path.basename(entry["file"]).rsplit(".", 1)[0] + ".c.su"
        candidates = []
        for root, _dirs, files in os.walk(entry["directory"]):
            if base in files:
                candidates.append(os.path.join(root, base))
        if not candidates:
            return None
        su_path = candidates[0]
    with open(su_path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 2:
                continue
            # parts[0] is "path:line:col:function_name"
            if parts[0].rsplit(":", 1)[-1] == function_name:
                try:
                    return int(parts[1])
                except ValueError:
                    continue
    return None


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build"
    cc_path = os.path.join(build_dir, "compile_commands.json")
    try:
        with open(cc_path) as f:
            compile_commands = json.load(f)
    except FileNotFoundError:
        print(f"check_stack_budget: {cc_path} not found -- run `idf.py build` at least once first "
              f"(from an ESP-IDF export.sh-sourced shell).", file=sys.stderr)
        return 2

    overall_ok = True
    skipped = []
    for chain in CHAINS:
        req = chain.get("requires_sdkconfig")
        if req and not sdkconfig_has(build_dir, req):
            skipped.append((chain["task"], req))
            continue
        total = 0
        rows = []
        chain_ok = True
        for rel_path, func in chain["members"]:
            entry = find_compile_command(compile_commands, rel_path)
            if entry is None:
                print(f"check_stack_budget: CHAIN_FAILURE -- {rel_path} is not in compile_commands.json "
                      f"(moved/renamed?) -- update the {chain['task']!r} chain in this script.", file=sys.stderr)
                chain_ok = False
                overall_ok = False
                continue
            size = stack_usage_for(entry, func)
            if size is None:
                print(f"check_stack_budget: CHAIN_FAILURE -- {func}() not found in {rel_path}'s own "
                      f"-fstack-usage output (renamed, removed, or inlined away) -- update the "
                      f"{chain['task']!r} chain in this script before trusting it again.", file=sys.stderr)
                chain_ok = False
                overall_ok = False
                continue
            rows.append((rel_path, func, size))
            total += size

        print(f"\n{chain['task']}: stack={chain['stack_bytes']} margin={chain['margin']*100:.0f}%")
        for rel_path, func, size in rows:
            print(f"  {size:6d} bytes  {func}  ({rel_path})")
        if not chain_ok:
            continue
        limit = chain["stack_bytes"] * chain["margin"]
        pct = 100.0 * total / chain["stack_bytes"]
        print(f"  ---------------------------------------------")
        print(f"  {total:6d} bytes total ({pct:.1f}% of {chain['stack_bytes']}-byte stack)")
        if total > limit:
            print(f"check_stack_budget: FAIL -- {chain['task']!r} chain uses {total} bytes, "
                  f"over its {limit:.0f}-byte ({chain['margin']*100:.0f}%) safety margin.", file=sys.stderr)
            overall_ok = False
        else:
            print(f"  OK -- under the {limit:.0f}-byte ({chain['margin']*100:.0f}%) safety margin")

    # Skipped chains are always reported: a variant losing coverage silently
    # is the failure this tool exists to prevent.
    for task, req in skipped:
        print(f"check_stack_budget: SKIPPED -- '{task}': {req} is not enabled in this build")
    print(f"check_stack_budget: {len(CHAINS) - len(skipped)} chain(s) measured, {len(skipped)} skipped")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
