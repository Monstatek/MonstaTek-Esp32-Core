# Capability manifest (build-time module state)

Hand-maintained per `docs/ARCHITECTURE.md`'s capability-manifest policy.
The *live*, per-request-profile capability state for every opcode is
always the canonical `GET_CAPABILITIES`/`GET_VERSION` wire response --
this document only records which optional modules a given build variant
was *compiled* with, and must be updated whenever `sdkconfig.defaults` or
`main/Kconfig.projbuild`'s module defaults change.

## Build variant: universal (release/factory), `sdkconfig.defaults`

**RC7 correction (independent audit P0 "The release artifact starts the
wrong transport for shipped M1 compatibility"):** the `MTEK_PRIMARY_
TRANSPORT_UART`/`MTEK_PRIMARY_TRANSPORT_SPI` build-time Kconfig choice
this section used to describe was **removed entirely** this round --
see `docs/DECISION_LOG.md`'s "real cross-transport AUTO selection"
entry. Every `CONFIG_MTEK_ADAPTER_*` component below is now both
compiled in AND boot-started UNCONDITIONALLY; `mtk_transport_claim_try`
(`components/mtek_transport_select`) is the real runtime cross-transport
exclusivity latch -- whichever adapter (factory UART, native M1 SPI v1,
or M1 Community Compatibility) first sees genuine protocol traffic on its own
bus wins exclusivity for the rest of the boot session, matching
`SPI_PROTOCOL_V1.md`'s own "Runtime transport selection" requirement
directly, with no build-time choice needed or possible.

| Component | State |
|---|---|
| Factory M1 UART adapter | Compiled in; boot-started unconditionally; may win the runtime AUTO-selection race |
| Native M1 SPI v1 adapter | Compiled in; boot-started unconditionally (AUTO-selected vs. M1 Community Compatibility on the shared SPI bus, then races UART for cross-transport exclusivity) |
| M1 Community Compatibility adapter (legacy Mtek Compatibility/C3 wire profile) | Compiled in; boot-started unconditionally (same AUTO/cross-transport selection as above) |
| Karma auto-responder module | Compiled in (not yet implemented -- reports `UNSUPPORTED`, see `docs/PROVENANCE.md`) |
| Probe-request flood module | Compiled in (not yet implemented -- reports `UNSUPPORTED`) |
| Captive portal module | Compiled in (not yet implemented -- reports `UNSUPPORTED`) |
| Wi-Fi beacon flood module | Compiled in (not yet implemented -- reports `UNSUPPORTED`) |
| SoftAP / PMKID capture | Compiled in, not module-gated (candidates for canonical-core "general reusable platform capability" status); not yet implemented -- reports `UNSUPPORTED` |
| Lab controls (`WIFI_MODE_SET`) | Disabled (`MTEK_LAB_CONTROLS_ENABLED=n`) |

**RC8 correction (independent audit P0-8 "Make capabilities truthful for
the exact build"):** every module row above marked "not yet implemented"
previously still reported `MTK_CAP_SUPPORTED` via the real
`GET_CAPABILITIES` wire response for the native-SPI/M1-Community-Compatibility profiles
(the generated registry's own static per-opcode table), even though
`mtek_wifi_logic.c`'s own dispatch switch unconditionally answered
`MTK_STATUS_UNSUPPORTED` for every one of them -- a real capability/
dispatch inconsistency, not merely a documentation gap. A runtime
overlay (`mtek_system_logic.c`'s `cap_for`) now corrects the REPORTED
state to `MTK_CAP_UNSUPPORTED` for exactly this set of opcodes, verified
by a property test (`host_tests/test_opcode_registry.c`) that walks
every opcode this build advertises via a real `GET_CAPABILITIES` call
and confirms agreement with real dispatch in both directions.

Required List B (never module-gated, always compiled in): deauthentication
(`DEAUTH_START/STOP/STATUS`, all three target modes), WPA handshake
capture, MonstaShark capture.

SoftAP, raw-TX, and monitor-mode canonical opcodes are not module-gated
(candidates for canonical-core "general reusable platform capability"
status per `docs/ARCHITECTURE.md`).

**Correction (raw-TX/monitor-mode foundation audit, 2026-09-19):** the
line above previously claimed all three were "not yet implemented at
the service-logic layer" -- true today only for **SoftAP**
(`SOFTAP_START/STOP/STA_LIST` fall through to `mtek_wifi_dispatch`'s
default `UNSUPPORTED` case, `mtek_wifi_service/mtek_wifi_logic.c`).
**Raw-TX** (`RAW_TX_SEND`) and **monitor-mode** (`mtek_capture_service`'s
`CAPTURE_START/STOP/STATUS/SESSION_INFO/STATS/POLL_READ`) are real,
audited, `GET_CAPABILITIES`-truthful implementations -- `cap_for`
(`mtek_system_logic.c`) does not include either in its
unimplemented-module downgrade list, and dispatch backs that up (real
`esp_wifi_80211_tx`/`esp_wifi_set_promiscuous` calls, arbiter classes
`RAW`/`M`, bounded 32-slot capture ring, `capture_teardown`/
`mtek_wifi_restore_and_release` cleanup on every exit path). See
`docs/ARCHITECTURE.md`'s own matching correction for the full evidence.

## Release artifact naming (owner-approved, coordinated change)

The canonical, permanent release artifact family name is **`MtkCore.bin`
/ `MtkCore.md5`** (owner-approved; supersedes two earlier working names,
`MtkEsp32-monstashark`/`Mtkv2Esp`, neither of which was ever an approved
customer-facing name). This name is used by: the ESP-IDF project
identifier (`CMakeLists.txt`), `tools/package_release.py`'s packaged
output, `host_tests/test_release_artifact.c`'s validation paths,
`release/RELEASE_NOTES.md`, and `release/merged_image_map.json`
(auto-generated by the packager, always reflects the current name).

`MtkEsp32-monstashark.bin` is retained **only** where it names the
historical factory/reference M1 firmware artifact evidence (none of
which exists in this clean-room tree; no factory binary was ever opened
or copied here) -- it must never be used to label this project's own
current or future release output again.

**Correction (RC5 independent audit P1 "Governance and integration
documentation is stale"):** an earlier candidate's version of this
section claimed the STM32/M1 SD-card updater's own expected-filename
configuration would need to change for this rename. Direct inspection of
`m1_esp32_fw_update.c` (the STM32/M1 app loader source, outside this
working tree) shows the SD updater accepts any lowercase `.bin` name and
derives the same-basename `.md5` sidecar -- it does not hard-code the old
basename. `MtkCore.bin`/`MtkCore.md5` therefore require **no STM32
updater code change** for this specific rename. That claim has been
removed below.

**Required future coordination (not performed in this task -- no
external repository was edited):** before any customer-facing release,
the following external systems must still be updated to expect
`MtkCore.bin`/`MtkCore.md5` instead of any prior working name:

- The Web Manager updater's release manifest.
- The factory flashing/provisioning procedure documentation.
- Any public release documentation or customer-facing download page.

Each of those lives outside this working tree and was explicitly
out-of-scope to edit in this task.

## Adding a new build variant

Copy the table above, change only the rows that differ (e.g. a
product-specific build disabling the M1 Community Compatibility adapter and
every optional module), and name the variant after its `sdkconfig`
overlay file. Do not remove a row for a component that still exists in
the source tree even if a variant disables it -- report it as "Compiled
out", never omit it, so this document always answers "what does this
exact binary actually contain" truthfully.

## RC12 hardening round (2026-09-07): TIME_SYNC_START capability correction

`TIME_SYNC_START` (service 0x0000, opcode 0x0008) capability_state changed
to reflect the truth that this candidate wires no SNTP client (the
operation always terminates FAILED/IO_ERROR):

| Profile       | Before      | After         |
|---------------|-------------|---------------|
| native_spi    | SUPPORTED   | **UNSUPPORTED** |
| compat_c3      | SUPPORTED   | **UNSUPPORTED** |
| factory_uart  | UNAVAILABLE | UNAVAILABLE (no change) |

`GET_CAPABILITIES` now reports UNSUPPORTED for TIME_SYNC_START on native and
Mtek Compatibility/C3. Restore to SUPPORTED only when a real SNTP client is implemented.
Host tests that used TIME_SYNC_START as a generic arbiter-free async vehicle
were migrated to a test-only opcode overlay (`mtk_opcode_overlay`, inert in
production; `host_tests/support/mtk_test_async_fixture.h`).

## RC12 blocker round (2026-09-07): TIME_SYNC_STOP capability correction

`TIME_SYNC_STOP` (service 0x0000, opcode 0x0009) native capability_state
changed SUPPORTED -> UNSUPPORTED, to match TIME_SYNC_START (UNSUPPORTED on
native/compat_c3 -- no SNTP client). factory_uart stays UNAVAILABLE,
compat_c3 stays UNSUPPORTED. Host tests needing a generic token-addressed
STOP use a test-only overlay opcode (0x00F1) compiled only into the
host-test build (MTK_ENABLE_TEST_OPCODES), never the ESP32 target.

| Opcode         | Profile      | Before      | After         |
|----------------|--------------|-------------|---------------|
| TIME_SYNC_STOP | native_spi   | SUPPORTED   | **UNSUPPORTED** |
| TIME_SYNC_STOP | compat_c3     | UNSUPPORTED | UNSUPPORTED (no change) |
| TIME_SYNC_STOP | factory_uart | UNAVAILABLE | UNAVAILABLE (no change) |
