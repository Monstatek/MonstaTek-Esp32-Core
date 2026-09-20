# Capability manifest (build-time module state)

Hand-maintained per `docs/ARCHITECTURE.md`'s capability-manifest policy.
The *live*, per-request-profile capability state for every opcode is
always the canonical `GET_CAPABILITIES`/`GET_VERSION` wire response --
this document only records which optional modules a given build variant
was *compiled* with, and must be updated whenever `sdkconfig.defaults` or
`main/Kconfig.projbuild`'s module defaults change.

## Build variant: universal (release/factory), `sdkconfig.defaults`

**Transport selection is runtime, not build-time.** The `MTEK_PRIMARY_
TRANSPORT_UART`/`MTEK_PRIMARY_TRANSPORT_SPI` build-time Kconfig choice
this section once described was **removed entirely** --
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
| Captive portal module | Compiled in and **implemented** (DNS hijack + HTTP portal over the SoftAP interface; capability-reported SUPPORTED) |
| Wi-Fi beacon flood module | Compiled in (not yet implemented -- reports `UNSUPPORTED`) |
| SoftAP | Compiled in, not module-gated; **implemented** (class `SAP`, capability-reported SUPPORTED) |
| PMKID capture | Compiled in, not module-gated; not yet implemented -- reports `UNSUPPORTED` |
| Lab controls (`WIFI_MODE_SET`) | Disabled (`MTEK_LAB_CONTROLS_ENABLED=n`) |

**Capabilities are truthful for the exact build.** Every module row above
marked "not yet implemented" once reported `MTK_CAP_SUPPORTED` via the real
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

**Correction:** an earlier version of this
section claimed the STM32/M1 SD-card updater's own expected-filename
configuration would need to change for this rename. Direct inspection of
`m1_esp32_fw_update.c` (the STM32/M1 app loader source, outside this
working tree) shows the SD updater accepts any lowercase `.bin` name and
derives the same-basename `.md5` sidecar -- it does not hard-code the old
basename. `MtkCore.bin`/`MtkCore.md5` therefore require **no STM32
updater code change** for this specific rename. That claim has been
removed below.

**Required future coordination (no external repository was edited
here):** before any customer-facing release,
the following external systems must still be updated to expect
`MtkCore.bin`/`MtkCore.md5` instead of any prior working name:

- The Web Manager updater's release manifest.
- The factory flashing/provisioning procedure documentation.
- Any public release documentation or customer-facing download page.

Each of those lives outside this working tree.

## Adding a new build variant

Copy the table above, change only the rows that differ (e.g. a
product-specific build disabling the M1 Community Compatibility adapter and
every optional module), and name the variant after its `sdkconfig`
overlay file. Do not remove a row for a component that still exists in
the source tree even if a variant disables it -- report it as "Compiled
out", never omit it, so this document always answers "what does this
exact binary actually contain" truthfully.

## TIME_SYNC_START capability correction (2026-09-07)

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

## TIME_SYNC_STOP capability correction (2026-09-07)

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

## Capability completion round (2026-09-20)

| Capability | State | Arbiter class | Notes |
|---|---|---|---|
| SoftAP | **Implemented** | `SAP` | `SOFTAP_START/STOP/STA_LIST`. Open or WPA2-PSK (8..63 char passphrase); the passphrase is never retained in service state. |
| Captive portal | **Implemented** | `SAP` (shared with SoftAP) | Brings up an open AP, then a DNS responder answering every A query with the AP address and an HTTP server. Captured credentials survive an ordinary stop and are zeroized on reset and on the next portal start. |
| ESP-NOW | **Implemented** | `ESPNOW` (new service `0x0006`) | Six opcodes: START/STOP/ADD_PEER/SEND/POLL_RECV/STATS. Shares the Wi-Fi radio, so it serializes against every Wi-Fi class. |
| IEEE 802.15.4 | **Not implemented** | `RESV_154` (still reserved) | See the blockers below. |

IEEE 802.15.4 is supported by the ESP32-C6 silicon (`SOC_IEEE802154_SUPPORTED`)
and ESP-IDF ships `esp_ieee802154.h`, but three things must be decided before
it can be built, none of which is a code-authoring question:

1. **Build configuration.** `CONFIG_IEEE802154_ENABLED` is not set in
   `sdkconfig.defaults`; enabling it changes the shipped universal artifact.
2. **Memory budget.** Free DIRAM after the capabilities above is 101,248 bytes
   against a documented 100,000-byte floor -- roughly 1.2KB of headroom. The
   802.15.4 MAC/PHY plus its receive-buffer pool does not fit in that, so
   enabling it requires either reclaiming memory elsewhere or an explicit,
   owner-approved change to the documented floor.
3. **Radio coexistence.** 802.15.4 shares the 2.4GHz radio with Wi-Fi and BLE.
   `RESV_154` is still a reserved class with a `DISABLED` self-pair, and giving
   it a real pairwise policy against the Wi-Fi and BLE classes is a coexistence
   decision, not a mechanical change.

## Build variants and IEEE 802.15.4 (2026-09-20)

Core ships three images. See `docs/BUILD_VARIANTS.md` for the full rationale.

| Capability | universal | mtkcore-154 | mtkcore-154-rcp |
|---|---|---|---|
| Wi-Fi (scan/connect/status) | yes | yes | yes |
| Deauth / handshake / capture | yes | yes | yes |
| SoftAP | yes | yes | yes |
| Captive portal | yes | compiled out | compiled out |
| BLE / GATT | yes | compiled out | compiled out |
| ESP-NOW | yes | yes | yes |
| 802.15.4 raw radio (0x0007/0x0001-0x0007) | UNAVAILABLE | **SUPPORTED** | UNAVAILABLE |
| 802.15.4 capture + hopping (0x0007/0x000B-0x000D) | UNAVAILABLE | **SUPPORTED** | UNAVAILABLE |
| 802.15.4 OpenThread RCP (0x0007/0x0008-0x000A) | UNAVAILABLE | UNAVAILABLE | **SUPPORTED** |

Two measured constraints produce this split, neither of which is negotiable:

1. Free DIRAM must stay at or above 100,000 bytes. The universal image has
   ~1.2KB of headroom, so the 802.15.4 radio (~12.9KB) and OpenThread RCP
   (~9.6KB) cannot be added to it. The 802.15.4 variants reclaim memory by
   compiling out the Bluetooth controller and the captive portal.
2. The ESP-IDF 802.15.4 driver has exactly one set of completion callbacks,
   and OpenThread's port layer defines the same symbols. Raw radio access and
   RCP therefore cannot coexist in one binary.

Capability reporting and dispatch are driven by the same build-time condition,
so `GET_CAPABILITIES` always matches what the image actually serves. Hosts
negotiate on capabilities; variant names are diagnostic only.

`GET_API_IDENTITY` (`0x0000/0x000A`) is served by all three images and
reports the frozen host-contract version (currently **API 1.0**) plus a
capability count computed from that image's own live opcode table. Variant
identity is diagnostic only.

Zigbee is intentionally absent on-device: a host-side stack consumes the raw
802.15.4 service across the same radio boundary Thread uses via RCP.
