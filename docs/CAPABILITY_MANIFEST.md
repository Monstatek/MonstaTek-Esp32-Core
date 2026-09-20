# Capability manifest

This document records the build-time state of the universal release/factory
configuration in `sdkconfig.defaults`. Live per-profile capability state is
available through `GET_CAPABILITIES` and `GET_VERSION`; those wire responses
remain the machine-readable source of truth.

## Universal build

All three transport adapters are compiled and started. The first adapter to
receive valid protocol traffic claims the canonical router for the boot session
through `mtk_transport_claim_try`; other adapters remain physically responsive
but return neutral/IDLE output.

| Component | State |
|---|---|
| Factory M1 UART | Compiled and started; strict shipped-interface parity |
| Native M1 SPI v1 | Compiled and started; canonical binary interface |
| Legacy SPI Compatibility | Compiled and started; established wire translation |
| Deauthentication | Compiled; required List B capability |
| WPA handshake capture | Compiled; required List B capability |
| MonstaShark capture | Compiled; required List B capability |
| Karma auto-responder | Compiled boundary; implementation unavailable, reports `UNSUPPORTED` |
| Probe-request flood | Compiled boundary; implementation unavailable, reports `UNSUPPORTED` |
| Captive portal | Compiled boundary; implementation unavailable, reports `UNSUPPORTED` |
| Wi-Fi beacon flood | Compiled boundary; implementation unavailable, reports `UNSUPPORTED` |
| SoftAP / PMKID capture | Not module-gated; implementation unavailable, reports `UNSUPPORTED` |
| Lab controls (`WIFI_MODE_SET`) | Disabled (`MTEK_LAB_CONTROLS_ENABLED=n`) |

The runtime capability overlay in `mtek_system_logic.c` ensures that an
unimplemented or disabled operation is never advertised as supported.

## Time synchronization

SoftAP, raw-TX and monitor-mode canonical opcodes are not module-gated: they
are general, reusable platform capabilities per `docs/ARCHITECTURE.md`. All
three are implemented -- `SOFTAP_START/STOP/STA_LIST` (class `SAP`),
`RAW_TX_SEND` (class `RAW`) and `mtek_capture_service`'s monitor-mode opcodes
(class `M`).

This firmware does not provide an SNTP client. Time synchronization therefore
has the following capability state:

| Opcode | native_spi | compat_spi | factory_uart |
|---|---|---|---|
| `TIME_SYNC_START` | `UNSUPPORTED` | `UNSUPPORTED` | `UNAVAILABLE` |
| `TIME_SYNC_STOP` | `UNSUPPORTED` | `UNSUPPORTED` | `UNAVAILABLE` |

Host lifecycle tests use test-only overlay opcodes. They are enabled only by
`MTK_ENABLE_TEST_OPCODES` and are absent from target builds. Time synchronization
must not report `SUPPORTED` until a real SNTP implementation exists.

## Release artifact naming

The release artifact family is `MtkCore.bin` and `MtkCore.md5`. The same names
must be used by the ESP32 build output, Web Manager manifest, factory flashing
procedure, and public release documentation.

The STM32 SD updater accepts a lowercase `.bin` filename and derives the
same-basename `.md5` sidecar, so this basename does not require an STM32 updater
code change.

## Adding a build variant

Document every component above as compiled in, compiled out, disabled, or
unsupported. Do not omit existing components. A variant may compile out an
unused adapter or optional module without changing canonical opcodes, schemas,
or capability negotiation.

## Adding a new build variant

Copy the table above and change only the rows that differ, naming the variant
after its `sdkconfig` overlay file. Do not remove a row for a component that
still exists in the source tree even if a variant disables it -- report it as
"Compiled out", never omit it, so this document always answers "what does this
image actually contain?" for every component.
