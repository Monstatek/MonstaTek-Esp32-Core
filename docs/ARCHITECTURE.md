# Architecture: canonical core, adapters, and optional modules

MonstaTek M1 is the canonical platform. Community/legacy implementations
(community/C3-lineage and any future third-party protocol) adapt to this
firmware's stable interfaces; their quirks are translated at the adapter
boundary and must never define or bloat the canonical core.

## Layers

```
 canonical services (mtek_wifi_service, mtek_ble_service,
   mtek_capture_service, mtek_system_service)
        |
 mtek_router  (capability-state gate, dispatch, optional async runner)
        |
 mtek_arbiter (radio-resource-class ownership)
        |
 mtek_core    (operation-token lifecycle, boot_epoch)
        |
 mtek_schema  (generated wire structs/codec/opcode registry from schemas.json)
        ^
        |  each adapter builds a canonical mtk_request_ctx_t and calls
        |  mtk_router_dispatch -- adapters never call each other or share
        |  translation code
        |
 +------+-------------------+-------------------+
 |                          |                    |
 factory UART adapter   native M1 SPI v1     M1 Community Compatibility
 (strict shipped parity) (canonical future-  (optional legacy/community
                          facing interface)   protocol translation)
```

### Canonical core

- **Schema/codec/core/arbiter/router** (`components/mtek_schema`,
  `mtek_core`, `mtek_arbiter`, `mtek_router`): the stable, transport-
  neutral superset. Generated from `schemas.json`, the sole
  machine-authoritative source of opcodes, capability states, and wire
  shapes -- never hand-edited, never diverged from to satisfy one
  adapter's convenience.
- **`mtek_async_queue`** (`components/mtek_core/include/mtek_async_queue.h`):
  a stable, reusable primitive (bounded thread-safe delivery FIFO) any
  adapter's persistent async response/event queue is built on -- a
  platform primitive, not compatibility-adapter-specific.
- **Canonical services** own product behavior (List A shipped behavior;
  required List B: `DEAUTH_START/STOP/STATUS`, `HANDSHAKE_START/STATUS/
  READ/STOP`, MonstaShark capture -- all explicitly owner-approved and
  compiled in unconditionally, never module-gated); stable radio/
  transport/storage primitives (Wi-Fi/BLE HAL abstractions, capture
  service); capability negotiation (`GET_CAPABILITIES`/`GET_VERSION`).
  SoftAP, raw-TX, and monitor-mode primitives live in canonical core
  because they are general, reusable platform capabilities (a station-
  mode radio can always be put into AP or monitor mode; that is not a
  community-specific behavior).

  **Correction (raw-TX/monitor-mode foundation audit, 2026-09-19):**
  raw-TX and monitor-mode are no longer scaffolding-only -- both have
  real, canonical, arbiter-mediated request/response logic, confirmed
  present and audited at this correction's own commit: `RAW_TX_SEND`
  (`handle_raw_tx_send`, `mtek_wifi_service/mtek_wifi_logic.c`, class
  `RAW`) transmits via `esp_wifi_80211_tx`; monitor-mode entry/exit,
  bounded frame delivery, and cancellation/cleanup live in
  `mtek_capture_service` (`CAPTURE_START/STOP/STATUS/SESSION_INFO/
  STATS/POLL_READ`, class `M`) via `esp_wifi_set_promiscuous`. Both are
  reachable identically from every transport that declares them
  `SUPPORTED` (native SPI and Mtek Compatibility/C3; `RAW_TX_SEND` is
  `UNAVAILABLE` on factory UART, `CAPTURE_START` is available there too),
  through the same `mtk_router_dispatch` path every other canonical
  opcode uses -- no parallel or transport-embedded implementation exists.
  **Correction (capability completion, 2026-09-20):** SoftAP is now
  implemented as well (`SOFTAP_START/STOP/STA_LIST`, class `SAP`), and the
  captive portal is layered on that same interface and lease rather than
  bringing up a second one (`CAPTIVE_PORTAL_START/STOP/GET_CREDENTIALS/
  GET_DIAGNOSTICS`, also class `SAP`, gated by
  `CONFIG_MTEK_MODULE_CAPTIVE_PORTAL`). ESP-NOW is implemented as its own
  canonical service (`mtek_espnow_service`, service `0x0006`, class
  `ESPNOW`); because it transmits through the Wi-Fi MAC it serializes
  against every Wi-Fi class rather than running alongside them.
  **IEEE 802.15.4 is implemented** as a first-class arbiter class (`IEEE154`)
  and its own canonical service (`mtek_ieee802154_service`, namespace
  `0x0007`). Because the ESP32-C6 has one 2.4GHz radio, `IEEE154` is
  serialized against every Wi-Fi class and `ESPNOW`, and cross-subsystem-busy
  against the BLE classes. The service is protocol-neutral: it moves PHY
  payloads and radio metadata and never parses or synthesises 802.15.4 MAC
  headers, so Thread (host-driven over an OpenThread RCP, which carries Spinel on the
M1's existing SPI wires and therefore takes that link over from the Core
transport in that image -- see `docs/BUILD_VARIANTS.md`), a host-side Zigbee
  stack, a sniffer or a bespoke protocol all sit above the same primitives.
  802.15.4 does not fit in the universal image and ships in dedicated
  variants -- see `docs/BUILD_VARIANTS.md` for the measured memory constraint
  and the driver-callback-ownership constraint that make that necessary.
  Core-managed capture with deterministic bounded channel hopping reuses the
  same service, ring and teardown rather than adding a second radio path.
  `GET_API_IDENTITY` freezes the host contract at a major/minor version;
  capability negotiation through `GET_CAPABILITIES` stays authoritative and
  hosts never branch on a variant name.

### Transport adapters (three, boot-exclusive, `SPI_PROTOCOL_V1.md`
"Runtime transport selection")

| Adapter | Role | Parity obligation |
|---|---|---|
| Factory M1 UART | Legacy ASCII REPL over UART0 | **Strict shipped parity** -- must not change shipped List A observable behavior |
| Native M1 SPI v1 | The canonical, future-facing binary interface | Owns the canonical wire shape; new canonical features are designed against this adapter first |
| M1 Community Compatibility (legacy Mtek Compatibility/C3 wire profile) | Optional translation of a legacy/community (community/C3-lineage) SPI wire protocol into the canonical API | No parity obligation of its own; translates faithfully from confirmed facts, never invents canonical behavior to match community quirks |

The M1 Community Compatibility adapter translates the community/C3 wire
protocol without changing its framing, opcode values, or behavior. Project-owned
source files use `mtek_compat_*`, C identifiers use `mtk_compat_*`, and the
build option is `CONFIG_MTEK_ADAPTER_COMPAT_C3`. Source/configuration names
have changed; existing integrations must update these names when rebuilding.
This is not a wire-protocol migration. Historical source identification remains
in `docs/THIRD_PARTY_NOTICES.md`; compatibility evidence is described in
`docs/COMPATIBILITY_LEDGER.md`.

### Optional capability modules

Community-specific attack conveniences that are not shipped List A
behavior and not owner-approved required List B features are **optional,
compile-time-selectable modules** (`main/Kconfig.projbuild`, "Optional
capability modules" menu): Karma auto-responder, probe-request flood,
captive portal, Wi-Fi beacon flood. Disabling a module's Kconfig option
means the canonical service layer answers `UNSUPPORTED` for its opcodes
regardless of what any adapter's own capability-state table says --
capability is decided once, centrally, by whether the module is compiled
in, not per-adapter. Of these four, the captive portal is implemented; the
Karma auto-responder, probe-request flood and beacon flood are not, and
their `#if`-guarded case labels still answer `UNSUPPORTED`
(`docs/PROVENANCE.md`).

The release/factory build (`sdkconfig.defaults`) enables all optional
modules by default -- this is a build-variant choice, not a canonical-API
change: the wire opcodes, schemas, and capability-negotiation surface are
identical whether a module is compiled in or out; only the runtime
behavior (UNSUPPORTED vs. real, once implemented) differs.

## Capability manifest and build-variant policy

A capability's *live* per-request state is always available via the
existing `GET_CAPABILITIES`/`GET_VERSION` canonical opcodes (unchanged --
this is the wire-level, machine-readable source of truth and is not
duplicated by a second, possibly-inconsistent mechanism). A capability's
*build-time* module state (which optional modules this specific firmware
image was compiled with) is documented in `docs/CAPABILITY_MANIFEST.md`,
generated by hand from `sdkconfig.defaults`/`main/Kconfig.projbuild` for
each named build variant -- this file must be updated whenever a module's
default changes or a new module is added, and `release/RELEASE_NOTES.md`
must cite it for every release candidate.

## Deliberate scope limits (see `docs/PROVENANCE.md` for full detail)

- No new community-feature radio behavior (Karma/probe-flood/captive-
  portal/beacon RF logic) was implemented -- explicitly out of scope for
  this architecture-correction pass, per instruction.
