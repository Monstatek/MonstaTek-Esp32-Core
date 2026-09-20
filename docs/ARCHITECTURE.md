# Architecture: canonical core, adapters, and optional modules

MonstaTek M1 is the canonical platform. Legacy SPI integrations and any
future additional protocol adapt to this firmware's stable interfaces; their
differences are translated at the adapter
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
 factory UART adapter   native M1 SPI v1     Legacy SPI Compatibility
 (strict shipped parity) (canonical future-  (optional legacy
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
  READ/STOP`, MonstaShark capture -- all required and
  compiled in unconditionally, never module-gated); stable radio/
  transport/storage primitives (Wi-Fi/BLE HAL abstractions, capture
  service); capability negotiation (`GET_CAPABILITIES`/`GET_VERSION`).
  SoftAP, raw-TX, and monitor-mode primitives live in canonical core
  because they are general, reusable platform capabilities (a station-
  mode radio can always be put into AP or monitor mode; that is not a
  community-specific behavior).

  Raw-TX and monitor-mode are fully implemented, not scaffolding: `RAW_TX_SEND`
  (`handle_raw_tx_send`, `mtek_wifi_service/mtek_wifi_logic.c`, class `RAW`)
  transmits via `esp_wifi_80211_tx`, and monitor-mode entry/exit, bounded frame
  delivery and cancellation/cleanup live in `mtek_capture_service`
  (`CAPTURE_START/STOP/STATUS/SESSION_INFO/STATS/POLL_READ`, class `M`) via
  `esp_wifi_set_promiscuous`. Both are reachable identically from every
  transport that declares them `SUPPORTED` (native SPI and legacy SPI
  compatibility; `RAW_TX_SEND` is `UNAVAILABLE` on factory UART, `CAPTURE_START`
  is available there), through the same `mtk_router_dispatch` path every other
  canonical opcode uses -- no parallel or transport-embedded implementation
  exists.

### Transport adapters (three, boot-exclusive, `SPI_PROTOCOL_V1.md`
"Runtime transport selection")

| Adapter | Role | Parity obligation |
|---|---|---|
| Factory M1 UART | Legacy ASCII REPL over UART0 | **Strict shipped parity** -- must not change shipped List A observable behavior |
| Native M1 SPI v1 | The canonical, future-facing binary interface | Owns the canonical wire shape; new canonical features are designed against this adapter first |
| Legacy SPI Compatibility | Optional translation of the established legacy SPI wire protocol into the canonical API | No parity obligation of its own; translates faithfully from confirmed facts, never invents canonical behavior to match legacy-protocol constraints |

The Legacy SPI Compatibility adapter translates the legacy SPI compatibility wire
protocol without changing its framing, opcode values, or behavior. Project-owned
source files use `mtek_compat_*`, C identifiers use `mtk_compat_*`, and the
build option is `CONFIG_MTEK_ADAPTER_COMPAT_SPI`. Source/configuration names
have changed; existing integrations must update these names when rebuilding.
This is not a wire-protocol migration. Compatibility evidence is described in
`docs/COMPATIBILITY_LEDGER.md`.

### Optional capability modules

Legacy-protocol-specific attack conveniences that are not shipped List A
behavior or required List B features are **optional,
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

- Karma, probe-flood and beacon RF logic remain unimplemented; their opcodes
  answer `UNSUPPORTED` rather than advertising behavior the image lacks.
