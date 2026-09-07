# Architecture: canonical core, adapters, and optional modules

MonstaTek M1 is the canonical platform. Community/legacy implementations
(Bedge/C3-lineage and any future third-party protocol) adapt to this
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
  Raw TX and the monitor-mode primitives used by deauthentication,
  handshake capture, and packet capture live in canonical core because
  they are general, reusable platform capabilities. SoftAP and PMKID
  capture remain explicit unsupported capability placeholders.

### Transport adapters (three, boot-exclusive, `SPI_PROTOCOL_V1.md`
"Runtime transport selection")

| Adapter | Role | Parity obligation |
|---|---|---|
| Factory M1 UART | Legacy ASCII REPL over UART0 | **Strict shipped parity** -- must not change shipped List A observable behavior |
| Native M1 SPI v1 | The canonical, future-facing binary interface | Owns the canonical wire shape; new canonical features are designed against this adapter first |
| M1 Community Compatibility (legacy Bedge/C3 wire profile) | Optional translation of a legacy/community (Bedge/C3-lineage) SPI wire protocol into the canonical API | No parity obligation of its own; translates faithfully from confirmed facts, never invents canonical behavior to match community quirks |

The M1 Community Compatibility adapter is exactly that -- a **compatibility**
layer. Its own wire opcodes, structs, and behavioral citations
(`mtek_bedge_*` source files, `mtk_bedge_*` C identifiers) retain their
historical Bedge/C3 naming internally, because that is where the
fact-citations legitimately live and a full internal rename was judged
too large a change to make safely under this session's time budget
without git version control to review it against. User-facing text
(Kconfig prompts, this document, release notes) uses "M1 Community
Compatibility" (or "M1 Community Compatibility (legacy Bedge/C3 wire
profile)" on first mention in a document); historical source-
identification lives in `docs/COMPATIBILITY_LEDGER.md`.

### Optional capability modules

Community-specific attack conveniences that are not shipped List A
behavior and not owner-approved required List B features are **optional,
compile-time-selectable modules** (`main/Kconfig.projbuild`, "Optional
capability modules" menu): Karma auto-responder, probe-request flood,
captive portal, Wi-Fi beacon flood. Disabling a module's Kconfig option
means the canonical service layer answers `UNSUPPORTED` for its opcodes
regardless of what any adapter's own capability-state table says --
capability is decided once, centrally, by whether the module is compiled
in, not per-adapter. None of these four modules' actual radio-behavior
logic is implemented yet this session (`docs/PROVENANCE.md`); the
Kconfig gates and explicit `#if`-guarded case labels in
`mtek_wifi_logic.c` establish the compile-time boundary now, so whichever
module gets implemented first does not have to also invent this
structure.

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

## What this session did NOT do (see `docs/PROVENANCE.md` for full detail)

- No new community-feature radio behavior (Karma/probe-flood/captive-
  portal/beacon RF logic) was implemented -- explicitly out of scope for
  this architecture-correction pass, per instruction.
- The M1 Community Compatibility adapter's internal C identifiers/file names
  were not renamed away from `mtek_bedge_*`/`mtk_bedge_*` -- a
  user-facing/documentation-level rename only, this session.
