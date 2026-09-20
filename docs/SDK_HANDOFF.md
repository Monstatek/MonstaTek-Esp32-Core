# Core → SDK / STM32 handoff

What a host needs to drive this firmware. Generated from the reconciled
source: `tools/schemas.json` is the machine-authoritative contract, and
everything below is an index into it, not a second source of truth.

## API identity

`GET_API_IDENTITY` (`0x0000`) reports the frozen host contract version.

| Field | Value |
|---|---|
| api_major | 1 |
| api_minor | 0 |

`api_minor` increments when capabilities are added without breaking an
existing host contract: new opcodes, new capability IDs, or an opcode moving
from `UNAVAILABLE` to `SUPPORTED` in some image. `api_major` increments only
for a change that breaks an existing host contract: a removed or renumbered
opcode, an incompatible request/response shape, or changed semantics for an
existing opcode.

**Hosts must negotiate through `GET_CAPABILITIES` and must never branch on a
variant name or id.** Three images ship; they differ in which opcodes are
dispatchable, and each reports that truthfully.

## Services and opcodes

| Service | Namespace | Opcodes |
|---|---|---|
| system | `0x0000` | 10 |
| wifi | `0x0001` | 41 |
| ble | `0x0002` | 23 |
| gatt | `0x0003` | 10 |
| capture | `0x0004` | 6 |
| diagnostics | `0x0005` | 4 |
| espnow | `0x0006` | 6 |
| ieee802154 | `0x0007` | 13 |
| **total** | | **113** |

Per-opcode request/response field lists, capability state per transport
profile, status outcomes, deadline bounds, cancellability, idempotence,
persistent effects and stop ownership are all in `tools/schemas.json`. The C
structs the wire maps to are generated into
`components/mtek_schema/include/mtek_schema_structs.h`; do not hand-write
them.

## Transport — read this before wiring the STM32 side

This is where the three images genuinely differ, and where an assumption
carried over from rc13 will fail.

| Image | Canonical Core protocol over SPI | Spinel | Factory UART0 REPL |
|---|---|---|---|
| universal | **yes** | no | yes |
| mtkcore-154 | **yes** | no | yes |
| mtkcore-154-rcp | **no** | yes | yes |

The ESP32-C6 has one general-purpose SPI peripheral, and the M1 routes one
SPI link between the STM32 and the ESP32-C6. In `mtkcore-154-rcp` that link
belongs to OpenThread: the STM32 speaks **Spinel** on those wires and the
ordinary Core SPI command channel **is not available**. Core's SPI runtime is
compiled out of that image entirely. A host that assumes it can still issue
canonical opcodes over SPI in RCP mode will get no response, because nothing
is listening.

The factory UART0 adapter remains available in all three images and still
reaches the canonical router, so RCP-mode builds are not left without a
control path.

### GPIO 6 changes meaning between images

| Image | Role | Asserted state |
|---|---|---|
| universal, mtkcore-154 | Core DATA_READY | **HIGH** = outbound data available |
| mtkcore-154-rcp | Spinel host interrupt | **LOW** = RCP has a frame to transfer |

The polarities are opposite. The STM32 must invert its interpretation
according to the flashed image. A host that does not will read "no data"
exactly when the RCP has a frame ready. The firmware cannot detect or correct
this. See `docs/BUILD_VARIANTS.md` for the driver-level detail and
`docs/HARDWARE_DEBT.md` for what remains unverified on real hardware.

## Radio arbitration

One 2.4GHz radio serves Wi-Fi, BLE and 802.15.4, so the arbiter grants one
active class at a time within a conflict group rather than letting two
subsystems configure the PHY at once.

Classes: `WMC`, `WS`, `BEACON`, `D`, `H`, `M`, `BS`, `BA`, `SM`, `GC`, `SAP`,
`RAW`, `ESPNOW`, `IEEE154`.

- `IEEE154` and `ESPNOW` serialize against every Wi-Fi class; `IEEE154` is
  cross-subsystem-busy against the BLE classes.
- `SAP` covers SoftAP and the captive portal, which share one interface lease
  rather than bringing up a second.
- A host that receives a busy status should retry rather than treat it as a
  failure; the pairwise policy table is in `tools/schemas.json`
  (`arbiter_pairwise_policy`).

## Capture

Capture sessions deliver into a bounded ring with drop-oldest semantics, and
the host drains via poll/read opcodes. A host that polls too slowly loses the
oldest frames rather than stalling the radio; session statistics report the
drop count so this is observable rather than silent.

802.15.4 Core-managed capture adds deterministic bounded channel hopping over
channels 11-26, and is served only by `mtkcore-154` (never by the RCP image,
where OpenThread owns the driver callbacks).

## Capability matrices per image

See `docs/CAPABILITY_MANIFEST.md`. In summary: the universal image serves no
`0x0007` opcodes at all; `mtkcore-154` serves the raw radio and capture
ranges; `mtkcore-154-rcp` serves only the RCP control opcodes. Capability
reporting and dispatch are driven by the same build-time condition, and a
property test fails loudly if they ever disagree.

## Build variants

`docs/BUILD_VARIANTS.md` has the full rationale. Build them with
`tools/build_variants.py`, which gives each variant its own sdkconfig and
asserts the resulting images really differ — `SDKCONFIG_DEFAULTS` alone
silently shares one configuration across variants.
