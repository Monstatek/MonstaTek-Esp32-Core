# Compatibility ledger: Legacy SPI Compatibility

This ledger records the legacy SPI compatibility boundary. The canonical wire
interface remains native M1 SPI v1.

## What this module is

An independently implemented (`docs/PROVENANCE.md`) optional translation layer
for the established `m1_link` SPI wire protocol on one side and the canonical
`mtk_router_dispatch` API on the other. It exists so a legacy host that only speaks that wire
protocol can still reach this firmware's real canonical services --
never the other way around: canonical behavior is never shaped to match
this module's quirks.

## Where the detailed citations live

Every opcode-level fact citation (exact wire byte layouts, confirmed
behaviors, and the specific contract-package section each is
drawn from) is kept inline, at the point of use, in:

- `components/mtek_transport_spi_compat/mtek_compat_frame.c`/`.h` -- the
  `m1_link` cell/header/CRC/fragmentation format.
- `components/mtek_transport_spi_compat/mtek_compat_dispatch.c` -- every
  one of the 44 `capability_state.compat_spi=SUPPORTED` opcodes' request/
  response translation, each with its own doc comment citing the exact
  fact and its confirmation source.
- `main/mtek_spi_runtime.c` -- the physical `spi_slave`/GPIO wiring facts
  (pins, mode, HANDSHAKE timing, DATAREADY default).

This ledger intentionally does not duplicate that per-opcode detail (it
would drift out of sync with the code); it is the *pointer* to where
compatibility-module evidence lives, and the statement of the boundary
rule: **internal protocol citations belong in this module's own files and
this ledger, never in `docs/ARCHITECTURE.md`, `docs/RESOURCE_BUDGET.md`,
or any canonical-core service's own documentation.**

## Naming

The project-owned adapter uses **Legacy SPI Compatibility** terminology:
`mtek_transport_spi_compat` for its component, `mtek_compat_*` for files,
`mtk_compat_*` / `MTK_COMPAT_*` for C symbols, and `compat_spi` for
schema adapter/capability keys. Enable it with
`CONFIG_MTEK_ADAPTER_COMPAT_SPI=y`. Downstream source integrations and saved
build configurations must migrate identifiers; old source aliases are not
retained. Existing devices still use the same wire protocol and opcode values.
The naming change does not alter the established wire protocol.

## Known scope limits (see `docs/PROVENANCE.md` for the full, current list)

- Wire layouts that are not established by the protocol contract remain
  explicitly unsupported rather than being guessed.
- The 12-opcode BLE compatibility family and `WIFI_MODE_GET`/`SET` are
  capability-`DISABLED` for this profile by design; this is not a
  module-boundary decision.
