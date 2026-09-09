# Compatibility ledger: M1 Community Compatibility (community/C3-lineage)

This ledger records the community/C3 compatibility boundary. The canonical
wire interface remains native M1 SPI v1. Source citations retain their file
basenames and technical findings; `references/compat/...` is a normalized
reference-root label, not a claim that the external source was renamed.
See `docs/THIRD_PARTY_NOTICES.md` for the original project's identity.

## What this module is

An independently clean-room-implemented (`docs/PROVENANCE.md`) optional
translation layer: a community/legacy SPI wire protocol (community/C3-lineage
`m1_link`, confirmed via `001-profile-bootstrap-feasibility.md` and
`002-adapter-translation-matrix.md` in the accepted facts-only contract
package) on one side, and the canonical `mtk_router_dispatch` API on the
other. It exists so a legacy/community host that only speaks that wire
protocol can still reach this firmware's real canonical services --
never the other way around: canonical behavior is never shaped to match
this module's quirks.

## Where the detailed citations live

Every opcode-level fact citation (exact wire byte layouts, source-
confirmed behaviors, and the specific facts-package section each is
drawn from) is kept inline, at the point of use, in:

- `components/mtek_transport_spi_compat/mtek_compat_frame.c`/`.h` -- the
  `m1_link` cell/header/CRC/fragmentation format.
- `components/mtek_transport_spi_compat/mtek_compat_dispatch.c` -- every
  one of the 44 `capability_state.compat_c3=SUPPORTED` opcodes' request/
  response translation, each with its own doc comment citing the exact
  fact and its confirmation source.
- `main/mtek_spi_runtime.c` -- the physical `spi_slave`/GPIO wiring facts
  (pins, mode, HANDSHAKE timing, DATAREADY default).

See `docs/THIRD_PARTY_NOTICES.md` for the community reference project
consulted to establish these facts and its license status.

This ledger intentionally does not duplicate that per-opcode detail (it
would drift out of sync with the code); it is the *pointer* to where
compatibility-module evidence lives, and the statement of the boundary
rule: **community source citations belong in this module's own files and
this ledger, never in `docs/ARCHITECTURE.md`, `docs/RESOURCE_BUDGET.md`,
or any canonical-core service's own documentation.**

## Naming

The project-owned adapter uses **Mtek Community Compatibility** terminology:
`mtek_transport_spi_compat` for its component, `mtek_compat_*` for files,
`mtk_compat_*` / `MTK_COMPAT_*` for C symbols, and `compat_c3` for
schema adapter/capability keys. Enable it with
`CONFIG_MTEK_ADAPTER_COMPAT_C3=y`. Downstream source integrations and saved
build configurations must migrate identifiers; old source aliases are not
retained. Existing devices still use the same wire protocol and opcode values.
The naming change does not transfer ownership of the referenced upstream work.

## Known scope limits (see `docs/PROVENANCE.md` for the full, current list)

- Two opcodes (`HANDSHAKE_READ`, `GATT_CONNECT`... now closed this
  review round from newly-supplied exact facts) and a handful of others
  whose exact Mtek Compatibility-side wire byte layout remains unconfirmed are
  explicit, cited, tested blockers -- never guessed.
- The 12-opcode BLE compatibility family and `WIFI_MODE_GET`/`SET` are
  capability-`DISABLED` for this profile per the accepted contract itself
  -- not a module-boundary decision, a contract fact.
