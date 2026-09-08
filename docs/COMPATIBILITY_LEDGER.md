# Compatibility ledger: M1 Community Compatibility (Bedge/C3-lineage)

This document is the single place historical source-lineage identifiers
(Bedge/C3, `m1_link`, `M1ESP_*` opcode names, `references/bedge/...` file
citations) are allowed to appear, per `docs/ARCHITECTURE.md`'s module
boundary. Canonical-core documentation must not name Bedge/C3 as if it
were the canonical protocol; the canonical wire interface is native M1
SPI v1.

## What this module is

An independently clean-room-implemented (`docs/PROVENANCE.md`) optional
translation layer: a community/legacy SPI wire protocol (Bedge/C3-lineage
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

- `components/mtek_transport_spi_bedge/mtek_bedge_frame.c`/`.h` -- the
  `m1_link` cell/header/CRC/fragmentation format.
- `components/mtek_transport_spi_bedge/mtek_bedge_dispatch.c` -- every
  one of the 44 `capability_state.bedge_c3=SUPPORTED` opcodes' request/
  response translation, each with its own doc comment citing the exact
  fact and its confirmation source.
- `main/mtek_spi_runtime.c` -- the physical `spi_slave`/GPIO wiring facts
  (pins, mode, HANDSHAKE timing, DATAREADY default).

This ledger intentionally does not duplicate that per-opcode detail (it
would drift out of sync with the code); it is the *pointer* to where
compatibility-module evidence lives, and the statement of the boundary
rule: **community source citations belong in this module's own files and
this ledger, never in `docs/ARCHITECTURE.md`, `docs/RESOURCE_BUDGET.md`,
or any canonical-core service's own documentation.**

## Naming

- RC11 release-finalization correction (independent audit): renamed
  from "M1 Compatibility SPI". User-facing name (Kconfig, release notes,
  this project's own documentation prose): **M1 Community Compatibility**
  (or **M1 Community Compatibility (legacy Bedge/C3 wire profile)** on
  first mention in a document, so a reader can still connect it to the
  historical lineage this ledger documents).
- Internal source identifiers (file names, C symbol prefixes): retain
  `mtek_bedge_*`/`mtk_bedge_*` this session -- a documentation-level
  rename only was completed; see `docs/ARCHITECTURE.md`'s "What this
  session did NOT do" for the reason (risk of a multi-file rename without
  git version control to review it against, under this session's time
  budget).

## Known scope limits (see `docs/PROVENANCE.md` for the full, current list)

- Two opcodes (`HANDSHAKE_READ`, `GATT_CONNECT`... now closed this
  review round from newly-supplied exact facts) and a handful of others
  whose exact Bedge-side wire byte layout remains unconfirmed are
  explicit, cited, tested blockers -- never guessed.
- The 12-opcode BLE compatibility family and `WIFI_MODE_GET`/`SET` are
  capability-`DISABLED` for this profile per the accepted contract itself
  -- not a module-boundary decision, a contract fact.
