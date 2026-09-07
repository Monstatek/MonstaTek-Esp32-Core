# Public engineering decisions

This concise log records the architecture decisions needed to understand the
current source. Detailed internal review transcripts and raw machine logs are
not part of the public repository.

## Canonical core and adapters

Product behavior lives in transport-neutral services. Factory UART, native SPI
v1, and Community/C3 are adapters into that core. All three are built and start
their physical loops; the first adapter receiving valid traffic claims the boot
session through the shared transport-selection latch.

## Resource bounds

Native SPI v1 uses an 8,192-byte reassembled-message limit and one active inbound
reassembly buffer while retaining four dispatched in-flight operation records.
This prevents unbounded static allocation on ESP32-C6 while rejecting overlap
explicitly instead of corrupting it. See `RESOURCE_BUDGET.md`.

## Candidate identity

RC12 uses a fixed, deliberate build epoch and embeds the candidate identifier.
Generated schema files are regenerated before the final build and must be byte-
idempotent. Application binaries are not updater packages; packaging follows
`RELEASE_PACKAGE.md`.

## Build integrity

Run stack-usage measurement only against a disposable measurement build because
the current tool recompiles selected objects. Keep the candidate build pristine
and confirm that a final Ninja dry-run schedules no source compilation or relink.
