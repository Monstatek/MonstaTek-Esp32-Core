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

## Token-family isolation

Every generic (token, boot_epoch)-addressed core API validates that a token
belongs to the family that minted it before acting on it, in the same lock
acquisition as the lookup -- a token from one operation family (e.g.
STA_SCAN) cannot be used to transition, finalize, or release resources for
an unrelated family (e.g. AP_SCAN) it was never issued against. GATT and
capture-poll handlers use their own service-local identity match instead,
since a live connection can outlive the operation record that created it.

## Callback lifetime under concurrent teardown

BLE HAL callbacks hold a single lock across their entire is-current-check,
use, and semaphore-signal span (`mtk_ble_op_lifecycle`), so a callback that
passes its liveness check cannot be preempted and then race a concurrent
timeout/retire path that has already torn down the context it is about to
use. This closes the check-to-use gap that a generation-only guard leaves
open.

## Community/Bedge deferred completion

A deferred (asynchronous) Community/C3 operation reports its accepted
status and its eventual terminal event as two independent parts that may
arrive in either order or in the same poll. The adapter's two-part
completion state machine preserves whichever half arrives first and only
produces the confirmed response once both are present, so a slow HAL
worker can never cause the accepted status or the terminal result (network
list, connection token, or failure) to be silently dropped.
