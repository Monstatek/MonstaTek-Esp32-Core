# Contributing

MonstaTek Core targets the ESP32-C6 in the MonstaTek M1. The STM32 application
loader and Web Manager are maintained separately.

## Before making changes

Read [Architecture](docs/ARCHITECTURE.md), [Development](docs/DEVELOPMENT.md),
and the [compatibility ledger](docs/COMPATIBILITY_LEDGER.md). Keep each pull
request focused on one problem and explain the externally visible result.

Existing shipped interfaces are compatibility requirements. Propose changes to
wire formats, identifiers, timing, partition layout, and updater behavior for
maintainer review before implementing them.

## Pull requests

Include the problem, scope, rationale, test results, and remaining limitations.
Distinguish source tests from physical-device tests. Identify the revision,
toolchain, configuration, and artifact hashes behind any build or hardware claim.

Keep generated files synchronized with their source definitions. Do not add
build products, local evidence logs, credentials, personal paths, or reference
material excluded by `.gitignore` and [Source scope](docs/SOURCE_SCOPE.md).

## Bug reports

Include the Core revision or package SHA-256, M1 firmware version, hardware
revision if known, steps to reproduce, expected result, and actual result.
State whether the behavior survives a full power cycle. Sanitize logs before
posting; do not attach passwords or third-party captured traffic.
