# MonstaTek Core

MonstaTek Core is ESP32-C6 firmware for the MonstaTek M1. It provides a shared service core with factory UART, native SPI, and Legacy SPI Compatibility transport adapters. The STM32 application loader and Web Manager are separate projects.

The implementation separates portable service and lifecycle logic from transport translation and ESP-IDF hardware interfaces. Host tests cover portable behavior; hardware validation establishes behavior on a complete M1.

Host verification covers portable service, lifecycle, and protocol behavior under AddressSanitizer/UndefinedBehaviorSanitizer and ThreadSanitizer, alongside generator idempotence and static resource and stack gates. That establishes source-level correctness and build reproducibility for a given commit; it does not establish hardware behavior, which is recorded separately. Identify any specific build from the generated `PACKAGING_MANIFEST.md` inside its package rather than from a version string -- see [build identity](docs/BUILD_IDENTITY.md).

Release preparation is in progress. Source verification and hardware-validation results must identify the exact revision and artifact tested. See [Release readiness](docs/RELEASE_READINESS.md) for the remaining gates.

Development prerequisites, verified build commands, testing distinctions, and known tooling constraints are described in [Development](docs/DEVELOPMENT.md). The exact files intentionally included and excluded are listed in [Source scope](docs/SOURCE_SCOPE.md).

Repository structure:

- `components/`: portable core, hardware interfaces, services, and transport adapters.
- `main/`: ESP-IDF application integration and configuration.
- `host_tests/`: host tests and their support code.
- `tools/`: schema generation and engineering checks.
- `docs/`: architecture, development, build identity, compatibility, and release-process documentation.
- `partitions.csv`: established partition layout.
- `sdkconfig.defaults`: reviewed build defaults.

Generated source needed by the build belongs in the selected source set. Build outputs, reference material, staging packages, and raw local logs do not.

An application-only binary is not an M1 updater package. Follow the [release-package contract](docs/RELEASE_PACKAGE.md) when preparing any hardware-validation artifact.

## License

GPL-3.0-only. See [LICENSE](LICENSE).

## Contributing and reporting issues

Read [Contributing](CONTRIBUTING.md) before submitting changes. Report reproducible
problems through this repository's Issues tab, including firmware identity and
observed behavior. Remove credentials, personal data, and captured network data
from public reports.
