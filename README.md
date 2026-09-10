# MonstaTek Core

MonstaTek Core is ESP32-C6 firmware for the MonstaTek M1. It provides a shared service core with factory UART, native SPI, and Legacy SPI Compatibility transport adapters. The STM32 application loader and Web Manager are separate projects.

The implementation separates portable service and lifecycle logic from transport translation and ESP-IDF hardware interfaces. Host tests cover portable behavior; hardware validation establishes behavior on a complete M1.

The current source passes host verification and a clean ESP32-C6 build check. Release-package identity and hardware-validation status are recorded separately for each exact artifact; source verification alone does not establish hardware compatibility.

Development prerequisites, verified build commands, testing distinctions, and known tooling constraints are described in [Development](docs/DEVELOPMENT.md). The exact files intentionally included and excluded are listed in [Source scope](docs/SOURCE_SCOPE.md).

Repository structure:

- `components/`: portable core, hardware interfaces, services, and transport adapters.
- `main/`: ESP-IDF application integration and configuration.
- `host_tests/`: host tests and their support code.
- `tools/`: schema generation and engineering checks.
- `docs/`: architecture, development, compatibility, and release-process documentation.
- `partitions.csv`: established partition layout.
- `sdkconfig.defaults`: reviewed build defaults.

Generated source needed by the build belongs in the selected source set. Build outputs, reference material, staging packages, and raw local logs do not.

An application-only binary is not an M1 updater package. Follow the [release-package contract](docs/RELEASE_PACKAGE.md) when preparing any hardware-validation artifact.

## License

GPL-3.0-only. See [LICENSE](LICENSE).
