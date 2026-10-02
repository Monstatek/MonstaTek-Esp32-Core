# MonstaTek Core

MonstaTek Core is ESP32-C6 firmware for the MonstaTek M1. It provides a shared service core with factory UART, native SPI, and Legacy SPI Compatibility transport adapters. 

Repository structure:

- `components/`: portable core, hardware interfaces, services, and transport adapters.
- `main/`: ESP-IDF application integration and configuration.
- `host_tests/`: host tests and their support code.
- `tools/`: schema generation and engineering checks.
- `docs/`: architecture, development, build identity, compatibility, and release-process documentation.
- `partitions.csv`: established partition layout.
- `sdkconfig.defaults`: reviewed build defaults.


## License

GPL-3.0-only. See [LICENSE](LICENSE).

## Contributing and reporting issues

Read [Contributing](CONTRIBUTING.md) before submitting changes. Report reproducible
problems through this repository's Issues tab, including firmware identity and
observed behavior. Remove credentials, personal data, and captured network data
from public reports.
