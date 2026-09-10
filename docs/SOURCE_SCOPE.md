# Public source scope

This repository contains the ESP32-C6 MonstaTek Core firmware only.

Included:

- ESP-IDF project files and reviewed defaults.
- Core, services, hardware abstraction layers, and three transport adapters.
- Generated schema and Legacy SPI Compatibility opcode sources required by the build.
- Host tests and reusable fake-HAL support headers.
- Schema generators, release tooling, and resource checks.
- Current public engineering documentation.

Excluded:

- STM32 firmware, the application loader, and Web Manager.
- Factory/reference binaries and locally packaged releases.
- Reference implementations and research folders.
- Schematics, BOMs, board documents, and private hardware material.
- Build directories, raw sanitizer logs, local evidence archives, editor state,
  machine-specific configuration, and credentials.

The source set was assembled from an explicit allowlist. It was not produced by
moving or cloning the engineering/reference workspace.
