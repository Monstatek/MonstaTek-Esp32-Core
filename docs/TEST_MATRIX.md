# Test matrix

The host suite contains 70 registered tests. It exercises schema encoding,
operation lifecycle, arbitration, malformed inputs, the three transport
adapters, asynchronous delivery, session invalidation, concurrency closures,
Wi-Fi service logic, BLE/GATT service logic, capture logic, and release-package
validation.

The reviewed RC12 evidence reported:

- 70/70 host tests passing under ASan+UBSan.
- 70/70 host tests passing under TSan with no reported races.
- Focused Community/C3 deferred-completion tests repeated 100 times under each
  sanitizer configuration.
- A clean ESP-IDF v6.0.1 ESP32-C6 target build.

These are engineering tests, not complete-device regression. Hardware
validation must separately cover the shipped scan, connect, signal-meter,
BLE/GATT, SD-card updater, Web Manager, reboot, persistence, and recovery paths.
Lab features must be recorded as separate tests rather than grouped into a
generic Wi-Fi result.

The source-only host command uses `MTK_RELEASE_PRE_PACKAGING=1`; therefore that
run does not validate a merged release package. See `DEVELOPMENT.md`.
