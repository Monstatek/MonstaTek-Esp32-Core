# Test matrix

The host suite contains 73 registered tests. It exercises schema encoding,
operation lifecycle, arbitration, malformed inputs, the three transport
adapters, asynchronous delivery, session invalidation, concurrency behavior,
Wi-Fi service logic, BLE/GATT service logic, capture logic, and release-package
validation.

Current source-only host verification:

- 73/73 host tests pass under ASan+UBSan.
- 73/73 host tests pass under TSan with no reported races.

The `rc13` candidate includes a factory-UART MonstaShark compatibility test
covering exact command parsing and recognition, the literal `PCAP ACK`, the
115200-to-3,000,000-baud transition order, `CAP_READY`, 32,768-byte credit
handling and over-credit prevention, a byte-exact `CAP_FRAME_BATCH` golden
vector with the 24-byte version-1 record header, `CAP_STOP`/`CAP_STOPPED`,
console-baud restoration followed by `PCAP DONE`, BLE-radio-busy rejection,
and cleanup at every injected transport-failure boundary. The full 73-test suite passes under
ASan+UBSan and TSan. This evidence is still host-side; the candidate remains
`NOT_YET_HARDWARE_VALIDATED`.

These are engineering tests, not complete-device regression. Hardware
validation must separately cover the shipped scan, connect, signal-meter,
BLE/GATT, SD-card updater, Web Manager, reboot, persistence, and recovery paths.
Lab features must be recorded as separate tests rather than grouped into a
generic Wi-Fi result.

The source-only host command uses `MTK_RELEASE_PRE_PACKAGING=1`; therefore that
run does not validate a merged release package. See `DEVELOPMENT.md`.
