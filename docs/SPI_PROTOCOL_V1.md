# Native M1 SPI v1

Native SPI v1 is the canonical binary transport between an M1 host and the
ESP32-C6 service core. The implementation is authoritative for byte-level
framing; this document summarizes its invariants.

- Fixed-size cells use the definitions in
  `components/mtek_transport_spi_native/include/mtek_spi_native_frame.h`.
- Discovery begins conservatively and negotiates the native cell size before
  normal traffic.
- Fragmented control messages are bounded by `MTK_SPI_NATIVE_MAX_MESSAGE`.
- One inbound message may be reassembled at a time; conflicting FIRST cells are
  rejected explicitly.
- Four fully received operations may remain in flight while asynchronous
  completion is pending.
- Requests, responses, events, credit, cancellation, duplicate detection, and
  peer-session invalidation are handled by `mtek_spi_native_dispatch.c`.
- A peer HELLO epoch identifies the STM32 session. The ESP boot epoch remains a
  separate local identity and is used for ESP-originated operation tokens.

Factory UART, native SPI, and Community/C3 physical loops start at boot. The
first valid protocol traffic claims the boot session through
`mtek_transport_claim_try`; other adapters remain physically responsive but do
not dispatch into the core during that session.

Any external protocol specification published later must be generated or
checked against the framing constants and host tests in this repository so it
cannot silently diverge from the implementation.
