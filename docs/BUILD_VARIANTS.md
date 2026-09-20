# Build variants

Core ships as more than one image because the ESP32-C6 cannot physically hold
every radio subsystem at once. Which image a device runs is a build-time
choice; what a host may call is discovered at runtime through
`GET_CAPABILITIES`, never inferred from a variant name.

| Variant | Config | Radios | 802.15.4 access |
|---|---|---|---|
| universal | `sdkconfig.defaults` | Wi-Fi, BLE, ESP-NOW | none |
| mtkcore-154 | `+ sdkconfig.154` | Wi-Fi, ESP-NOW, 802.15.4 | raw radio service |
| mtkcore-154-rcp | `+ sdkconfig.154 + sdkconfig.154-rcp` | Wi-Fi, ESP-NOW, 802.15.4 | OpenThread RCP (Spinel over the SPI link; Core SPI transport stands down — see "Spinel transport") |

```
idf.py -B build.universal -D SDKCONFIG=build.universal/sdkconfig \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults" build

idf.py -B build.154 -D SDKCONFIG=build.154/sdkconfig \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.154" build

idf.py -B build.154-rcp -D SDKCONFIG=build.154-rcp/sdkconfig \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.154;sdkconfig.154-rcp" build
```

## Why more than one image

Two independent constraints force this, both measured rather than assumed.

**1. Memory.** Free DIRAM must stay at or above 100,000 bytes
(`docs/RESOURCE_BUDGET.md`). The universal image sits at 101,244 bytes free --
roughly 1.2KB of headroom. Adding the 802.15.4 radio to it costs about 12.9KB
and OpenThread RCP a further ~9.6KB, either of which puts the image under the
floor. The 802.15.4 variants pay for the radio by compiling out the Bluetooth
controller (~28KB) and the captive portal's ~4.4KB credential store, which is
why they end up with more headroom than the universal image, not less.

Measured free DIRAM, each variant built from a clean tree with its own
`SDKCONFIG` output file:

| Variant | DIRAM used | free | vs. 100,000 floor |
|---|---|---|---|
| universal | 350,868 | **101,244** | +1,244 |
| mtkcore-154 | 334,180 | **117,932** | +17,932 |
| mtkcore-154-rcp | 267,276 | **184,836** | +84,836 |

The RCP image is the roomiest because it gives up the most: no Bluetooth
controller, no captive portal, and no Core SPI transport (the link belongs to
Spinel -- see "Spinel transport"). The floor is never lowered to make
something fit.

Build each variant with a distinct `SDKCONFIG` path. `SDKCONFIG_DEFAULTS` only
seeds a config file that does not yet exist, so reusing the default root
`sdkconfig` across variants silently rebuilds whichever config was written
last and yields three identical binaries.

**2. One driver, one callback owner.** The ESP-IDF 802.15.4 driver exposes a
single set of completion callbacks (`esp_ieee802154_receive_done`,
`transmit_done`, `transmit_failed`, `energy_detect_done`). OpenThread's port
layer defines those same symbols. Two definitions cannot coexist in one
binary, so raw radio access and OpenThread RCP are mutually exclusive at link
time. This is a property of the driver, not a Core design choice, and no
amount of runtime arbitration can work around a link-time symbol conflict.

That second constraint is why `mtkcore-154` and `mtkcore-154-rcp` are separate
images rather than one image with a runtime switch.

## What each 802.15.4 image serves

Both build the same service (namespace `0x0007`) and hold the same `IEEE154`
arbiter class; they differ in which half of the opcode range is served, and
capability reporting and dispatch are driven by the same build-time condition
so they can never disagree.

- **mtkcore-154 (raw):** serves `0x0001`–`0x0007` (start/stop/status, retune,
  energy scan, raw TX, metadata-carrying receive) and `0x000B`–`0x000D`
  (Core-managed capture). The RCP opcodes report `UNAVAILABLE`.
- **mtkcore-154-rcp:** serves `0x0008`–`0x000A` — RCP start/stop/status. The
  raw radio and capture opcodes report `UNAVAILABLE` because OpenThread owns
  the driver.
- **universal:** the service is never registered; every `0x0007` opcode
  reports `UNAVAILABLE`.

## Thread

Thread runs on the host, not on Core. `mtkcore-154-rcp` builds OpenThread in
`RADIO_MODE_NATIVE` with an RCP host connection, which links the radio and
Spinel layers only — no Thread application stack, no network state, no
dataset storage on the device. The host owns all Thread behaviour.

### Spinel transport

Spinel travels over the M1's **existing STM32↔ESP32 SPI wires**: MOSI 12,
MISO 13, SCLK 7, CS 15, with GPIO 6 — the line the Core transport uses for
DATA_READY — carrying Spinel's flow-control interrupt. No new routing is
required, and no separate UART is used, because the production M1 routes only
that one SPI link plus HANDSHAKE and DATA_READY between the two parts; there
is no second UART to speak Spinel over.

This has a consequence that must not be glossed over. The ESP32-C6 has one
general-purpose SPI peripheral (SPI0/SPI1 serve flash), and both Core's own
`spi_slave` transport and OpenThread's RCP host connection drive a slave on
it. **They cannot both run in one image.** In `mtkcore-154-rcp` the link
therefore belongs to Spinel: `main/mtek_spi_runtime.c` stands the Core SPI
transport down under `CONFIG_OPENTHREAD_RADIO`, and the host on those wires
talks to a Thread radio co-processor, not to the canonical Core protocol. The
factory UART0 adapter is unaffected and still reaches the canonical router, so
the image is not left without a control path.

The transport is a compile-time choice, not a runtime request:
`CONFIG_OPENTHREAD_RCP_SPI=y` is set in `sdkconfig.154-rcp` because ESP-IDF
`#if`-gates the port layer on it and defaults to UART. Setting only the
runtime `host_connection_mode` field links the UART host path and silently
ignores the SPI pins — verified by inspecting which host object the map file
pulls in.

## Zigbee

Zigbee is deliberately **not** implemented on-device. No ZBOSS or Espressif
Zigbee managed component is added. Two reasons, in order of importance:

1. The same boundary that serves Thread serves Zigbee. A host-side Zigbee
   stack consumes the radio through the raw 802.15.4 service, which moves PHY
   payloads and never parses or synthesises MAC headers. Nothing about the
   Core interface is Thread-specific.
2. It would not fit. ZBOSS is heavier than OpenThread, which already consumes
   most of the memory reclaimed by dropping BLE.

A future Zigbee host therefore needs host-side work and hardware validation,
not another Core redesign.

## Capture

`mtkcore-154` captures with Core managing the channel plan. `IEEE154_CAPTURE_START`
takes either a fixed channel or a hopping mask over channels 11–26 with a
per-channel dwell. Hopping is deterministic: the next selected channel in
ascending order, wrapping at the top of the mask, advanced only once a full
dwell has elapsed, at most one retune per service tick. A single-channel mask
does not retune at all. Hopping is driven from the service tick rather than
its own task, so it shares the session's lifetime and cannot outlive a
teardown -- stop, peer reset, arbitration loss and start failure all end it
and release the lease.

Captured frames are drained through `IEEE154_POLL_RECV`, the same bounded ring
the raw session uses, and carry complete MPDUs plus the metadata a PCAP export
needs: SFD timestamp, channel, RSSI, LQI, original length, truncation flag.
Ring overflow drops the oldest frame and counts it, reported by
`IEEE154_CAPTURE_STATUS`. Core decodes no application protocol.

## Versioned host contract

`GET_API_IDENTITY` (`0x0000/0x000A`) is served by every image and reports
`api_major`/`api_minor`, a `capability_count` computed from the live opcode
table for that image, and a diagnostic `variant_id`/`variant_name`.

The current contract is **API 1.0**. `api_minor` increments for additive,
backward-compatible growth: new opcodes, new capability IDs, or an opcode
moving from `UNAVAILABLE` to `SUPPORTED` in some image. `api_major` increments
only for a change that breaks an existing host contract: a removed or
renumbered opcode, an incompatible request/response shape, or changed
semantics for an existing opcode.

A host must negotiate features through `GET_CAPABILITIES` and must never
branch on `variant_id` or `variant_name`, which carry no feature meaning.

## Capability negotiation

A host must not branch on variant names. `GET_CAPABILITIES` reports the real
per-opcode state for the running image, and `IEEE154_RCP_STATUS` exposes
link details diagnostically. Service registration, the capability table and
actual dispatch are driven by one build-time condition and are enforced to
agree by a property test that walks every registered opcode
(`host_tests/test_opcode_registry.c`).

## Arbitration

All three images share one arbiter and one radio-ownership model. `IEEE154`
is a first-class class: `SERIALIZED` against every Wi-Fi class and `ESPNOW`
(one 2.4GHz radio), `CROSS_SUBSYSTEM_BUSY` against the BLE classes. A raw
session, an energy scan and the RCP runtime each take the same single lease,
so they can never overlap each other or Wi-Fi. Stop, start failure and peer
reset all funnel through one teardown that releases the lease.

## Stack budget and variants

`tools/check_stack_budget.py` is variant-aware: a chain may declare
`requires_sdkconfig`, and chains whose subsystem is compiled out are skipped
and reported. A symbol missing while its option is enabled still fails loud,
preserving the rename/removal guarantee. Skips are always printed so a
variant cannot lose coverage silently.
