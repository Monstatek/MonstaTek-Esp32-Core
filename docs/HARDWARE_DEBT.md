# Hardware debt

Items that cannot be resolved without measurements from a real board. Each is
a deliberate, disclosed choice, not an unknown. None is a defect in the
host-verifiable logic; all are places where the correct value or sequence is
an empirical property of the silicon.

## 1. Fixed 50 ms radio settle delay

`esp32_restore_sta_mode` (`components/mtek_wifi_service/mtek_wifi_hal_esp32.c`)
waits a fixed 50 ms between `esp_wifi_stop()` and `esp_wifi_set_mode()`.

- **Why it exists.** A mode change is only well-defined against a stopped
  driver. The delay gives the driver time to finish stopping.
- **Why it was not replaced.** A retry-on-error loop would only help if a
  premature mode change reported a failed return; ESP-IDF does not guarantee
  that, so the failure could be silent. An event wait on `WIFI_EVENT_STA_STOP`
  races a stop that has already completed. Both replace a known-conservative
  delay with an unverified mechanism on the single path every radio teardown
  uses (capture, handshake, deauth, raw TX, SoftAP, captive portal, ESP-NOW).
- **What would close it.** On real hardware, measure the minimum interval at
  which `esp_wifi_set_mode` followed by `esp_wifi_start` reliably restores
  station mode across repeated teardowns, including immediately after a
  promiscuous capture and after an AP teardown. Either reduce the constant to
  a measured-safe value with margin, or replace it with a condition the
  measurement shows is actually observable.

## 2. Wi-Fi driver callback stack budget

`promisc_trampoline` runs on a stack the Wi-Fi driver owns, not one this
project configures. `tools/check_stack_budget.py` assumes it is no larger than
the smallest stack this project's own `sdkconfig` configures (2,304 bytes) and
measures 1,024 bytes of use against that. The real budget is unconfirmed.

- **What would close it.** Measure actual high-water usage of the Wi-Fi driver
  task on hardware, or obtain the figure from the ESP-IDF configuration in use.

## 3. Captive portal and SoftAP runtime behaviour

Implemented and host-tested against a fake HAL; never exercised against a real
client. Unverified: association and DHCP lease issuance, WPA2 handshake with a
real station, whether OS connectivity probes (`/generate_204`,
`/hotspot-detect.html`) actually surface the portal page, DNS hijack behaviour
against real resolvers, form-POST decoding from real browsers, and the DNS
task's 3,072-byte stack under sustained query load.

## 4. ESP-NOW runtime behaviour

Implemented and host-tested against a fake HAL. Unverified: real peer pairing,
encrypted (LMK) delivery, the send-status callback's success/failure
accounting, reported RSSI values, and behaviour when the receive queue
saturates under real traffic.

## 5. Radio coexistence under real load

The arbiter serializes Wi-Fi, SoftAP/portal and ESP-NOW by construction, and
that serialization is host-tested. Whether the radio itself behaves correctly
across real mode transitions under sustained traffic is a hardware property.

## RCP Spinel transport (mtkcore-154-rcp)

None of the following has been exercised against real M1 hardware. Each is a
software-verified configuration awaiting a target.

| Item | Verified in software | Pending on hardware |
|---|---|---|
| Spinel over the STM32<->ESP32 SPI link | `CONFIG_OPENTHREAD_RCP_SPI=y`; `esp_openthread_spi_slave.c.obj` linked and `esp_openthread_host_rcp_spi_init` present in the ELF; UART host path absent | A real STM32 Spinel host completing frame exchange |
| GPIO 6 polarity inversion | Both drivers read; polarities confirmed opposite from source | Scope/logic-analyser confirmation, and an STM32 build that inverts its reading in RCP mode |
| SPI mode 1 (CPOL=0, CPHA=1) | Set to match the mode Core already uses on these wires | Confirmed against the STM32 master's actual configuration |
| Core SPI transport absent in RCP image | `spi_runtime_task` absent from the RCP ELF, present in the other two (`tools/build_variants.py`) | Confirmation that the host tolerates the canonical protocol being unavailable on that link |
| Clock rate | Not configurable in slave mode | The master's asserted rate against Spinel framing |

The GPIO 6 item is the one most likely to bite. The two images assert the
same wire with opposite polarity, and a host that does not invert its
interpretation will read "no data available" exactly when the RCP has a frame
ready. No firmware-side change can detect this.
