/* Blocking hardware abstraction for the ESP32-C6 IEEE 802.15.4 radio.
 *
 * Deliberately protocol-neutral: it moves PHY payloads and reports radio
 * metadata, and knows nothing about Thread, Zigbee or any other 802.15.4
 * consumer. Whatever runs above it -- a host-side Thread stack over RCP, a
 * host-side Zigbee stack, a sniffer, or a bespoke protocol -- uses the same
 * primitives, so adding a new 802.15.4 consumer does not require changing
 * this firmware.
 *
 * A target build supplies the ESP-IDF-backed implementation; host tests link
 * a deterministic fake, so the whole service lifecycle is testable without
 * hardware. */
#pragma once
#include <stdint.h>
#include "mtek_hal_common.h"
#include "mtek_core.h"
#include "mtek_router.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 2.4GHz O-QPSK channel page: IEEE 802.15.4 channels 11..26. */
#define MTK_154_CHANNEL_MIN 11
#define MTK_154_CHANNEL_MAX 26
/* aMaxPHYPacketSize. A frame is never longer than this on the air. */
#define MTK_154_MAX_PHY_LEN 127

/* Delivers one received frame. `data` is valid only for the duration of the
 * call, so an implementation must copy whatever it keeps. Called from a safe
 * (non-ISR, non-driver) context -- `service` below is the drain hook for an
 * implementation that queues instead of calling straight through, mirroring
 * the Wi-Fi HAL's promiscuous path. */
typedef void (*mtk_hal_154_rx_cb_t)(void *user, uint64_t timestamp_us, uint8_t channel,
                                     int8_t rssi, uint8_t lqi,
                                     const uint8_t *data, uint8_t len);

typedef struct mtk_ieee802154_hal {
    /* Enables the radio on `channel` and installs the receive callback.
     * `promiscuous` disables address filtering so every frame on the channel
     * is delivered, which is what a sniffer needs. Returns 0 only if the
     * radio is genuinely receiving. */
    int (*start)(uint8_t channel, uint8_t promiscuous, mtk_hal_154_rx_cb_t cb, void *user);
    /* Always safe to call, including when start never succeeded. */
    void (*stop)(void);
    /* Drains queued receives. */
    void (*service)(void);
    /* Retunes a live session. Returns 0 on success. */
    int (*set_channel)(uint8_t channel);
    /* Transmits one PHY payload (1..127 bytes). `cca` requests clear-channel
     * assessment before transmit. Returns 0 on success. */
    int (*transmit)(const uint8_t *data, uint8_t len, uint8_t cca);
    /* Measures peak energy on `channel` over `dwell_ms`. Returns 0 on
     * success and writes the peak RSSI. */
    int (*energy_scan)(uint8_t channel, uint16_t dwell_ms, int8_t *peak_rssi_out);
    /* Transmit counters observed by the driver's own completion path. */
    int (*stats)(uint32_t *transmitted_out, uint32_t *tx_failures_out);

    /* OpenThread Radio Co-Processor. Hands the radio to the RCP runtime so a
     * host can drive Thread over Spinel; Core runs no Thread application
     * stack. A seam rather than a direct call so the portable lifecycle is
     * host-testable and so the real implementation is genuinely linked --
     * a weak-symbol shim would let the linker keep the stub and silently
     * drop the runtime. Returns 0 on success; start unwinds fully on
     * failure and stop is always safe to call. */
    int (*rcp_start)(void);
    void (*rcp_stop)(void);
    int (*rcp_is_running)(void);
} mtk_ieee802154_hal_t;

void mtek_ieee802154_set_hal(const mtk_ieee802154_hal_t *hal);
mtk_register_result_t mtek_ieee802154_service_register(void);
void mtek_ieee802154_service_init(uint64_t (*now_ms_fn)(void));
void mtek_ieee802154_set_lock(void (*lock)(void), void (*unlock)(void));
void mtek_ieee802154_service_tick(void);
/* Tears down a session left running by a departed peer session. */
mtk_op_id_t mtek_ieee802154_cancel_active_for_peer_reset(void);

const mtk_ieee802154_hal_t *mtek_ieee802154_hal_esp32_get(void);
int mtek_ieee802154_hal_esp32_init(void);

#ifdef __cplusplus
}
#endif
