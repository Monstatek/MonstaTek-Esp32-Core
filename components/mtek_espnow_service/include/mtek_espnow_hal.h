/* Blocking hardware abstraction for ESP-NOW, mirroring mtek_wifi_hal.h's
 * contract: every call returns a definite result, so the portable service
 * logic needs no async machinery of its own. A target build supplies the
 * ESP-IDF-backed implementation; host tests link a deterministic fake. */
#pragma once
#include <stdint.h>
#include "mtek_hal_common.h"
#include "mtek_core.h"
#include "mtek_router.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Delivers one received ESP-NOW frame. `data` is valid only for the duration
 * of the call, so an implementation must copy whatever it keeps. Called from a
 * safe (non-driver) context -- espnow_service below is the drain hook for an
 * implementation that queues instead of calling straight through. */
typedef void (*mtk_hal_espnow_rx_cb_t)(void *user, mtk_hal_mac6_t src,
                                        int8_t rssi, const uint8_t *data, uint8_t len);

typedef struct mtk_espnow_hal {
    /* Initialises ESP-NOW on `channel` and installs the receive callback.
     * ESP-NOW shares the Wi-Fi radio, so the caller holds the ESPNOW arbiter
     * class across the whole session. Returns 0 only if ESP-NOW is genuinely
     * initialised. */
    int (*start)(uint8_t channel, mtk_hal_espnow_rx_cb_t cb, void *user);
    /* Always safe to call, including when start never succeeded: the teardown
     * path runs it unconditionally. Forgets every peer. */
    void (*stop)(void);
    /* Drains queued receives, mirroring the Wi-Fi HAL's promisc_service. */
    void (*service)(void);

    /* Adds one peer. `lmk_len` is 0 for an unencrypted peer or 16 for an
     * encrypted one. Returns 0 on success, 1 if the peer table is full, and a
     * negative value for any other failure. */
    int (*add_peer)(mtk_hal_mac6_t peer, uint8_t channel, uint8_t encrypt,
                     const uint8_t *lmk, uint8_t lmk_len);
    /* Sends one frame (<= 250 bytes). Returns 0 on success. */
    int (*send)(mtk_hal_mac6_t peer, const uint8_t *data, uint8_t len);
    /* Transmit counters observed by the send-status callback. */
    int (*stats)(uint32_t *sent_ok_out, uint32_t *send_fail_out);
} mtk_espnow_hal_t;

void mtek_espnow_set_hal(const mtk_espnow_hal_t *hal);
mtk_register_result_t mtek_espnow_service_register(void);
void mtek_espnow_service_init(uint64_t (*now_ms_fn)(void));
void mtek_espnow_set_lock(void (*lock)(void), void (*unlock)(void));
void mtek_espnow_service_tick(void);
/* Tears down an ESP-NOW session left running by a departed peer session. */
mtk_op_id_t mtek_espnow_cancel_active_for_peer_reset(void);

const mtk_espnow_hal_t *mtek_espnow_hal_esp32_get(void);
int mtek_espnow_hal_esp32_init(void);

#ifdef __cplusplus
}
#endif
