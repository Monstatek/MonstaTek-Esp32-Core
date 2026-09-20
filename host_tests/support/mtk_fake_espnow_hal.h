/* Deterministic ESP-NOW HAL fake. Mirrors the real HAL's contract: receives
 * are staged by the test and delivered on the next service() drain, never from
 * inside start(), which is how the target HAL defers delivery out of the Wi-Fi
 * driver task. */
#pragma once
#include "mtek_espnow_hal.h"
#include <string.h>

typedef struct {
    unsigned start_count, stop_count, service_count;
    int start_rc;
    uint8_t last_channel;
    mtk_hal_espnow_rx_cb_t cb; void *cb_user;

    unsigned add_peer_count;
    int add_peer_rc;
    mtk_hal_mac6_t last_peer;
    uint8_t last_peer_channel, last_peer_encrypt, last_peer_lmk_len;

    unsigned send_count;
    int send_rc;
    uint8_t last_send[250]; uint8_t last_send_len;

    uint32_t sent_ok, send_fail;
    int stats_rc;

    struct { mtk_hal_mac6_t src; int8_t rssi; uint8_t len; uint8_t data[250]; } pending[32];
    unsigned pending_count;
} mtk_fake_espnow_t;

static mtk_fake_espnow_t g_fake_espnow;

static int fake_espnow_start(uint8_t channel, mtk_hal_espnow_rx_cb_t cb, void *user) {
    g_fake_espnow.start_count++;
    g_fake_espnow.last_channel = channel;
    int rc = g_fake_espnow.start_rc;
    if (rc == 0) { g_fake_espnow.cb = cb; g_fake_espnow.cb_user = user; }
    return rc;
}
static void fake_espnow_stop(void) {
    g_fake_espnow.stop_count++;
    g_fake_espnow.cb = NULL; g_fake_espnow.cb_user = NULL;
    g_fake_espnow.pending_count = 0;
}
static void fake_espnow_service(void) {
    g_fake_espnow.service_count++;
    mtk_hal_espnow_rx_cb_t cb = g_fake_espnow.cb;
    void *user = g_fake_espnow.cb_user;
    unsigned n = g_fake_espnow.pending_count;
    g_fake_espnow.pending_count = 0;
    if (!cb) return;
    for (unsigned i = 0; i < n; i++) {
        cb(user, g_fake_espnow.pending[i].src, g_fake_espnow.pending[i].rssi,
           g_fake_espnow.pending[i].data, g_fake_espnow.pending[i].len);
    }
}
static int fake_espnow_add_peer(mtk_hal_mac6_t peer, uint8_t channel, uint8_t encrypt,
                                 const uint8_t *lmk, uint8_t lmk_len) {
    (void)lmk;
    g_fake_espnow.add_peer_count++;
    g_fake_espnow.last_peer = peer;
    g_fake_espnow.last_peer_channel = channel;
    g_fake_espnow.last_peer_encrypt = encrypt;
    g_fake_espnow.last_peer_lmk_len = lmk_len;
    return g_fake_espnow.add_peer_rc;
}
static int fake_espnow_send(mtk_hal_mac6_t peer, const uint8_t *data, uint8_t len) {
    (void)peer;
    g_fake_espnow.send_count++;
    g_fake_espnow.last_send_len = len;
    memset(g_fake_espnow.last_send, 0, sizeof(g_fake_espnow.last_send));
    if (len) memcpy(g_fake_espnow.last_send, data, len);
    if (g_fake_espnow.send_rc == 0) g_fake_espnow.sent_ok++; else g_fake_espnow.send_fail++;
    return g_fake_espnow.send_rc;
}
static int fake_espnow_stats(uint32_t *sent_ok_out, uint32_t *send_fail_out) {
    int rc = g_fake_espnow.stats_rc;
    if (rc == 0) { *sent_ok_out = g_fake_espnow.sent_ok; *send_fail_out = g_fake_espnow.send_fail; }
    return rc;
}

static const mtk_espnow_hal_t g_fake_espnow_hal = {
    fake_espnow_start, fake_espnow_stop, fake_espnow_service,
    fake_espnow_add_peer, fake_espnow_send, fake_espnow_stats,
};

static inline void mtk_fake_espnow_reset(void) {
    memset(&g_fake_espnow, 0, sizeof(g_fake_espnow));
}

/* Stages one inbound frame for the next service() drain. */
static inline void mtk_fake_espnow_deliver(const uint8_t mac[6], int8_t rssi,
                                            const uint8_t *data, uint8_t len) {
    if (g_fake_espnow.pending_count >= 32) return;
    unsigned i = g_fake_espnow.pending_count++;
    memset(&g_fake_espnow.pending[i], 0, sizeof(g_fake_espnow.pending[i]));
    memcpy(g_fake_espnow.pending[i].src.b, mac, 6);
    g_fake_espnow.pending[i].rssi = rssi;
    g_fake_espnow.pending[i].len = len;
    if (len) memcpy(g_fake_espnow.pending[i].data, data, len);
}
