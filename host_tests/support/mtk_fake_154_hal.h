/* Deterministic IEEE 802.15.4 HAL fake. Receives are staged by the test and
 * delivered on the next service() drain, never from inside start(), matching
 * how the target HAL defers delivery out of the radio driver's context. */
#pragma once
#include "mtek_ieee802154_hal.h"
#include <string.h>

typedef struct {
    unsigned start_count, stop_count, service_count, set_channel_count;
    int start_rc, set_channel_rc, transmit_rc, energy_rc, stats_rc;
    uint8_t last_channel, last_promiscuous;
    mtk_hal_154_rx_cb_t cb; void *cb_user;

    unsigned transmit_count;
    uint8_t last_tx[MTK_154_MAX_PHY_LEN]; uint8_t last_tx_len; uint8_t last_tx_cca;
    uint32_t transmitted, tx_failures;

    unsigned energy_count;
    int8_t energy_peak;
    uint8_t energy_channels[32]; unsigned energy_channel_count;

    unsigned rcp_start_count, rcp_stop_count;
    int rcp_start_rc;
    unsigned rcp_running;

    struct { uint64_t ts; uint8_t ch; int8_t rssi; uint8_t lqi;
             uint8_t data[MTK_154_MAX_PHY_LEN]; uint8_t len; } pending[32];
    unsigned pending_count;
} mtk_fake_154_t;

static mtk_fake_154_t g_fake_154;

static int fake_154_start(uint8_t channel, uint8_t promiscuous, mtk_hal_154_rx_cb_t cb, void *user) {
    g_fake_154.start_count++;
    g_fake_154.last_channel = channel;
    g_fake_154.last_promiscuous = promiscuous;
    int rc = g_fake_154.start_rc;
    if (rc == 0) { g_fake_154.cb = cb; g_fake_154.cb_user = user; }
    return rc;
}
static void fake_154_stop(void) {
    g_fake_154.stop_count++;
    g_fake_154.cb = NULL; g_fake_154.cb_user = NULL;
    g_fake_154.pending_count = 0;
}
static void fake_154_service(void) {
    g_fake_154.service_count++;
    mtk_hal_154_rx_cb_t cb = g_fake_154.cb; void *user = g_fake_154.cb_user;
    unsigned n = g_fake_154.pending_count;
    g_fake_154.pending_count = 0;
    if (!cb) return;
    for (unsigned i = 0; i < n; i++) {
        cb(user, g_fake_154.pending[i].ts, g_fake_154.pending[i].ch,
           g_fake_154.pending[i].rssi, g_fake_154.pending[i].lqi,
           g_fake_154.pending[i].data, g_fake_154.pending[i].len);
    }
}
static int fake_154_set_channel(uint8_t channel) {
    g_fake_154.set_channel_count++;
    int rc = g_fake_154.set_channel_rc;
    if (rc == 0) g_fake_154.last_channel = channel;
    return rc;
}
static int fake_154_transmit(const uint8_t *data, uint8_t len, uint8_t cca) {
    g_fake_154.transmit_count++;
    g_fake_154.last_tx_len = len;
    g_fake_154.last_tx_cca = cca;
    memset(g_fake_154.last_tx, 0, sizeof(g_fake_154.last_tx));
    if (len) memcpy(g_fake_154.last_tx, data, len);
    if (g_fake_154.transmit_rc == 0) g_fake_154.transmitted++; else g_fake_154.tx_failures++;
    return g_fake_154.transmit_rc;
}
static int fake_154_energy_scan(uint8_t channel, uint16_t dwell_ms, int8_t *peak_rssi_out) {
    (void)dwell_ms;
    if (g_fake_154.energy_channel_count < 32) g_fake_154.energy_channels[g_fake_154.energy_channel_count++] = channel;
    g_fake_154.energy_count++;
    int rc = g_fake_154.energy_rc;
    if (rc == 0) *peak_rssi_out = g_fake_154.energy_peak;
    return rc;
}
static int fake_154_stats(uint32_t *tx_out, uint32_t *fail_out) {
    int rc = g_fake_154.stats_rc;
    if (rc == 0) { *tx_out = g_fake_154.transmitted; *fail_out = g_fake_154.tx_failures; }
    return rc;
}

static int fake_154_rcp_start(void) {
    g_fake_154.rcp_start_count++;
    int rc = g_fake_154.rcp_start_rc;
    if (rc == 0) g_fake_154.rcp_running = 1;
    return rc;
}
static void fake_154_rcp_stop(void) { g_fake_154.rcp_stop_count++; g_fake_154.rcp_running = 0; }
static int fake_154_rcp_is_running(void) { return g_fake_154.rcp_running ? 1 : 0; }

static const mtk_ieee802154_hal_t g_fake_154_hal = {
    fake_154_start, fake_154_stop, fake_154_service, fake_154_set_channel,
    fake_154_transmit, fake_154_energy_scan, fake_154_stats,
    fake_154_rcp_start, fake_154_rcp_stop, fake_154_rcp_is_running,
};

static inline void mtk_fake_154_reset(void) { memset(&g_fake_154, 0, sizeof(g_fake_154)); }

/* Stages one inbound frame for the next service() drain. */
static inline void mtk_fake_154_deliver(uint64_t ts, uint8_t ch, int8_t rssi, uint8_t lqi,
                                         const uint8_t *data, uint8_t len) {
    if (g_fake_154.pending_count >= 32) return;
    unsigned i = g_fake_154.pending_count++;
    memset(&g_fake_154.pending[i], 0, sizeof(g_fake_154.pending[i]));
    g_fake_154.pending[i].ts = ts; g_fake_154.pending[i].ch = ch;
    g_fake_154.pending[i].rssi = rssi; g_fake_154.pending[i].lqi = lqi;
    g_fake_154.pending[i].len = len;
    if (len) memcpy(g_fake_154.pending[i].data, data, len);
}
