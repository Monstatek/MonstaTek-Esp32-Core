/* Header-only deterministic fake mtk_ble_hal_t for host tests. */
#pragma once
#include "mtek_ble_hal.h"
#include <string.h>
#include <unistd.h>

typedef struct {
    mtk_hal_ble_adv_t scan_results[8]; unsigned scan_count;
    /* Mirrors mtk_fake_wifi_hal.h's own connect_delay_ms/promisc_start_delay_ms
     * precedent -- an optional REAL delay (ms) before scan/gatt_connect check
     * their own injected result, giving a test's own concurrent thread a genuine
     * window to invalidate this exact operation's token (a simulated native-SPI
     * peer reboot) before the blocking HAL call returns. 0 (default) = no delay,
     * every other test's existing timing unaffected. */
    unsigned scan_delay_ms;
    unsigned gatt_connect_delay_ms;
    int adv_start_rc;
    int signal_rc; int8_t signal_rssi; uint8_t signal_is_random;
    int gatt_connect_rc; uint16_t gatt_vendor_handle;
    mtk_hal_gatt_service_t gatt_services[4]; unsigned gatt_service_count;
    /* Filtered by requested [start_handle, end_handle] at call time (like a real
     * HAL would), so a test can populate the full connection's chars/descs once
     * and different per-service/per-characteristic discovery calls each see only
     * the subset whose handle actually falls in their own real range. */
    mtk_hal_gatt_char_t gatt_chars[8]; unsigned gatt_char_count;
    mtk_hal_gatt_desc_t gatt_descs[8]; unsigned gatt_desc_count;
    uint8_t gatt_read_data[64]; uint16_t gatt_read_len; int gatt_read_rc;
    int gatt_write_rc;
    int gatt_subscribe_rc, gatt_unsubscribe_rc;
    /* Records the exact (attr_handle, end_handle) this HAL's subscribe/
     * unsubscribe calls actually received, so a test can prove the real
     * discovered service end_handle was threaded through -- not a
     * guessed window or attr_handle+1. */
    uint16_t last_subscribe_attr_handle, last_subscribe_end_handle;
    uint16_t last_unsubscribe_attr_handle, last_unsubscribe_end_handle;
    uint16_t notify_handle; uint8_t notify_data[32]; uint16_t notify_len; int notify_pending;
    int remote_disconnect_pending; /* test sets this to simulate a real BLE_GAP_EVENT_DISCONNECT */
    uint8_t remote_disconnect_reason; /* Test sets this to simulate a real
                                       * HCI-level reason code */
    uint32_t notify_dropped_count; /* Test sets this to simulate real HAL-side
                                    * overflow */
    unsigned gatt_disconnect_call_count; /* Proves a peer-reset invalidation
                                          * genuinely tore down a live/mid-connect
                                          * GATT session */
    /* Mirrors scan_delay_ms/gatt_ connect_delay_ms's own established pattern --
     * an optional REAL delay (ms) before gatt_discover/gatt_read check their own
     * injected result, giving a test's own concurrent thread a genuine window to
     * disconnect-then-reconnect (deliberately reusing the exact same
     * vendor_handle for the new connection) while the call is blocked, so the
     * canonical layer's own post-call identity re-validation (mtek_ble_logic.c's
     * gatt_revalidate_identity) can be proven to actually refuse
     * merging/publishing a stale result into the new session rather than merely
     * trusting connection_token alone. 0 (default) = no delay, every other
     * test's existing timing unaffected. */
    unsigned gatt_op_delay_ms;
    /* Lets a test inject a genuine HAL-layer discovery failure (mirroring a real
     * xSemaphoreCreateBinary failure in mtek_ble_hal_esp32.c's
     * esp32_gatt_discover/_chars/_descs) so the canonical service layer's own
     * handling of a negative return can be proven -- an honest
     * MTK_STATUS_IO_ERROR, never an apparently successful empty discovery. 0
     * (default) = no injected failure. */
    int gatt_discover_force_fail;
} mtk_fake_ble_state_t;

static mtk_fake_ble_state_t g_fake_ble;
static inline void mtk_fake_ble_reset(void) { memset(&g_fake_ble, 0, sizeof(g_fake_ble)); }

/* Mirrors mtk_fake_wifi_hal.h's own established lock pattern exactly -- a real
 * TSan run against the new concurrent BLE_SCAN/GATT_CONNECT tests caught a
 * genuine data race between a background worker's writes into g_fake_ble (e.g.
 * gatt_disconnect_call_count) and the main thread's own polling reads of the
 * same fields, with no lock of any kind previously guarding this struct.
 * Optional, safe no-op default for every other (single-threaded) host test that
 * never registers one. */
typedef void (*mtk_fake_ble_lock_fn)(void);
static mtk_fake_ble_lock_fn s_fake_ble_lock_fn, s_fake_ble_unlock_fn;
static inline void mtk_fake_ble_set_lock(mtk_fake_ble_lock_fn lock, mtk_fake_ble_lock_fn unlock) {
    s_fake_ble_lock_fn = lock; s_fake_ble_unlock_fn = unlock;
}
static inline void fake_ble_lock(void) { if (s_fake_ble_lock_fn) s_fake_ble_lock_fn(); }
static inline void fake_ble_unlock(void) { if (s_fake_ble_unlock_fn) s_fake_ble_unlock_fn(); }

static int fake_ble_scan(uint8_t mode, uint16_t duration_ms, const char *name_filter,
                          mtk_hal_ble_adv_t *out, unsigned max_out) {
    (void)mode; (void)duration_ms; (void)name_filter;
    if (g_fake_ble.scan_delay_ms) usleep(g_fake_ble.scan_delay_ms * 1000);
    unsigned n = g_fake_ble.scan_count < max_out ? g_fake_ble.scan_count : max_out;
    memcpy(out, g_fake_ble.scan_results, n * sizeof(*out));
    return (int)n;
}
static int fake_ble_adv_start(const uint8_t *name, uint8_t name_len) { (void)name; (void)name_len; return g_fake_ble.adv_start_rc; }
static void fake_ble_adv_stop(void) {}
static int fake_ble_signal_sample(mtk_hal_mac6_t addr, uint8_t addr_type, int8_t *rssi_out, uint8_t *is_random_out) {
    (void)addr; (void)addr_type;
    *rssi_out = g_fake_ble.signal_rssi; *is_random_out = g_fake_ble.signal_is_random;
    return g_fake_ble.signal_rc;
}
static int fake_ble_gatt_connect(mtk_hal_mac6_t addr, uint8_t addr_type, uint32_t timeout_ms, uint16_t *vendor_handle_out) {
    (void)addr; (void)addr_type; (void)timeout_ms;
    if (g_fake_ble.gatt_connect_delay_ms) usleep(g_fake_ble.gatt_connect_delay_ms * 1000);
    *vendor_handle_out = g_fake_ble.gatt_vendor_handle;
    return g_fake_ble.gatt_connect_rc;
}
static void fake_ble_gatt_disconnect(uint16_t vendor_handle) {
    (void)vendor_handle;
    fake_ble_lock(); g_fake_ble.gatt_disconnect_call_count++; fake_ble_unlock();
}
static int fake_ble_gatt_discover(uint16_t vendor_handle, mtk_hal_gatt_service_t *out, unsigned max_out) {
    (void)vendor_handle;
    /* gatt_op_ delay_ms exists precisely so a test can reconfigure g_fake_ble.
     * gatt_service_count/gatt_services[] on a DIFFERENT thread while this call
     * is genuinely blocked -- reading them (and the delay itself) under
     * fake_ble_lock/unlock (a real mutex when a test registers one, a safe no-op
     * otherwise, matching every other test's existing single-threaded usage)
     * avoids racing that concurrent reconfigure. */
    fake_ble_lock();
    unsigned delay = g_fake_ble.gatt_op_delay_ms;
    fake_ble_unlock();
    if (delay) usleep(delay * 1000);
    fake_ble_lock();
    int force_fail = g_fake_ble.gatt_discover_force_fail;
    unsigned n = g_fake_ble.gatt_service_count < max_out ? g_fake_ble.gatt_service_count : max_out;
    if (!force_fail) memcpy(out, g_fake_ble.gatt_services, n * sizeof(*out));
    fake_ble_unlock();
    if (force_fail) return -1;
    return (int)n;
}
static int fake_ble_gatt_discover_chars(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                         mtk_hal_gatt_char_t *out, unsigned max_out) {
    (void)vendor_handle;
    unsigned n = 0;
    for (unsigned i = 0; i < g_fake_ble.gatt_char_count && n < max_out; i++) {
        if (g_fake_ble.gatt_chars[i].def_handle >= start_handle && g_fake_ble.gatt_chars[i].def_handle <= end_handle) {
            out[n++] = g_fake_ble.gatt_chars[i];
        }
    }
    return (int)n;
}
static int fake_ble_gatt_discover_descs(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                         mtk_hal_gatt_desc_t *out, unsigned max_out) {
    (void)vendor_handle;
    unsigned n = 0;
    for (unsigned i = 0; i < g_fake_ble.gatt_desc_count && n < max_out; i++) {
        if (g_fake_ble.gatt_descs[i].handle >= start_handle && g_fake_ble.gatt_descs[i].handle <= end_handle) {
            out[n++] = g_fake_ble.gatt_descs[i];
        }
    }
    return (int)n;
}
static int fake_ble_gatt_read(uint16_t vendor_handle, uint16_t attr_handle, uint8_t *out, uint16_t max_len, uint16_t *len_out) {
    (void)vendor_handle; (void)attr_handle;
    fake_ble_lock();
    unsigned delay = g_fake_ble.gatt_op_delay_ms;
    fake_ble_unlock();
    if (delay) usleep(delay * 1000);
    fake_ble_lock();
    uint16_t n = g_fake_ble.gatt_read_len < max_len ? g_fake_ble.gatt_read_len : max_len;
    memcpy(out, g_fake_ble.gatt_read_data, n);
    int rc = g_fake_ble.gatt_read_rc;
    fake_ble_unlock();
    *len_out = n;
    return rc;
}
static int fake_ble_gatt_write(uint16_t vendor_handle, uint16_t attr_handle, const uint8_t *data, uint16_t len, uint8_t with_response) {
    (void)vendor_handle; (void)attr_handle; (void)data; (void)len; (void)with_response;
    return g_fake_ble.gatt_write_rc;
}
static int fake_ble_gatt_subscribe(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle, uint8_t mode) {
    (void)vendor_handle; (void)mode;
    fake_ble_lock();
    unsigned delay = g_fake_ble.gatt_op_delay_ms;
    fake_ble_unlock();
    if (delay) usleep(delay * 1000);
    fake_ble_lock();
    g_fake_ble.last_subscribe_attr_handle = attr_handle;
    g_fake_ble.last_subscribe_end_handle = end_handle;
    int rc = g_fake_ble.gatt_subscribe_rc;
    fake_ble_unlock();
    return rc;
}
/* Mirrors fake_ble_gatt_ subscribe's own gatt_op_delay_ms/locking pattern
 * exactly -- previously this function had no delay hook and no lock of any kind,
 * so no test could genuinely race a concurrent disconnect/reconnect against an
 * in-flight UNSUBSCRIBE the way the existing subscribe/discover/read tests
 * already do. */
static int fake_ble_gatt_unsubscribe(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle) {
    (void)vendor_handle;
    fake_ble_lock();
    unsigned delay = g_fake_ble.gatt_op_delay_ms;
    fake_ble_unlock();
    if (delay) usleep(delay * 1000);
    fake_ble_lock();
    g_fake_ble.last_unsubscribe_attr_handle = attr_handle;
    g_fake_ble.last_unsubscribe_end_handle = end_handle;
    int rc = g_fake_ble.gatt_unsubscribe_rc;
    fake_ble_unlock();
    return rc;
}
static int fake_ble_gatt_poll_notify(uint16_t vendor_handle, uint16_t *attr_handle_out, uint8_t *data_out, uint16_t *len_out) {
    (void)vendor_handle;
    if (!g_fake_ble.notify_pending) return 0;
    *attr_handle_out = g_fake_ble.notify_handle;
    memcpy(data_out, g_fake_ble.notify_data, g_fake_ble.notify_len);
    *len_out = g_fake_ble.notify_len;
    g_fake_ble.notify_pending = 0;
    return 1;
}

static int fake_ble_gatt_poll_disconnected(uint16_t vendor_handle, uint8_t *reason_out) {
    (void)vendor_handle;
    if (!g_fake_ble.remote_disconnect_pending) return 0;
    g_fake_ble.remote_disconnect_pending = 0;
    if (reason_out) *reason_out = g_fake_ble.remote_disconnect_reason;
    return 1;
}

static uint32_t fake_ble_gatt_notify_dropped_count(void) { return g_fake_ble.notify_dropped_count; }

static const mtk_ble_hal_t g_fake_ble_hal = {
    fake_ble_scan, fake_ble_adv_start, fake_ble_adv_stop, fake_ble_signal_sample,
    fake_ble_gatt_connect, fake_ble_gatt_disconnect, fake_ble_gatt_discover,
    fake_ble_gatt_discover_chars, fake_ble_gatt_discover_descs, fake_ble_gatt_read,
    fake_ble_gatt_write, fake_ble_gatt_subscribe, fake_ble_gatt_unsubscribe, fake_ble_gatt_poll_notify,
    fake_ble_gatt_poll_disconnected, fake_ble_gatt_notify_dropped_count,
};
