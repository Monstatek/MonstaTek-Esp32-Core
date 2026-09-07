/* Header-only deterministic fake mtk_wifi_hal_t for host tests. Every call
 * is synchronous: promisc_start immediately replays whatever canned frames
 * the test configured, matching this session's synchronous-HAL design
 * (mtek_wifi_hal.h). */
#pragma once
#include "mtek_wifi_hal.h"
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#define MTK_FAKE_MAX_FRAMES 8

/* RC11 round 10 verification (TSan): named so a caller can snapshot the
 * staged frames into a local batch under the lock and then invoke the
 * frame callback with the lock released -- see mtk_fake_wifi_deliver_
 * frames/fake_wifi_promisc_start below. Layout and field names are
 * unchanged from the anonymous struct this replaces. */
typedef struct { uint8_t data[1000]; uint16_t len; int8_t rssi; uint8_t channel; } mtk_fake_wifi_frame_t;

typedef struct {
    mtk_hal_ap_record_t ap_results[8]; unsigned ap_count;
    mtk_hal_station_record_t sta_results[8]; unsigned sta_count;
    mtk_hal_connect_result_t connect_result; int connect_rc;
    /* P0 correction (Codex read-only re-audit, "genuine peer-session
     * ownership"): mirrors promisc_start_delay_ms's own established
     * precedent -- an optional REAL delay (ms) before connect() checks its
     * own injected result, giving a test's own concurrent thread a
     * genuine window to invalidate this exact operation's token (a
     * simulated native-SPI peer reboot) before the blocking HAL call
     * returns, proving handle_sta_connect's own fencing (gate every
     * externally-visible effect on winning the transition, never release
     * a newer operation's arbiter lease) resolves correctly regardless of
     * which side reaches its own finish line first. 0 (default) = no
     * delay, every other connect() test's existing timing unaffected. */
    unsigned connect_delay_ms;
    unsigned disconnect_call_count; /* proves a fenced-out late "connected" completion tears itself back down */
    mtk_hal_mac6_t last_sta_scan_bssid; uint8_t last_sta_scan_channel;
    unsigned deauth_sent_count;
    mtk_hal_mac6_t last_deauth_ap, last_deauth_station;
    /* 1000 bytes: matches MonstaShark's own max snap_len (mtek_capture_
     * logic.c's handle_capture_start rejects a larger one at
     * CAPTURE_START), not just a typical 802.11 management/control frame
     * size, so a test can stage a full-size capture frame without
     * silently overflowing into the neighboring array elements of this
     * struct (a real, previously-undetected latent bug: a smaller cap
     * here let test_monstashark_capture.c's own 1000-byte canned frame
     * silently corrupt sibling fields -- harmless there only because that
     * test never asserted on the corrupted frame's actual byte content,
     * see test_spi_native_events_streams.c which does and caught this). */
    mtk_fake_wifi_frame_t frames[MTK_FAKE_MAX_FRAMES];
    unsigned frame_count;
    uint8_t mode;
    mtk_hal_mac6_t mac;
    int raw_tx_rc; unsigned raw_tx_count;
    unsigned restore_count;
    unsigned promisc_start_count, promisc_stop_count;
    int promisc_start_rc; /* RC8 independent audit P0-9: test-settable failure injection */
    unsigned pace_delay_call_count; /* counts calls without actually sleeping, so host tests stay fast */
    unsigned sta_was_connected;     /* mirrors mtek_wifi_hal_esp32.c's own s_sta_was_connected for restore-mode tests */
    unsigned reconnect_attempted_count;
    uint8_t current_channel;
    uint8_t last_requested_channel;
    unsigned set_channel_call_count;
    int set_channel_rc; /* RC8 independent audit P0-9: test-settable failure injection */
    int defer_frames; /* if set, promisc_start only records the callback;
                        * mtk_fake_wifi_deliver_frames() replays it later,
                        * simulating frames that arrive after START returns */
    mtk_hal_frame_cb_t pending_cb; void *pending_cb_user;
    /* RC11 round 10 verification (TSan): bumped under the fake HAL's own
     * lock immediately AFTER pending_cb/pending_cb_user are published, so
     * a test can wait for "the deferred registration is actually visible"
     * rather than for promisc_start_count, which fake_wifi_promisc_start
     * bumps EARLY -- before its own promisc_start_delay_ms sleep and long
     * before it publishes the registration at all. A test that staged a
     * deliberately-stale pending_cb_user after only the latter was racing
     * the very worker whose registration it meant to supersede. */
    unsigned promisc_registered_count;
    /* RC8 independent audit P0-9: mirrors mtek_wifi_hal_esp32.c's own
     * capture_prior_state_once/esp32_restore_sta_mode contract -- a test
     * sets `mode`/`current_channel` to whatever the "prior" state should
     * be BEFORE driving a List B operation, then asserts these two fields
     * hold those exact values again after the operation's own
     * restore_sta_mode call (not hard-coded back to some fixed mode/
     * channel), proving a real, non-STA-only, non-zero-channel prior
     * state actually round-trips. */
    uint8_t prior_state_valid;
    uint8_t prior_mode;
    uint8_t prior_channel;
    uint8_t prior_was_connected;
    /* RC9 independent correction order P0 "station-target scan bypasses
     * the transactional radio lifecycle": mirrors mtek_wifi_hal_esp32.c's
     * own esp32_sta_scan/esp32_sta_scan_cancel contract. sta_scan_rc
     * injects a transactional-entry failure (a real negative return, not
     * "0 stations found"). sta_scan_poll_count_per_call/
     * sta_scan_polls_done let a test prove a real STOP-during-scan
     * genuinely shortens the call (fewer polls consumed than the full
     * configured duration would need) instead of merely being ignored
     * until the full wait elapses. */
    int sta_scan_rc;
    volatile int sta_scan_cancel_requested;
    unsigned sta_scan_cancel_count;
    unsigned sta_scan_poll_count_per_call; /* how many 5ms polls one full-duration call performs */
    unsigned sta_scan_polls_done;          /* set by the call itself: how many polls it actually did before returning */
    unsigned sta_scan_call_count;
    /* RC10 independent correction order P0 "transactional entry/teardown
     * errors are still discarded": mirrors mtek_wifi_hal_esp32.c's own
     * esp32_sta_scan teardown -- when nonzero, the fake's own teardown
     * step fails just like a real esp_wifi_set_promiscuous(false)/
     * esp_wifi_set_promiscuous_rx_cb(NULL) failure would, and the call
     * must report failure (-1) even though real results may have been
     * found. */
    int sta_scan_teardown_promisc_off_rc;
    int sta_scan_teardown_cb_clear_rc;
    /* RC10 independent correction order P0 "restore is not actually
     * failure-atomic": mirrors mtek_wifi_hal_esp32.c's own
     * esp32_restore_sta_mode -- when set, the fake's own restore fails at
     * the named step (and, per the real HAL's own short-circuit rule,
     * skips whatever depends on it) instead of always succeeding. */
    int restore_promisc_off_rc;
    int restore_stop_rc;
    int restore_mode_rc;
    int restore_start_rc;
    int restore_channel_rc;
    int restore_reconnect_rc;
    unsigned quiescence_wait_call_count;
    /* RC11 independent correction order P0 "make WIFI_STOP_ALL retry
     * verified recovery before releasing ownership": when nonzero, the
     * fake's own restore_sta_mode forces a transient failure (never
     * touching the per-step *_rc fields above, and never clearing the
     * preserved prior-state snapshot) and decrements this counter --
     * letting a test prove a bounded-retry caller genuinely recovers
     * after N transient failures, not just that it gives up on the
     * first one or retries forever. 0 (default): no transient failure
     * injected here at all (the per-step *_rc fields above, if any,
     * still apply normally). */
    unsigned restore_fail_countdown;
    /* RC11 independent correction order P0 "AP scan has the same async
     * STOP race and false-success behavior": mirrors the sta_scan fields
     * above exactly, for the same reason (a real pthread AP-scan START/
     * STOP race test). */
    int ap_scan_rc;
    volatile int ap_scan_cancel_requested;
    unsigned ap_scan_cancel_count;
    unsigned ap_scan_poll_count_per_call;
    unsigned ap_scan_polls_done;
    unsigned ap_scan_call_count;
    /* P0 correction (Codex read-only re-audit, "final focused
     * concurrency-correction round", issue 4): when set, ap_scan_cancel()
     * still records that cancellation was genuinely signalled
     * (ap_scan_cancel_count still increments) but deliberately does NOT
     * set ap_scan_cancel_requested -- simulating a real-world case where
     * the HAL's own cancel signal does not take effect promptly (e.g. a
     * transient hardware-busy state), so a test can force the peer-reset
     * canceller's own bounded quiescence wait to genuinely time out
     * rather than observe the worker finish early, exactly like every
     * other test in this suite that DOES observe a prompt cancellation. */
    unsigned ap_scan_ignore_cancel;
    unsigned sta_scan_ignore_cancel;
    /* RC9 independent correction order P0 "handshake capture ignores
     * monitor-entry failure": an optional real delay (ms) before
     * promisc_start checks its own injected failure/success, giving a
     * test's own concurrent thread a genuine window to dispatch a STOP
     * against the same operation token before promisc_start returns --
     * proving handle_handshake_start's monitor-entry failure path and a
     * genuine concurrent STOP resolve to exactly one cleanup, whichever
     * wins mtk_op_transition. 0 (default) = no delay, every other
     * promisc_start test's existing timing is unaffected. */
    unsigned promisc_start_delay_ms;
    /* RC9 independent correction order P0 "the prior-state snapshot is
     * not complete or failure-safe": mirrors mtek_wifi_hal_esp32.c's own
     * capture_prior_state_once -- when set, a fresh (not-yet-captured)
     * prior-state snapshot fails entirely (a required mode/channel/
     * promiscuous read failing on the real HAL), and every real "disturb
     * the radio" entry point must treat that as its own transactional-
     * entry failure, never proceeding over unknown prior state. */
    int prior_state_capture_rc;
} mtk_fake_wifi_state_t;

static mtk_fake_wifi_state_t g_fake_wifi;
static inline void mtk_fake_wifi_reset(void) { memset(&g_fake_wifi, 0, sizeof(g_fake_wifi)); }

/* RC8 independent audit "Run a supported ThreadSanitizer build" (this
 * round's own verification pass): a real TSan run against
 * test_deauth_continuous.c's genuine pthread worker caught a real data
 * race on g_fake_wifi's own prior-state/channel/mode fields between the
 * async worker thread's in-flight fake_wifi_send_deauth call and a
 * concurrent DEAUTH_STOP's fake_wifi_restore_sta_mode call on another
 * thread -- the exact same real hazard this round's P0-9 fix newly
 * exposed in production too (mtek_wifi_hal_esp32.c's own s_prior_state
 * now needs the analogous fix, see that file). Optional lock hooks,
 * mirroring mtek_wifi_service_set_lock/mtk_core_set_lock's own
 * established pattern -- a no-op default is safe for every other host
 * test (single-threaded, never registers one); only a test with a real
 * concurrent dispatch path (test_deauth_continuous.c) needs to. */
typedef void (*mtk_fake_wifi_lock_fn)(void);
static mtk_fake_wifi_lock_fn s_fake_wifi_lock_fn, s_fake_wifi_unlock_fn;
static inline void mtk_fake_wifi_set_lock(mtk_fake_wifi_lock_fn lock, mtk_fake_wifi_lock_fn unlock) {
    s_fake_wifi_lock_fn = lock; s_fake_wifi_unlock_fn = unlock;
}
static inline void fake_wifi_lock(void) { if (s_fake_wifi_lock_fn) s_fake_wifi_lock_fn(); }
static inline void fake_wifi_unlock(void) { if (s_fake_wifi_unlock_fn) s_fake_wifi_unlock_fn(); }
/* A frame batch copied out of g_fake_wifi under the lock, so the callback
 * itself can run with the lock RELEASED: the callback re-enters production
 * code that can call straight back into this same fake HAL (set_channel,
 * promisc_stop, restore_sta_mode, ...), and these lock hooks are plain
 * non-recursive mutexes -- holding one across the callback would deadlock. */
typedef struct { mtk_fake_wifi_frame_t frames[MTK_FAKE_MAX_FRAMES]; unsigned count; } mtk_fake_wifi_frame_batch_t;

/* Caller must hold the fake HAL lock. */
static inline void fake_wifi_snapshot_frames_locked(mtk_fake_wifi_frame_batch_t *out) {
    unsigned n = g_fake_wifi.frame_count;
    if (n > MTK_FAKE_MAX_FRAMES) n = MTK_FAKE_MAX_FRAMES;
    out->count = n;
    for (unsigned i = 0; i < n; i++) out->frames[i] = g_fake_wifi.frames[i];
}

/* RC11 round 10 verification (TSan): every g_fake_wifi access here was
 * previously bare. A real full-suite ThreadSanitizer run caught both
 * halves of the resulting race against a concurrently-dispatched worker
 * thread's own fake_wifi_promisc_start: this function's `pending_cb` read
 * vs. that function's `pending_cb = cb` write, and a test's own staged
 * `pending_cb_user` write vs. its `pending_cb_user = user` write. The
 * registration and the frames are now snapshotted under the lock; the
 * callback runs outside it, for the deadlock reason above. */
static inline void mtk_fake_wifi_deliver_frames(void) {
    fake_wifi_lock();
    mtk_hal_frame_cb_t cb = g_fake_wifi.pending_cb;
    void *user = g_fake_wifi.pending_cb_user;
    mtk_fake_wifi_frame_batch_t batch;
    fake_wifi_snapshot_frames_locked(&batch);
    fake_wifi_unlock();
    if (!cb) return;
    for (unsigned i = 0; i < batch.count; i++) {
        cb(user, batch.frames[i].data, batch.frames[i].len, batch.frames[i].rssi, batch.frames[i].channel);
    }
}

static int fake_wifi_connect(const uint8_t *ssid, uint8_t ssid_len, mtk_hal_mac6_t bssid_hint, uint8_t channel_hint,
                              uint8_t auth_mode, const uint8_t *psk, uint8_t psk_len, uint8_t ip_mode,
                              mtk_hal_ipv4_t static_ip, mtk_hal_ipv4_t static_netmask, mtk_hal_ipv4_t static_gateway,
                              uint32_t timeout_ms, mtk_hal_connect_result_t *out) {
    (void)ssid; (void)ssid_len; (void)bssid_hint; (void)channel_hint; (void)auth_mode; (void)psk; (void)psk_len;
    (void)ip_mode; (void)static_ip; (void)static_netmask; (void)static_gateway; (void)timeout_ms;
    fake_wifi_lock();
    unsigned delay_ms = g_fake_wifi.connect_delay_ms;
    fake_wifi_unlock();
    if (delay_ms) usleep(delay_ms * 1000);
    *out = g_fake_wifi.connect_result;
    fake_wifi_lock();
    if (g_fake_wifi.connect_rc == 0 && g_fake_wifi.connect_result.connected) g_fake_wifi.sta_was_connected = 1;
    fake_wifi_unlock();
    return g_fake_wifi.connect_rc;
}
static void fake_wifi_disconnect(void) { fake_wifi_lock(); g_fake_wifi.sta_was_connected = 0; g_fake_wifi.disconnect_call_count++; fake_wifi_unlock(); }
static void fake_wifi_get_status(mtk_hal_sta_status_t *out) { memset(out, 0, sizeof(*out)); }
static bool fake_wifi_capture_prior_state_once(void) {
    /* Locked in full (not just the individual field writes): a second
     * caller must never observe `prior_state_valid` set but the other
     * three fields still mid-write -- mirrors mtek_wifi_hal_esp32.c's own
     * capture_prior_state_once, now under its own real FreeRTOS mutex.
     * Returns false (a real transactional-entry failure) if
     * prior_state_capture_rc is set and nothing was captured yet --
     * never sets prior_state_valid around unknown state. */
    fake_wifi_lock();
    if (g_fake_wifi.prior_state_valid) { fake_wifi_unlock(); return true; }
    if (g_fake_wifi.prior_state_capture_rc != 0) { fake_wifi_unlock(); return false; }
    g_fake_wifi.prior_mode = g_fake_wifi.mode;
    g_fake_wifi.prior_channel = g_fake_wifi.current_channel;
    g_fake_wifi.prior_was_connected = (uint8_t)g_fake_wifi.sta_was_connected;
    g_fake_wifi.prior_state_valid = 1;
    fake_wifi_unlock();
    return true;
}
static void fake_wifi_ap_scan_cancel(void) {
    fake_wifi_lock();
    if (!g_fake_wifi.ap_scan_ignore_cancel) g_fake_wifi.ap_scan_cancel_requested = 1;
    g_fake_wifi.ap_scan_cancel_count++;
    fake_wifi_unlock();
}
static int fake_wifi_ap_scan(uint8_t band, uint8_t fixed_channel, uint32_t duration_ms,
                             mtk_hal_ap_record_t *out, unsigned max_out) {
    (void)band; (void)fixed_channel; (void)duration_ms;
    if (!fake_wifi_capture_prior_state_once()) return -1;
    fake_wifi_lock();
    g_fake_wifi.ap_scan_call_count++;
    g_fake_wifi.ap_scan_cancel_requested = 0;
    int rc = g_fake_wifi.ap_scan_rc;
    unsigned total_polls = g_fake_wifi.ap_scan_poll_count_per_call ? g_fake_wifi.ap_scan_poll_count_per_call : 1;
    fake_wifi_unlock();
    if (rc != 0) return rc;
    unsigned done = 0;
    for (; done < total_polls; done++) {
        fake_wifi_lock();
        int cancelled = g_fake_wifi.ap_scan_cancel_requested;
        fake_wifi_unlock();
        if (cancelled) break;
        usleep(5000);
    }
    fake_wifi_lock();
    g_fake_wifi.ap_scan_polls_done = done;
    unsigned n = g_fake_wifi.ap_count < max_out ? g_fake_wifi.ap_count : max_out;
    if (out && n) memcpy(out, g_fake_wifi.ap_results, n * sizeof(*out));
    fake_wifi_unlock();
    return (int)n;
}
static void fake_wifi_sta_scan_cancel(void) {
    fake_wifi_lock();
    if (!g_fake_wifi.sta_scan_ignore_cancel) g_fake_wifi.sta_scan_cancel_requested = 1;
    g_fake_wifi.sta_scan_cancel_count++;
    fake_wifi_unlock();
}
/* RC9 independent correction order P0 "station-target scan bypasses the
 * transactional radio lifecycle": mirrors mtek_wifi_hal_esp32.c's own
 * esp32_sta_scan -- capture_prior_state_once, a real (test-injectable)
 * transactional-entry failure, and a short-poll cancellable wait (5ms
 * real sleeps, so a genuine concurrent pthread STOP -- test_sta_scan_
 * *.c's own real worker -- can be observed taking effect) instead of one
 * uninterruptible block, with sta_scan_polls_done left behind so a test
 * can prove a STOP-triggered return consumed fewer polls than the full
 * configured duration would have. */
static int fake_wifi_sta_scan(mtk_hal_mac6_t bssid, uint8_t channel, uint16_t duration_ms,
                               mtk_hal_station_record_t *out, unsigned max_out) {
    (void)duration_ms;
    if (!fake_wifi_capture_prior_state_once()) return -1;
    fake_wifi_lock();
    g_fake_wifi.sta_scan_call_count++;
    g_fake_wifi.sta_scan_cancel_requested = 0;
    int rc = g_fake_wifi.sta_scan_rc;
    fake_wifi_unlock();
    if (rc != 0) return rc; /* transactional-entry failure: never touches out[]/count */

    g_fake_wifi.last_sta_scan_bssid = bssid;
    g_fake_wifi.last_sta_scan_channel = channel;

    fake_wifi_lock();
    unsigned total_polls = g_fake_wifi.sta_scan_poll_count_per_call ? g_fake_wifi.sta_scan_poll_count_per_call : 1;
    fake_wifi_unlock();
    unsigned done = 0;
    for (; done < total_polls; done++) {
        fake_wifi_lock();
        int cancelled = g_fake_wifi.sta_scan_cancel_requested;
        fake_wifi_unlock();
        if (cancelled) break;
        usleep(5000); /* 5ms -- real, short, so a concurrent pthread STOP has a genuine chance to be observed */
    }
    fake_wifi_lock();
    g_fake_wifi.sta_scan_polls_done = done;
    unsigned n = g_fake_wifi.sta_count < max_out ? g_fake_wifi.sta_count : max_out;
    memcpy(out, g_fake_wifi.sta_results, n * sizeof(*out));
    /* RC10 independent correction order P0 "transactional entry/teardown
     * errors are still discarded": mirrors mtek_wifi_hal_esp32.c's own
     * esp32_sta_scan teardown -- a real (test-injected) callback-
     * unregister or promiscuous-disable failure here means the call
     * itself must fail, even though `n` real results were already found
     * and copied above. */
    int promisc_off_rc = g_fake_wifi.sta_scan_teardown_promisc_off_rc;
    int cb_clear_rc = g_fake_wifi.sta_scan_teardown_cb_clear_rc;
    fake_wifi_unlock();
    if (promisc_off_rc != 0 || cb_clear_rc != 0) return -1;
    return (int)n;
}
static int fake_wifi_send_deauth(mtk_hal_mac6_t ap_bssid, mtk_hal_mac6_t station, uint8_t channel) {
    if (!fake_wifi_capture_prior_state_once()) return -1;
    /* Mirrors mtek_wifi_hal_esp32.c's own esp32_send_deauth, which selects
     * the requested channel before every single transmit -- so the
     * "operating" channel genuinely disturbs current_channel here too,
     * making a test's own post-restore channel assertion meaningful
     * (not vacuously true just because nothing ever touched it). */
    fake_wifi_lock();
    g_fake_wifi.set_channel_call_count++;
    g_fake_wifi.current_channel = channel;
    g_fake_wifi.deauth_sent_count++;
    g_fake_wifi.last_deauth_ap = ap_bssid;
    g_fake_wifi.last_deauth_station = station;
    fake_wifi_unlock();
    return 0;
}
static int fake_wifi_promisc_start(uint8_t channel, mtk_hal_frame_cb_t cb, void *user) {
    (void)channel;
    if (!fake_wifi_capture_prior_state_once()) return -1;
    fake_wifi_lock();
    g_fake_wifi.promisc_start_count++;
    unsigned delay_ms = g_fake_wifi.promisc_start_delay_ms;
    fake_wifi_unlock();
    if (delay_ms) usleep(delay_ms * 1000);
    /* RC8 independent audit P0-9: test-settable failure injection --
     * proves a real transactional-failure path, not just the always-
     * succeeds default every other promisc_start test relies on.
     *
     * RC11 round 10 verification (TSan): promisc_start_rc, defer_frames,
     * the pending_cb/pending_cb_user publication and the staged frames
     * were all read/written here with no lock at all, while this very
     * function runs on a dispatched worker thread and a test's own main
     * thread reaches into the same fields -- the real, TSan-confirmed
     * data race (see mtk_fake_wifi_deliver_frames above). All of it is
     * now one locked section; promisc_registered_count is bumped inside
     * it, immediately after the registration becomes visible, so a test
     * can wait for publication rather than for mere entry. The callback
     * is invoked with the lock RELEASED, for the deadlock reason
     * documented on mtk_fake_wifi_frame_batch_t. */
    fake_wifi_lock();
    int start_rc = g_fake_wifi.promisc_start_rc;
    if (start_rc != 0) { fake_wifi_unlock(); return start_rc; }
    if (g_fake_wifi.defer_frames) {
        g_fake_wifi.pending_cb = cb; g_fake_wifi.pending_cb_user = user;
        g_fake_wifi.promisc_registered_count++;
        fake_wifi_unlock();
        return 0;
    }
    mtk_fake_wifi_frame_batch_t batch;
    fake_wifi_snapshot_frames_locked(&batch);
    g_fake_wifi.promisc_registered_count++;
    fake_wifi_unlock();
    for (unsigned i = 0; i < batch.count; i++) {
        cb(user, batch.frames[i].data, batch.frames[i].len, batch.frames[i].rssi, batch.frames[i].channel);
    }
    return 0;
}
/* RC11 round 10 verification (TSan): counter bump locked for the same
 * reason as promisc_start's own -- this runs on dispatched worker threads
 * while a test's main thread reads the same counter. */
static void fake_wifi_promisc_stop(void) {
    fake_wifi_lock();
    g_fake_wifi.promisc_stop_count++;
    fake_wifi_unlock();
}
/* RC11 independent correction order P0 #1/#2: this fake HAL already
 * delivers every frame synchronously, inline, from inside promisc_start
 * itself (see its own doc comment) -- there is no real Wi-Fi driver task
 * here to be unsafe on in the first place, so there is nothing to
 * service/drain. A real target HAL's own esp32_promisc_service is what
 * mtek_wifi_service_tick actually calls; this no-op exists only so every
 * host test's own tick call has a valid, harmless function pointer. */
static void fake_wifi_promisc_service(void) {}
static int fake_wifi_set_channel(uint8_t channel) {
    if (!fake_wifi_capture_prior_state_once()) return -1;
    fake_wifi_lock();
    g_fake_wifi.set_channel_call_count++;
    /* RC11 owner-approved correction fallout (RAW_TX_SEND now genuinely
     * restores prior state afterward, mtek_wifi_logic.c's own handle_raw_
     * tx_send): current_channel itself is no longer a reliable post-call
     * signal of "what channel was this transmit actually sent on" -- a
     * successful restore reverts it to the prior value before the test
     * ever gets to look. Recorded unconditionally (even on injected
     * failure, matching current_channel's own real i.e. only-on-success
     * semantics being a separate concern) so a test can still prove the
     * REQUESTED value was genuinely passed through, independent of
     * whether it was later restored away. */
    g_fake_wifi.last_requested_channel = channel;
    int rc = g_fake_wifi.set_channel_rc;
    if (rc == 0) g_fake_wifi.current_channel = channel;
    fake_wifi_unlock();
    return rc;
}
static int fake_wifi_get_mode(uint8_t *mode_out) { *mode_out = g_fake_wifi.mode; return 0; }
static int fake_wifi_set_mode(uint8_t mode) { g_fake_wifi.mode = mode; return 0; }
static void fake_wifi_get_mac(mtk_hal_mac6_t *out) { *out = g_fake_wifi.mac; }
static int fake_wifi_raw_tx(const uint8_t *frame, uint16_t len) { (void)frame; (void)len; g_fake_wifi.raw_tx_count++; return g_fake_wifi.raw_tx_rc; }
/* Mirrors mtek_wifi_hal_esp32.c's own restore_sta_mode contract: a
 * reconnect is only attempted (tracked here via reconnect_attempted, for
 * test assertions) if the station was genuinely connected before the
 * List B operation borrowed the radio. */
/* RC10 independent correction order P0 "restore is not actually failure-
 * atomic": mirrors mtek_wifi_hal_esp32.c's own esp32_restore_sta_mode --
 * returns 0 only if every step it actually attempted succeeded, -1
 * otherwise; the snapshot is preserved (not cleared) until this function
 * reaches its own defined safe terminal state at the end; a failing
 * "stop" short-circuits mode/start/channel/reconnect exactly like the
 * real HAL (their own real precondition, a stopped driver, could not be
 * confirmed). */
static int fake_wifi_restore_sta_mode(void) {
    fake_wifi_lock();
    g_fake_wifi.restore_count++;
    int ok = 1;

    if (g_fake_wifi.restore_fail_countdown > 0) {
        g_fake_wifi.restore_fail_countdown--;
        fake_wifi_unlock();
        return -1; /* transient failure: prior-state snapshot untouched, matching the real HAL's own "clear only on success" rule */
    }

    int promisc_off_rc = g_fake_wifi.restore_promisc_off_rc;
    if (promisc_off_rc != 0) ok = 0;
    int stop_rc = g_fake_wifi.restore_stop_rc;
    if (stop_rc != 0) ok = 0;

    if (stop_rc == 0) {
        if (g_fake_wifi.restore_mode_rc != 0) ok = 0;
        if (g_fake_wifi.prior_state_valid) g_fake_wifi.mode = g_fake_wifi.prior_mode;
        int start_rc = g_fake_wifi.restore_start_rc;
        if (start_rc != 0) ok = 0;
        if (start_rc == 0) {
            if (g_fake_wifi.restore_channel_rc != 0) ok = 0;
            else if (g_fake_wifi.prior_state_valid) g_fake_wifi.current_channel = g_fake_wifi.prior_channel;
            uint8_t was_connected = g_fake_wifi.prior_state_valid ? g_fake_wifi.prior_was_connected : (uint8_t)g_fake_wifi.sta_was_connected;
            if (was_connected) {
                if (g_fake_wifi.restore_reconnect_rc != 0) ok = 0;
                else g_fake_wifi.reconnect_attempted_count++;
            }
        }
    } else {
        ok = 0; /* stop failed: mode/start/channel/reconnect all skipped as unverified, matching the real HAL */
    }

    /* RC11 independent correction order P0 "retain the prior-state
     * snapshot ... on a restore failure": this used to clear
     * prior_state_valid/mode/channel/was_connected UNCONDITIONALLY here,
     * contradicting this very function's own doc comment above ("the
     * snapshot is preserved ... until this function reaches its own
     * defined safe terminal state") and mekt_wifi_hal_esp32.c's own real
     * esp32_restore_sta_mode, which clears it ONLY on success -- a real
     * fidelity gap that would have made a host test's own "does a retry
     * recover to the EXACT original prior state" assertion pass by
     * accident (or not at all) regardless of whether production logic
     * actually preserves it. Mirrors the real HAL exactly: forgotten only
     * on success. */
    if (ok) {
        g_fake_wifi.prior_state_valid = 0;
        g_fake_wifi.prior_mode = 0;
        g_fake_wifi.prior_channel = 0;
        g_fake_wifi.prior_was_connected = 0;
    }
    fake_wifi_unlock();
    return ok ? 0 : -1;
}
static void fake_wifi_pace_delay_ms(uint32_t ms) { (void)ms; g_fake_wifi.pace_delay_call_count++; /* no real sleep -- host tests stay fast and deterministic */ }
/* RC10 independent correction order P0 "STOP restores/releases the radio
 * before the worker has stopped touching it": mirrors mtek_wifi_hal_
 * esp32.c's own esp32_quiescence_wait_ms -- deliberately a REAL short
 * sleep (unlike pace_delay_ms's intentional no-op), since this is the
 * only thing giving a genuinely concurrent worker thread real wall-clock
 * time to finish inside a host test's own bounded STOP-quiescence wait;
 * nothing else in this suite depends on this call staying instantaneous. */
static void fake_wifi_quiescence_wait_ms(uint32_t ms) {
    fake_wifi_lock(); g_fake_wifi.quiescence_wait_call_count++; fake_wifi_unlock();
    usleep(ms * 1000);
}

static const mtk_wifi_hal_t g_fake_wifi_hal = {
    fake_wifi_ap_scan, fake_wifi_ap_scan_cancel, fake_wifi_sta_scan, fake_wifi_sta_scan_cancel, fake_wifi_connect, fake_wifi_disconnect, fake_wifi_get_status,
    fake_wifi_send_deauth, fake_wifi_promisc_start, fake_wifi_promisc_stop, fake_wifi_promisc_service, fake_wifi_set_channel,
    fake_wifi_get_mode, fake_wifi_set_mode, fake_wifi_get_mac, fake_wifi_raw_tx, fake_wifi_restore_sta_mode,
    fake_wifi_pace_delay_ms, fake_wifi_quiescence_wait_ms,
};
