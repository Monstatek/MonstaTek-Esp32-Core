/* Clean-room implementation from MonstaTek contract. Blocking hardware
 * abstraction the portable Wi-Fi service logic (mtek_wifi_logic.c) calls
 * into. A target build supplies a real ESP-IDF-backed implementation
 * (mtek_wifi_hal_esp32.c, ESP-IDF component only); host tests link a
 * deterministic fake (mtek_wifi_hal_fake.c) instead. Every call blocks
 * until it has a definite result -- matching ESP-IDF's own blocking
 * esp_wifi_scan_start()/connect-and-wait-for-event patterns -- so the
 * service logic itself needs no async/callback machinery; a target build
 * runs each *_START opcode's blocking work on its own FreeRTOS task so it
 * never stalls the adapter's request-dispatch loop. */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "mtek_hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mtk_hal_mac6_t bssid;
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint8_t channel;
    int8_t rssi;
    uint8_t authmode;
} mtk_hal_ap_record_t;

typedef struct {
    mtk_hal_mac6_t mac;
    int8_t rssi;
} mtk_hal_station_record_t;

typedef struct {
    uint8_t connected;
    uint8_t ssid[32];
    uint8_t ssid_len;
    mtk_hal_mac6_t bssid;
    uint8_t channel;
    uint8_t ip_present;
    mtk_hal_ipv4_t ip;
    int8_t rssi;
} mtk_hal_sta_status_t;

typedef struct {
    uint8_t connected;      /* final result */
    mtk_hal_mac6_t bssid;
    uint8_t channel;
    uint8_t ip_present;
    mtk_hal_ipv4_t ip;
    uint8_t timed_out;
} mtk_hal_connect_result_t;

/* Called by the service for each captured 802.11 frame while a promiscuous
 * capture (handshake/MonstaShark) is active. `frame` is the raw MAC frame
 * (header + body), never parsed by the HAL itself -- EAPOL/frame-type
 * inspection is the service's own portable logic, kept host-testable. */
typedef void (*mtk_hal_frame_cb_t)(void *user, const uint8_t *frame, uint16_t len, int8_t rssi, uint8_t channel);

typedef struct mtk_wifi_hal {
    /* AP scan: blocks until complete or the deadline elapses; returns the
     * number of APs written into `out` (capacity `max_out`). */
    int (*ap_scan)(uint8_t band, uint8_t fixed_channel /*0=hop*/, uint32_t duration_ms,
                    mtk_hal_ap_record_t *out, unsigned max_out);
    /* RC11 independent correction order P0 "AP scan has the same async
     * STOP race and false-success behavior": mirrors sta_scan_cancel's
     * own contract exactly -- requests an in-progress ap_scan (blocked
     * inside esp_wifi_scan_start(..., true) on another task) to stop as
     * soon as possible via a real esp_wifi_scan_stop() call. NULL-safe;
     * a caller that cannot cancel must not release the radio's arbiter
     * lease until ap_scan itself actually returns. */
    void (*ap_scan_cancel)(void);
    /* Station-target scan against one AP/channel.
     *
     * RC9 independent correction order P0 "station-target scan bypasses
     * the transactional radio lifecycle": this call must now follow the
     * same transactional entry/exit contract as promisc_start/
     * restore_sta_mode -- capture the real prior state before the first
     * mutation, check every callback/channel/promiscuous-mode return
     * value, unwind cleanly on any partial-entry failure (never leaving
     * the callback registered or promiscuous mode on), and guarantee
     * teardown (promiscuous mode off, callback cleared) before
     * returning, however the wait ends. Returns the number of stations
     * written into `out` on success (0..max_out), or a negative value if
     * the transactional entry itself failed (callback/channel/
     * promiscuous-mode step) -- the caller must treat a negative return
     * as a real failure, never silently substitute 0. */
    int (*sta_scan)(mtk_hal_mac6_t bssid, uint8_t channel, uint16_t duration_ms,
                     mtk_hal_station_record_t *out, unsigned max_out);
    /* Requests an in-progress sta_scan (on another task, via a real
     * async runner) to stop as soon as possible: sta_scan itself waits
     * in short, bounded polls (not one long uninterruptible delay) and
     * checks this signal between them, tearing down and returning early
     * once seen -- closing "STA_SCAN_STOP merely changes the operation
     * record and releases ownership while the worker continues touching
     * the radio". NULL-safe (older/host-fake HALs that predate this may
     * omit it; a caller that cannot cancel must not release the radio's
     * arbiter lease until sta_scan itself actually returns). */
    void (*sta_scan_cancel)(void);
    /* Blocks until associated+IP (or DHCP skipped for static), timed out,
     * or failed. */
    int (*connect)(const uint8_t *ssid, uint8_t ssid_len, mtk_hal_mac6_t bssid_hint,
                    uint8_t channel_hint, uint8_t auth_mode, const uint8_t *psk, uint8_t psk_len,
                    uint8_t ip_mode, mtk_hal_ipv4_t static_ip, mtk_hal_ipv4_t static_netmask,
                    mtk_hal_ipv4_t static_gateway, uint32_t timeout_ms, mtk_hal_connect_result_t *out);
    void (*disconnect)(void);
    void (*get_status)(mtk_hal_sta_status_t *out);

    /* Sends one 802.11 deauthentication frame (AP->station direction,
     * reason code 2). Returns 0 on success. */
    int (*send_deauth)(mtk_hal_mac6_t ap_bssid, mtk_hal_mac6_t station, uint8_t channel);

    /* Fixes the Wi-Fi channel and installs the single promiscuous
     * callback; `user` is passed back on every frame. Returns 0 on
     * success. */
    int (*promisc_start)(uint8_t channel, mtk_hal_frame_cb_t cb, void *user);
    void (*promisc_stop)(void);
    /* RC11 independent correction order P0 "never call Wi-Fi control APIs
     * from the Wi-Fi promiscuous callback": `cb` (registered via
     * promisc_start) is no longer invoked directly from the Wi-Fi driver
     * task -- a target HAL now queues raw frames there and defers actual
     * delivery to this call, which the portable layer's own periodic
     * tick (mtek_wifi_service_tick) must call regularly from a normal
     * FreeRTOS task. NULL-safe/no-op for a HAL that predates this or
     * delivers synchronously (e.g. every host test's fake HAL, which has
     * no real Wi-Fi driver task to be unsafe on in the first place). */
    void (*promisc_service)(void);
    /* Switches the currently-fixed channel while promiscuous mode stays
     * active (no restart) -- used by a hop-mode MonstaShark capture's
     * periodic channel plan (mtek_capture_logic.c's
     * mtek_capture_channel_hop_tick, RC5 independent audit P1 "capture
     * filters and channel plans are stored but not applied").
     *
     * RC8 independent audit P0-9 "Correct raw-radio channel and monitor-
     * mode entry behavior": "Current raw TX validates the requested
     * channel but does not select it. Deauth logs a channel-set failure
     * and may still transmit on the wrong channel." Returns 0 on success,
     * nonzero on failure -- callers that need to TRANSMIT on a specific
     * channel (RAW_TX_SEND, deauth) must check this and never transmit
     * on a channel selection failure; a hop-mode capture tick's own
     * channel switch is comparatively low-stakes (a missed hop just
     * means one dwell period stays on the old channel, not a frame sent
     * on the WRONG one) but the real result is still available to it
     * now instead of silently discarded. */
    int (*set_channel)(uint8_t channel);

    int (*get_mode)(uint8_t *mode_out);      /* 0=STA,1=AP,2=APSTA */
    int (*set_mode)(uint8_t mode);
    void (*get_mac)(mtk_hal_mac6_t *out);

    int (*raw_tx)(const uint8_t *frame, uint16_t len);

    /* Deterministic restoration to the prior connected-mode state (RC5
     * independent audit P1 "Wi-Fi restoration after monitor mode is
     * incomplete"), called after every List B operation's stop/
     * cancellation/timeout/error path -- not just a bare mode switch:
     * stop (promiscuous off) / delay (let the radio settle) / mode (back
     * to STA) / start (idempotent) / reconnect (re-issue connect only if
     * the station was genuinely connected before this operation borrowed
     * the radio). The exact settle delay is an engineering choice, not an
     * independently confirmed timing value from the accepted facts
     * package (see mtek_wifi_hal_esp32.c's own doc comment on this call).
     *
     * RC10 independent correction order P0 "restore is not actually
     * failure-atomic": now returns 0 only if every real step it performed
     * succeeded, nonzero if any of them failed -- the caller must treat a
     * nonzero return as a real, honest failure (never silently report
     * clean completion/release as healthy). The captured prior-state
     * snapshot is preserved until restoration reaches its own defined
     * safe terminal state (not cleared up front), and a failing
     * prerequisite step short-circuits the steps that depend on it
     * (mtek_wifi_hal_esp32.c's own doc comment on this call has the exact
     * per-step short-circuit rules). */
    int (*restore_sta_mode)(void); /* used by every stop/reset/timeout path */

    /* Pacing delay for a run-until-stopped background operation (deauth
     * count=0/continuous mode) between transmit rounds -- lets a
     * long-running worker task yield instead of spinning a CPU core, and
     * gives mtk_op_state_is_terminal(rec->state) (checked between rounds)
     * a real chance to observe a concurrent STOP promptly. NULL-safe:
     * host tests may omit it (see mtk_fake_wifi_hal.h), in which case the
     * caller must not depend on any actual delay occurring. */
    void (*pace_delay_ms)(uint32_t ms);

    /* RC10 independent correction order P0 "STOP restores/releases the
     * radio before the worker has stopped touching it": a REAL, bounded
     * wait used only by a STOP handler's own quiescence handshake (poll
     * "has the worker reached a terminal state yet" between short real
     * waits) -- deliberately separate from pace_delay_ms, which
     * intentionally never sleeps in host tests (kept fast/deterministic
     * for a purpose unrelated to this one: pacing an already-running
     * worker's own send loop, never used to wait for ANOTHER thread to
     * finish). NULL-safe: a caller with no real wait available (single-
     * threaded host tests with no async runner) has nothing to wait for
     * in the first place, since nothing runs concurrently there. */
    void (*quiescence_wait_ms)(uint32_t ms);
} mtk_wifi_hal_t;

void mtek_wifi_set_hal(const mtk_wifi_hal_t *hal);

#ifdef __cplusplus
}
#endif
