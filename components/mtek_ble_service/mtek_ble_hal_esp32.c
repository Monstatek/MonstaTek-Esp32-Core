/* Clean-room implementation from MonstaTek contract, built against
 * official ESP-IDF v6.0.1 NimBLE host APIs (host/ble_gap.h, host/ble_gatt.h).
 * Real radio-facing HAL for the ESP32-C6 target build -- see
 * docs/PROVENANCE.md: host-tested only; hardware behavior is not
 * validated here.
 *
 * NimBLE's own API is asynchronous/event-driven throughout; every
 * blocking wrapper below follows the same pattern (issue the async NimBLE
 * call, block on a short-lived binary semaphore that the GAP/GATT event
 * callback signals, then read out the result the callback stashed) to
 * match mtek_ble_hal.h's blocking-call contract (mtek_wifi_hal.h has the
 * same rationale for the Wi-Fi side). */
#include "sdkconfig.h"
#if CONFIG_BT_ENABLED
#include "mtek_ble_hal_esp32.h"
#include "mtk_ble_op_generation.h"
#include "mtk_ble_op_lifecycle.h"
#include "mtek_ble_disc_status.h"
#include "mtek_ble_adv_merge.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>

static const char *TAG = "mtk_ble_hal";
static uint8_t s_own_addr_type;
/* Real synchronized publication of the host-synced flag, replacing
 * a bare `volatile int`. `volatile` orders neither this flag against the
 * s_own_addr_type write that must precede it (on_sync) nor a reader's own
 * subsequent radio calls -- it only forbids the compiler from caching the value.
 * An atomic store (default seq_cst) in on_sync establishes a release fence so
 * s_own_addr_type is genuinely visible to any task that then observes
 * s_synced==1 before it issues a GAP/GATT operation using s_own_addr_type. */
static atomic_int s_synced;
/* Protects s_notify_queue/head/count and s_disconnected_handle (declared further
 * down, near their own usage) across the NimBLE host-task callback and this
 * file's polling calls -- see the fuller doc comment at those declarations.
 * Declared here, early, so mtek_ble_hal_esp32_init (which creates it) can appear
 * before the GATT client section further down without a forward-declaration. */
static SemaphoreHandle_t s_notify_mutex;
/* XSemaphoreCreateMutex can fail (heap exhaustion) and every call site below
 * previously called xSemaphoreTake/Give on s_notify_mutex unconditionally -- a
 * NULL handle passed to either is undefined behavior on FreeRTOS. Null-checked
 * here as defense-in-depth (NimBLE's own host-task callbacks, conn_gap_cb below,
 * keep running regardless of whether mtek_ble_hal_esp32_init's own reported
 * failure below caused app_main.c to leave this HAL unregistered) -- the PRIMARY
 * fix is that a failed s_notify_mutex allocation makes mtek_ble_hal_esp32_init
 * report failure at all (see its own doc comment), so app_main.c never wires
 * this HAL in for real use in that case. */
static void notify_lock(void) { if (s_notify_mutex) xSemaphoreTake(s_notify_mutex, portMAX_DELAY); }
static void notify_unlock(void) { if (s_notify_mutex) xSemaphoreGive(s_notify_mutex); }

/* One shared lifecycle object guards every
 * one of this file's nine static-context callback sites below (scan/signal,
 * connect, GATT service/char/descriptor discovery, read, write, CCCD-descriptor
 * discovery, characteristic-bound search) -- a single instance is correct
 * because this HAL only ever has ONE such blocking operation in flight at a time
 * (each is a synchronous call that waits on its own semaphore before returning),
 * so a fresh arm always means "whatever was previously outstanding is now
 * superseded", regardless of which of the nine kinds it was. Unlike the prior
 * generation-only guard, this closes the check-to-use race (see
 * mtk_ble_op_lifecycle.h): a callback holds the lifetime lock across its whole
 * is-current-check + context-use + semaphore-give span, and every arm/retire is
 * under that same lock, so a timeout's teardown can never free the
 * context/semaphore out from under an already-entered callback. The lock itself
 * (s_op_lc_mutex) is a dedicated FreeRTOS mutex, mandatory at init: a failed
 * allocation makes the whole HAL report not-ready (mtek_ble_hal_esp32_init), so
 * app_main never wires it in. */
static mtk_ble_op_lifecycle_t s_op_lc;
static SemaphoreHandle_t s_op_lc_mutex;
static void op_lc_take(void *ctx) { if (ctx) xSemaphoreTake((SemaphoreHandle_t)ctx, portMAX_DELAY); }
static void op_lc_give(void *ctx) { if (ctx) xSemaphoreGive((SemaphoreHandle_t)ctx); }

void ble_store_config_init(void);

static void on_reset(int reason) { ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason); atomic_store(&s_synced, 0); }
static void on_sync(void) {
    /* BOTH address-initialization results are now checked -- host
     * readiness is published only if both genuinely succeeded. A failed
     * ensure_addr/infer_auto previously left s_own_addr_type undefined-but-used
     * while s_synced was set to 1 anyway, so every subsequent GAP/GATT call ran
     * with an unresolved identity address. */
    int rc_addr = ble_hs_util_ensure_addr(0);
    int rc_infer = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc_addr != 0 || rc_infer != 0) {
        ESP_LOGE(TAG, "NimBLE address init failed (ensure_addr=%d infer_auto=%d) -- host NOT marked ready",
                 rc_addr, rc_infer);
        atomic_store(&s_synced, 0);
        return;
    }
    atomic_store(&s_synced, 1); /* release fence: s_own_addr_type above is visible before any reader sees synced==1 */
    ESP_LOGI(TAG, "NimBLE host synced");
}
static void host_task(void *param) { (void)param; nimble_port_run(); nimble_port_freertos_deinit(); }

/* A failed s_notify_mutex allocation previously left every
 * conn_gap_cb/esp32_gatt_poll_notify/_disconnected/ notify_dropped_count call
 * site taking/giving a NULL FreeRTOS handle -- undefined behavior, not a clean
 * failure. notify_lock/notify_unlock are now null-safe (defense-in-depth, since
 * NimBLE's own host-task callback keeps running regardless), but the real fix is
 * reporting this failure here so app_main.c never wires this HAL in for real use
 * at all -- every BLE opcode then honestly refuses via mtek_ble_logic.c's
 * existing `s_hal &&` guards, never running with a partially-initialized,
 * silently unlocked notification/disconnect path. */
int mtek_ble_hal_esp32_init(void) {
    mtk_ble_op_lifecycle_init(&s_op_lc);
    /* The lifetime lock is a mandatory resource -- without it the
     * callback-lifetime protocol degrades to a single-thread-only no-op, which
     * is exactly the unsafe check-to-use window exists to close. A failed
     * allocation therefore makes the whole HAL report not-ready (like
     * s_notify_mutex below), so app_main never installs it and every BLE opcode
     * honestly refuses. */
    s_op_lc_mutex = xSemaphoreCreateMutex();
    if (!s_op_lc_mutex) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex(s_op_lc_mutex) failed -- BLE HAL will not be wired in this boot session");
        return -1;
    }
    mtk_ble_op_lifecycle_set_lock(&s_op_lc, op_lc_take, op_lc_give, s_op_lc_mutex);
    s_notify_mutex = xSemaphoreCreateMutex();
    if (!s_notify_mutex) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex(s_notify_mutex) failed -- BLE HAL will not be wired in this boot session");
        return -1;
    }
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init failed: %d", err); return -1; }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_device_name_set("MonstaTek-M1");
    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    return 0;
}


/* scan (also reused for the signal-meter's short single-target scan) */

typedef struct {
    mtk_hal_ble_adv_t *out;
    unsigned max, count;
    SemaphoreHandle_t done;
    uint8_t target_only;
    mtk_hal_mac6_t target;
    uint8_t target_type;
} scan_ctx_t;
/* The per-operation context is no longer held in a separate
 * file-scope pointer that a callback reads outside the lifetime lock -- it is
 * published INTO the lifecycle (arm) and handed back to the callback atomically
 * with the generation check (callback_begin), closing the check-to-use window.
 * Same for every other operation's context below
 * (connect/discover/read/write/CCCD/char-bound). */

/* + "name filtering is prefix/length- asymmetric rather than exact": a real BLE
 * device advertises repeatedly throughout a whole scan window, so without dedup
 * the same physical device previously consumed multiple `out[]` slots and
 * misrepresented "how many devices found" -- now checked against every
 * already-recorded address (linear scan of the small in-progress result set,
 * correct and simple for the bounded max_out this HAL ever scans with) before a
 * new entry is added. `o->flags`/`o->mfg_data`/`o->mfg_len` (already present in
 * mtk_hal_ble_adv_t, just never populated) are now filled from NimBLE's own
 * parsed advertisement fields. Name filtering now requires an EXACT
 * length-and-content match, not `strncmp` over min(advertised_name_len, 32)
 * bytes -- the previous form could match a shorter advertised name against a
 * longer filter whenever the shorter name was itself a byte-for-byte PREFIX of
 * the filter (e.g. filter "ABCDEF" incorrectly matching an advertised name
 * "ABC"). */
/* The BLE_GAP_EVENT_DISC body, extracted so scan_gap_cb can guarantee a
 * single matched callback_begin/callback_end pair even though this body
 * has several internal early-outs (name-filter mismatch, dedup hit). It
 * runs with the lifetime lock held (its caller entered via
 * mtk_ble_op_lifecycle_callback_begin), touching only the published `c`. */
static void scan_handle_disc(scan_ctx_t *c, struct ble_gap_event *event) {
    uint8_t address[6];
    mtk_ble_addr_order(address, event->disc.addr.val);
    mtk_ble_adv_observe(c->out, c->max, &c->count, address,
        event->disc.addr.type, c->target_only ? c->target.b : NULL, c->target_type,
        event->disc.data, event->disc.length_data,
        event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP,
        event->disc.event_type, event->disc.rssi, (uint32_t)(esp_timer_get_time() / 1000));
}

/* Every NimBLE GAP/GATT callback in this file now enters through
 * mtk_ble_op_lifecycle_callback_begin (which takes the lifetime lock and hands
 * back the currently-published context only if this callback's own generation is
 * still current) and leaves through mtk_ble_op_lifecycle_callback_end -- so a
 * timeout's retire can never free the context/semaphore mid-callback. All
 * context access and the semaphore give happen inside this locked span. */
static int scan_gap_cb(struct ble_gap_event *event, void *arg) {
    scan_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (c) {
        if (event->type == BLE_GAP_EVENT_DISC) {
            scan_handle_disc(c, event);
        } else if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
            if (c->done) xSemaphoreGive(c->done);
        }
    }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}

/* A scan that reaches its bounded wait without the callback
 * signalling completion must actively cancel the still-running NimBLE discovery
 * procedure (ble_gap_disc_cancel, a supported GAP cancellation API) BEFORE
 * retiring the lifecycle -- the application giving up waiting does not itself
 * stop the controller from clocking more DISC events into scan_gap_cb. Cancel is
 * issued WITHOUT the lifetime lock held (it can synchronously deliver a final
 * DISC_COMPLETE into scan_gap_cb, which takes that same lock itself); the
 * context is still live and its generation still current at that point, so such
 * a callback is safe. retire then quiesces all future callback access before the
 * semaphore is deleted and ctx leaves scope. */
static void esp32_ble_scan_teardown(scan_ctx_t *ctx, int signaled) {
    if (!signaled) ble_gap_disc_cancel();
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx->done);
}

static int esp32_ble_scan(uint8_t mode, uint16_t duration_ms, const char *name_filter,
                           mtk_hal_ble_adv_t *out, unsigned max_out) {
    if (!atomic_load(&s_synced)) return -1;
    scan_ctx_t ctx = {0};
    ctx.out = out; ctx.max = max_out;
    ctx.done = xSemaphoreCreateBinary();
    /* A failed allocation must never reach xSemaphoreTake/vSemaphoreDelete on a
     * NULL handle (undefined behavior) -- fail this one operation honestly
     * instead, before ever arming the lifecycle/starting the real NimBLE call. */
    if (!ctx.done) return -1;
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);

    struct ble_gap_disc_params params = {0};
    params.passive = (mode == 1);
    params.itvl = 0x0010; params.window = 0x0010; params.filter_duplicates = 0;
    uint32_t dur = duration_ms ? duration_ms : 10000;
    int rc = ble_gap_disc(s_own_addr_type, dur, &params, scan_gap_cb, (void *)(uintptr_t)gen);
    if (rc != 0) { mtk_ble_op_lifecycle_retire(&s_op_lc); vSemaphoreDelete(ctx.done); return -1; }
    int signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(dur + 1000)) == pdTRUE);
    esp32_ble_scan_teardown(&ctx, signaled);
    unsigned kept = 0;
    for (unsigned i = 0; i < ctx.count; ++i) {
        if (name_filter && *name_filter &&
            (strlen(name_filter) != out[i].name_len || memcmp(name_filter, out[i].name, out[i].name_len))) continue;
        if (kept != i) out[kept] = out[i];
        kept++;
    }
    return (int)kept;
}

/* The shipping parity record is exact -- advertising must be non-connectable,
 * carry no scan response, and contain name + flags only (no TX-power field).
 * This is List A (frozen), not a design-freedom List B surface, so it is
 * restored byte-for-byte rather than treated as an opportunity to add a field a
 * real M1 client was never built to expect. */
static int esp32_ble_adv_start(const uint8_t *name, uint8_t name_len) {
    if (!atomic_load(&s_synced)) return -1;
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = name; fields.name_len = name_len; fields.name_is_complete = 1;
    /* No tx_pwr_lvl_is_present: the frozen shipped payload is name + flags
     * only, nothing else. */
    if (ble_gap_adv_set_fields(&fields) != 0) return -1;
    /* No ble_gap_adv_rsp_set_fields call: no scan response, matching the
     * frozen shipped behavior exactly. */

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; /* non-connectable, frozen shipped behavior */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    return ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL) == 0 ? 0 : -1;
}
static void esp32_ble_adv_stop(void) { ble_gap_adv_stop(); }

/* One persistent discovery context, protected by the existing callback lifetime
 * lock. The service arbiter owns it for either discovery or signal tracking.
 * No GATT operation is used to obtain RSSI or a local name. */
static mtk_hal_ble_adv_t s_live_items[40];
static scan_ctx_t s_live_ctx;
static uint8_t s_live_running;

static void esp32_live_stop(void) {
    if (!s_live_running) return;
    ble_gap_disc_cancel();
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    s_live_running = 0;
}

static int esp32_live_start(uint8_t target_only, mtk_hal_mac6_t target, uint8_t type,
                            const mtk_hal_ble_adv_t *previous, unsigned count) {
    if (!atomic_load(&s_synced) || s_live_running) return -1;
    memset(&s_live_ctx, 0, sizeof(s_live_ctx));
    s_live_ctx.out = s_live_items;
    s_live_ctx.max = target_only ? 1 : 40;
    s_live_ctx.target_only = target_only;
    s_live_ctx.target = target;
    s_live_ctx.target_type = type;
    if (previous) {
        s_live_ctx.count = count < s_live_ctx.max ? count : s_live_ctx.max;
        memcpy(s_live_items, previous, s_live_ctx.count * sizeof(*previous));
    }
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &s_live_ctx);
    struct ble_gap_disc_params params = {0};
    params.passive = target_only; /* picker collects scan responses; meter listens */
    params.itvl = 0x0010; params.window = 0x0010; params.filter_duplicates = 0;
    if (ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, scan_gap_cb,
                     (void *)(uintptr_t)gen) != 0) {
        mtk_ble_op_lifecycle_retire(&s_op_lc);
        return -1;
    }
    s_live_running = 1;
    return 0;
}
static int esp32_live_discovery_start(const mtk_hal_ble_adv_t *previous, unsigned count) {
    mtk_hal_mac6_t unused = {{0}};
    return esp32_live_start(0, unused, 0, previous, count);
}
static int esp32_live_snapshot(mtk_hal_ble_adv_t *out, unsigned max) {
    op_lc_take(s_op_lc_mutex);
    unsigned n = s_live_ctx.count < max ? s_live_ctx.count : max;
    memcpy(out, s_live_items, n * sizeof(*out));
    op_lc_give(s_op_lc_mutex);
    return (int)n;
}
static int esp32_signal_sample(mtk_hal_mac6_t addr, uint8_t addr_type, int8_t *rssi_out, uint8_t *is_random_out) {
    if (!s_live_running && esp32_live_start(1, addr, addr_type, NULL, 0) != 0) return -1;
    int rc = -1;
    op_lc_take(s_op_lc_mutex);
    if (s_live_ctx.target_only && s_live_ctx.count &&
        s_live_items[0].rssi != 127) {
        *rssi_out = s_live_items[0].rssi;
        s_live_items[0].rssi = 127; /* consumed; only a new advertisement re-arms it */
        *is_random_out = (addr_type != 0);
        rc = 0;
    }
    op_lc_give(s_op_lc_mutex);
    return rc;
}

/* GATT client ---------------------------- */

typedef struct { int status; uint16_t conn_handle; SemaphoreHandle_t done; } conn_ctx_t;

#define NOTIFY_QUEUE_LEN 8
typedef struct { uint16_t handle; uint8_t data[32]; uint16_t len; } pending_notify_t;
static pending_notify_t s_notify_queue[NOTIFY_QUEUE_LEN];
static unsigned s_notify_head, s_notify_count;
/* Incremented whenever BLE_GAP_EVENT_NOTIFY_RX arrives with the queue already
 * full (see conn_gap_cb below); reset to 0 alongside the rest of this queue's
 * own state on every new connection (esp32_gatt_connect). Read by
 * esp32_gatt_notify_dropped_count, wired into mtk_gatt_status_resp_t's own
 * dropped_notification_count field by mtek_ble_logic.c -- previously declared
 * and reported but never actually incremented anywhere. */
static uint32_t s_notify_dropped_count;
static uint16_t s_disconnected_handle = 0xFFFF; /* 0xFFFF sentinel: none pending */
/* The real HCI-level disconnect reason (event->disconnect.reason), guarded by
 * the same s_notify_mutex as s_disconnected_handle -- never a hard-coded
 * placeholder. */
static uint8_t s_disconnect_reason;
/* "Notification queue access crosses callback/task contexts without
 * synchronization." s_notify_queue/head/count and s_disconnected_handle above
 * are written from conn_gap_cb (NimBLE's own host task) and read/cleared from
 * esp32_gatt_poll_notify/ esp32_gatt_poll_disconnected (called from
 * mtek_ble_gatt_tick, on whatever task the target wires that periodic tick to --
 * main/ app_main.c's ble_tick_task) -- two genuinely different FreeRTOS tasks,
 * unlike this file's other blocking calls, which correctly rendezvous via their
 * own per-call semaphore. s_notify_mutex (declared near the top of this file,
 * created in mtek_ble_hal_esp32_init) protects both shared structures. */

static int conn_gap_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        /* A CONNECT event belongs to whichever esp32_gatt_connect
         * call registered THIS callback instance -- callback_begin rejects it
         * (and never hands back a context) if that call has already timed out
         * and a NEWER connect attempt has re-armed the lifecycle, and holds the
         * lifetime lock across the context write + semaphore give so a
         * concurrent retire can never free ctx mid-write. NOTIFY_RX/DISCONNECT
         * below are unrelated to any single connect attempt's own timeout --
         * they are legitimate for this callback's whole connection lifetime, so
         * they are deliberately NOT gated on the lifecycle and continue to use
         * s_notify_mutex. */
        conn_ctx_t *c = NULL;
        if (mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) {
            if (c) {
                c->status = event->connect.status;
                c->conn_handle = event->connect.conn_handle;
                if (c->done) xSemaphoreGive(c->done);
            }
            mtk_ble_op_lifecycle_callback_end(&s_op_lc);
        }
    } else if (event->type == BLE_GAP_EVENT_NOTIFY_RX) {
        notify_lock();
        if (s_notify_count < NOTIFY_QUEUE_LEN) {
            pending_notify_t *n = &s_notify_queue[(s_notify_head + s_notify_count) % NOTIFY_QUEUE_LEN];
            n->handle = event->notify_rx.attr_handle;
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len > sizeof(n->data)) len = sizeof(n->data);
            ble_hs_mbuf_to_flat(event->notify_rx.om, n->data, len, &len);
            n->len = len;
            s_notify_count++;
        } else {
            /* Previously silent. */
            s_notify_dropped_count++;
        }
        notify_unlock();
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        /* "Remote disconnect is ignored, leaving connection/arbiter state
         * stale." A remote/peer-initiated disconnect must be observable by the
         * service layer so it can release the GC arbiter lease and clear its own
         * connected-state bookkeeping, exactly as a local GATT_DISCONNECT
         * already does -- not just silently discovered the next time some other
         * call happens to fail. Recorded here (this callback fires on NimBLE's
         * own host task) and drained by esp32_gatt_poll_ disconnected below,
         * polled from mtek_ble_gatt_tick's existing periodic cadence
         * (mtek_ble_logic.c). */
        notify_lock();
        s_disconnected_handle = event->disconnect.conn.conn_handle;
        s_disconnect_reason = (uint8_t)event->disconnect.reason;
        notify_unlock();
    }
    return 0;
}

static int esp32_gatt_connect(mtk_hal_mac6_t addr, uint8_t addr_type, uint32_t timeout_ms, uint16_t *vendor_handle_out) {
    if (!atomic_load(&s_synced)) return -1;
    ble_addr_t peer; peer.type = addr_type; mtk_ble_addr_order(peer.val, addr.b);
    conn_ctx_t ctx = { -1, 0, xSemaphoreCreateBinary() };
    /* See esp32_ble_scan's own doc comment. */
    if (!ctx.done) return -1;
    /* "Zero-filled connection parameters rather than known-valid defaults."
     * All-zero params.itvl_min/itvl_max are outside the valid BLE
     * connection-interval range (Core Spec Vol 3 Part C 9.3.9: 6..3200, in
     * 1.25ms units), so a zero-filled request risks outright rejection by the
     * peer's link layer or an undefined interval, not merely a "default" one.
     * These are NimBLE's own documented typical example values, not
     * independently confirmed shipping-parity numbers from the accepted facts
     * package -- the requirement here is "known-valid", not "byte-identical to
     * the reference": 30-50ms connection interval, no slave latency, a 2.56s
     * supervision timeout (comfortably > 6x the max interval, per the same spec
     * section's own required relationship), matching common NimBLE
     * peripheral/central example configurations. */
    struct ble_gap_conn_params params;
    memset(&params, 0, sizeof(params));
    params.scan_itvl = 0x0010;
    params.scan_window = 0x0010;
    params.itvl_min = 24;             /* 30ms (24 * 1.25ms) */
    params.itvl_max = 40;             /* 50ms (40 * 1.25ms) */
    params.latency = 0;
    params.supervision_timeout = 256; /* 2.56s (256 * 10ms) */
    params.min_ce_len = 0x0010;
    params.max_ce_len = 0x0300;
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int rc = ble_gap_connect(s_own_addr_type, &peer, (int32_t)timeout_ms, &params, conn_gap_cb, (void *)(uintptr_t)gen);
    if (rc != 0) { mtk_ble_op_lifecycle_retire(&s_op_lc); vSemaphoreDelete(ctx.done); return -1; }
    int signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(timeout_ms + 1000)) == pdTRUE);
    /* On the unsignalled (timeout) path actively cancel the
     * still-pending connection attempt (ble_gap_conn_cancel, a supported GAP
     * cancellation API) BEFORE retiring -- otherwise the controller can still
     * deliver a late BLE_GAP_EVENT_CONNECT into conn_gap_cb after this stack ctx
     * is gone. Issued without the lifetime lock held (cancel can synchronously
     * deliver that final CONNECT event, whose callback takes the same lock
     * itself); ctx is still live and current at that instant. */
    if (!signaled) ble_gap_conn_cancel();
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    if (ctx.status != 0) return -1;
    *vendor_handle_out = ctx.conn_handle;
    /* "[notification queue is] not reliably reset between connections" -- a
     * stale notification or disconnect flag left over from a prior connection
     * must never be misread as belonging to this new one. */
    notify_lock();
    s_notify_head = 0; s_notify_count = 0;
    s_notify_dropped_count = 0;
    s_disconnected_handle = 0xFFFF;
    notify_unlock();
    return 0;
}
static void esp32_gatt_disconnect(uint16_t vendor_handle) { ble_gap_terminate(vendor_handle, BLE_ERR_REM_USER_CONN_TERM); }

/* "The target HAL... passes stack contexts to asynchronous
 * discovery/read/write/descriptor callbacks. Timeout paths delete the semaphore
 * and return while a late NimBLE callback can still write the expired stack or
 * signal the deleted semaphore." Discover/read/write/
 * descriptor-discovery/characteristic-bound below used to pass `&ctx` (a
 * STACK-LOCAL struct) directly as the NimBLE callback's own `void *arg` --
 * unlike esp32_ble_scan/esp32_gatt_connect above (already safe: a static
 * `s_scan_ctx`/`s_conn_ctx` global pointer, set NULL before the semaphore is
 * deleted, checked by the callback before touching anything), NimBLE retains
 * that raw `arg` pointer internally and can still invoke the callback with it at
 * any later time -- including after a timeout, once this function has already
 * returned and `ctx`'s stack memory has been reused by something else entirely.
 * Every one of these five operations is now backed by a STATIC context plus the
 * identical global-pointer-nulled-before-delete pattern already proven safe
 * above -- correct because this whole HAL is a strictly sequential, single-
 * outstanding-blocking-call-at-a-time design (mtek_ble_logic.c never issues a
 * second GATT operation before the first one's blocking call returns), so one
 * shared static per operation type is never concurrently reused by two in-flight
 * calls. */
/* `done_status` records the FINAL ble_gatt_error.status this callback ever
 * observed (never overwritten by an earlier, per-result status of 0) -- NimBLE's
 * own documented convention for a "discover all..." procedure is a callback
 * invocation per real result (status BLE_HS_ EOK/0) followed by exactly one
 * final invocation carrying either BLE_HS_ EDONE (the procedure completed
 * normally, count is a real, honest total -- possibly genuinely 0) or a real,
 * different nonzero status (the procedure failed mid-flight, e.g. a disconnect
 * or ATT error -- whatever was collected before that point must never be
 * reported as a clean result). Distinguishing these three (still 0/not-yet-done,
 * EDONE, real error) is exactly what every discovery function below now checks
 * after its own semaphore wait, instead of trusting a plain "the wait returned"
 * signal to mean "this was a clean completion". */
typedef struct { mtk_hal_gatt_service_t *out; unsigned max, count; SemaphoreHandle_t done; int done_status; } disc_ctx_t;
static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg) {
    (void)conn_handle;
    /* See mtk_ble_op_lifecycle.h -- the whole body runs under the
     * lifetime lock (callback_begin), so a concurrent timeout retire cannot free
     * ctx mid-callback. */
    disc_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (!c) { mtk_ble_op_lifecycle_callback_end(&s_op_lc); return 0; }
    if (service && error->status == 0 && c->count < c->max) {
        mtk_hal_gatt_service_t *o = &c->out[c->count];
        /* A 16-bit UUID only ever writes uuid_value[0..1] below, but the
         * canonical schema always serializes all 16 bytes of uuid.value onto the
         * wire regardless of width -- zero the whole field first so bytes
         * [2..15] are a defined, honest 0, never whatever (possibly
         * uninitialized, possibly a PRIOR service record's own 128-bit UUID
         * tail) happened to occupy this slot before. */
        memset(o->uuid_value, 0, sizeof(o->uuid_value));
        if (service->uuid.u.type == BLE_UUID_TYPE_16) {
            o->uuid_width = 0;
            o->uuid_value[0] = (uint8_t)service->uuid.u16.value;
            o->uuid_value[1] = (uint8_t)(service->uuid.u16.value >> 8);
        } else {
            o->uuid_width = 1;
            memcpy(o->uuid_value, service->uuid.u128.value, 16);
        }
        o->start_handle = service->start_handle;
        o->end_handle = service->end_handle;
        c->count++;
    }
    if (error->status != 0) { c->done_status = error->status; if (c->done) xSemaphoreGive(c->done); }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static int esp32_gatt_discover(uint16_t vendor_handle, mtk_hal_gatt_service_t *out, unsigned max_out) {
    static disc_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.out = out; ctx.max = max_out; ctx.done = xSemaphoreCreateBinary();
    /* -1 is a real, distinct failure, never conflated with a genuine
     * zero-service result (0 already means "nothing discovered"). */
    if (!ctx.done) return -1;
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int start_rc = ble_gattc_disc_all_svcs(vendor_handle, disc_svc_cb, (void *)(uintptr_t)gen);
    /* An immediate, nonzero return from the NimBLE "start" call itself means
     * disc_svc_cb was NEVER armed at all -- no callback will ever fire, so
     * waiting on ctx.done would either hang for the full timeout or (worse, if a
     * PRIOR operation's callback happened to still be pending under a stale
     * generation) never genuinely correspond to this attempt. Fail immediately
     * and honestly instead of falling through to report ctx.count (always 0
     * here) as if this were a real, empty discovery. */
    /* The semaphore wait's own return value is now checked too -- a timeout
     * (pdFALSE) means the procedure never genuinely completed (no BLE_HS_EDONE,
     * no real error either -- NimBLE simply never got there in time).
     * mtek_ble_disc_failed (mtek_ble_disc_status.h) is the single, shared
     * classification this file's five discovery call sites all use -- see its
     * own doc comment for the full rationale and
     * host_tests/test_ble_disc_status.c for its portable proof. */
    int semaphore_signaled = 0;
    if (start_rc == 0) semaphore_signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(5000)) == pdTRUE);
    int failed = mtek_ble_disc_failed(start_rc, semaphore_signaled, ctx.done_status, BLE_HS_EDONE);
    /* Retire quiesces every future callback (there is no supported
     * per-procedure cancel for a GATT "discover all..." in NimBLE -- it
     * completes or the link drops) and only then is the semaphore deleted / ctx
     * allowed to leave scope. */
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    return failed ? -1 : (int)ctx.count;
}

/* Full-result-set discovery, companion to disc_svc_cb above (which only ever
 * enumerated services). Unlike char_bound_cb/dsc_cb below (which narrow their
 * own internal CCCD search to a single match), these two collect every real
 * characteristic/descriptor NimBLE reports in the requested range, up to max_out
 * -- the canonical GATT_DISCOVER_CHARS/GATT_DISCOVER_DESCS opcodes (new, purely
 * additive; see schemas.json) surface these results over the wire for the UART
 * `connect`/`services` commands. */
typedef struct { mtk_hal_gatt_char_t *out; unsigned max, count; SemaphoreHandle_t done; int done_status; } disc_chr_ctx_t;
static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    (void)conn_handle;
    /* See mtk_ble_op_lifecycle.h. */
    disc_chr_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (!c) { mtk_ble_op_lifecycle_callback_end(&s_op_lc); return 0; }
    if (chr && error->status == 0 && c->count < c->max) {
        mtk_hal_gatt_char_t *o = &c->out[c->count];
        memset(o->uuid_value, 0, sizeof(o->uuid_value));
        if (chr->uuid.u.type == BLE_UUID_TYPE_16) {
            o->uuid_width = 0;
            o->uuid_value[0] = (uint8_t)chr->uuid.u16.value;
            o->uuid_value[1] = (uint8_t)(chr->uuid.u16.value >> 8);
        } else {
            o->uuid_width = 1;
            memcpy(o->uuid_value, chr->uuid.u128.value, 16);
        }
        o->def_handle = chr->def_handle;
        o->val_handle = chr->val_handle;
        o->properties = chr->properties;
        c->count++;
    }
    if (error->status != 0) { c->done_status = error->status; if (c->done) xSemaphoreGive(c->done); }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static int esp32_gatt_discover_chars(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                      mtk_hal_gatt_char_t *out, unsigned max_out) {
    if (end_handle < start_handle) return 0;
    static disc_chr_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.out = out; ctx.max = max_out; ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) return -1; /* See esp32_gatt_discover's own doc comment: -1 is
                               * a real, distinct failure, never conflated with a
                               * genuine zero-characteristic result */
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int start_rc = ble_gattc_disc_all_chrs(vendor_handle, start_handle, end_handle, disc_chr_cb, (void *)(uintptr_t)gen);
    /* See esp32_gatt_discover's own doc comment for the full rationale --
     * mtek_ble_disc_failed (mtek_ble_disc_status.h) is the single, shared
     * classification. */
    int semaphore_signaled = 0;
    if (start_rc == 0) semaphore_signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(5000)) == pdTRUE);
    int failed = mtek_ble_disc_failed(start_rc, semaphore_signaled, ctx.done_status, BLE_HS_EDONE);
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    return failed ? -1 : (int)ctx.count;
}

typedef struct { mtk_hal_gatt_desc_t *out; unsigned max, count; SemaphoreHandle_t done; int done_status; } disc_dsc_ctx_t;
static int disc_dsc_full_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc, void *arg) {
    (void)conn_handle; (void)chr_val_handle;
    /* See mtk_ble_op_lifecycle.h. */
    disc_dsc_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (!c) { mtk_ble_op_lifecycle_callback_end(&s_op_lc); return 0; }
    if (dsc && error->status == 0 && c->count < c->max) {
        mtk_hal_gatt_desc_t *o = &c->out[c->count];
        memset(o->uuid_value, 0, sizeof(o->uuid_value));
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16) {
            o->uuid_width = 0;
            o->uuid_value[0] = (uint8_t)dsc->uuid.u16.value;
            o->uuid_value[1] = (uint8_t)(dsc->uuid.u16.value >> 8);
        } else {
            o->uuid_width = 1;
            memcpy(o->uuid_value, dsc->uuid.u128.value, 16);
        }
        o->handle = dsc->handle;
        c->count++;
    }
    if (error->status != 0) { c->done_status = error->status; if (c->done) xSemaphoreGive(c->done); }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static int esp32_gatt_discover_descs(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                      mtk_hal_gatt_desc_t *out, unsigned max_out) {
    if (end_handle < start_handle) return 0;
    static disc_dsc_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.out = out; ctx.max = max_out; ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) return -1; /* See esp32_gatt_discover's own doc comment: -1 is
                               * a real, distinct failure, never conflated with a
                               * genuine zero-descriptor result */
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int start_rc = ble_gattc_disc_all_dscs(vendor_handle, start_handle, end_handle, disc_dsc_full_cb, (void *)(uintptr_t)gen);
    /* See esp32_gatt_discover's own doc comment for the full rationale. */
    int semaphore_signaled = 0;
    if (start_rc == 0) semaphore_signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(5000)) == pdTRUE);
    int failed = mtek_ble_disc_failed(start_rc, semaphore_signaled, ctx.done_status, BLE_HS_EDONE);
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    return failed ? -1 : (int)ctx.count;
}

typedef struct { uint8_t *out; uint16_t max, len; int rc; SemaphoreHandle_t done; } read_ctx_t;
static int read_attr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
    (void)conn_handle;
    /* See mtk_ble_op_lifecycle.h. */
    read_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (c) {
        c->rc = error->status;
        if (error->status == 0 && attr && attr->om) {
            uint16_t n = OS_MBUF_PKTLEN(attr->om);
            if (n > c->max) n = c->max;
            ble_hs_mbuf_to_flat(attr->om, c->out, n, &n);
            c->len = n;
        }
        if (c->done) xSemaphoreGive(c->done);
    }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static int esp32_gatt_read(uint16_t vendor_handle, uint16_t attr_handle, uint8_t *out, uint16_t max_len, uint16_t *len_out) {
    static read_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.out = out; ctx.max = max_len; ctx.rc = -1; ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) { *len_out = 0; return -1; } /* Item 4 -- see esp32_ble_scan's own doc
                                                 * comment */
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int rc = ble_gattc_read(vendor_handle, attr_handle, read_attr_cb, (void *)(uintptr_t)gen);
    if (rc == 0) xSemaphoreTake(ctx.done, pdMS_TO_TICKS(3000));
    /* No supported per-procedure cancel for a GATT read in NimBLE -- the
     * lifetime retire quiesces any late callback before ctx/semaphore teardown
     * (see mtk_ble_op_lifecycle.h). */
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    *len_out = ctx.len;
    return ctx.rc == 0 ? 0 : -1;
}

typedef struct { int rc; SemaphoreHandle_t done; } write_ctx_t;
static int write_attr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
    (void)conn_handle; (void)attr;
    /* See mtk_ble_op_lifecycle.h. */
    write_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (c) {
        c->rc = error->status;
        if (c->done) xSemaphoreGive(c->done);
    }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static int esp32_gatt_write(uint16_t vendor_handle, uint16_t attr_handle, const uint8_t *data, uint16_t len, uint8_t with_response) {
    if (!with_response) return ble_gattc_write_no_rsp_flat(vendor_handle, attr_handle, data, len) == 0 ? 0 : -1;
    static write_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.rc = -1; ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) return -1; /* Item 4 -- see esp32_ble_scan's own doc comment */
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int rc = ble_gattc_write_flat(vendor_handle, attr_handle, data, len, write_attr_cb, (void *)(uintptr_t)gen);
    if (rc == 0) xSemaphoreTake(ctx.done, pdMS_TO_TICKS(3000));
    /* No supported per-procedure cancel for a GATT write in NimBLE -- the
     * lifetime retire quiesces any late callback before ctx/semaphore teardown
     * (see mtk_ble_op_lifecycle.h). */
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    return ctx.rc == 0 ? 0 : -1;
}

/* CCCD discovery: runs a real "Discover All Characteristic Descriptors"
 * procedure over the exact range (attr_handle+1.. search_end) the caller
 * supplies. Looks for the standard Client Characteristic Configuration
 * descriptor (UUID16 0x2902, Bluetooth SIG-assigned). If no 0x2902 descriptor is
 * found within that exact, real range, this fails cleanly (returns 0) -- no
 * attr_handle+1 or any other fallback guess. */
typedef struct { uint16_t cccd_handle; int found; SemaphoreHandle_t done; int done_status; } dsc_ctx_t;
static int dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                   const struct ble_gatt_dsc *dsc, void *arg) {
    (void)conn_handle; (void)chr_val_handle;
    /* See mtk_ble_op_lifecycle.h. */
    dsc_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (c) {
        if (dsc && error->status == 0 && dsc->uuid.u.type == BLE_UUID_TYPE_16 && dsc->uuid.u16.value == 0x2902) {
            c->cccd_handle = dsc->handle;
            c->found = 1;
        }
        if (error->status != 0) { c->done_status = error->status; if (c->done) xSemaphoreGive(c->done); }
    }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
/* Distinct from this function's own existing 0 return ("no 0x2902 descriptor was
 * found in a real search that actually ran") -- a real GATT attribute handle on
 * this HAL's own small, custom services never reaches 0xFFFF, so it is available
 * as an unambiguous "this was never actually searched for at all" / "this search
 * did not genuinely complete" signal, now covering allocation failure AND an
 * immediate NimBLE start failure, a semaphore timeout, or a real
 * callback-reported error -- none of these mean "genuinely no CCCD found"
 * (MTK_STATUS_NOT_FOUND, blaming the peer's own GATT topology); they mean this
 * search itself could not be trusted, and esp32_gatt_ subscribe/unsubscribe
 * below must report an honest IO failure instead. */
#define MTK_GATT_CCCD_ALLOC_FAILED 0xFFFF
static uint16_t find_cccd_handle_in_range(uint16_t vendor_handle, uint16_t search_start, uint16_t search_end) {
    if (search_end < search_start) return 0;
    static dsc_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) return MTK_GATT_CCCD_ALLOC_FAILED;
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int start_rc = ble_gattc_disc_all_dscs(vendor_handle, search_start, search_end, dsc_cb, (void *)(uintptr_t)gen);
    /* See esp32_gatt_discover's own doc comment for the full rationale -- an
     * immediate start failure, a semaphore timeout, or a real callback-reported
     * error must never be reported the same way as a genuine, complete search
     * that simply found no 0x2902 descriptor. */
    int semaphore_signaled = 0;
    if (start_rc == 0) {
        semaphore_signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(3000)) == pdTRUE);
    }
    int failed = mtek_ble_disc_failed(start_rc, semaphore_signaled, ctx.done_status, BLE_HS_EDONE);
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    if (failed) return MTK_GATT_CCCD_ALLOC_FAILED;
    return ctx.found ? ctx.cccd_handle : 0;
}

/* The previous search range was (attr_handle, service_end_handle] -- correct
 * only for a characteristic's own descriptors when it happens to be the LAST one
 * in its service; for any earlier characteristic, that range also covers every
 * LATER characteristic's own declaration/value/descriptors, including their own
 * 0x2902 CCCD if they have one, and ble_gattc_disc_all_dscs (like the real GATT
 * "Discover All Characteristic Descriptors" procedure it wraps) would happily
 * return the FIRST 0x2902 found in that whole range -- possibly belonging to a
 * characteristic the caller never asked about. Bounds the search instead to
 * `(attr_handle, next_characteristic_declaration_handle - 1]`: discovers every
 * characteristic in (attr_handle, service_end_handle] via a real "Discover All
 * Characteristics" procedure, finds the SMALLEST declaration handle greater than
 * attr_handle (the next characteristic in the service, if any), and uses one
 * less than that as the new ceiling -- falling back to the unmodified
 * service_end_handle only when attr_handle's own characteristic is genuinely the
 * last one in the service (no narrower bound exists). This remains an internal
 * correctness fix only for SUBSCRIBE/UNSUBSCRIBE's own CCCD search --
 * GATT_DISCOVER itself still stays services-only. (update: full
 * characteristic/descriptor enumeration IS now separately exposed over the wire,
 * via the new, purely additive GATT_DISCOVER_CHARS/GATT_DISCOVER_DESCS opcodes
 * and disc_chr_cb/ disc_dsc_full_cb above -- this bound_to_own_characteristic
 * call and its dedicated char_bound_cb remain their own narrow, internal-only
 * search, unrelated to those wire opcodes.) */
typedef struct { uint16_t next_char_def_handle; SemaphoreHandle_t done; int done_status; } char_bound_ctx_t;
static int char_bound_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    (void)conn_handle;
    /* See mtk_ble_op_lifecycle.h. */
    char_bound_ctx_t *c = NULL;
    if (!mtk_ble_op_lifecycle_callback_begin(&s_op_lc, (uint32_t)(uintptr_t)arg, (void **)&c)) return 0;
    if (c) {
        if (chr && error->status == 0) {
            if (c->next_char_def_handle == 0 || chr->def_handle < c->next_char_def_handle) {
                c->next_char_def_handle = chr->def_handle;
            }
        }
        if (error->status != 0) { c->done_status = error->status; if (c->done) xSemaphoreGive(c->done); }
    }
    mtk_ble_op_lifecycle_callback_end(&s_op_lc);
    return 0;
}
static uint16_t bound_to_own_characteristic(uint16_t vendor_handle, uint16_t attr_handle, uint16_t service_end_handle) {
    if (service_end_handle <= attr_handle) return service_end_handle;
    static char_bound_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.done = xSemaphoreCreateBinary();
    /* service_end_ handle is this function's own legitimate "no narrower bound
     * found" fallback -- but that fallback means "a real Discover All
     * Characteristics procedure ran and genuinely completed, finding none
     * narrower", not "this was never actually attempted" (allocation failure) or
     * "this could not be trusted to have actually completed" (an immediate start
     * failure, a semaphore timeout, or a real callback-reported error). Reusing
     * service_end_ handle for any of those is precisely "broaden a CCCD search
     * as a fallback": the caller's own search would silently widen to the FULL,
     * unnarrowed service range instead of failing. 0 is never a legitimate
     * return from this function otherwise (service_end_handle/
     * next_char_def_handle-1 are always real, nonzero GATT handles), so it is
     * available as an unambiguous, distinct "this bound cannot be trusted"
     * signal for all four cases. */
    if (!ctx.done) return 0;
    uint32_t gen = mtk_ble_op_lifecycle_arm(&s_op_lc, &ctx);
    int start_rc = ble_gattc_disc_all_chrs(vendor_handle, (uint16_t)(attr_handle + 1), service_end_handle, char_bound_cb, (void *)(uintptr_t)gen);
    int semaphore_signaled = 0;
    if (start_rc == 0) semaphore_signaled = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(3000)) == pdTRUE);
    int failed = mtek_ble_disc_failed(start_rc, semaphore_signaled, ctx.done_status, BLE_HS_EDONE);
    mtk_ble_op_lifecycle_retire(&s_op_lc);
    vSemaphoreDelete(ctx.done);
    if (failed) return 0;
    if (ctx.next_char_def_handle != 0 && ctx.next_char_def_handle > attr_handle) {
        return (uint16_t)(ctx.next_char_def_handle - 1);
    }
    return service_end_handle; /* no later characteristic found -- attr_handle's own char is last in the service */
}

/* `end_handle`: the real containing SERVICE's end_handle from an actual
 * prior GATT_DISCOVER call (mtek_ble_logic.c tracks and supplies this --
 * never a guessed window). Narrowed via bound_to_own_characteristic
 * before the CCCD search itself, per this file's own doc comment above. */
static int esp32_gatt_subscribe(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle, uint8_t mode) {
    uint16_t search_end = bound_to_own_characteristic(vendor_handle, attr_handle, end_handle);
    /* A resource failure while narrowing the search bound must never fall
     * through into a search over the full, unnarrowed service range -- see
     * bound_to_own_ characteristic's own doc comment. Reported as a plain IO
     * failure (-1), distinct from MTK_HAL_GATT_SUBSCRIBE_NO_CCCD (-2): this is
     * not a claim about the peer's own GATT topology. */
    if (!search_end) return -1;
    uint16_t cccd = find_cccd_handle_in_range(vendor_handle, (uint16_t)(attr_handle + 1), search_end);
    /* See find_cccd_handle_in_range's own doc comment -- an allocation failure
     * is never conflated with a genuine "no CCCD found" here either. */
    if (cccd == MTK_GATT_CCCD_ALLOC_FAILED) return -1;
    /* A distinct return value for "no CCCD found" -- never conflated with an
     * ordinary write failure -- so the canonical service (mtek_ble_logic.c) and,
     * from there, the UART adapter can surface the accepted baseline's own
     * distinct error text instead of a generic one. */
    if (!cccd) return MTK_HAL_GATT_SUBSCRIBE_NO_CCCD; /* 0x2902 not found in the real characteristic range -- fail cleanly, no fallback */
    uint8_t val[2] = { (uint8_t)(mode == 1 ? 2 : 1), 0 };
    return esp32_gatt_write(vendor_handle, cccd, val, 2, 1);
}
static int esp32_gatt_unsubscribe(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle) {
    uint16_t search_end = bound_to_own_characteristic(vendor_handle, attr_handle, end_handle);
    /* See esp32_gatt_subscribe's own doc comment immediately above. */
    if (!search_end) return -1;
    uint16_t cccd = find_cccd_handle_in_range(vendor_handle, (uint16_t)(attr_handle + 1), search_end);
    if (cccd == MTK_GATT_CCCD_ALLOC_FAILED) return -1;
    if (!cccd) return -1;
    uint8_t val[2] = {0, 0};
    return esp32_gatt_write(vendor_handle, cccd, val, 2, 1);
}

static int esp32_gatt_poll_notify(uint16_t vendor_handle, uint16_t *attr_handle_out, uint8_t *data_out, uint16_t *len_out) {
    (void)vendor_handle;
    notify_lock();
    if (s_notify_count == 0) { notify_unlock(); return 0; }
    pending_notify_t n = s_notify_queue[s_notify_head];
    s_notify_head = (s_notify_head + 1) % NOTIFY_QUEUE_LEN;
    s_notify_count--;
    notify_unlock();
    *attr_handle_out = n.handle;
    memcpy(data_out, n.data, n.len);
    *len_out = n.len;
    return 1;
}

/* Real remote-disconnect visibility for the service layer -- see conn_gap_cb's
 * BLE_GAP_EVENT_DISCONNECT doc comment above. Returns 1 (and clears the pending
 * flag) exactly once per remote disconnect the service hasn't yet observed,
 * matching esp32_gatt_poll_notify's own poll-and-consume shape; `vendor_handle`
 * lets the service confirm this disconnect belongs to the connection it
 * currently thinks is active (a stale flag from an already-superseded connection
 * is otherwise possible in principle, though the arbiter's
 * single-active-GC-class model means at most one is ever really live). */
static int esp32_gatt_poll_disconnected(uint16_t vendor_handle, uint8_t *reason_out) {
    notify_lock();
    int hit = (s_disconnected_handle == vendor_handle);
    if (hit) {
        s_disconnected_handle = 0xFFFF;
        if (reason_out) *reason_out = s_disconnect_reason;
    }
    notify_unlock();
    return hit;
}

/* See s_notify_dropped_count's own doc comment above. */
static uint32_t esp32_gatt_notify_dropped_count(void) {
    notify_lock();
    uint32_t c = s_notify_dropped_count;
    notify_unlock();
    return c;
}

static const mtk_ble_hal_t s_hal_impl = {
    esp32_ble_scan, esp32_ble_adv_start, esp32_ble_adv_stop, esp32_signal_sample,
    esp32_gatt_connect, esp32_gatt_disconnect, esp32_gatt_discover,
    esp32_gatt_discover_chars, esp32_gatt_discover_descs, esp32_gatt_read,
    esp32_gatt_write, esp32_gatt_subscribe, esp32_gatt_unsubscribe, esp32_gatt_poll_notify,
    esp32_gatt_poll_disconnected, esp32_gatt_notify_dropped_count,
    esp32_live_discovery_start, esp32_live_snapshot, esp32_live_stop, esp32_live_stop,
};

const mtk_ble_hal_t *mtek_ble_hal_esp32_get(void) {
    ESP_LOGI(TAG, "ESP32-C6 BLE (NimBLE) HAL ready (host-tested; hardware behavior not validated)");
    return &s_hal_impl;
}

#endif /* CONFIG_BT_ENABLED -- controller compiled out of this build variant */
