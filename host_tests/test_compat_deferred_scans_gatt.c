/* RC12 final blocker correction, item 2 "real deferred Community execution":
 * DETERMINISTIC proof (forced condvar synchronization -- never timing-only
 * sleeps) that the Community/Mtek Compatibility adapter's two-part response/event state
 * machine (mtek_compat_dispatch.c: dispatch_async_with_event +
 * mtek_compat_dispatch_poll_outbound) is correct for AP_SCAN_START,
 * STA_SCAN_START, and GATT_CONNECT under a genuinely deferred async runner
 * (the ESP32 target's own configuration).
 *
 * The exact P0 this closes: a real worker emits its canonical ACCEPTED
 * response, then remains inside a slow HAL scan/connect, and only emits its
 * terminal event (AP/STA result_generation, GATT connection_token) SEVERAL
 * polls later. The pre-correction "arm the continuation only when no
 * response was seen" logic returned as soon as the ACCEPTED response drained
 * and silently DROPPED the later event -- losing the scan list / generation
 * / connection token.
 *
 * For each of the three features this proves ALL FIVE interleavings, each
 * forced deterministically (no reliance on which thread happens to be
 * scheduled first):
 *   (1) neither response nor event present when dispatch returns;
 *   (2) ACCEPTED response present but the terminal event still pending;
 *   (3) both response and event arriving together in one poll drain
 *       (continuation path);
 *   (4) both response and event already present in the initial drain
 *       (synchronous path -- never armed);
 *   (5) the initial response rejecting the operation, with no terminal event
 *       ever following (synchronous NAK).
 * And the externally visible result per feature:
 *   - AP scan returns the confirmed network-list bytes;
 *   - STA results-page works off the captured generation;
 *   - GATT disconnect reaches the fake HAL off the captured connection token;
 *   - no duplicate response, and no indefinite IDLE.
 *
 * Forcing mechanism: a custom async runner plus a gated HAL. The runner runs
 * the deferred handler on a real worker thread that blocks at a START gate
 * (so the main thread deterministically observes "nothing emitted yet" =
 * case 1). Each gated HAL scan/connect -- called by the handler AFTER it has
 * already emitted ACCEPTED and BEFORE it emits its terminal event -- signals
 * "in HAL" and blocks at a HAL gate (so the main thread deterministically
 * observes "ACCEPTED present, event pending" = case 2). Opening the HAL gate
 * lets the worker emit the real terminal event and finish. Cases 3/4/5 use
 * an inline / manual / failing runner variant with the gate inactive. No
 * unsolicited Community wire event or new opcode is introduced. */
#include "mtk_test.h"
#include "mtek_compat_dispatch.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_ble_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include "mtk_fake_ble_hal.h"
#include "mtek_codec_api.h"
#include "mtek_schema_message_descs.h"
#include "mtek_async_queue.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

/* ---- stack/core/router/service/HAL lock (single non-recursive mutex,
 * exactly as every other threaded host test wires it -- never nested). --- */
static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void v_lock(void) { pthread_mutex_lock(&s_mutex); }
static void v_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void q_lock(void *c) { (void)c; pthread_mutex_lock(&s_mutex); }
static void q_unlock(void *c) { (void)c; pthread_mutex_unlock(&s_mutex); }

/* ---- forced-synchronization gates (a SEPARATE mutex from the stack lock,
 * so a worker parked in the HAL gate never holds the stack lock the main
 * thread's poll needs). ------------------------------------------------- */
static pthread_mutex_t gmx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  gcv = PTHREAD_COND_INITIALIZER;
static int g_start_gate;   /* 0 = worker parked before running the handler */
static int g_hal_gate;     /* 0 = handler parked inside the HAL, event not yet emitted */
static int g_in_hal;       /* worker has reached the gated HAL (ACCEPTED already emitted) */
static int g_worker_done;  /* the deferred handler has fully returned (event emitted) */
static int g_gate_active;  /* 1 = the gated HAL actually parks; 0 = pass through */

static void gate_reset(void) {
    pthread_mutex_lock(&gmx);
    g_start_gate = 0; g_hal_gate = 0; g_in_hal = 0; g_worker_done = 0;
    pthread_mutex_unlock(&gmx);
}
static void gate_open_start(void) {
    pthread_mutex_lock(&gmx); g_start_gate = 1; pthread_cond_broadcast(&gcv); pthread_mutex_unlock(&gmx);
}
static void gate_wait_in_hal(void) {
    pthread_mutex_lock(&gmx); while (!g_in_hal) pthread_cond_wait(&gcv, &gmx); pthread_mutex_unlock(&gmx);
}
static void gate_open_hal(void) {
    pthread_mutex_lock(&gmx); g_hal_gate = 1; pthread_cond_broadcast(&gcv); pthread_mutex_unlock(&gmx);
}
static void gate_wait_done(void) {
    pthread_mutex_lock(&gmx); while (!g_worker_done) pthread_cond_wait(&gcv, &gmx); pthread_mutex_unlock(&gmx);
}
/* Called from inside a gated HAL scan/connect (i.e. after ACCEPTED, before
 * the terminal event): announce arrival, then block until released. */
static void hal_gate_park(void) {
    if (!g_gate_active) return;
    pthread_mutex_lock(&gmx);
    g_in_hal = 1; pthread_cond_broadcast(&gcv);
    while (!g_hal_gate) pthread_cond_wait(&gcv, &gmx);
    pthread_mutex_unlock(&gmx);
}

/* ---- gated HALs: copies of the fake HALs with only the blocking scan/
 * connect entry points wrapped, delegating to the originals after the
 * gate. ------------------------------------------------------------------ */
static mtk_wifi_hal_t g_gated_wifi;
static mtk_ble_hal_t  g_gated_ble;

static int gated_ap_scan(uint8_t band, uint8_t ch, uint32_t dur, mtk_hal_ap_record_t *out, unsigned max) {
    hal_gate_park();
    return g_fake_wifi_hal.ap_scan(band, ch, dur, out, max);
}
static int gated_sta_scan(mtk_hal_mac6_t bssid, uint8_t ch, uint16_t dur, mtk_hal_station_record_t *out, unsigned max) {
    hal_gate_park();
    return g_fake_wifi_hal.sta_scan(bssid, ch, dur, out, max);
}
static int gated_gatt_connect(mtk_hal_mac6_t addr, uint8_t addr_type, uint32_t to, uint16_t *vh) {
    hal_gate_park();
    return g_fake_ble_hal.gatt_connect(addr, addr_type, to, vh);
}

/* ---- runner variants (swapped per phase via mtk_router_set_async_runner) - */
static void (*g_pending_fn)(void *);
static void *g_pending_arg;
static int g_have_pending;

static void *thr_main(void *u) {
    (void)u;
    pthread_mutex_lock(&gmx);
    while (!g_start_gate) pthread_cond_wait(&gcv, &gmx);
    pthread_mutex_unlock(&gmx);
    g_pending_fn(g_pending_arg);                 /* async_trampoline -> real handler */
    pthread_mutex_lock(&gmx);
    g_worker_done = 1; pthread_cond_broadcast(&gcv);
    pthread_mutex_unlock(&gmx);
    return NULL;
}
static int threaded_runner(void (*fn)(void *), void *arg) {
    g_pending_fn = fn; g_pending_arg = arg;
    pthread_t th;
    if (pthread_create(&th, NULL, thr_main, NULL) != 0) return -1;
    pthread_detach(th);
    return 0;
}
static int inline_runner(void (*fn)(void *), void *arg) { fn(arg); return 0; }
static int manual_runner(void (*fn)(void *), void *arg) { g_pending_fn = fn; g_pending_arg = arg; g_have_pending = 1; return 0; }
static void run_pending(void) { MTK_CHECK(g_have_pending); g_have_pending = 0; g_pending_fn(g_pending_arg); }
static int failing_runner(void (*fn)(void *), void *arg) { (void)fn; (void)arg; return -1; }

/* Worker that runs the deferred handler immediately (no start gate) -- used
 * by blocking_runner below to force the exact original dispatch boundary:
 * the handler emits ACCEPTED and parks in the gated HAL (event still held)
 * BEFORE the runner returns, so dispatch_async_with_event's OWN initial
 * drain observes response-present / event-absent. */
static void *thr_main_nostart(void *u) {
    (void)u;
    g_pending_fn(g_pending_arg);
    pthread_mutex_lock(&gmx);
    g_worker_done = 1; pthread_cond_broadcast(&gcv);
    pthread_mutex_unlock(&gmx);
    return NULL;
}
static int blocking_runner(void (*fn)(void *), void *arg) {
    g_pending_fn = fn; g_pending_arg = arg;
    pthread_t th;
    if (pthread_create(&th, NULL, thr_main_nostart, NULL) != 0) return -1;
    pthread_detach(th);
    /* Block until the worker has emitted ACCEPTED and parked inside the
     * gated HAL (g_in_hal). Only then return, so the caller's own initial
     * drain sees exactly {response present, terminal event withheld}. */
    pthread_mutex_lock(&gmx);
    while (!g_in_hal) pthread_cond_wait(&gcv, &gmx);
    pthread_mutex_unlock(&gmx);
    return 0;
}

/* Legitimate queue seam: push a synthetic frame into the adapter's own
 * event queue (the same public API the router's sink uses), to drive the
 * poll_outbound continuation through interleavings that production ordering
 * never produces on its own (see the documented invariant in the test). */
static void push_event_generation(mtk_compat_dispatch_ctx_t *d, const char *name,
                                  const mtk_struct_desc_t *desc, const void *ev) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_EVENT;
    strncpy(f.event_name, name, MTK_ASYNC_EVENT_NAME_MAX - 1);
    size_t blen = 0;
    mtk_encode(desc, ev, f.body, sizeof(f.body), &blen);
    f.body_len = blen;
    MTK_CHECK(mtk_async_queue_push(&d->event_queue, &f) == 1); /* push returns 1 on success */
}
static void push_response_status(mtk_compat_dispatch_ctx_t *d, uint8_t status) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_RESPONSE;
    f.seq_or_status = status;
    MTK_CHECK(mtk_async_queue_push(&d->event_queue, &f) == 1); /* push returns 1 on success */
}

static mtk_compat_dispatch_ctx_t dctx;

static void make_req(mtk_compat_header_t *req, uint16_t msg_id, uint16_t plen) {
    memset(req, 0, sizeof(*req));
    req->magic = MTK_COMPAT_MAGIC; req->version = MTK_COMPAT_VERSION;
    req->msg_type = MTK_COMPAT_MSG_REQ; req->msg_id = msg_id; req->payload_len = plen;
}

/* ================================================================= */
MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_lock(v_lock, v_unlock);
    mtk_core_set_lock(v_lock, v_unlock);
    mtek_wifi_service_set_lock(v_lock, v_unlock);
    mtk_fake_wifi_set_lock(v_lock, v_unlock);
    mtk_fake_ble_set_lock(v_lock, v_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtek_ble_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtk_fake_ble_reset();

    /* Gated HALs = fake HALs with only the blocking entry points wrapped. */
    g_gated_wifi = g_fake_wifi_hal; g_gated_wifi.ap_scan = gated_ap_scan; g_gated_wifi.sta_scan = gated_sta_scan;
    g_gated_ble  = g_fake_ble_hal;  g_gated_ble.gatt_connect = gated_gatt_connect;
    mtek_wifi_set_hal(&g_gated_wifi);
    mtek_ble_set_hal(&g_gated_ble);

    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_ble_service_register();

    mtek_compat_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, q_lock, q_unlock, NULL);

    mtk_compat_header_t rh; uint8_t rp[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t rl = 0;
    mtk_compat_header_t req;

    /* Canned HAL results reused across every phase. */
    v_lock();
    g_fake_wifi.ap_count = 2;
    memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    memcpy(g_fake_wifi.ap_results[0].ssid, "HomeNet", 7); g_fake_wifi.ap_results[0].ssid_len = 7;
    g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -45; g_fake_wifi.ap_results[0].authmode = 3;
    memcpy(g_fake_wifi.ap_results[1].bssid.b, (uint8_t[]){0x11,0x22,0x33,0x44,0x55,0x66}, 6);
    memcpy(g_fake_wifi.ap_results[1].ssid, "OpenNet", 7); g_fake_wifi.ap_results[1].ssid_len = 7;
    g_fake_wifi.ap_results[1].channel = 1; g_fake_wifi.ap_results[1].rssi = -70; g_fake_wifi.ap_results[1].authmode = 0;
    g_fake_wifi.sta_count = 2;
    memcpy(g_fake_wifi.sta_results[0].mac.b, (uint8_t[]){0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, 6);
    g_fake_wifi.sta_results[0].rssi = -55;
    memcpy(g_fake_wifi.sta_results[1].mac.b, (uint8_t[]){0x1A,0x1B,0x1C,0x1D,0x1E,0x1F}, 6);
    g_fake_wifi.sta_results[1].rssi = -60;
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 77;
    v_unlock();

    const uint8_t AP_PL_NONE = 0; (void)AP_PL_NONE;
    uint8_t sta_pl[8] = { 0x02,0x02,0x03,0x04,0x05,0x06, 6 /*channel*/, 1 /*dur s*/ };
    uint8_t gatt_pl[7] = { 0x04,0x04,0x04,0x04,0x04,0x04, 0 /*addr_type*/ };

    /* Verifies the confirmed AP network-list bytes in a completed RESP. */
    #define VERIFY_AP_LIST(hdr, pl) do {                                     \
        MTK_CHECK_EQ((hdr).msg_type, MTK_COMPAT_MSG_RESP);                    \
        uint16_t _n = (uint16_t)((pl)[0] | ((pl)[1] << 8));                 \
        MTK_CHECK_EQ(_n, 2);                                                 \
        MTK_CHECK(memcmp((pl) + 2, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6) == 0); \
        MTK_CHECK_EQ((int8_t)(pl)[8], -45);                                 \
        MTK_CHECK_EQ((pl)[9], 6);                                            \
        MTK_CHECK_EQ((pl)[10], 3);                                           \
        MTK_CHECK_EQ((pl)[11], 7);                                           \
        MTK_CHECK(memcmp((pl) + 12, "HomeNet", 7) == 0);                     \
    } while (0)

    /* ===================================================================
     * Per-feature CASE 1 + CASE 2 + completion, forced with the threaded
     * runner + gated HAL. Walks: nothing emitted -> ACCEPTED only ->
     * terminal event -> confirmed reply.
     * =================================================================== */
    struct { const char *name; uint16_t msg; const uint8_t *pl; uint16_t plen; } FEAT[] = {
        { "AP_SCAN",     0x0103, NULL,    0 },
        { "STA_SCAN",    0x030E, sta_pl,  (uint16_t)sizeof(sta_pl) },
        { "GATT_CONNECT",0x0409, gatt_pl, (uint16_t)sizeof(gatt_pl) },
    };
    for (unsigned fi = 0; fi < 3; fi++) {
        gate_reset();
        g_gate_active = 1;
        mtk_router_set_async_runner(threaded_runner);

        /* -- CASE 1: neither response nor event present when dispatch returns. */
        make_req(&req, FEAT[fi].msg, FEAT[fi].plen);
        mtek_compat_dispatch_request(&dctx, &req, FEAT[fi].pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_have_response, 0);
        MTK_CHECK_EQ(dctx.pending_have_event, 0);

        /* -- CASE 2: worker emits ACCEPTED, parks in the HAL; event pending. */
        gate_open_start();
        gate_wait_in_hal();
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);          /* still IDLE, never a premature reply */
        MTK_CHECK_EQ(dctx.pending_have_response, 1);            /* ACCEPTED captured internally */
        MTK_CHECK_EQ(dctx.pending_have_event, 0);              /* terminal event genuinely not yet seen */
        MTK_CHECK_EQ(dctx.pending_start_msg_id, FEAT[fi].msg); /* continuation still owed */

        /* -- release the HAL: worker emits the terminal event and finishes. */
        gate_open_hal();
        gate_wait_done();
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);         /* confirmed reply, not fabricated earlier */
        MTK_CHECK_EQ(rh.msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);            /* continuation cleared */

        /* -- externally visible result + no indefinite IDLE / no duplicate. */
        if (fi == 0) {
            VERIFY_AP_LIST(rh, rp);
            MTK_CHECK(dctx.ap_scan_has_generation);
        } else if (fi == 1) {
            MTK_CHECK(dctx.sta_scan_has_generation);
            /* STA results page works off the captured generation. */
            make_req(&req, 0x030F, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
            uint16_t sn = (uint16_t)(rp[0] | (rp[1] << 8));
            MTK_CHECK_EQ(sn, 2);
            MTK_CHECK(memcmp(rp + 2, (uint8_t[]){0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, 6) == 0);
            MTK_CHECK_EQ((int8_t)rp[8], -55);
        } else {
            MTK_CHECK(dctx.gatt_conn_token != 0);
            /* GATT disconnect reaches the fake HAL off the captured token. */
            unsigned before = g_fake_ble.gatt_disconnect_call_count;
            make_req(&req, 0x040A, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
            MTK_CHECK_EQ(dctx.gatt_conn_token, 0);
            MTK_CHECK_EQ(g_fake_ble.gatt_disconnect_call_count, before + 1);
        }
        /* No duplicate response: the next poll is a clean IDLE. */
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);
    }

    /* ===================================================================
     * CASE 3: response AND terminal event arrive TOGETHER in one poll drain
     * (continuation path). Manual runner arms the continuation (nothing run
     * yet), then the handler is run to completion so BOTH frames sit in the
     * queue, and a SINGLE poll drains them together.
     * =================================================================== */
    g_gate_active = 0;              /* gated HAL passes straight through */
    mtk_router_set_async_runner(manual_runner);
    for (unsigned fi = 0; fi < 3; fi++) {
        make_req(&req, FEAT[fi].msg, FEAT[fi].plen);
        mtek_compat_dispatch_request(&dctx, &req, FEAT[fi].pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);         /* armed, nothing run yet (case-1 state) */
        MTK_CHECK_EQ(dctx.pending_have_response, 0);
        MTK_CHECK_EQ(dctx.pending_have_event, 0);

        run_pending();                                         /* emits ACCEPTED + terminal event into the queue */

        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl); /* one drain sees both */
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(rh.msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);
        if (fi == 0) VERIFY_AP_LIST(rh, rp);
        if (fi == 2) { /* reset the live connection for the next connect */
            make_req(&req, 0x040A, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
        }
    }

    /* ===================================================================
     * CASE 4: both response and event already present in the INITIAL drain
     * (synchronous path -- never deferred, never armed). Inline runner runs
     * the whole handler before dispatch_async_with_event drains.
     * =================================================================== */
    mtk_router_set_async_runner(inline_runner);
    for (unsigned fi = 0; fi < 3; fi++) {
        make_req(&req, FEAT[fi].msg, FEAT[fi].plen);
        mtek_compat_dispatch_request(&dctx, &req, FEAT[fi].pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);         /* completed synchronously, no IDLE */
        MTK_CHECK_EQ(rh.msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);            /* never armed */
        if (fi == 0) VERIFY_AP_LIST(rh, rp);
        if (fi == 2) {
            MTK_CHECK(dctx.gatt_conn_token != 0);
            make_req(&req, 0x040A, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
        }
    }

    /* ===================================================================
     * CASE 5: the initial response REJECTS the operation (runner cannot
     * start the worker) -- a synchronous NAK, with no terminal event ever
     * following and no continuation armed.
     * =================================================================== */
    mtk_router_set_async_runner(failing_runner);
    for (unsigned fi = 0; fi < 3; fi++) {
        make_req(&req, FEAT[fi].msg, FEAT[fi].plen);
        mtek_compat_dispatch_request(&dctx, &req, FEAT[fi].pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_NAK);          /* rejection relayed now, not awaited */
        MTK_CHECK_EQ(rh.msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);            /* nothing left pending */
        /* No indefinite IDLE / no phantom later reply: the next poll is IDLE. */
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);
    }

    /* ===================================================================
     * PHASE 6 (RC12 RC12 closure item 1): the EXACT original dispatch
     * boundary. blocking_runner returns only after the worker has emitted
     * ACCEPTED and parked in the gated HAL, so dispatch_async_with_event's
     * OWN INITIAL DRAIN sees {response present, terminal event withheld} --
     * it must preserve the response half, arm the continuation, and return
     * IDLE; a later poll then receives the terminal event and produces the
     * confirmed final Community response. This is the response-only initial-
     * drain path the earlier phases (worker gated shut until after dispatch)
     * did not exercise. Forced entirely by condvar handshakes.
     * =================================================================== */
    for (unsigned fi = 0; fi < 3; fi++) {
        gate_reset();
        g_gate_active = 1;
        mtk_router_set_async_runner(blocking_runner);

        make_req(&req, FEAT[fi].msg, FEAT[fi].plen);
        mtek_compat_dispatch_request(&dctx, &req, FEAT[fi].pl, &rh, rp, &rl);
        /* The initial drain saw ACCEPTED but not the event: */
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_have_response, 1);   /* response half preserved from the INITIAL drain */
        MTK_CHECK_EQ(dctx.pending_have_event, 0);      /* event deliberately still withheld */
        MTK_CHECK_EQ(dctx.pending_start_msg_id, FEAT[fi].msg);

        /* Release the HAL -> worker emits the terminal event and finishes. */
        gate_open_hal();
        gate_wait_done();
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(rh.msg_id, FEAT[fi].msg);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);

        if (fi == 0) {
            VERIFY_AP_LIST(rh, rp);
            MTK_CHECK(dctx.ap_scan_has_generation);
        } else if (fi == 1) {
            MTK_CHECK(dctx.sta_scan_has_generation);
            make_req(&req, 0x030F, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
            uint16_t sn = (uint16_t)(rp[0] | (rp[1] << 8));
            MTK_CHECK_EQ(sn, 2);
            MTK_CHECK(memcmp(rp + 2, (uint8_t[]){0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, 6) == 0);
            MTK_CHECK_EQ((int8_t)rp[8], -55);
        } else {
            MTK_CHECK(dctx.gatt_conn_token != 0);
            unsigned before = g_fake_ble.gatt_disconnect_call_count;
            make_req(&req, 0x040A, 0);
            mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
            MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);
            MTK_CHECK_EQ(dctx.gatt_conn_token, 0);
            MTK_CHECK_EQ(g_fake_ble.gatt_disconnect_call_count, before + 1);
        }
        /* Response emitted exactly once; adapter not stuck in IDLE. */
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);
    }

    /* ===================================================================
     * PHASE 7 (RC12 RC12 closure item 1, event-only preservation).
     *
     * Production ordering makes event-before-response IMPOSSIBLE: every
     * ACCEPTED_ASYNC handler calls respond(ACCEPTED) BEFORE the blocking HAL
     * call that precedes any terminal-event emit (e.g. mtek_wifi_logic.c
     * handle_ap_scan_start respond() then ap_scan() then AP_SCAN_COMPLETE;
     * mtek_ble_logic.c handle_gatt_connect respond() at :706 then
     * gatt_connect() then GATT_CONNECT_COMPLETE at :794), and the async
     * queue is priority-ordered so a drain seeing both returns the RESPONSE
     * first; dispatch_async_with_event also resets the queue before the
     * drain, so nothing can be pre-seeded into its own initial drain.
     * The state machine nonetheless handles an event arriving while the
     * response is still awaited. This proves that defensive branch of
     * poll_outbound via the legitimate public queue seam (mtk_async_queue_
     * push -- the same call the router's sink uses): the continuation is
     * armed with neither half, ONLY the terminal event is delivered, and it
     * must be carried into pending state (generation retained) while the
     * response is still awaited (IDLE); the later response then completes.
     * =================================================================== */
    {
        g_gate_active = 0;
        mtk_router_set_async_runner(manual_runner);
        make_req(&req, 0x0103, 0);
        mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);      /* armed, neither half present */
        MTK_CHECK_EQ(dctx.pending_have_response, 0);
        MTK_CHECK_EQ(dctx.pending_have_event, 0);

        /* Deliver ONLY the terminal event (response not yet emitted). */
        mtk_ap_scan_complete_ev_t ev; memset(&ev, 0, sizeof(ev)); ev.result_generation = 0xABCDu;
        push_event_generation(&dctx, "AP_SCAN_COMPLETE", &mtk_ap_scan_complete_ev_t_desc, &ev);
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);      /* event carried, response still awaited */
        MTK_CHECK_EQ(dctx.pending_have_event, 1);           /* already-captured event preserved in pending state */
        MTK_CHECK_EQ(dctx.pending_event_generation, 0xABCDu);
        MTK_CHECK_EQ(dctx.pending_have_response, 0);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0103);    /* continuation still owed */

        /* Now the response arrives -> completion, using the carried event. */
        push_response_status(&dctx, MTK_STATUS_ACCEPTED);
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_RESP);      /* emitted once, not stuck in IDLE */
        MTK_CHECK_EQ(rh.msg_id, 0x0103);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);
        /* No real AP scan ran (synthetic generation 0xABCD has no results
         * page), so the confirmed list is a well-formed empty (count=0) --
         * the point here is the carry-forward, not the bytes. */
        MTK_CHECK_EQ((uint16_t)(rp[0] | (rp[1] << 8)), 0);
        run_pending(); /* release the held async slot (real handler self-completes; stale frames wiped by the next reset) */
    }

    /* ===================================================================
     * PHASE 8 (RC12 RC12 closure item 2): honest GATT terminal-failure.
     * A failed connect (HAL rc != 0 -> terminal GATT_CONNECT_COMPLETE status
     * TIMEOUT -- mtek_ble_logic.c handle_gatt_connect's only failure status)
     * must NAK, store NO token, and leave a later GATT_DISCONNECT unable to
     * reach the HAL. Proven equivalent on the deferred and synchronous
     * paths, plus one acceptance-level failure (NO_MEMORY).
     * =================================================================== */
    v_lock(); g_fake_ble.gatt_connect_rc = -1; v_unlock(); /* connect fails -> terminal TIMEOUT */

    /* -- 8a: DEFERRED terminal TIMEOUT (case1 -> case2 -> terminal failure). */
    {
        gate_reset(); g_gate_active = 1;
        mtk_router_set_async_runner(threaded_runner);
        unsigned disc_before = g_fake_ble.gatt_disconnect_call_count;

        make_req(&req, 0x0409, sizeof(gatt_pl));
        mtek_compat_dispatch_request(&dctx, &req, gatt_pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);      /* deferred (case 1) */

        gate_open_start();
        gate_wait_in_hal();
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);      /* ACCEPTED present, terminal pending (case 2) */
        MTK_CHECK_EQ(dctx.pending_have_response, 1);

        gate_open_hal();
        gate_wait_done();
        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_NAK);       /* terminal failure -> NAK, not a false OK */
        MTK_CHECK_EQ(rp[0], MTK_COMPAT_STATUS_ERR_TIMEOUT);  /* existing canonical->Community mapping */
        MTK_CHECK_EQ(dctx.gatt_conn_token, 0);              /* no connection token retained */
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);         /* continuation cleared */

        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);      /* no duplicate / no indefinite IDLE */

        make_req(&req, 0x040A, 0);                          /* GATT_DISCONNECT */
        mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
        MTK_CHECK_EQ(g_fake_ble.gatt_disconnect_call_count, disc_before); /* HAL NOT reached (no connection) */
    }

    /* -- 8b: SYNCHRONOUS terminal TIMEOUT -- equivalent outcome/same HAL. */
    {
        g_gate_active = 0;
        mtk_router_set_async_runner(inline_runner);
        unsigned disc_before = g_fake_ble.gatt_disconnect_call_count;

        make_req(&req, 0x0409, sizeof(gatt_pl));
        mtek_compat_dispatch_request(&dctx, &req, gatt_pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_NAK);       /* same NAK as the deferred path */
        MTK_CHECK_EQ(rp[0], MTK_COMPAT_STATUS_ERR_TIMEOUT);
        MTK_CHECK_EQ(dctx.gatt_conn_token, 0);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);

        make_req(&req, 0x040A, 0);
        mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
        MTK_CHECK_EQ(g_fake_ble.gatt_disconnect_call_count, disc_before);
    }

    /* -- 8c: one other available failure result -- acceptance-level rejection
     * (NO_MEMORY): the terminal event only ever carries OK or TIMEOUT, so the
     * remaining GATT failures are acceptance-level and map to their own NAK. */
    {
        mtk_router_set_async_runner(failing_runner);
        unsigned disc_before = g_fake_ble.gatt_disconnect_call_count;

        make_req(&req, 0x0409, sizeof(gatt_pl));
        mtek_compat_dispatch_request(&dctx, &req, gatt_pl, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_NAK);
        MTK_CHECK_EQ(rp[0], MTK_COMPAT_STATUS_ERR_NO_MEM);
        MTK_CHECK_EQ(dctx.gatt_conn_token, 0);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0);

        mtek_compat_dispatch_poll_outbound(&dctx, &rh, rp, &rl);
        MTK_CHECK_EQ(rh.msg_type, MTK_COMPAT_MSG_IDLE);

        make_req(&req, 0x040A, 0);
        mtek_compat_dispatch_request(&dctx, &req, NULL, &rh, rp, &rl);
        MTK_CHECK_EQ(g_fake_ble.gatt_disconnect_call_count, disc_before);
    }

    v_lock(); g_fake_ble.gatt_connect_rc = 0; v_unlock(); /* restore for hygiene */

MTK_TEST_MAIN_END
