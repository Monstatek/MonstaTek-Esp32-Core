/* ROUND 4 (follow-up read-only audit, "final focused concurrency-correction
 * round"): round 3's own peer- session invalidation fenced
 * STA_CONNECT/BLE_SCAN/GATT_CONNECT's own op-table state, but a further
 * read-only re-audit found the arbiter itself was still released too early for
 * these three uncancellable blocking HAL operations -- the peer-reset canceller
 * released the radio-arbiter lease the instant it won the op-table transition,
 * even though the OLD blocking HAL call (connect/scan/gatt_connect, none of
 * which have a cancel hook) could still genuinely be running on another thread.
 * That let a brand-new operation of the SAME class acquire the arbiter and start
 * a SECOND, physically overlapping HAL call against the same radio hardware
 * while the first one was still in flight -- a real, unsafe overlap, not merely
 * a stale/duplicate response.
 *
 * The fix (mtek_wifi_logic.c/mtek_ble_logic.c, see their own doc comments): the
 * peer-reset canceller now ONLY fences the operation's own TOKEN for these
 * classes (WMC/BS/GC), never releasing the arbiter itself. The arbiter stays
 * genuinely HELD (so any replacement request of the same class is correctly
 * refused BUSY) until the ORIGINAL worker's own tail -- once its real blocking
 * HAL call actually returns -- performs a token-ownership-checked release
 * (mtk_arbiter_active_token == id.token), independent of whether it also won the
 * op-table state- publish race. AP/STA scan's own bounded quiescence-wait
 * canceller additionally now force-finalizes the TOKEN (never the arbiter) if
 * its own 1-second wait times out, so the token never lingers queryable forever
 * even when the radio genuinely stays busy longer than that.
 *
 * This file proves, with REAL concurrent pthread workers wherever the production
 * HAL call is genuinely blocking (BLE_SCAN, GATT_CONNECT, AP_SCAN), the exact
 * scenarios the re-audit demanded: A. Blocked BLE scan -> reset -> immediate
 * replacement BLE operation: refused BUSY while genuinely in flight, no stale
 * BLE_DEVICE_FOUND/ BLE_SCAN_COMPLETE for the invalidated token, arbiter
 * released exactly once by the real worker's own tail, replacement then succeeds
 * normally. B. Real concurrently blocked GATT connect -> reset -> immediate
 * replacement GATT connect: same shape, plus the stale worker's own lost-race
 * teardown (gatt_disconnect on ITS OWN local vendor handle) is proven to fire
 * without ever touching a newer connection. C. Forced AP/STA scan cancellation
 * timeout followed by a new-session request: the fake HAL's own cancel signal is
 * deliberately ignored (simulating a real HAL that does not honor cancellation
 * promptly), forcing the peer-reset canceller's bounded quiescence wait to
 * genuinely time out; the token is immediately force-finalized/ evictable, an
 * immediate replacement is still correctly refused BUSY (the arbiter was
 * deliberately left held), and only the real worker's own eventual completion
 * frees it, with no stale event ever emitted for the invalidated token.
 *
 * "Blocked STA connect -> reset -> immediate replacement STA operation" (the
 * fourth adversarial scenario the re-audit asked for) is already proven with a
 * real concurrent worker in test_peer_session_invalidation_ services.c's own
 * section 2 (extended in this same round to add the
 * immediate-replacement-refused-BUSY proof) -- not duplicated here. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }

/* A SEPARATE mutex for mtk_op_set_publish_lock, distinct from s_mutex above --
 * see mtek_core.h's own doc comment on mtk_op_begin_publish_ guard for why it
 * must never share a lock with anything a sink's own emit call can reach. */
static pthread_mutex_t s_pub_mutex = PTHREAD_MUTEX_INITIALIZER;
static void pub_lock_fn(void) { pthread_mutex_lock(&s_pub_mutex); }
static void pub_unlock_fn(void) { pthread_mutex_unlock(&s_pub_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

static int sta_query_always_ready(void) { return 1; }

static uint32_t poll_for_op_token(mtk_fake_sink_state_t *sink, const mtk_opcode_entry_t *op) {
    (void)op;
    uint32_t token = 0;
    for (int i = 0; i < 3000 && token == 0; i++) {
        mtk_fake_sink_lock();
        int got = sink->response.set && sink->response.status == MTK_STATUS_ACCEPTED;
        struct { uint32_t operation_token; } r = {0};
        if (got) memcpy(&r, sink->response.body, sizeof(r) < sink->response.body_len ? sizeof(r) : sink->response.body_len);
        mtk_fake_sink_unlock();
        if (got) { token = r.operation_token; break; }
        usleep(500);
    }
    return token;
}

/* Waits (bounded) for a response to actually land, returning its status --
 * see test_peer_session_invalidation_services.c's own identical helper
 * for the full rationale (a deferred worker, not the calling thread,
 * writes this response; reading it without waiting races that worker and
 * can trip ASan's stack-use-after-scope detector on the caller's own
 * stack-local sink). Returns 0xFF (never a real wire status) on timeout. */
static uint8_t poll_for_response_status(mtk_fake_sink_state_t *sink) {
    for (int i = 0; i < 3000; i++) {
        mtk_fake_sink_lock();
        int got = sink->response.set;
        uint8_t status = sink->response.status;
        mtk_fake_sink_unlock();
        if (got) return status;
        usleep(500);
    }
    return 0xFFu;
}

static uint8_t op_status(uint32_t token) {
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GET_OPERATION_STATUS");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 999);
    mtk_get_operation_status_req_t req = {0}; req.operation_token = token;
    mtk_test_call(&ctx, status_op, &req);
    return sink.response.status;
}

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_arbiter_set_lock(router_lock, router_unlock);
    mtk_router_set_lock(router_lock, router_unlock);
    mtk_op_set_publish_lock(pub_lock_fn, pub_unlock_fn);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_ble_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);
    mtek_system_set_sta_query(sta_query_always_ready);
    mtk_router_set_async_runner(pthread_runner);

    const mtk_opcode_entry_t *ble_scan_start = mtk_test_find_op("BLE_SCAN_START");
    const mtk_opcode_entry_t *gatt_connect_op = mtk_test_find_op("GATT_CONNECT");
    const mtk_opcode_entry_t *gatt_status_op = mtk_test_find_op("GATT_STATUS");
    const mtk_opcode_entry_t *ap_start = mtk_test_find_op("AP_SCAN_START");
    MTK_CHECK(ble_scan_start && gatt_connect_op && gatt_status_op && ap_start);

    /* ==== A. Blocked BLE scan -> reset -> immediate replacement BLE
     * operation. =========================================================== */
    {
        mtk_fake_ble_reset();
        g_fake_ble.scan_delay_ms = 200; /* real, genuinely blocking */
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_ble_scan_start_req_t req; memset(&req, 0, sizeof(req));
        req.mode = 0; req.duration_ms = 100;
        mtk_test_call(&ctx, ble_scan_start, &req);
        uint32_t tok = poll_for_op_token(&sink, ble_scan_start);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);

        mtk_op_id_t cancelled = mtek_ble_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, tok);
        /* The arbiter is deliberately NOT released -- the old scan call has no
         * cancel hook and may still genuinely be running. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);
        mtk_op_evict_all_terminal();
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_NOT_FOUND); /* token evicted */

        /* Immediate replacement, while the old scan call is STILL genuinely
         * blocked, must be refused -- never admitted to start a second,
         * overlapping scan. */
        {
            mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
            mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 2);
            mtk_test_call(&busy_ctx, ble_scan_start, &req);
            MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);
        }

        /* Let the stale worker's scan call finally return. It must never publish
         * s_scan/emit BLE_DEVICE_FOUND/BLE_SCAN_COMPLETE for the invalidated
         * token, but it IS the one place that safely releases MTK_ARB_BS once
         * its own real call returns. */
        usleep(400000);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* released by the stale worker's own tail, via token ownership */
        mtk_fake_sink_lock();
        unsigned stale_events = sink.event_count;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stale_events, 0u); /* no BLE_DEVICE_FOUND/BLE_SCAN_COMPLETE emitted for the invalidated token */

        /* A NEW BLE_SCAN_START now completes normally end-to-end. */
        g_fake_ble.scan_delay_ms = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 3);
        mtk_test_call(&ctx2, ble_scan_start, &req);
        uint32_t tok2 = poll_for_op_token(&sink2, ble_scan_start);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK); /* the new operation's own scan genuinely completed */
    }

    /* ==== B. Real concurrently blocked GATT connect -> reset ->
     * immediate replacement GATT connect. ================================= */
    {
        mtk_fake_ble_reset();
        g_fake_ble.gatt_connect_delay_ms = 200; /* real, genuinely blocking */
        g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 0x2222;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 4);
        mtk_gatt_connect_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target.addr.b, 0xDD, 6);
        mtk_test_call(&ctx, gatt_connect_op, &req);
        uint32_t tok = poll_for_op_token(&sink, gatt_connect_op);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_GC);

        mtk_op_id_t cancelled = mtek_ble_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, tok);
        /* The arbiter is deliberately NOT released -- the old gatt_connect call
         * has no cancel hook and may still genuinely be running. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_GC);
        mtk_op_evict_all_terminal();
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_NOT_FOUND); /* token evicted */

        /* Immediate replacement, while the old gatt_connect call is STILL
         * genuinely blocked, must be refused -- never admitted to start a
         * second, overlapping connect. */
        {
            mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
            mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 5);
            mtk_gatt_connect_req_t breq; memset(&breq, 0, sizeof(breq));
            memset(breq.target.addr.b, 0xEE, 6);
            mtk_test_call(&busy_ctx, gatt_connect_op, &breq);
            MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);
        }

        /* Let the stale worker's gatt_connect call finally return "connected" --
         * it lost the op-table race (already invalidated), so it must never
         * publish s_gatt as connected nor emit GATT_ CONNECT_COMPLETE for the
         * invalidated token; it must tear its OWN just-formed real connection
         * back down (gatt_disconnect on its own local vendor handle) and only
         * THEN release MTK_ARB_GC via token ownership. */
        usleep(400000);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* released by the stale worker's own tail */
        fake_ble_lock();
        unsigned gatt_disconnects = g_fake_ble.gatt_disconnect_call_count;
        fake_ble_unlock();
        MTK_CHECK(gatt_disconnects >= 1); /* the stale worker's own fallback teardown fired */
        mtk_fake_sink_lock();
        unsigned stale_events = sink.event_count;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stale_events, 0u); /* GATT_CONNECT_COMPLETE was never emitted for the invalidated token */

        /* A NEW GATT_CONNECT now completes normally end-to-end. */
        g_fake_ble.gatt_connect_delay_ms = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 6);
        mtk_test_call(&ctx2, gatt_connect_op, &req);
        uint32_t tok2 = poll_for_op_token(&sink2, gatt_connect_op);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK);

        mtk_fake_sink_state_t status_sink; mtk_fake_sink_reset(&status_sink);
        mtk_request_ctx_t status_ctx = mtk_test_ctx(&status_sink, 7);
        mtk_gatt_status_req_t sreq = {0}; sreq.connection_token = tok2;
        mtk_test_call(&status_ctx, gatt_status_op, &sreq);
        mtk_gatt_status_resp_t st = {0};
        mtk_decode(gatt_status_op->resp_desc, &st, status_sink.response.body, status_sink.response.body_len, NULL);
        MTK_CHECK_EQ(st.connected, 1); /* the new operation's own connect genuinely succeeded */

        /* GATT_CONNECT deliberately keeps MTK_ARB_GC held for as long as
         * the connection itself exists (this file's own established
         * design, unlike Wi-Fi STA connect) -- disconnect now so it does
         * not linger held into section C below. */
        const mtk_opcode_entry_t *gatt_disconnect_op = mtk_test_find_op("GATT_DISCONNECT");
        MTK_CHECK(gatt_disconnect_op);
        mtk_fake_sink_state_t dc_sink; mtk_fake_sink_reset(&dc_sink);
        mtk_request_ctx_t dc_ctx = mtk_test_ctx(&dc_sink, 100);
        mtk_gatt_disconnect_req_t dreq = {0}; dreq.connection_token = tok2;
        mtk_test_call(&dc_ctx, gatt_disconnect_op, &dreq);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

    /* ==== C. Forced AP scan cancellation timeout followed by a
     * new-session request. ================================================= */
    {
        mtk_fake_wifi_reset();
        /* The fake HAL's own cancel signal is deliberately ignored (still
         * recorded as genuinely signalled) -- simulating a real HAL whose
         * own cancellation does not take effect promptly, forcing the
         * canceller's bounded ~1000ms quiescence wait to genuinely
         * time out rather than observe an early return. 260 polls * 5ms =
         * 1300ms, comfortably longer than the 1000ms bound. */
        g_fake_wifi.ap_scan_ignore_cancel = 1;
        g_fake_wifi.ap_scan_poll_count_per_call = 260;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 8);
        mtk_ap_scan_start_req_t req = {0}; req.band = 0; req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, ap_start, &req);
        uint32_t tok = poll_for_op_token(&sink, ap_start);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        mtk_op_id_t cancelled = mtek_wifi_cancel_active_for_peer_reset(); /* blocks ~1000ms (the real bounded quiescence wait) */
        MTK_CHECK_EQ(cancelled.token, tok);
        MTK_CHECK(g_fake_wifi.ap_scan_cancel_count >= 1); /* cancellation was genuinely signalled, even though not honored */
        /* Bounded wait expired: the token is force-finalized to STOPPED
         * (queryable/evictable immediately -- never left as a live "old
         * session" operation), but the arbiter is deliberately left held
         * (the radio may still genuinely be in use). */
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK);
        mtk_op_evict_all_terminal();
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        /* An immediate replacement is still correctly refused -- the
         * radio is genuinely still busy (the old worker has not reached
         * poll 260 yet: only ~200 of its own 5ms polls have elapsed by
         * now, matching the ~1000ms the cancel call itself just spent). */
        {
            mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
            mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 9);
            mtk_test_call(&busy_ctx, ap_start, &req);
            MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);
        }

        /* Let the stale worker finish its own remaining polls (up to its
         * own full 260 * 5ms = 1300ms) and reach its own tail. It lost
         * the op-table race (already force-finalized above), so it must
         * never publish s_ap/emit AP_SCAN_COMPLETE for the invalidated
         * token -- but it IS the one place that safely restores the
         * radio and releases MTK_ARB_WS, via token ownership, once its
         * own real call actually returns. */
        usleep(1200000);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* released by the stale worker's own tail */
        mtk_fake_sink_lock();
        unsigned stale_events = sink.event_count;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stale_events, 0u); /* no AP_SCAN_COMPLETE emitted for the invalidated token */

        /* A NEW AP_SCAN_START now completes normally end-to-end, proving
         * the radio really was freed and nothing about the old session's
         * state lingers. */
        g_fake_wifi.ap_scan_ignore_cancel = 0;
        g_fake_wifi.ap_scan_poll_count_per_call = 1;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 10);
        mtk_test_call(&ctx2, ap_start, &req);
        uint32_t tok2 = poll_for_op_token(&sink2, ap_start);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK);
    }

MTK_TEST_MAIN_END
