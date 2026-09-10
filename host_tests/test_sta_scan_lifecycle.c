/* RC9 independent correction order P0 "station-target scan bypasses the
 * transactional radio lifecycle": proves esp32_sta_scan's own new
 * transactional contract (mirrored by mtk_fake_wifi_hal.h's
 * fake_wifi_sta_scan) -- a real prior-state snapshot/restore round trip,
 * a real transactional-entry failure surfaced honestly (never silently
 * reported as "0 stations found"), and a genuine early-return when
 * STA_SCAN_STOP is dispatched from another thread while a real async
 * worker is still inside the blocking sta_scan call. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }

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

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_router_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    /* RC9 independent correction order verification sweep: a real TSan run
     * caught this test's own polling reads of a stack-local fake sink
     * racing the async worker thread's own writes into it -- see
     * mtk_fake_sink.h's own doc comment on mtk_fake_sink_set_lock. */
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    const mtk_opcode_entry_t *start_op = mtk_test_find_op("STA_SCAN_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("STA_SCAN_STOP");
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("STA_SCAN_STATUS");
    MTK_CHECK(start_op && stop_op && status_op);

    /* ---- Part 1: transactional-entry failure is surfaced honestly --
     * never silently rewritten to "0 stations found". */
    {
        mtk_fake_sink_state_t sink; memset(&sink, 0, sizeof(sink));
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_scan_rc = -1; /* callback/channel/promiscuous-mode step fails */
        g_fake_wifi.mode = 1; g_fake_wifi.current_channel = 9; g_fake_wifi.sta_was_connected = 1;

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* not MTK_STATUS_OK with result_count=0 */
        MTK_CHECK(g_fake_wifi.restore_count >= 1); /* still restored even on entry failure */
        MTK_CHECK_EQ(g_fake_wifi.mode, 1);
        MTK_CHECK_EQ(g_fake_wifi.current_channel, 9);

        /* The arbiter (MTK_ARB_WS) must be genuinely free again -- a
         * second scan attempt (this time succeeding) must not be
         * rejected BUSY. */
        g_fake_wifi.sta_scan_rc = 0;
        mtk_sta_scan_start_req_t req2 = req;
        mtk_test_call(&ctx, start_op, &req2);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    }

    /* ---- Part 2: prior-state snapshot/restore round trip for a
     * SUCCESSFUL scan (a non-default mode/channel, genuinely disturbed
     * by the scan's own channel select, must come back exactly). */
    {
        mtk_fake_sink_state_t sink; memset(&sink, 0, sizeof(sink));
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.mode = 2; g_fake_wifi.current_channel = 11; g_fake_wifi.sta_was_connected = 1;
        g_fake_wifi.sta_count = 1;
        memset(g_fake_wifi.sta_results[0].mac.b, 0x11, 6); g_fake_wifi.sta_results[0].rssi = -40;

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(ev.result_count, 1u);
        MTK_CHECK(g_fake_wifi.restore_count >= 1);
        MTK_CHECK_EQ(g_fake_wifi.mode, 2);              /* restored to the real prior mode (APSTA), not hard-coded STA */
        MTK_CHECK_EQ(g_fake_wifi.current_channel, 11);  /* restored to the real prior channel, not left at 6 */
        MTK_CHECK(g_fake_wifi.reconnect_attempted_count >= 1);
    }

    /* ---- Part 3: STOP genuinely interrupts a live blocking scan -- a
     * real async worker (pthread_runner) is still inside the fake HAL's
     * own poll loop when STOP is dispatched from this thread. */
    {
        mtk_router_set_async_runner(pthread_runner);
        mtk_fake_sink_state_t sink; memset(&sink, 0, sizeof(sink));
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_scan_poll_count_per_call = 40; /* 40 * 5ms = 200ms if never cancelled */

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 5000;
        mtk_test_call(&ctx, start_op, &req);
        /* Not asserting "not yet answered" here -- a real scheduling race
         * with no defined winner (see test_handshake_monitor_entry_
         * failure.c's own identical note); the poll loop below is the
         * real proof this was genuinely deferred to another thread. */

        /* Poll for the ACCEPTED response the worker thread writes. */
        uint32_t op_token = 0;
        for (int i = 0; i < 2000 && op_token == 0; i++) {
            mtk_fake_sink_lock();
            int got = sink.response.set && sink.response.status == MTK_STATUS_ACCEPTED;
            mtk_sta_scan_start_resp_t started = {0};
            if (got) mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
            mtk_fake_sink_unlock();
            if (got) op_token = started.operation_token;
            else usleep(500);
        }
        MTK_CHECK(op_token != 0);

        /* Give the worker a moment to genuinely enter its poll loop, then
         * STOP it well before the full 40-poll/200ms duration. */
        usleep(15000); /* ~15ms -- a handful of 5ms polls in, nowhere near 40 */
        mtk_sta_scan_stop_req_t stopreq = {0}; stopreq.operation_token = op_token;
        mtk_test_call(&ctx, stop_op, &stopreq);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK); /* STA_SCAN_STOP is SYNCHRONOUS -- answered directly on this thread, no race */

        /* Bounded wait for the worker to actually finish (it must, within
         * roughly one more poll interval after seeing the cancel flag). */
        int terminal = 0;
        for (int i = 0; i < 2000 && !terminal; i++) {
            mtk_sta_scan_status_req_t sreq = {0}; sreq.operation_token = op_token;
            mtk_fake_sink_state_t status_sink; memset(&status_sink, 0, sizeof(status_sink));
            mtk_request_ctx_t sctx = mtk_test_ctx(&status_sink, 2);
            mtk_test_call(&sctx, status_op, &sreq);
            if (status_sink.response.status == MTK_STATUS_OK) {
                mtk_sta_scan_status_resp_t st = {0};
                mtk_decode(status_op->resp_desc, &st, status_sink.response.body, status_sink.response.body_len, NULL);
                if (mtk_op_state_is_terminal((mtk_op_state_t)st.state)) terminal = 1;
            }
            if (!terminal) usleep(1000);
        }
        MTK_CHECK(terminal);

        /* The critical proof: STOP genuinely shortened the call -- it did
         * NOT run all 40 configured polls. Locked: these fields are the
         * same ones fake_wifi_sta_scan itself writes from the worker
         * thread (mtk_fake_wifi_hal.h's own fake_wifi_lock/unlock). */
        fake_wifi_lock();
        unsigned cancel_count = g_fake_wifi.sta_scan_cancel_count;
        unsigned polls_done = g_fake_wifi.sta_scan_polls_done;
        fake_wifi_unlock();
        MTK_CHECK(cancel_count >= 1);
        MTK_CHECK(polls_done < 40);

        /* Radio state is safely restored, and the arbiter is genuinely
         * free -- a fresh operation on the same class must not be
         * rejected BUSY (would indicate a leaked lease from the
         * still-technically-running worker). */
        usleep(5000);
        mtk_fake_sink_state_t sink2; memset(&sink2, 0, sizeof(sink2));
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 3);
        mtk_sta_scan_start_req_t req2; memset(&req2, 0, sizeof(req2));
        memset(req2.target_bssid.b, 0xAA, 6); req2.channel = 6; req2.duration_ms = 10;
        g_fake_wifi.sta_scan_poll_count_per_call = 1;
        mtk_test_call(&ctx2, start_op, &req2);
        int sink2_accepted = 0;
        for (int i = 0; i < 2000 && !sink2_accepted; i++) {
            mtk_fake_sink_lock();
            sink2_accepted = (sink2.response.status == MTK_STATUS_ACCEPTED);
            mtk_fake_sink_unlock();
            if (!sink2_accepted) usleep(500);
        }
        MTK_CHECK(sink2_accepted);
    }

MTK_TEST_MAIN_END
