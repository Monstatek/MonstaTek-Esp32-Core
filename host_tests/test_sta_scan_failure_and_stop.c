/* RC10 independent target-concurrency correction order: proves the
 * fixes to esp32_sta_scan's own transactional entry/teardown honesty
 * (P0 #4), esp32_restore_sta_mode's own failure-atomicity (P0 #5), and
 * s_sta/status/results-page synchronization (P0 #3) -- via the fake
 * HAL's own mirrored contract (mtk_fake_wifi_hal.h), since
 * mtek_wifi_hal_esp32.c itself is ESP-IDF-only and cannot be compiled or
 * exercised on the host. A genuinely concurrent STA scan STOP/result-
 * publication test (real pthread) is included per the order's own
 * explicit requirement. */
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
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    const mtk_opcode_entry_t *start_op = mtk_test_find_op("STA_SCAN_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("STA_SCAN_STOP");
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("STA_SCAN_STATUS");
    const mtk_opcode_entry_t *page_op = mtk_test_find_op("STA_SCAN_RESULTS_PAGE");
    MTK_CHECK(start_op && stop_op && status_op && page_op);

    /* ---- Part 1: teardown-failure honesty -- real results were found,
     * but a teardown step failed, so the whole call must fail. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_count = 2;
        memset(g_fake_wifi.sta_results[0].mac.b, 0x11, 6); g_fake_wifi.sta_results[0].rssi = -40;
        memset(g_fake_wifi.sta_results[1].mac.b, 0x22, 6); g_fake_wifi.sta_results[1].rssi = -50;
        g_fake_wifi.sta_scan_teardown_cb_clear_rc = -1; /* callback-unregister failure */

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* never a clean OK despite 2 real stations found */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* still released -- no permanent lockout */
    }
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_count = 1;
        memset(g_fake_wifi.sta_results[0].mac.b, 0x11, 6);
        g_fake_wifi.sta_scan_teardown_promisc_off_rc = -1; /* promiscuous-disable failure */

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

    /* ---- Part 2: restore-step failure honesty + short-circuit + no
     * double restore/release. Each restore step failure is injected in
     * turn; a successful scan must still report IO_ERROR when its own
     * post-scan restore fails, and restore_count must be exactly 1
     * (never called twice for one operation). */
    const char *step_names[] = {"promisc_off", "stop", "mode", "start", "channel", "reconnect"};
    for (int step = 0; step < 6; step++) {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_count = 1;
        memset(g_fake_wifi.sta_results[0].mac.b, 0x33, 6);
        g_fake_wifi.sta_was_connected = (step == 5); /* only exercise reconnect when testing that step */
        switch (step) {
            case 0: g_fake_wifi.restore_promisc_off_rc = -1; break;
            case 1: g_fake_wifi.restore_stop_rc = -1; break;
            case 2: g_fake_wifi.restore_mode_rc = -1; break;
            case 3: g_fake_wifi.restore_start_rc = -1; break;
            case 4: g_fake_wifi.restore_channel_rc = -1; break;
            case 5: g_fake_wifi.restore_reconnect_rc = -1; break;
        }

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK(sink.event_count >= 1);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* truthful: this specific restore step failed */
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u);  /* called exactly once -- never double-restored */
        /* RC11 independent correction order P0 "if radio restoration
         * fails, retain the prior-state snapshot and keep the radio lease
         * quarantined -- never report/expose it as healthy/free": a
         * restore-step failure must no longer release the arbiter (RC10's
         * own "still released exactly once" expectation is exactly the
         * defect this order fixes) -- the lease stays held against the
         * SAME class that owned it, and mekt_wifi_radio_is_quarantined()
         * reports the quarantine honestly. mtk_arbiter_reset() below is
         * pure test-harness cleanup (mirrors mtk_fake_wifi_reset() already
         * used to isolate each of these six independent scenarios from
         * each other) -- production code never force-clears a quarantine
         * this way; only a later confirmed-successful restore does. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        mtk_arbiter_reset();
        (void)step_names;
    }

    /* Short-circuit proof: a "stop" failure must skip mode/start/channel/
     * reconnect entirely (the real HAL's own precondition rule) -- the
     * fake models this by never touching mode/current_channel/reconnect_
     * attempted_count when restore_stop_rc is set, which this asserts. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.mode = 2; g_fake_wifi.current_channel = 9; g_fake_wifi.sta_was_connected = 1;
        g_fake_wifi.restore_stop_rc = -1;
        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(g_fake_wifi.mode, 2);              /* untouched -- mode/start step never attempted */
        MTK_CHECK_EQ(g_fake_wifi.current_channel, 9);   /* untouched by restore (the fake's own sta_scan does not itself disturb current_channel, unlike send_deauth's) */
        MTK_CHECK_EQ(g_fake_wifi.reconnect_attempted_count, 0u); /* reconnect never attempted either */
        /* RC11 independent correction order P0: same quarantine-on-
         * restore-failure fix as the loop above -- this scenario's own
         * injected "stop" failure must also leave the lease held, not
         * released; clear it (test-harness only) before Part 3 needs a
         * genuinely free arbiter to start its own fresh scan. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        mtk_arbiter_reset();
    }

    /* ---- Part 3: a genuinely concurrent STA scan STOP racing result
     * publication and STATUS/RESULTS_PAGE reads (real pthread worker). */
    {
        mtk_router_set_async_runner(pthread_runner);
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_scan_poll_count_per_call = 30; /* 30 * 5ms = 150ms if never cancelled */
        g_fake_wifi.sta_count = 3;
        memset(g_fake_wifi.sta_results[0].mac.b, 0x01, 6);
        memset(g_fake_wifi.sta_results[1].mac.b, 0x02, 6);
        memset(g_fake_wifi.sta_results[2].mac.b, 0x03, 6);

        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 5000;
        mtk_test_call(&ctx, start_op, &req);

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

        /* Hammer STA_SCAN_STATUS and STA_SCAN_RESULTS_PAGE from THIS
         * thread while the worker is still mid-scan and about to publish
         * s_sta -- proves handle_sta_scan_status/_results_page's own
         * wifi_lock/unlock makes every read see one coherent generation/
         * count/items snapshot (TSan would flag anything less). */
        for (int i = 0; i < 20; i++) {
            mtk_fake_sink_state_t ssink; mtk_fake_sink_reset(&ssink);
            mtk_request_ctx_t sctx = mtk_test_ctx(&ssink, 100 + (uint32_t)i);
            mtk_sta_scan_status_req_t sreq = {0}; sreq.operation_token = op_token;
            mtk_test_call(&sctx, status_op, &sreq);
            MTK_CHECK_EQ(ssink.response.status, MTK_STATUS_OK);

            mtk_fake_sink_state_t psink; mtk_fake_sink_reset(&psink);
            mtk_request_ctx_t pctx = mtk_test_ctx(&psink, 200 + (uint32_t)i);
            mtk_sta_scan_results_page_req_t preq = {0}; preq.result_generation = 0xFFFFFFFF; /* a real prior generation is unknown yet -- NOT_FOUND is a valid, coherent answer */
            mtk_test_call(&pctx, page_op, &preq);
            MTK_CHECK(psink.response.status == MTK_STATUS_OK || psink.response.status == MTK_STATUS_NOT_FOUND);
            usleep(1000);
        }

        /* Now STOP it mid-flight. */
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 300);
        mtk_sta_scan_stop_req_t stopreq = {0}; stopreq.operation_token = op_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK(stopsink.response.status == MTK_STATUS_OK || stopsink.response.status == MTK_STATUS_BUSY);

        /* Continue hammering STATUS/RESULTS_PAGE concurrently with the
         * worker's own teardown+result-publication, then confirm a
         * coherent final result once terminal. */
        int terminal = 0;
        mtk_sta_scan_status_resp_t last_status = {0};
        for (int i = 0; i < 2000 && !terminal; i++) {
            mtk_fake_sink_state_t ssink; mtk_fake_sink_reset(&ssink);
            mtk_request_ctx_t sctx = mtk_test_ctx(&ssink, 400 + (uint32_t)i);
            mtk_sta_scan_status_req_t sreq = {0}; sreq.operation_token = op_token;
            mtk_test_call(&sctx, status_op, &sreq);
            if (ssink.response.status == MTK_STATUS_OK) {
                mtk_decode(status_op->resp_desc, &last_status, ssink.response.body, ssink.response.body_len, NULL);
                if (mtk_op_state_is_terminal((mtk_op_state_t)last_status.state)) terminal = 1;
            }
            if (!terminal) usleep(1000);
        }
        MTK_CHECK(terminal);
        MTK_CHECK(g_fake_wifi.sta_scan_polls_done < 30); /* STOP genuinely shortened it */

        /* Radio genuinely free afterward -- a fresh scan is accepted. */
        usleep(5000);
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 500);
        g_fake_wifi.sta_scan_poll_count_per_call = 1;
        mtk_sta_scan_start_req_t req2; memset(&req2, 0, sizeof(req2));
        memset(req2.target_bssid.b, 0xAA, 6); req2.channel = 6; req2.duration_ms = 10;
        mtk_test_call(&ctx2, start_op, &req2);
        int accepted2 = 0;
        for (int i = 0; i < 2000 && !accepted2; i++) {
            mtk_fake_sink_lock();
            accepted2 = (sink2.response.status == MTK_STATUS_ACCEPTED);
            mtk_fake_sink_unlock();
            if (!accepted2) usleep(500);
        }
        MTK_CHECK(accepted2);
    }

MTK_TEST_MAIN_END
