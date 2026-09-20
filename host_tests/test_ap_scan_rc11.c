/* Owner-approved RC11 correction: "AP scan has the same async STOP race and
 * false-success behavior" as station-target scan. Mirrors test_sta_scan_rc10.c's
 * own structure closely (a real pthread async runner for the concurrent-STOP
 * section, matching that file's own proven pattern) but AP scan has no separate
 * promiscuous-mode teardown step of its own to fail
 * (esp32_ap_scan/fake_wifi_ap_scan never register a promiscuous callback), so
 * this covers: honest transactional-entry failure reporting, restore-step
 * failure honesty + quarantine (never released), and a genuinely concurrent STOP
 * racing natural completion. */
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

    const mtk_opcode_entry_t *start_op = mtk_test_find_op("AP_SCAN_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("AP_SCAN_STOP");
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("AP_SCAN_STATUS");
    MTK_CHECK(start_op && stop_op && status_op);

    /* Part 1: transactional-entry failure honesty -- previously silently
     * rewritten to 0 results / MTK_STATUS_OK. -- */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_scan_rc = -1;
        mtk_ap_scan_start_req_t req = {0}; req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_ap_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_ap_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* never a clean OK on a real entry failure */
        MTK_CHECK_EQ(ev.result_count, 0u);
        mtk_arbiter_reset();
    }

    /* Part 2: restore-step failure honesty -- a successful scan must still
     * report IO_ERROR when its own post-scan restore fails, and the lease must
     * stay quarantined (never released) rather than freed over unconfirmed radio
     * state. -- */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_count = 2;
        g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -40;
        g_fake_wifi.ap_results[1].channel = 11; g_fake_wifi.ap_results[1].rssi = -55;
        g_fake_wifi.restore_stop_rc = -1;

        mtk_ap_scan_start_req_t req = {0}; req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_ap_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_ap_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* truthful: the restore step failed */
        MTK_CHECK_EQ(ev.result_count, 2u); /* real results stay published -- only restoration is in question */
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u); /* called exactly once -- never double-restored */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS); /* lease HELD (quarantined), not released */
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        mtk_arbiter_reset(); /* test-harness cleanup only */
    }

    /* Part 3: a genuinely concurrent AP scan STOP racing natural completion
     * (real pthread worker). -- */
    {
        mtk_router_set_async_runner(pthread_runner);
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_scan_poll_count_per_call = 30; /* 30 * 5ms = 150ms if never cancelled */
        g_fake_wifi.ap_count = 1;
        g_fake_wifi.ap_results[0].channel = 6;

        mtk_ap_scan_start_req_t req = {0}; req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;
        mtk_test_call(&ctx, start_op, &req);

        uint32_t op_token = 0;
        for (int i = 0; i < 2000 && op_token == 0; i++) {
            mtk_fake_sink_lock();
            int got = sink.response.set && sink.response.status == MTK_STATUS_ACCEPTED;
            mtk_ap_scan_start_resp_t started = {0};
            if (got) mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
            mtk_fake_sink_unlock();
            if (got) op_token = started.operation_token;
            else usleep(500);
        }
        MTK_CHECK(op_token != 0);

        /* Now STOP it mid-flight. */
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 300);
        mtk_ap_scan_stop_req_t stopreq = {0}; stopreq.operation_token = op_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK(stopsink.response.status == MTK_STATUS_OK || stopsink.response.status == MTK_STATUS_BUSY);

        int terminal = 0;
        mtk_ap_scan_status_resp_t last_status = {0};
        for (int i = 0; i < 2000 && !terminal; i++) {
            mtk_fake_sink_state_t ssink; mtk_fake_sink_reset(&ssink);
            mtk_request_ctx_t sctx = mtk_test_ctx(&ssink, 400 + (uint32_t)i);
            mtk_ap_scan_status_req_t sreq = {0}; sreq.operation_token = op_token;
            mtk_test_call(&sctx, status_op, &sreq);
            if (ssink.response.status == MTK_STATUS_OK) {
                mtk_decode(status_op->resp_desc, &last_status, ssink.response.body, ssink.response.body_len, NULL);
                if (mtk_op_state_is_terminal((mtk_op_state_t)last_status.state)) terminal = 1;
            }
            if (!terminal) usleep(1000);
        }
        MTK_CHECK(terminal);
        MTK_CHECK(g_fake_wifi.ap_scan_polls_done < 30); /* STOP genuinely shortened it */

        /* Radio genuinely free afterward (assuming restore succeeded --
         * no restore-failure injected in this part) -- a fresh scan is
         * accepted. */
        usleep(5000);
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 500);
        g_fake_wifi.ap_scan_poll_count_per_call = 1;
        mtk_ap_scan_start_req_t req2 = {0}; req2.band = 2; req2.channel_plan.mode = 1; req2.channel_plan.band = 2;
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
