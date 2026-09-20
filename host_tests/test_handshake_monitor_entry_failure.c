/* promisc_start's return value was previously discarded entirely -- a
 * callback/channel/promiscuous-mode failure on the real HAL left the operation
 * RUNNING forever with no cleanup, and the initiating deauth burst still fired
 * regardless. Proves the fix: a monitor-entry failure transitions to a terminal
 * IO-error state exactly once, runs full cleanup (promisc_stop already never
 * armed, restore_sta_mode, arbiter release) exactly once, emits
 * HANDSHAKE_STOPPED, and never sends a single deauth frame. Also proves a
 * genuine concurrent STOP racing a slow, eventually-failing promisc_start
 * resolves to exactly one cleanup either way. */
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
    /* Verification sweep: a real TSan run caught this test's own polling reads
     * of a stack-local fake sink racing the async worker thread's own writes
     * into it (Part 2 below) -- see mtk_fake_sink.h's own doc comment on
     * mtk_fake_sink_set_lock. */
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    const mtk_opcode_entry_t *start_op = mtk_test_find_op("HANDSHAKE_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("HANDSHAKE_STOP");
    MTK_CHECK(start_op && stop_op);

    /* Part 1: synchronous monitor-entry failure -- no async runner registered
     * yet, so this runs entirely on the calling thread, the simplest and most
     * direct proof. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.promisc_start_rc = -1; /* callback/channel/promiscuous-mode step fails */

        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        req.channel = 6; req.deauth_count = 5; /* would send 5 deauth frames if this bug were still present */

        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED); /* ACCEPTED_ASYNC's own immediate accept, unrelated to the later failure */
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0u); /* never sent a single frame after monitor entry failed */

        const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
        MTK_CHECK(stopped != NULL);
        if (stopped) {
            mtk_handshake_stopped_ev_t done = {0};
            mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
            MTK_CHECK_EQ(done.status, MTK_STATUS_IO_ERROR);
        }
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* lease released, never leaked */

        /* A fresh handshake attempt must not be rejected BUSY -- the
         * arbiter is genuinely free after the failure's own cleanup. */
        g_fake_wifi.promisc_start_rc = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 2);
        mtk_handshake_start_req_t req2 = req; req2.deauth_count = 0;
        mtk_test_call(&ctx2, start_op, &req2);
        MTK_CHECK_EQ(sink2.response.status, MTK_STATUS_ACCEPTED);
        /* Clean up this second, genuinely-armed session before Part 2. */
        mtk_handshake_start_resp_t started = {0};
        mtk_decode(start_op->resp_desc, &started, sink2.response.body, sink2.response.body_len, NULL);
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 3);
        mtk_handshake_stop_req_t stopreq = {0}; stopreq.operation_token = started.operation_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
    }

    /* Part 2: a genuine concurrent STOP racing a slow, eventually- failing
     * promisc_start (real async worker, real pthread) -- proves the race
     * resolves to exactly one HANDSHAKE_STOPPED, one restore, one arbiter
     * release, regardless of which side (the worker's own failure path, or the
     * STOP) actually wins mtk_op_transition. */
    {
        mtk_router_set_async_runner(pthread_runner);
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.promisc_start_rc = -1;
        g_fake_wifi.promisc_start_delay_ms = 40; /* real window for the STOP below to race it */

        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        req.channel = 6; req.deauth_count = 0;
        mtk_test_call(&ctx, start_op, &req);
        /* Not asserting "not yet answered" here: whether the worker
         * thread has already written the response by the time control
         * returns to this thread is a real scheduling race with no
         * defined winner (TSan's own added instrumentation overhead
         * measurably shifts it versus a plain build) -- this call being
         * deferred to another thread at all is already proven by the
         * poll loop below succeeding via a real cross-thread write, not
         * by a timing assumption about how far that thread has gotten
         * before this one checks. */

        /* Poll for the ACCEPTED response the worker thread writes. */
        uint32_t op_token = 0;
        for (int i = 0; i < 2000 && op_token == 0; i++) {
            mtk_fake_sink_lock();
            int got = sink.response.set && sink.response.status == MTK_STATUS_ACCEPTED;
            mtk_handshake_start_resp_t started = {0};
            if (got) mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
            mtk_fake_sink_unlock();
            if (got) op_token = started.operation_token;
            else usleep(500);
        }
        MTK_CHECK(op_token != 0);

        /* STOP while the worker is still inside its own 40ms-delayed
         * promisc_start call, well before it observes its own injected
         * failure. */
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 2);
        mtk_handshake_stop_req_t stopreq = {0}; stopreq.operation_token = op_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK_EQ(stopsink.response.status, MTK_STATUS_OK);

        /* Bounded wait for the worker to actually finish (promisc_start's
         * own 40ms delay, plus scheduling slack). */
        usleep(150000);

        /* Exactly one HANDSHAKE_STOPPED total -- handshake_finish always
         * emits via the session's own originally-captured sink (`sink`,
         * from HANDSHAKE_START's own ctx), regardless of whether the
         * worker's own failure path or this STOP call actually won
         * mtk_op_transition, so cleanup ran exactly once, never twice. */
        mtk_fake_sink_lock();
        unsigned stopped_count = 0;
        for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "HANDSHAKE_STOPPED") == 0) stopped_count++;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stopped_count, 1u);
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0u);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
