/* Release-tooling-round P0 correction, ROUND 2 (follow-up read-only audit,
 * "the same slot-reuse/ABA hazard remains through all 11 production
 * mtk_op_alloc() call sites"): the prior round fixed every mtk_op_find()
 * call site (never retain a raw pointer past the lock that produced it)
 * but left mtk_op_alloc()'s own 11 production callers dereferencing the
 * freshly-minted record's pointer for everything from the initial RUNNING
 * transition through the ACCEPTED response and any HAL call that follows
 * it -- the identical hazard, just at mint time instead of lookup time.
 * That round's fix: mtk_op_alloc_id() (mtek_core.h) copies out the
 * operation's immutable {token, boot_epoch} identity atomically at mint
 * time, under the same lock that allocated it, and every one of the 11
 * production call sites now uses ONLY that identity thereafter -- never a
 * retained mtk_operation_record_t* past the allocating expression.
 *
 * This is the deterministic, SERVICE-LEVEL proof the audit asked for: it
 * forces genuine table-slot reuse (fills the fixed 8-slot table with
 * terminal records, then allocates a 9th, forcing mtk_op_alloc's own
 * documented "evict the oldest terminal record" path to free and reuse a
 * REAL slot for a brand-new, unrelated operation) and proves, entirely
 * through the public request/response wire-shape API (GET_OPERATION_
 * STATUS, DEAUTH_STOP) rather than any mtek_core.c internal:
 *   1. The evicted ("victim") operation's own OLD token is genuinely
 *      unusable afterward -- GET_OPERATION_STATUS/DEAUTH_STOP against it
 *      both report NOT_FOUND, never silently resolving to whatever now
 *      occupies its former slot.
 *   2. The replacement operation (freshly minted into that same reused
 *      slot) reports its own correct, independent state via STATUS,
 *      genuinely RUNNING, never contaminated by the victim's own final
 *      FAILED status.
 *   3. Attempting to STOP the replacement using the victim's OLD (now
 *      unrelated) token is rejected (NOT_FOUND) and provably does NOT
 *      mutate the replacement's own real state -- confirmed by a
 *      follow-up STATUS call on the replacement's own token showing it is
 *      still exactly as it was, unaffected.
 *   4. STOPping the replacement with its OWN correct token succeeds
 *      normally (STOPPED), completing a full alloc -> STATUS -> STOP path
 *      entirely through the post-reuse slot with no cross-contamination
 *      at any step. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtk_test_async_fixture.h"
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
    usleep(50000); /* genuinely deferred: still pending when this test's own main thread checks it */
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

static void wait_for_deauth_count(unsigned expected) {
    for (int i = 0; i < 3000; i++) {
        fake_wifi_lock();
        unsigned count = g_fake_wifi.deauth_sent_count;
        fake_wifi_unlock();
        if (count >= expected) break;
        usleep(1000);
    }
}

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_router_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);
    mtek_system_set_sta_query(sta_query_always_ready);
    /* No async runner registered yet: TIME_SYNC_START is itself an
     * ACCEPTED_ASYNC opcode (mtk_router_dispatch only ever defers when a
     * runner IS registered), and every one of these 8 calls must complete
     * fully synchronously, on this thread, into a plain stack-local sink
     * -- registered only later, right before the one call (DEAUTH_START)
     * that actually needs genuine backgrounding. */

    const mtk_opcode_entry_t *ts_start_op = mtk_test_async_fixture_install() /* RC12 item 5: test-only overlay async op, was TIME_SYNC_START */;
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GET_OPERATION_STATUS");
    const mtk_opcode_entry_t *deauth_start_op = mtk_test_find_op("DEAUTH_START");
    const mtk_opcode_entry_t *deauth_stop_op = mtk_test_find_op("DEAUTH_STOP");
    MTK_CHECK(ts_start_op && status_op && deauth_start_op && deauth_stop_op);

    /* ---- 1. Fill all 8 operation-table slots with immediately-terminal
     * TIME_SYNC_START operations (no SNTP client wired in -> each one
     * synchronously transitions FAILED before its own request even
     * returns). Slot/eviction order is by terminal_at_ms, oldest first --
     * these 8 calls execute strictly in sequence on this single thread, so
     * victim_token (the FIRST one minted) is unambiguously the oldest
     * terminal record once the table is full. ---------------------------- */
    uint32_t victim_token = 0;
    for (int i = 0; i < 8; i++) {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1000 + i);
        mtk_time_sync_start_req_t req = {0}; req.timeout_ms = 0;
        mtk_test_call(&ctx, ts_start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        mtk_time_sync_start_resp_t r = {0};
        mtk_decode(ts_start_op->resp_desc, &r, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK(r.operation_token != 0);
        if (i == 0) victim_token = r.operation_token;
    }
    MTK_CHECK(victim_token != 0);

    /* Confirm the victim really is present and FAILED (terminal) before
     * eviction -- otherwise the eviction this test forces below would not
     * be exercising the documented "oldest TERMINAL record" path at all. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 2000);
        mtk_get_operation_status_req_t req = {0}; req.operation_token = victim_token;
        mtk_test_call(&ctx, status_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_FAILED);
    }

    /* ---- 2. The table is now genuinely FULL (8/8, all terminal). A 9th
     * allocation -- a genuinely long-running DEAUTH_START -- forces
     * mtk_op_alloc's own documented eviction of the oldest terminal record
     * (victim_token's own slot) to mint this new, unrelated operation.
     * ACCEPTED (never NO_MEMORY) here is itself proof the eviction really
     * happened. ------------------------------------------------------------ */
    uint32_t replacement_token = 0;
    mtk_router_set_async_runner(pthread_runner); /* only now -- DEAUTH_START needs genuine backgrounding */
    /* start_sink/start_ctx are declared here, NOT inside a nested block --
     * they must stay alive for the rest of this function, since the
     * deferred background worker (handle_deauth_start's own tail code,
     * mtek_wifi_logic.c) retains a pointer to start_sink via its own
     * persistent async-pool copy of start_ctx.sink and may still touch it
     * well after this section returns (until this test's own later STOP
     * call below wins the claim race and the worker's own natural-
     * completion attempt then finds it already lost, per deauth_finalize's
     * own single-winner semantics, and skips its own event emission
     * entirely -- but that only holds AFTER the STOP call, so start_sink
     * must not go out of scope before then). */
    mtk_fake_sink_state_t start_sink; mtk_fake_sink_reset(&start_sink);
    mtk_request_ctx_t start_ctx = mtk_test_ctx(&start_sink, 3000);
    {
        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req.channel = 6; req.target_mode = 2 /* BROADCAST */;
        req.count = 0 /* run until stopped */; req.interval_ms = 0;
        mtk_test_call(&start_ctx, deauth_start_op, &req);

        /* Poll for the ACCEPTED response the worker thread writes (genuinely
         * deferred -- never assume it lands synchronously). */
        int got = 0;
        for (int i = 0; i < 3000 && !got; i++) {
            mtk_fake_sink_lock();
            got = start_sink.response.set && start_sink.response.status == MTK_STATUS_ACCEPTED;
            mtk_deauth_start_resp_t r = {0};
            if (got) mtk_decode(deauth_start_op->resp_desc, &r, start_sink.response.body, start_sink.response.body_len, NULL);
            mtk_fake_sink_unlock();
            if (got) { replacement_token = r.operation_token; break; }
            usleep(1000);
        }
        MTK_CHECK(got); /* NOT NO_MEMORY: the table genuinely had room after eviction */
        MTK_CHECK(replacement_token != 0);
        MTK_CHECK(replacement_token != victim_token); /* a genuinely NEW token, never reissued */
    }
    wait_for_deauth_count(1); /* the replacement is genuinely running in the background */

    /* ---- 3. The victim's OLD token is now unusable: NOT_FOUND, never
     * silently resolving to the replacement now occupying its old slot. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 4000);
        mtk_get_operation_status_req_t req = {0}; req.operation_token = victim_token;
        mtk_test_call(&ctx, status_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_NOT_FOUND);
    }

    /* ---- 4. The replacement reports its OWN correct state (RUNNING),
     * never the victim's stale FAILED status. --------------------------- */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 5000);
        mtk_get_operation_status_req_t req = {0}; req.operation_token = replacement_token;
        mtk_test_call(&ctx, status_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    }

    /* ---- 5. Attempting to STOP the replacement using the VICTIM's own
     * (now stale/unrelated) token is rejected -- and, critically, does NOT
     * mutate the replacement's own real state. --------------------------- */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 6000);
        mtk_deauth_stop_req_t req = {0}; req.operation_token = victim_token;
        mtk_test_call(&ctx, deauth_stop_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_NOT_FOUND);
    }
    {
        /* Re-check the replacement: still RUNNING, completely unaffected
         * by the stale-token STOP attempt above -- the old operation
         * genuinely can never emit or mutate using the replacement's own
         * token/slot. */
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 7000);
        mtk_get_operation_status_req_t req = {0}; req.operation_token = replacement_token;
        mtk_test_call(&ctx, status_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    }

    /* ---- 6. STOPping the replacement with its OWN correct token succeeds
     * normally, completing the full alloc -> STATUS -> STOP path through
     * the reused slot with no cross-contamination at any step. ---------- */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 8000);
        mtk_deauth_stop_req_t req = {0}; req.operation_token = replacement_token;
        mtk_test_call(&ctx, deauth_stop_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        mtk_deauth_stop_resp_t r = {0};
        mtk_decode(deauth_stop_op->resp_desc, &r, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK_EQ(r.final_state, MTK_OPS_STOPPED);
    }

MTK_TEST_MAIN_END
