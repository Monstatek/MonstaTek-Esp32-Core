/* Router async-pool safety, tested in isolation per the documented
 * contract (mtek_router.h): every sink target used with a registered
 * async runner here is heap-allocated with a lifetime the test itself
 * manages past the deferred call -- never a stack local -- so this test
 * proves the pool allocate/release/failure mechanics are memory-safe
 * without depending on any adapter rework. Build this test with
 * -fsanitize=address,undefined (see host_tests/CMakeLists.txt) to get an
 * active UAF/leak check, not just a manual lifetime argument. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <stdlib.h>
#include <string.h>

#define DEFER_QUEUE_LEN 8
static struct { void (*fn)(void *); void *arg; } s_deferred[DEFER_QUEUE_LEN];
static unsigned s_deferred_count;
static unsigned s_lock_calls, s_unlock_calls;
static int s_fail_next_runner;

static int defer_runner(void (*fn)(void *arg), void *arg) {
    if (s_fail_next_runner) { s_fail_next_runner = 0; return -1; }
    if (s_deferred_count >= DEFER_QUEUE_LEN) return -1;
    s_deferred[s_deferred_count].fn = fn;
    s_deferred[s_deferred_count].arg = arg;
    s_deferred_count++;
    return 0;
}
static void run_one_deferred(void) {
    MTK_CHECK(s_deferred_count > 0);
    if (s_deferred_count == 0) return;
    s_deferred[0].fn(s_deferred[0].arg);
    for (unsigned i = 1; i < s_deferred_count; i++) s_deferred[i - 1] = s_deferred[i];
    s_deferred_count--;
}
static void lock_fn(void) { s_lock_calls++; }
static void unlock_fn(void) { s_unlock_calls++; }

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_router_set_async_runner(defer_runner);
    mtk_router_set_lock(lock_fn, unlock_fn);

    /* Heap-allocated sink target: valid for as long as the test needs it,
     * unlike every adapter's current stack-local capture struct. */
    mtk_fake_sink_state_t *sink = (mtk_fake_sink_state_t *)malloc(sizeof(mtk_fake_sink_state_t));
    mtk_fake_sink_reset(sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);

    const mtk_opcode_entry_t *op = mtk_test_find_op("AP_SCAN_START");
    mtk_ap_scan_start_req_t req = {0}; req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;

    /* Dispatch returns immediately (deferred), no response yet -- proves
     * the call did not run synchronously on this thread. */
    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink->response.set, 0);
    MTK_CHECK_EQ(s_deferred_count, 1);
    MTK_CHECK(s_lock_calls > 0);

    /* Running the deferred call later (simulating the worker task) is
     * where the response/terminal event actually land -- into the still-
     * valid heap sink, never a freed stack frame. */
    run_one_deferred();
    MTK_CHECK_EQ(sink->response.set, 1);
    MTK_CHECK_EQ(sink->response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK(mtk_fake_find_event(sink, "AP_SCAN_COMPLETE") != NULL);
    free(sink); /* only freed now that we're certain nothing will touch it again */

    /* Pool exhaustion: fill all 4 slots without draining -> the 5th
     * dispatch is a checked, synchronous NO_MEMORY, never a silent drop
     * or an unbounded 5th background task. */
    mtk_fake_sink_state_t *sinks[5];
    for (int i = 0; i < 5; i++) {
        sinks[i] = (mtk_fake_sink_state_t *)malloc(sizeof(mtk_fake_sink_state_t));
        mtk_fake_sink_reset(sinks[i]);
        mtk_request_ctx_t c = mtk_test_ctx(sinks[i], (uint32_t)(10 + i));
        mtk_test_call(&c, op, &req);
    }
    MTK_CHECK_EQ(sinks[4]->response.set, 1);
    MTK_CHECK_EQ(sinks[4]->response.status, MTK_STATUS_NO_MEMORY);
    for (int i = 0; i < 4; i++) MTK_CHECK_EQ(sinks[i]->response.set, 0); /* still pending */
    while (s_deferred_count) run_one_deferred(); /* drain to release all 4 slots */
    for (int i = 0; i < 5; i++) free(sinks[i]);

    /* Runner-creation failure: slot is released, not leaked, and the
     * caller gets a checked NO_MEMORY -- the pool remains fully usable
     * for the next call afterward. */
    mtk_fake_sink_state_t *fsink = (mtk_fake_sink_state_t *)malloc(sizeof(mtk_fake_sink_state_t));
    mtk_fake_sink_reset(fsink);
    mtk_request_ctx_t fctx = mtk_test_ctx(fsink, 99);
    s_fail_next_runner = 1;
    mtk_test_call(&fctx, op, &req);
    MTK_CHECK_EQ(fsink->response.set, 1);
    MTK_CHECK_EQ(fsink->response.status, MTK_STATUS_NO_MEMORY);
    MTK_CHECK_EQ(s_deferred_count, 0);
    free(fsink);

    /* Pool is usable again (the failed slot was released, not stuck). */
    mtk_fake_sink_state_t *again = (mtk_fake_sink_state_t *)malloc(sizeof(mtk_fake_sink_state_t));
    mtk_fake_sink_reset(again);
    mtk_request_ctx_t actx = mtk_test_ctx(again, 100);
    mtk_test_call(&actx, op, &req);
    MTK_CHECK_EQ(s_deferred_count, 1);
    run_one_deferred();
    MTK_CHECK_EQ(again->response.status, MTK_STATUS_ACCEPTED);
    free(again);

    mtk_router_set_async_runner(NULL); /* restore synchronous default for any later test in this binary */
    mtk_router_set_lock(NULL, NULL);

MTK_TEST_MAIN_END
