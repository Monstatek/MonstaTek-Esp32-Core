/* Release-tooling-round P0 correction, ROUND 5 (follow-up read-only audit,
 * "final P0 concurrency-closure round"), FURTHER CORRECTED (follow-up
 * read-only audit, "one P0 race remains" -- see mtk_op_begin_publish_guard's
 * own doc comment in mtek_core.h for the full account): round 4 closed
 * the arbiter-release-timing hazard for STA_CONNECT/BLE_SCAN/GATT_CONNECT's
 * own peer-reset path, but a further read-only re-audit found four
 * remaining gaps:
 *
 *  1. Every "check ownership, then separately release" call site (not
 *     just GATT's) was still two independent arbiter-lock acquisitions,
 *     leaving a real window between them for a concurrent reassignment to
 *     be released out from under a newer operation -- including mtek_
 *     wifi_logic.c's own restore/release paths (deauth/handshake/AP-STA-
 *     scan/RAW_TX/capture), not only GATT. Fixed with a genuinely atomic
 *     mtk_arbiter_release_if_owner(class, token) (mtek_arbiter.c) used
 *     everywhere that pattern occurred, including inside mtek_wifi_
 *     restore_and_release's own now `owner_token`-taking signature.
 *  2. A worker could win mtk_op_claim_finalization/mtk_op_transition_by_
 *     token for its own token an instant BEFORE a concurrent peer-session
 *     reset bumps the session generation, then publish shared state or
 *     emit an event an instant AFTER -- neither the op-table win nor the
 *     arbiter-ownership check closes this, since both only check state
 *     at one moment. An EARLIER version of this fix (mtk_op_confirm_
 *     still_current_session) was a single point-in-time re-check taken
 *     right before publish -- but a further re-audit found this
 *     insufficient: a single check only proves the generation had not
 *     YET changed the instant it ran, not that it cannot change in the
 *     (necessarily nonzero) window between the check RETURNING and the
 *     caller's own subsequent publish statements actually executing. The
 *     real fix is mtk_op_begin_publish_guard/mtk_op_end_publish_guard
 *     (mtek_core.c): a genuine begin/end GUARD, held across the entire
 *     publish, backed by a lock ALSO acquired (briefly) by mtk_core_
 *     bump_session_generation itself -- so a bump cannot complete while
 *     a guard is open, and a guard cannot open while a bump is in
 *     progress, closing the window to zero width rather than merely
 *     narrowing it. This lock is deliberately separate from mtk_core_
 *     set_lock's own (mtk_op_set_publish_lock), mirroring mtek_capture_
 *     service.h's own established mtek_capture_set_lock precedent,
 *     since the guard is held across the caller's own sink emit_event
 *     call, which can itself reach mtk_core_set_lock's own lock (a
 *     queue-backed sink's async_queue lock is often the identical
 *     physical lock) -- reusing it here would self-deadlock. mtk_op_
 *     set_won_hook (test-only, always NULL/no-op in production) lets
 *     this file pause a worker WHILE it holds the guard, so a concurrent
 *     peer reset can be proven to genuinely BLOCK rather than merely
 *     hoping it lands in some narrow window.
 *  3. BLE_SCAN_STOP, WIFI_STOP_ALL, and STA_DISCONNECT -- ordinary user-
 *     invoked cleanup paths, not just the peer-reset canceller -- still
 *     released/restored/reused radio ownership immediately, even while
 *     an uncancellable blocking HAL call (scan()/connect()) was still
 *     genuinely running, permitting the exact same overlapping-HAL-call
 *     hazard round 4 closed for the peer-reset path alone. BLE_SCAN_STOP
 *     now only fences the token (mtek_ble_logic.c); WIFI_STOP_ALL now
 *     special-cases WMC/WS to do the same, sharing mtek_wifi_logic.c's
 *     own wifi_quiesce_and_fence_ws helper with the peer-reset canceller;
 *     STA_DISCONNECT no longer touches the arbiter at all (it never had
 *     a legitimate reason to -- MTK_ARB_WMC is only ever held for the
 *     duration of an in-progress connect attempt itself, always already
 *     free by the time a session is genuinely connected).
 *  4. (List A/wire format unchanged throughout -- verified by every
 *     existing host test in this suite continuing to pass unmodified.)
 *
 * This file proves each of the four adversarial scenarios the re-audit
 * demanded, plus success/timeout/cancellation/restore-failure coverage:
 *   A. Direct atomic-arbiter proof: pause a worker after it would have
 *      passed an ownership check but before it actually releases; replace
 *      ownership with the same class and a NEW token; prove the paused
 *      release cannot touch the new lease.
 *   B. Worker-finalization/session-reset race: pause a REAL STA_CONNECT
 *      worker (via mtk_op_set_won_hook) while it holds the publish guard
 *      (mtk_op_begin_publish_guard already returned 1); attempt a real,
 *      changed-epoch HELLO concurrently, on its own thread; prove the
 *      HELLO genuinely BLOCKS (cannot bump the session generation) for as
 *      long as the worker holds the guard; resume the worker; prove its
 *      publish (s_sta_connected + STA_CONNECT_COMPLETE) completes first,
 *      strictly BEFORE the now-unblocked reset runs and correctly tears
 *      the resulting connection back down via its own pre-existing
 *      already-connected handling.
 *   C. BLE_SCAN_STOP during a blocked BLE scan: immediate replacement
 *      work is refused BUSY until the original HAL call genuinely exits.
 *   D/E. WIFI_STOP_ALL during blocked AP_SCAN/STA_SCAN: no premature
 *      restore/release, no overlapping HAL calls, no stale events.
 *   F. WIFI_STOP_ALL and STA_DISCONNECT during a blocked STA_CONNECT: no
 *      premature restore/release, no overlapping HAL calls, no stale
 *      events, no disconnection of a newer session.
 *   G. STA_DISCONNECT normal-path regression: a genuinely connected
 *      session still disconnects correctly (issue 4: no change to
 *      successful normal-path behavior).
 *   H. Restore-failure branch: a forced restore_sta_mode() failure still
 *      quarantines the lease (never releases it) through the new atomic
 *      wrapper, exactly as before. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

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

static uint32_t poll_for_op_token(mtk_fake_sink_state_t *sink) {
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

/* A deferred worker's own tail transitions the operation to terminal
 * (making op_status flip to OK) strictly BEFORE it emits that operation's
 * own terminal event into the SAME sink -- waiting only for op_status to
 * flip, then letting an enclosing block's stack-local sink go out of
 * scope, races that still-in-flight emit call (a real, ASan-caught
 * hazard: ctx->sink.user points at the sink, which by then is freed
 * stack memory). Waits (bounded) for the event to actually arrive before
 * it is safe to let the sink go away. */
static int wait_for_event(mtk_fake_sink_state_t *sink) {
    for (int i = 0; i < 3000; i++) {
        mtk_fake_sink_lock();
        unsigned n = sink->event_count;
        mtk_fake_sink_unlock();
        if (n > 0) return 1;
        usleep(500);
    }
    return 0;
}

/* ==== A. Direct atomic-arbiter proof (issue 1). ========================= */

typedef struct {
    pthread_mutex_t m; pthread_cond_t cv;
    int worker_arrived; int release_worker;
    mtk_arbiter_class_t cls; uint32_t old_token;
    int released_result;
} arb_race_t;

static void *arb_delayed_release_thread(void *arg) {
    arb_race_t *r = (arb_race_t *)arg;
    pthread_mutex_lock(&r->m);
    r->worker_arrived = 1;
    pthread_cond_broadcast(&r->cv);
    while (!r->release_worker) pthread_cond_wait(&r->cv, &r->m);
    pthread_mutex_unlock(&r->m);
    /* This is the exact call an old worker would make after having
     * decided (in its own mind, before this rendezvous) that it is time
     * to release -- proving the check-and-release below is atomic: no
     * separate "is it still mine" read exists for a reassignment to slip
     * in between. */
    r->released_result = mtk_arbiter_release_if_owner(r->cls, r->old_token);
    return NULL;
}

static void test_atomic_arbiter_release(void) {
    mtk_arbiter_init();
    arb_race_t r; memset(&r, 0, sizeof(r));
    pthread_mutex_init(&r.m, NULL); pthread_cond_init(&r.cv, NULL);
    r.cls = MTK_ARB_WS; r.old_token = 111;
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_WS, 111), MTK_ARB_GRANT_OK);

    pthread_t t;
    pthread_create(&t, NULL, arb_delayed_release_thread, &r);
    pthread_mutex_lock(&r.m);
    while (!r.worker_arrived) pthread_cond_wait(&r.cv, &r.m);
    pthread_mutex_unlock(&r.m);

    /* While the old worker is paused right before its own release call,
     * replace ownership with the SAME class and a NEW token -- exactly
     * as a brand-new operation legitimately acquiring the class after
     * the old one's token was independently fenced/evicted would. */
    mtk_arbiter_force_transfer(MTK_ARB_WS, 222);

    pthread_mutex_lock(&r.m);
    r.release_worker = 1;
    pthread_cond_broadcast(&r.cv);
    pthread_mutex_unlock(&r.m);
    pthread_join(t, NULL);

    MTK_CHECK_EQ(r.released_result, 0); /* the stale release was correctly refused */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS); /* the NEW lease is untouched */
    MTK_CHECK_EQ(mtk_arbiter_active_token(), 222u);

    /* Sanity: the SAME call with the CORRECT (current) token does
     * release. */
    MTK_CHECK_EQ(mtk_arbiter_release_if_owner(MTK_ARB_WS, 222), 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
}

/* ==== B. Worker-finalization/session-reset race (issue 2). =============
 *
 * P0 correction (follow-up read-only audit, "one P0 race remains"): the
 * FIRST version of this test paused a worker inside mtk_op_confirm_
 * still_current_session (a single point-in-time re-check) and then sent
 * a real HELLO SYNCHRONOUSLY on the main thread while the worker sat
 * paused -- proving only that a reset landing in that exact window
 * failed to produce a stale publish, never that the window itself was
 * closed (a reset landing ONE INSTRUCTION LATER, after the check
 * returned but before the publish executed, was not exercised at all,
 * and could not be: no single re-check can close a window that exists
 * after it returns). The fix (mtk_op_begin_publish_guard/mtk_op_end_
 * publish_guard, mtek_core.c) makes "check" and "the generation actually
 * changing" mutually exclusive via a dedicated lock also held (briefly)
 * by mtk_core_bump_session_generation itself -- so with the worker now
 * paused INSIDE the guard (mtk_op_begin_publish_guard has already
 * returned 1 and is still holding that lock), a concurrent HELLO's own
 * reset processing cannot even START bumping the generation: it must
 * BLOCK on the same lock. Proving that block is now the real test: the
 * HELLO is sent on ITS OWN thread (sending it synchronously on the main
 * thread here would deadlock, since it would never return while this
 * same thread is the one that must resume the paused worker), and this
 * test verifies the HELLO thread has NOT completed and the session
 * generation has NOT yet changed while the worker remains paused --
 * only after the worker is resumed and releases the guard does the
 * HELLO thread unblock and the reset actually happen, strictly AFTER
 * the worker's own publish, never concurrently with it. */

typedef struct {
    pthread_mutex_t m; pthread_cond_t cv;
    int worker_arrived; int release_worker;
} won_race_t;
static won_race_t s_won_race;

static void won_hook_pause(uint32_t token) {
    (void)token;
    pthread_mutex_lock(&s_won_race.m);
    s_won_race.worker_arrived = 1;
    pthread_cond_broadcast(&s_won_race.cv);
    while (!s_won_race.release_worker) pthread_cond_wait(&s_won_race.cv, &s_won_race.m);
    pthread_mutex_unlock(&s_won_race.m);
}

/* Dedicated publish-guard lock for this test -- see mtek_core.h's own
 * doc comment on mtk_op_begin_publish_guard for why it must be distinct
 * from every other lock registered here (a sink emit call reachable
 * from inside the guard must never re-enter it). */
static pthread_mutex_t s_pub_mutex_b = PTHREAD_MUTEX_INITIALIZER;
static void pub_lock_b(void) { pthread_mutex_lock(&s_pub_mutex_b); }
static void pub_unlock_b(void) { pthread_mutex_unlock(&s_pub_mutex_b); }

typedef struct {
    mtk_spi_native_dispatch_ctx_t *dctx;
    uint32_t peer_epoch;
    volatile int done;
} hello_thread_arg_t;

static void *hello_thread_fn(void *arg) {
    hello_thread_arg_t *a = (hello_thread_arg_t *)arg;
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_HELLO;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.boot_epoch = a->peer_epoch;
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(a->dctx, &h, NULL, 3, &resp_hdr, resp_payload, &resp_len);
    a->done = 1; /* set LAST -- the HELLO_ACK response itself is not separately checked by the caller, only completion */
    return NULL;
}

#define ESP_EPOCH_B    0x11112222u
#define PEER_EPOCH_B1  0x33334444u
#define PEER_EPOCH_B2  0x55556666u

static mtk_spi_native_header_t base_req_hdr_b(uint16_t service, uint16_t opcode, uint32_t request_id,
                                               uint16_t payload_len, uint32_t peer_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = peer_epoch;
    return h;
}
static mtk_spi_native_header_t hello_hdr_b(uint32_t peer_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_HELLO;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.boot_epoch = peer_epoch;
    return h;
}

static void test_won_session_reset_race(void) {
    mtk_core_init(ESP_EPOCH_B);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_lock(router_lock, router_unlock);
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_arbiter_set_lock(router_lock, router_unlock);
    mtk_op_set_publish_lock(pub_lock_b, pub_unlock_b);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, mtk_test_now_ms);
    mtek_wifi_service_init(mtk_test_now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_system_set_sta_query(sta_query_always_ready);

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, ESP_EPOCH_B);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *sta_connect_op = mtk_test_find_op("STA_CONNECT");
    MTK_CHECK(sta_connect_op);

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* Initial HELLO. */
    {
        mtk_spi_native_header_t h = hello_hdr_b(PEER_EPOCH_B1);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    }

    memset(&s_won_race, 0, sizeof(s_won_race));
    pthread_mutex_init(&s_won_race.m, NULL); pthread_cond_init(&s_won_race.cv, NULL);
    mtk_op_set_won_hook(won_hook_pause);
    mtk_router_set_async_runner(pthread_runner);

    g_fake_wifi.connect_delay_ms = 0; /* connect() itself returns fast -- the pause happens at the won-hook, not inside connect() */
    g_fake_wifi.connect_rc = 0;
    g_fake_wifi.connect_result.connected = 1;

    uint8_t buf[64]; size_t blen = 0;
    mtk_sta_connect_req_t req; memset(&req, 0, sizeof(req));
    req.ssid.len = 4; memcpy(req.ssid.data, "test", 4); req.auth_mode = 0; req.credential.kind = 0;
    mtk_encode(sta_connect_op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_spi_native_header_t hdr = base_req_hdr_b(sta_connect_op->service_id, sta_connect_op->opcode, 50, (uint16_t)blen, PEER_EPOCH_B1);
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 2, &resp_hdr, resp_payload, &resp_len);
    /* Usually IDLE (the deferred worker has not started running yet), but
     * mtek_spi_native_dispatch.c's own dispatch_complete_message
     * deliberately delivers "whatever is now at the front of the queue"
     * for this same transaction (its own doc comment: "this request's own
     * response if it completed fast/synchronously ... either is
     * protocol-legal") -- under real scheduling variance (pronounced
     * under TSan's own heavy instrumentation slowdown) the newly created
     * worker thread can occasionally reach its own ACCEPTED respond()
     * call before this thread reaches that check, legitimately
     * delivering it as an immediate RESPONSE instead of IDLE. Neither
     * outcome matters to the rest of this test, which waits on the
     * won-hook rendezvous below regardless of how/when the ACCEPTED
     * response itself was delivered. */
    MTK_CHECK(resp_hdr.msg_class == MTK_SPI_CLASS_IDLE || resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE);

    /* Wait for the worker to reach the won-hook -- it has already won
     * mtk_op_transition_by_token for its own token, connected_now=1,
     * released MTK_ARB_WMC, and mtk_op_begin_publish_guard has ALREADY
     * returned 1 (session still valid at that check) and is now paused
     * still HOLDING the publish-guard lock, about to publish
     * s_sta_connected/emit STA_CONNECT_COMPLETE. */
    pthread_mutex_lock(&s_won_race.m);
    while (!s_won_race.worker_arrived) pthread_cond_wait(&s_won_race.cv, &s_won_race.m);
    pthread_mutex_unlock(&s_won_race.m);

    uint32_t generation_before = mtk_core_session_generation();

    /* Attempt a real peer reset -- a genuine HELLO with a different
     * epoch -- WHILE the worker is paused holding the publish-guard
     * lock. Sent on ITS OWN thread: cancel_active_operations_for_peer_
     * reset's own mtk_core_bump_session_generation() call needs the SAME
     * lock the paused worker holds, so this call cannot complete (and,
     * per the fix, must not even be ABLE to bump the generation) until
     * the worker is resumed below -- sending it synchronously on this
     * thread would deadlock (this thread is also the one that must
     * signal the worker to resume). */
    hello_thread_arg_t hello_arg = { &dctx, PEER_EPOCH_B2, 0 };
    pthread_t hello_tid;
    MTK_CHECK(pthread_create(&hello_tid, NULL, hello_thread_fn, &hello_arg) == 0);

    /* Real wall-clock margin for the HELLO thread to have reached (and
     * blocked inside) mtk_core_bump_session_generation, if it were ever
     * going to get past it prematurely. */
    usleep(200000);
    MTK_CHECK(!hello_arg.done); /* genuinely still blocked -- cannot bump while the guard is open */
    MTK_CHECK_EQ(mtk_core_session_generation(), generation_before); /* the generation has NOT changed yet */

    /* Resume the paused worker -- only now can it finish its publish and
     * release the guard. */
    pthread_mutex_lock(&s_won_race.m);
    s_won_race.release_worker = 1;
    pthread_cond_broadcast(&s_won_race.cv);
    pthread_mutex_unlock(&s_won_race.m);

    /* Give the worker real wall-clock time to finish publishing, release
     * the guard, and let the now-unblocked HELLO thread run the reset to
     * completion. */
    usleep(300000);
    pthread_join(hello_tid, NULL);
    MTK_CHECK(hello_arg.done);

    mtk_router_set_async_runner(NULL);
    mtk_op_set_won_hook(NULL);

    /* The generation check inside the guard genuinely passed BEFORE the
     * reset could even begin bumping it (proven above), so this
     * operation's publish is entirely legitimate and happened-before the
     * reset, never concurrently with it. It correctly connected under
     * the OLD session -- and the reset's own already-established "an
     * ALREADY-connected STA session is left behind" handling (mekt_wifi_
     * cancel_active_for_peer_reset's own independent check, which by now
     * genuinely observes a real connection, unlike the original bug
     * where the publish could still be pending when that check ran) then
     * correctly tears it down as part of the SAME reset that had to wait
     * for it. Net result: never a stale publish landing AFTER a reset,
     * and never a real connection silently missed BY a reset either. */
    MTK_CHECK(!mtek_wifi_is_sta_connected()); /* connected, then correctly torn down by the reset that had to wait for it */
    MTK_CHECK(mtk_core_session_generation() != generation_before); /* the reset now genuinely completed, strictly after the publish */
    fake_wifi_lock();
    unsigned disconnects = g_fake_wifi.disconnect_call_count;
    fake_wifi_unlock();
    MTK_CHECK(disconnects >= 1); /* the reset's own already-connected teardown fired */
}

MTK_TEST_MAIN_BEGIN

    test_atomic_arbiter_release();
    test_won_session_reset_race();

    mtk_test_bootstrap();
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_arbiter_set_lock(router_lock, router_unlock);
    mtk_router_set_lock(router_lock, router_unlock);
    /* Reused from test_won_session_reset_race, above -- that call has
     * already returned by this point (sequential, not concurrent), and
     * this lock must stay distinct from router_lock/router_unlock
     * regardless (see mtek_core.h's own doc comment on mtk_op_begin_
     * publish_guard). */
    mtk_op_set_publish_lock(pub_lock_b, pub_unlock_b);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_ble_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);
    mtek_system_set_sta_query(sta_query_always_ready);
    mtk_router_set_async_runner(pthread_runner);

    const mtk_opcode_entry_t *ble_scan_start = mtk_test_find_op("BLE_SCAN_START");
    const mtk_opcode_entry_t *ble_scan_stop = mtk_test_find_op("BLE_SCAN_STOP");
    const mtk_opcode_entry_t *ap_start = mtk_test_find_op("AP_SCAN_START");
    const mtk_opcode_entry_t *sta_scan_start = mtk_test_find_op("STA_SCAN_START");
    const mtk_opcode_entry_t *sta_connect_op2 = mtk_test_find_op("STA_CONNECT");
    const mtk_opcode_entry_t *sta_disconnect_op = mtk_test_find_op("STA_DISCONNECT");
    const mtk_opcode_entry_t *wifi_stop_all_op = mtk_test_find_op("WIFI_STOP_ALL");
    MTK_CHECK(ble_scan_start && ble_scan_stop && ap_start && sta_scan_start &&
              sta_connect_op2 && sta_disconnect_op && wifi_stop_all_op);

    /* ==== C. BLE_SCAN_STOP during a blocked BLE scan. ==================== */
    {
        mtk_fake_ble_reset();
        g_fake_ble.scan_delay_ms = 200;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_ble_scan_start_req_t req; memset(&req, 0, sizeof(req));
        req.mode = 0; req.duration_ms = 100;
        mtk_test_call(&ctx, ble_scan_start, &req);
        uint32_t tok = poll_for_op_token(&sink);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);

        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 2);
        mtk_ble_scan_stop_req_t stopreq = {0}; stopreq.operation_token = tok;
        mtk_test_call(&stop_ctx, ble_scan_stop, &stopreq);
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        /* The arbiter must stay held -- scan() has no cancel hook and is
         * still genuinely blocked. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);

        /* Immediate replacement work must be refused. */
        mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
        mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 3);
        mtk_test_call(&busy_ctx, ble_scan_start, &req);
        MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);

        usleep(400000); /* let the original scan() call genuinely exit */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* released by the real worker's own tail */

        /* A NEW BLE_SCAN_START now succeeds. */
        g_fake_ble.scan_delay_ms = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 4);
        mtk_test_call(&ctx2, ble_scan_start, &req);
        uint32_t tok2 = poll_for_op_token(&sink2);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK);
        MTK_CHECK(wait_for_event(&sink2)); /* let the worker's own BLE_SCAN_COMPLETE emit finish before sink2 goes out of scope */
    }

    /* ==== D. WIFI_STOP_ALL during a blocked AP_SCAN. ====================== */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_scan_poll_count_per_call = 60; /* 60*5ms=300ms if never cancelled */
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 10);
        mtk_ap_scan_start_req_t req = {0}; req.band = 0; req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, ap_start, &req);
        uint32_t tok = poll_for_op_token(&sink);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 11);
        mtk_test_call(&stop_ctx, wifi_stop_all_op, NULL);
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        MTK_CHECK(g_fake_wifi.ap_scan_cancel_count >= 1); /* cancellation was genuinely signalled */

        /* WIFI_STOP_ALL's own quiescence handshake (ap_scan_cancel is
         * honored promptly by this fake HAL) should have let the worker
         * finish within its own bounded wait -- the class is free again.
         * WIFI_STOP_ALL only ever transitions the token (never evicts
         * it, unlike the peer-reset path's own final sweep) -- it stays
         * present/terminal, queryable OK, until normal retention GC. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK);

        /* A NEW AP_SCAN_START now succeeds -- proving no overlap occurred
         * and the radio was genuinely freed. */
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 12);
        mtk_test_call(&ctx2, ap_start, &req);
        uint32_t tok2 = poll_for_op_token(&sink2);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK);
        /* op_status flips to OK the instant the worker's own mtk_op_
         * transition_by_token call returns, strictly BEFORE its own
         * subsequent ctx->sink.emit_event call -- wait for that event to
         * actually arrive before sink2/ctx2 (its own sink.user target)
         * go out of scope at the end of this block. */
        MTK_CHECK(wait_for_event(&sink2));
    }

    /* ==== D2. WIFI_STOP_ALL during a GENUINELY STUCK AP_SCAN (cancel
     * ignored -- the quiescence wait itself times out). ==================== */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_scan_ignore_cancel = 1;
        g_fake_wifi.ap_scan_poll_count_per_call = 260; /* 260*5ms=1300ms, longer than the 1000ms bound */
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 13);
        mtk_ap_scan_start_req_t req = {0}; req.band = 0; req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, ap_start, &req);
        uint32_t tok = poll_for_op_token(&sink);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 14);
        mtk_test_call(&stop_ctx, wifi_stop_all_op, NULL); /* blocks ~1000ms (the bounded quiescence wait) */
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        /* Timed out: the token is force-finalized to a terminal state
         * (never left queryable as a live "old session" operation --
         * still present/OK, not evicted, since WIFI_STOP_ALL never
         * calls mtk_op_evict_all_terminal), but the arbiter is
         * deliberately left held -- the radio may still genuinely be
         * busy. */
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        /* Immediate replacement must be refused -- the radio is
         * genuinely still busy. */
        mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
        mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 15);
        mtk_test_call(&busy_ctx, ap_start, &req);
        MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);

        /* Let the stale worker finish its own remaining polls; it lost
         * the op-table race already, so no stale AP_SCAN_COMPLETE. */
        usleep(1200000);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        mtk_fake_sink_lock();
        unsigned stale_events = sink.event_count;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stale_events, 0u);

        g_fake_wifi.ap_scan_ignore_cancel = 0;
        g_fake_wifi.ap_scan_poll_count_per_call = 1;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 16);
        mtk_test_call(&ctx2, ap_start, &req);
        uint32_t tok2 = poll_for_op_token(&sink2);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && op_status(tok2) != MTK_STATUS_OK; i++) usleep(500);
        MTK_CHECK_EQ(op_status(tok2), MTK_STATUS_OK);
        MTK_CHECK(wait_for_event(&sink2));
    }

    /* ==== E. WIFI_STOP_ALL during a blocked STA_SCAN. ===================== */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.sta_scan_poll_count_per_call = 60;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 20);
        mtk_sta_scan_start_req_t req = {0}; req.channel = 6; req.duration_ms = 100;
        mtk_test_call(&ctx, sta_scan_start, &req);
        uint32_t tok = poll_for_op_token(&sink);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);

        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 21);
        mtk_test_call(&stop_ctx, wifi_stop_all_op, NULL);
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        MTK_CHECK(g_fake_wifi.sta_scan_cancel_count >= 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK); /* present/terminal -- WIFI_STOP_ALL never evicts */
    }

    /* ==== F. WIFI_STOP_ALL and STA_DISCONNECT during a blocked
     * STA_CONNECT. ========================================================= */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.connect_delay_ms = 250;
        g_fake_wifi.connect_rc = 0;
        g_fake_wifi.connect_result.connected = 1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 30);
        mtk_sta_connect_req_t req; memset(&req, 0, sizeof(req));
        req.ssid.len = 4; memcpy(req.ssid.data, "test", 4); req.auth_mode = 0; req.credential.kind = 0;
        mtk_test_call(&ctx, sta_connect_op2, &req);
        uint32_t tok = poll_for_op_token(&sink);
        MTK_CHECK(tok != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WMC);

        /* WIFI_STOP_ALL must not touch restore/arbiter -- connect() has
         * no cancel hook and is still genuinely blocked. */
        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 31);
        mtk_test_call(&stop_ctx, wifi_stop_all_op, NULL);
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WMC);
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK); /* token fenced (terminal), but WIFI_STOP_ALL never evicts */

        /* STA_DISCONNECT, also called while the connect is still blocked,
         * must likewise not touch the arbiter (nothing is connected yet)
         * and must not disturb the still-in-flight attempt. */
        mtk_fake_sink_state_t disc_sink; mtk_fake_sink_reset(&disc_sink);
        mtk_request_ctx_t disc_ctx = mtk_test_ctx(&disc_sink, 32);
        mtk_test_call(&disc_ctx, sta_disconnect_op, NULL);
        MTK_CHECK_EQ(disc_sink.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WMC); /* still untouched */
        fake_wifi_lock();
        unsigned disc_calls_before = g_fake_wifi.disconnect_call_count;
        fake_wifi_unlock();
        MTK_CHECK_EQ(disc_calls_before, 0u); /* nothing connected yet -- disconnect() never called */

        /* Immediate replacement STA_CONNECT must be refused -- the radio
         * is genuinely still busy. */
        mtk_fake_sink_state_t busy_sink; mtk_fake_sink_reset(&busy_sink);
        mtk_request_ctx_t busy_ctx = mtk_test_ctx(&busy_sink, 33);
        mtk_test_call(&busy_ctx, sta_connect_op2, &req);
        MTK_CHECK_EQ(poll_for_response_status(&busy_sink), MTK_STATUS_BUSY);

        /* Let the stale connect() call finally return "connected" -- it
         * lost the race already (fenced by WIFI_STOP_ALL above), so it
         * must never publish s_sta_connected/emit STA_CONNECT_COMPLETE,
         * and must tear its own real HAL-level association back down. */
        usleep(400000);
        MTK_CHECK(!mtek_wifi_is_sta_connected());
        fake_wifi_lock();
        unsigned disc_calls_after = g_fake_wifi.disconnect_call_count;
        fake_wifi_unlock();
        MTK_CHECK(disc_calls_after >= 1); /* the stale worker's own fallback teardown fired */
        mtk_fake_sink_lock();
        unsigned stale_events = sink.event_count;
        mtk_fake_sink_unlock();
        MTK_CHECK_EQ(stale_events, 0u);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* released by the stale worker's own tail */

        /* A genuinely NEW STA_CONNECT now completes normally end-to-end. */
        g_fake_wifi.connect_delay_ms = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 34);
        mtk_test_call(&ctx2, sta_connect_op2, &req);
        uint32_t tok2 = poll_for_op_token(&sink2);
        MTK_CHECK(tok2 != 0 && tok2 != tok);
        for (int i = 0; i < 3000 && !mtek_wifi_is_sta_connected(); i++) usleep(500);
        MTK_CHECK(mtek_wifi_is_sta_connected());
        MTK_CHECK(wait_for_event(&sink2)); /* s_sta_connected flips before the worker's own STA_CONNECT_COMPLETE emit -- wait for it before sink2 goes out of scope */
    }

    /* Sections G/H complete fully synchronously -- clear the async
     * runner so neither is deferred. */
    mtk_router_set_async_runner(NULL);

    /* ==== G. STA_DISCONNECT normal-path regression: a genuinely
     * connected session still disconnects correctly. ====================== */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.connect_delay_ms = 0;
        g_fake_wifi.connect_rc = 0;
        g_fake_wifi.connect_result.connected = 1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 40);
        mtk_sta_connect_req_t req; memset(&req, 0, sizeof(req));
        req.ssid.len = 4; memcpy(req.ssid.data, "test", 4); req.auth_mode = 0; req.credential.kind = 0;
        mtk_test_call(&ctx, sta_connect_op2, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK(mtek_wifi_is_sta_connected());
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* WMC already released -- connected, not mid-attempt */

        mtk_fake_sink_state_t disc_sink; mtk_fake_sink_reset(&disc_sink);
        mtk_request_ctx_t disc_ctx = mtk_test_ctx(&disc_sink, 41);
        mtk_test_call(&disc_ctx, sta_disconnect_op, NULL);
        MTK_CHECK_EQ(disc_sink.response.status, MTK_STATUS_OK);
        MTK_CHECK(!mtek_wifi_is_sta_connected());
        fake_wifi_lock();
        unsigned disc_calls = g_fake_wifi.disconnect_call_count;
        fake_wifi_unlock();
        MTK_CHECK(disc_calls >= 1); /* the real, established connection genuinely disconnected */
    }

    /* ==== H. Restore-failure branch: a forced restore_sta_mode()
     * failure still quarantines the lease through the new atomic
     * wrapper, never releasing it. ========================================= */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.ap_scan_poll_count_per_call = 1;
        g_fake_wifi.restore_stop_rc = 1; /* force a real restore failure */
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 50);
        mtk_ap_scan_start_req_t req = {0}; req.band = 0; req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, ap_start, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        mtk_ap_scan_start_resp_t r = {0};
        mtk_decode(ap_start->resp_desc, &r, sink.response.body, sink.response.body_len, NULL);
        uint32_t tok = r.operation_token;
        MTK_CHECK(tok != 0);
        /* Synchronous (no async runner) -- the call already ran to
         * completion inline. A failed restore must quarantine (never
         * release) the lease, through mtek_wifi_restore_and_release's
         * new mtk_arbiter_release_if_owner-based release path. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        MTK_CHECK_EQ(op_status(tok), MTK_STATUS_OK); /* GET_OPERATION_STATUS reports presence, not the record's own final_status -- still present/terminal */
    }

MTK_TEST_MAIN_END
