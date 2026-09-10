/* Real cross-thread proof that the async lifetime/delivery pattern now
 * proven for DEAUTH_START (test_compat_async_deauth.c) genuinely extends
 * to every other ACCEPTED_ASYNC opcode this dispatch layer translates,
 * using a real pthread-based mtk_router async runner. Covers: ordering
 * (two sequential deferred operations complete in the order they were
 * issued, never interleaved/corrupted), cancellation (STOP against a
 * still-pending operation is safely rejected for HANDSHAKE_START, same
 * as DEAUTH_START), reset (an operation's dctx-owned event_queue is safe
 * to reset mid-flight and reusable afterward), and overflow/backpressure
 * (a worker task producing more frames than the bounded queue holds
 * never corrupts state, and the accept response itself is always
 * delivered even under queue pressure from other traffic).
 *
 * Correction: the previous version used a fixed-sleep, detached async
 * runner and asserted each dispatch returned IDLE. That assumed the
 * worker always defers -- but dispatch_start_track_token_async
 * (mtek_compat_dispatch.c) explicitly and correctly ALSO completes
 * synchronously when a real worker happens to run fast enough to push its
 * ACCEPTED before the dispatching call's own pop_response_frame check. A
 * loaded run (e.g. ThreadSanitizer under CPU contention) makes the worker
 * win that race, so the IDLE assertion failed intermittently even though
 * production was behaving correctly, and a worker left running across a
 * sub-test boundary could push a late frame that the next sub-test read
 * back as its own reply. Every deferred dispatch below now uses the same
 * bounded mutex/condition-variable rendezvous as test_compat_async_deauth.c:
 * the tracked worker signals `started`, blocks until explicitly released,
 * then runs the real production callback and signals `done`. The test
 * observes IDLE while the worker is provably still parked (so the
 * deferral is deterministic, not a timing accident), releases it, drains
 * the real RESP, then joins it before the next sub-test begins. No
 * assertion's meaning changed; the scheduling nondeterminism was removed. */
#include "mtk_test.h"
#include "mtek_compat_dispatch.h"
#include "mtek_compat_opcode_map.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_capture_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

/* Absolute CLOCK_REALTIME deadline `timeout_ms` from now, for use with
 * pthread_cond_timedwait -- shared by every bounded wait below. */
static void abstime_after_ms(struct timespec *ts, long timeout_ms) {
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec += timeout_ms / 1000;
    ts->tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) { ts->tv_nsec -= 1000000000L; ts->tv_sec += 1; }
}

/* Release-gated worker (identical contract to test_compat_async_deauth.c):
 * a genuine, tracked, joinable thread that signals `started`, blocks until
 * explicitly `release`d (with the rendezvous mutex NOT held while it then
 * runs the real production callback), then signals `done`. Its mutex/cv is
 * entirely separate from router_lock/queue_lock -- never held while
 * production code runs, so it cannot form a lock-order cycle. */
typedef struct {
    pthread_mutex_t m;
    pthread_cond_t cv;
    int started;
    int release;
    int done;
} worker_rv_t;

typedef struct {
    worker_rv_t rv;
    void (*fn)(void *arg);
    void *arg;
    pthread_t tid;
} worker_t;

static void *worker_trampoline(void *arg) {
    worker_t *w = (worker_t *)arg;
    pthread_mutex_lock(&w->rv.m);
    w->rv.started = 1;
    pthread_cond_broadcast(&w->rv.cv);
    while (!w->rv.release) pthread_cond_wait(&w->rv.cv, &w->rv.m);
    pthread_mutex_unlock(&w->rv.m); /* dropped BEFORE executing production code */

    w->fn(w->arg);

    pthread_mutex_lock(&w->rv.m);
    w->rv.done = 1;
    pthread_cond_broadcast(&w->rv.cv);
    pthread_mutex_unlock(&w->rv.m);
    return NULL;
}

/* Bounded wait (up to timeout_ms) for `started`. A timeout is reported as
 * a test failure AND releases the worker unconditionally, so a worker that
 * was parked just after this check can never remain stuck on `release`
 * forever and hang cleanup. */
static int worker_wait_started(worker_t *w, long timeout_ms) {
    struct timespec deadline; abstime_after_ms(&deadline, timeout_ms);
    pthread_mutex_lock(&w->rv.m);
    while (!w->rv.started) {
        if (pthread_cond_timedwait(&w->rv.cv, &w->rv.m, &deadline) != 0) break;
    }
    int started = w->rv.started;
    pthread_mutex_unlock(&w->rv.m);
    if (!started) {
        MTK_CHECK(0);
        pthread_mutex_lock(&w->rv.m);
        w->rv.release = 1;
        pthread_cond_broadcast(&w->rv.cv);
        pthread_mutex_unlock(&w->rv.m);
    }
    return started;
}

static void worker_release(worker_t *w) {
    pthread_mutex_lock(&w->rv.m);
    w->rv.release = 1;
    pthread_cond_broadcast(&w->rv.cv);
    pthread_mutex_unlock(&w->rv.m);
}

/* Bounded wait (up to timeout_ms) for `done`. pthread_join is only ever
 * called after this returns true -- a timeout is a reported test failure,
 * never the first (unbounded, hang-capable) detector. */
static int worker_wait_done(worker_t *w, long timeout_ms) {
    struct timespec deadline; abstime_after_ms(&deadline, timeout_ms);
    pthread_mutex_lock(&w->rv.m);
    while (!w->rv.done) {
        if (pthread_cond_timedwait(&w->rv.cv, &w->rv.m, &deadline) != 0) break;
    }
    int done = w->rv.done;
    pthread_mutex_unlock(&w->rv.m);
    MTK_CHECK(done);
    return done;
}

/* The router's async-runner slot: exactly one worker is expected per
 * mtek_compat_dispatch_request call that defers -- set immediately before
 * each such call, consumed and cleared by the runner itself. A dispatch
 * that unexpectedly tries to defer with no worker assigned fails closed
 * (returns -1, i.e. NO_MEMORY) rather than silently spawning an untracked,
 * unjoined thread. */
static worker_t *s_next_worker;
static int tracked_runner(void (*fn)(void *arg), void *arg) {
    worker_t *w = s_next_worker;
    s_next_worker = NULL;
    if (!w) return -1;
    memset(&w->rv, 0, sizeof(w->rv));
    pthread_mutex_init(&w->rv.m, NULL);
    pthread_cond_init(&w->rv.cv, NULL);
    w->fn = fn; w->arg = arg;
    if (pthread_create(&w->tid, NULL, worker_trampoline, w) != 0) return -1;
    return 0;
}

/* Bounded poll for the real, cross-thread-delivered response, exactly as
 * the physical SPI transaction loop would on repeated IDLE exchanges. A
 * hang here would itself be a test failure (CTest's own timeout), an
 * acceptable way to catch a genuinely broken delivery path. */
static int poll_until_resp(mtk_compat_dispatch_ctx_t *dctx, mtk_compat_header_t *resp_hdr,
                           uint8_t *resp_payload, uint16_t *resp_len) {
    for (int i = 0; i < 2000; i++) {
        mtek_compat_dispatch_poll_outbound(dctx, resp_hdr, resp_payload, resp_len);
        if (resp_hdr->msg_type != MTK_COMPAT_MSG_IDLE) return 1;
        usleep(1000);
    }
    return 0;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(tracked_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* RC11 independent correction order P0 verification fallout (same
     * real TSan-caught gap as test_spi_native_dup_cache.c/test_spi_
     * native_async.c's own identical fix): a real async runner is
     * registered above, and this file exercises both DEAUTH_START and
     * CAPTURE_START's own completion paths, each genuinely touching
     * shared operation-table/wifi-service/capture-service/fake-HAL state
     * from a different thread than a concurrent STOP/status request --
     * every one of these must share the SAME real mutex as mtk_router_
     * set_lock above. */
    mtk_core_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtek_capture_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtek_capture_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_capture_service_register();

    mtk_compat_dispatch_ctx_t dctx;
    mtek_compat_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;

    /* ---- HANDSHAKE_START: deferred, delivered later, STOP-while-pending
     * rejected (same cancellation-safety pattern as DEAUTH_START). ---- */
    {
        uint8_t hs_payload[9] = {1,2,3,4,5,6, 6, 0,0};
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION; hdr.msg_type = MTK_COMPAT_MSG_REQ;
        hdr.msg_id = 0x0310; hdr.payload_len = sizeof(hs_payload);

        worker_t w;
        s_next_worker = &w;
        mtek_compat_dispatch_request(&dctx, &hdr, hs_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE); /* genuinely deferred */
        MTK_CHECK_EQ(dctx.handshake_token, 0);
        /* worker is now provably parked (started, blocked on release, real
         * production callback not yet run) -- so the IDLE above was a real
         * deferral, not the worker simply not having been scheduled yet. */
        MTK_CHECK(worker_wait_started(&w, 5000));

        /* Cancellation: STOP against the still-pending (not yet accepted) operation. */
        mtk_compat_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0313; stop_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_NAK);
        MTK_CHECK_EQ(resp_payload[0], MTK_COMPAT_STATUS_ERR_NOT_RUNNING);

        worker_release(&w); /* worker now runs the real callback, publishing ACCEPTED */
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_hdr.msg_id, 0x0310);
        MTK_CHECK(worker_wait_done(&w, 5000));
        pthread_join(w.tid, NULL); /* happens-before edge for the fields read below */
        MTK_CHECK(dctx.handshake_token != 0);
        MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 1); /* real HAL reached on the worker thread */

        /* Clean up so the next sub-test starts from a known state. */
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.handshake_token, 0);
    }

    /* ---- CAPTURE_START: deferred delivery of the special raw-4-byte-
     * errno response format (not the generic bare-status convention) --
     * proves the per-opcode response-shape override survives deferral. */
    {
        uint8_t cap_payload[2] = {0, 0}; /* channel=0(hop), band=0 */
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION; hdr.msg_type = MTK_COMPAT_MSG_REQ;
        hdr.msg_id = 0x0300; hdr.payload_len = sizeof(cap_payload);

        worker_t w;
        s_next_worker = &w;
        mtek_compat_dispatch_request(&dctx, &hdr, cap_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.capture_token, 0);
        MTK_CHECK(worker_wait_started(&w, 5000));

        worker_release(&w);
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 4); /* raw 4-byte LE esp_err_t, not a bare 0-byte status */
        MTK_CHECK(worker_wait_done(&w, 5000));
        pthread_join(w.tid, NULL);
        MTK_CHECK(dctx.capture_token != 0);

        mtk_compat_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0301; stop_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.capture_token, 0);
    }

    /* ---- Ordering: two sequential deferred DEAUTH_START/STOP operations
     * complete in issue order, never interleaved. ---- */
    {
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION; hdr.msg_type = MTK_COMPAT_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);
        mtk_compat_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;

        /* First deferred DEAUTH_START, driven to completion in issue order. */
        worker_t wa;
        s_next_worker = &wa;
        mtek_compat_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK(worker_wait_started(&wa, 5000));
        worker_release(&wa);
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_hdr.msg_id, 0x0302); /* the FIRST deferred op's own reply, not a stale/mixed one */
        MTK_CHECK(worker_wait_done(&wa, 5000));
        pthread_join(wa.tid, NULL);
        MTK_CHECK(dctx.deauth_token != 0);
        uint32_t first_token = dctx.deauth_token;

        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.deauth_token, 0);

        /* A second, independent deferred DEAUTH_START gets a genuinely
         * new token, proving no state bled across the two operations. */
        worker_t wb;
        s_next_worker = &wb;
        mtek_compat_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK(worker_wait_started(&wb, 5000));
        worker_release(&wb);
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK(worker_wait_done(&wb, 5000));
        pthread_join(wb.tid, NULL);
        MTK_CHECK(dctx.deauth_token != 0);
        MTK_CHECK(dctx.deauth_token != first_token);
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    }

    /* ---- Reset: dctx's event_queue is safe to reset mid-flight (e.g. a
     * boot_epoch reset) and fully reusable afterward for an unrelated
     * later operation. ---- */
    {
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION; hdr.msg_type = MTK_COMPAT_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);

        worker_t worb;
        s_next_worker = &worb;
        mtek_compat_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);
        MTK_CHECK(worker_wait_started(&worb, 5000));

        /* worker is parked, genuinely in flight -- reset lands here,
         * exactly simulating a reset arriving in that window. */
        mtk_async_queue_reset(&dctx.event_queue); /* simulated reset-intent */
        dctx.pending_start_msg_id = 0;

        /* Let the orphaned worker actually run and push its (now orphaned)
         * frame into the queue AFTER the reset -- a real, bounded, joined
         * completion, never a fixed sleep guessing when it is done. */
        worker_release(&worb);
        MTK_CHECK(worker_wait_done(&worb, 5000));
        pthread_join(worb.tid, NULL);

        /* At most the orphaned ACCEPTED response plus its own terminal
         * DEAUTH_STOPPED event (this short-lived count=1 operation emits
         * both, same as test_compat_async_deauth.c's own reset sub-test),
         * never more -- no corruption, no duplication. */
        MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) <= 2);
        mtk_async_queue_reset(&dctx.event_queue);
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0);

        /* Reusable: a fresh operation on the same dctx works normally. */
        worker_t wfresh;
        s_next_worker = &wfresh;
        mtek_compat_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK(worker_wait_started(&wfresh, 5000));
        worker_release(&wfresh);
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK(worker_wait_done(&wfresh, 5000));
        pthread_join(wfresh.tid, NULL);
        MTK_CHECK(dctx.deauth_token != 0);
        mtk_compat_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    }

    /* ---- Overflow/backpressure: the underlying mtk_async_queue's own
     * bounded-drop behavior (proven generically in test_async_queue.c)
     * is exercised here at the adapter level by flooding dctx's queue
     * with synthetic frames before a real dispatch, proving the real
     * accept response is still correctly found/delivered despite queue
     * pressure from unrelated traffic, and the drop counter is honest. */
    {
        mtk_async_queue_reset(&dctx.event_queue);
        for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
            mtk_async_frame_t f; memset(&f, 0, sizeof(f));
            f.kind = MTK_ASYNC_FRAME_EVENT;
            mtk_async_queue_push(&dctx.event_queue, &f);
        }
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), MTK_ASYNC_QUEUE_DEPTH);
        /* RC7 independent audit P0 "Native scheduling can starve or drop
         * control and terminal traffic": mtk_async_queue_push now evicts
         * a lower-priority occupied slot to make room for a higher-
         * priority arrival (RESPONSE > EVENT > STREAM) -- `overflow_frame`
         * must match the fill kind (EVENT) to prove the genuinely-full
         * same-priority case here; priority eviction itself is proven
         * directly in test_async_queue.c. */
        mtk_async_frame_t overflow_frame; memset(&overflow_frame, 0, sizeof(overflow_frame));
        overflow_frame.kind = MTK_ASYNC_FRAME_EVENT;
        MTK_CHECK_EQ(mtk_async_queue_push(&dctx.event_queue, &overflow_frame), 0); /* dropped, queue full */
        MTK_CHECK(mtk_async_queue_dropped_count(&dctx.event_queue) >= 1);

        /* dispatch_start_track_token_async's own mtk_async_queue_reset
         * before every fresh dispatch (see mtek_compat_dispatch.c) means a
         * real new operation is never blocked by stale queue pressure
         * from unrelated prior traffic -- proven here directly. */
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION; hdr.msg_type = MTK_COMPAT_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);

        worker_t w;
        s_next_worker = &w;
        mtek_compat_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK(worker_wait_started(&w, 5000));
        worker_release(&w);
        MTK_CHECK(poll_until_resp(&dctx, &resp_hdr, resp_payload, &resp_len));
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK(worker_wait_done(&w, 5000));
        pthread_join(w.tid, NULL);
        MTK_CHECK(dctx.deauth_token != 0);
        mtk_compat_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    }

MTK_TEST_MAIN_END
