/* Real cross-thread proof of the async lifetime/delivery mechanism for
 * DEAUTH_START (a required List B feature, explicitly owner-approved) --
 * not a single-threaded simulation. Registers a genuine pthread-based
 * mtk_router async runner + a real pthread_mutex lock, dispatches
 * DEAUTH_START through the full Legacy SPI Compatibility wire path, and proves: the
 * dispatching call returns immediately without blocking on the
 * background thread; the current transaction correctly answers IDLE
 * while genuinely deferred; the persistent, dctx-owned event_queue
 * survives past the dispatching call's return; the real canonical deauth
 * logic actually ran on the OTHER thread (fake HAL reached); the
 * eventual real response is delivered correctly on a later poll with the
 * right operation_token; STOP against a still-pending (not yet accepted)
 * operation is safely rejected, not a race/crash; and the whole sequence
 * is ASan/UBSan- and TSan-clean under real concurrent execution.
 *
 * M4 correction (a genuine TSan data race caught by the required 100-run
 * stress repetition, not by the ordinary single-pass suite): the
 * dispatching call only proves request ACCEPTANCE returned; it does not
 * by itself establish a happens-before edge to the background worker's
 * LATER fake-HAL write. The previous version inferred HAL completion from
 * a fixed worker-side sleep and read g_fake_wifi's shared fields directly
 * from the main thread with no lock and no join -- correct on ordinary
 * schedules, but not actually synchronized, exactly the gap ThreadSanitizer
 * exists to catch. Every cross-thread milestone below is now a real,
 * bounded mutex/condition-variable rendezvous with an explicitly tracked,
 * joined worker; every read or write of the shared fake-HAL fixture goes
 * through fake_wifi_lock/fake_wifi_unlock via one coherent snapshot/reset
 * helper, never a direct field access. */
#include "mtk_test.h"
#include "mtek_compat_dispatch.h"
#include "mtek_compat_opcode_map.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_router_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_router_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_router_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_router_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_router_mutex); }

/* M4 correction: a shared helper for every bounded, condition-variable
 * wait below -- computes an absolute CLOCK_REALTIME deadline
 * `timeout_ms` from now, for use with pthread_cond_timedwait. */
static void abstime_after_ms(struct timespec *ts, long timeout_ms) {
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec += timeout_ms / 1000;
    ts->tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) { ts->tv_nsec -= 1000000000L; ts->tv_sec += 1; }
}

/* M4 correction (diagnosis "lock plus an explicit worker-lifecycle
 * rendezvous"): replaces the detached, sleep-delayed async runner. Each
 * worker is a genuine, tracked, joinable thread that signals `started`,
 * blocks until explicitly `release`d (with the rendezvous mutex NOT held
 * while it then runs the real production callback), then signals `done`.
 * A test-only mutex/condition-variable pair, entirely separate from
 * fake_wifi_lock/router_lock -- never held while production code runs,
 * so it cannot form a lock-order cycle with anything production touches. */
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
 * a test failure AND releases the worker unconditionally -- a worker that
 * never signaled started may still be genuinely parked just after this
 * check ran; releasing it regardless ensures it can never remain stuck
 * waiting on `release` forever, so cleanup cannot hang the test. */
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
 * called by this file after this returns true -- a timeout is a reported
 * test failure, never the first (unbounded, hang-capable) detector. */
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

/* The router's own async-runner slot: exactly one worker is expected to
 * be spawned per mtek_compat_dispatch_request call that defers (this test
 * drives exactly two such calls, worker0 then worker1) -- set immediately
 * before each such call, consumed and cleared by the runner itself. A
 * dispatch that unexpectedly tries to defer with no worker assigned fails
 * closed (returns -1, i.e. NO_MEMORY) rather than silently spawning an
 * untracked, unjoined thread. */
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

/* M4 correction: coherent fixture access. The fake-HAL writer
 * (fake_wifi_send_deauth, mtk_fake_wifi_hal.h) already acquires
 * fake_wifi_lock before writing deauth_sent_count and both MAC fields
 * together; every reader/writer of those same three fields in this file
 * must go through the SAME lock, and must copy all three fields in ONE
 * critical section so the count a caller observes always corresponds to
 * the AP/station values it observes alongside it -- never a torn
 * combination from two separate lock acquisitions. */
typedef struct {
    unsigned count;
    mtk_hal_mac6_t ap;
    mtk_hal_mac6_t station;
} deauth_fixture_snapshot_t;

static deauth_fixture_snapshot_t deauth_fixture_snapshot(void) {
    fake_wifi_lock();
    deauth_fixture_snapshot_t s;
    s.count = g_fake_wifi.deauth_sent_count;
    s.ap = g_fake_wifi.last_deauth_ap;
    s.station = g_fake_wifi.last_deauth_station;
    fake_wifi_unlock();
    return s;
}
static void deauth_fixture_reset_count(void) {
    fake_wifi_lock();
    g_fake_wifi.deauth_sent_count = 0;
    fake_wifi_unlock();
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(tracked_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* Verification fallout (same real TSan-caught gap as
     * test_spi_native_dup_cache.c/test_spi_ native_async.c's own identical fix):
     * a real async runner is registered above, and DEAUTH_START's own completion
     * path (deauth_finalize) genuinely touches shared operation-table/wifi-
     * service/fake-HAL state from a different thread than a concurrent
     * DEAUTH_STOP/status request -- every one of these must share the SAME real
     * mutex as mtk_router_set_lock above. */
    mtk_core_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();

    mtk_compat_dispatch_ctx_t dctx;
    mtek_compat_dispatch_init(&dctx, 0x1234);
    /* The dctx-owned event_queue is genuinely shared across the calling
     * thread and the background worker thread the tracked runner above
     * spawns -- it needs real mutual exclusion, exactly as target glue
     * would register real FreeRTOS mutex hooks. */
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    /* m1esp_deauth_req_t {bssid[6], channel, station[6], count:u16, interval_ms:u16} */
    uint8_t req_payload[6 + 1 + 6 + 2 + 2];
    memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
    req_payload[6] = 6;
    memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req_payload[13] = 1; req_payload[14] = 0; /* count = 1 LE */
    req_payload[15] = 0; req_payload[16] = 0; /* interval_ms = 0 */

    mtk_compat_header_t req_hdr = {0};
    req_hdr.magic = MTK_COMPAT_MAGIC; req_hdr.version = MTK_COMPAT_VERSION;
    req_hdr.msg_type = MTK_COMPAT_MSG_REQ; req_hdr.msg_id = 0x0302; req_hdr.payload_len = sizeof(req_payload);

    mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;

    worker_t worker0, worker1;
    s_next_worker = &worker0;
    mtek_compat_dispatch_request(&dctx, &req_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);

    /* Dispatching returned immediately: the transport loop is never
     * blocked waiting for the background deauth work to finish. */
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
    MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);

    /* worker0 is now genuinely parked -- signaled started, blocked on
     * release, has not yet executed the real production callback at all.
     * This is what actually proves "the real HAL has not been reached
     * yet", replacing the previous fixed-sleep inference with a real
     * happens-before edge (the started signal itself). */
    MTK_CHECK(worker_wait_started(&worker0, 5000));
    deauth_fixture_snapshot_t snap0 = deauth_fixture_snapshot();
    MTK_CHECK_EQ(snap0.count, 0u);
    MTK_CHECK_EQ(dctx.deauth_token, 0); /* not yet minted/harvested */

    /* A STOP against an operation whose token this dispatch layer does
     * not have yet (because the ACCEPTED response is still in flight on
     * the other thread, still parked) is safely rejected -- no race, no
     * crash, no guessed token. */
    mtk_compat_header_t stop_hdr = req_hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
    mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_NAK);
    MTK_CHECK_EQ(resp_payload[0], MTK_COMPAT_STATUS_ERR_NOT_RUNNING);

    /* Release worker0: it now runs the real production callback (with
     * the rendezvous mutex already dropped), reaching the fake HAL and
     * publishing ACCEPTED into the queue. */
    worker_release(&worker0);

    /* Poll (as the physical SPI transaction loop would on repeated IDLE
     * exchanges from the peer) until the real, cross-thread-delivered
     * response arrives. Bounded retry -- this must complete well within
     * a couple of seconds on any real machine; a hang here would itself
     * be a test failure (CTest's own timeout), which is an acceptable,
     * intentional way to catch a genuinely broken delivery path. */
    int got_resp = 0;
    for (int i = 0; i < 2000 && !got_resp; i++) {
        mtek_compat_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        if (resp_hdr.msg_type != MTK_COMPAT_MSG_IDLE) got_resp = 1;
        else usleep(1000);
    }
    MTK_CHECK(got_resp);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
    MTK_CHECK_EQ(resp_hdr.msg_id, 0x0302); /* answers the original DEAUTH_START request */
    MTK_CHECK_EQ(dctx.pending_start_msg_id, 0); /* cleared once delivered */

    /* Consuming the queued ACCEPTED response only orders state that
     * preceded ITS OWN push -- it does not by itself prove the LATER
     * fake-HAL write has happened yet. worker0's own `done` signal is the
     * real happens-before edge for that; only after observing it (and
     * joining the thread) is a snapshot of the fixture actually coherent
     * with "the real canonical deauth logic genuinely ran". */
    MTK_CHECK(worker_wait_done(&worker0, 5000));
    pthread_join(worker0.tid, NULL);

    /* The real canonical deauth logic genuinely ran on the background
     * thread (not the thread that called mtek_compat_dispatch_request):
     * the fake HAL was actually reached, and the operation_token was
     * correctly harvested across the thread boundary via the persistent,
     * dctx-owned event_queue -- not a stack-local capture that would
     * have gone out of scope long before the worker thread got to it.
     * One coherent locked snapshot -- count and both MAC fields together. */
    deauth_fixture_snapshot_t snap1 = deauth_fixture_snapshot();
    MTK_CHECK_EQ(snap1.count, 1u);
    MTK_CHECK(memcmp(snap1.ap.b, req_payload, 6) == 0);
    MTK_CHECK(memcmp(snap1.station.b, req_payload + 7, 6) == 0);
    MTK_CHECK(dctx.deauth_token != 0);

    /* Subsequent poll (nothing else outstanding): well-formed IDLE, not
     * a repeat of the already-delivered response. */
    mtek_compat_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);

    /* Now that the token is real, DEAUTH_STOP works normally (also
     * dispatched synchronously here since STOP is lifecycle SYNCHRONOUS,
     * never deferred by the router). */
    mtek_compat_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
    MTK_CHECK_EQ(dctx.deauth_token, 0);

    /* Cancellation/reset of a still-pending deferred operation: start a
     * second DEAUTH_START (deferred again, worker1 parked exactly like
     * worker0 was), then simulate a boot_epoch reset by resetting the
     * event_queue directly WHILE worker1 is still genuinely in flight --
     * the queue must accept being drained mid-flight without leaving
     * dctx in a state where a later, unrelated poll misinterprets a
     * stale frame as belonging to a new request. */
    {
        deauth_fixture_reset_count();
        s_next_worker = &worker1;
        mtek_compat_dispatch_request(&dctx, &req_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);
        MTK_CHECK(worker_wait_started(&worker1, 5000));

        /* worker1 is parked, genuinely in flight -- reset here, exactly
         * simulating a reset landing in that window. */
        mtk_async_queue_reset(&dctx.event_queue);
        dctx.pending_start_msg_id = 0; /* the adapter's own reset-intent handling would clear this alongside the queue */

        /* Let worker1 actually run and push its (now orphaned) frame
         * into the queue AFTER the reset, proving the queue is safely
         * reusable and does not corrupt state even though nothing is
         * listening for that frame anymore -- a real, bounded, joined
         * completion, never a fixed sleep guessing when it is done. */
        worker_release(&worker1);
        MTK_CHECK(worker_wait_done(&worker1, 5000));
        pthread_join(worker1.tid, NULL);

        /* At most the orphaned ACCEPTED response plus its own terminal
         * DEAUTH_STOPPED event (this short-lived count=1 operation emits
         * both), never more -- no corruption, no duplication, no runaway
         * growth from the reset racing the worker thread. */
        MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) <= 2);
        mtk_async_queue_reset(&dctx.event_queue);
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0);
    }

MTK_TEST_MAIN_END
