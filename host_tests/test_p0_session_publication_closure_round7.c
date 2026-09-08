/* Release-tooling-round P0 correction, ROUND 7 (follow-up read-only audit,
 * "next focused P0 session-publication closure round"): rounds 5/6 closed
 * the publish/reset race for STA_CONNECT alone (mtk_op_begin_publish_guard,
 * test_p0_concurrency_closure_round5.c's own test_won_session_reset_race).
 * This round extends the SAME guard to every other delayed/background
 * producer this tree defines -- deauth's own natural-completion tail,
 * handshake's hs_frame_cb/handshake_finish, MonstaShark capture's frame_cb/
 * mtek_capture_channel_hop_tick/capture_teardown, BLE's mtek_ble_signal_
 * meter_tick, and GATT's mtek_ble_gatt_tick -- and adds a genuinely new
 * dedicated lock domain (mtek_ble_service_set_lock) protecting s_sig/
 * s_gatt, which previously had no lock of any kind at all.
 *
 * This file proves, for each of the five producer classes named above:
 *   A. A real changed-epoch HELLO landing while the producer is paused
 *      INSIDE mtk_op_begin_publish_guard (mtk_op_set_won_hook, exactly
 *      round 5's own test seam) genuinely BLOCKS -- the session generation
 *      does not change until the producer resumes and releases the guard
 *      -- and the producer's own already-legitimate publish then completes
 *      strictly BEFORE the now-unblocked reset's own cleanup runs.
 *   B. A producer whose long-lived session struct was captured under an
 *      OLDER session_generation, invoked AFTER a real HELLO has already
 *      bumped the generation (this exact struct possibly already torn
 *      down by the SAME reset's own per-service cancel-for-peer-reset
 *      call), publishes nothing at all -- mtk_op_begin_publish_guard
 *      itself refuses before any sink call is ever reached -- and a
 *      genuinely NEW session of the same kind, started immediately
 *      afterward under the new generation, works normally end-to-end
 *      with no leftover state from the old one.
 * Plus two smaller, direct proofs:
 *   C. Every non-native adapter's request carries session_generation==0
 *      (mtk_test_ctx's own established convention) -- mtk_op_begin_
 *      publish_guard(0) always succeeds, completely unaffected by however
 *      many real peer-session resets have happened in the meantime.
 *   D. mtk_op_set_publish_lock(NULL, NULL) / mtek_ble_service_set_lock
 *      (NULL, NULL) never CRASH: every lock/unlock call the guard and the
 *      BLE lock domain make is itself NULL-checked (mtek_core.c's pub_
 *      lock/pub_unlock, mtek_ble_logic.c's ble_lock/ble_unlock), so an
 *      unregistered lock degrades to a no-op critical section rather than
 *      invoking UB on a NULL FreeRTOS handle -- proven here by running a
 *      full SIGNAL_METER_START/tick/STOP cycle with both explicitly
 *      unregistered.
 *
 *      P0 correction (follow-up read-only audit, "Round 8: final
 *      concurrency and resource-failure closure", item 5): an EARLIER
 *      version of this doc comment (and of main/app_main.c's own design)
 *      mischaracterized this as itself a SAFE degraded mode on a real
 *      target. It is not: this proof establishes only NULL-CALL
 *      TOLERANCE (the wrapper cannot crash) -- this single-threaded host
 *      test never has two tasks genuinely racing s_sig under an
 *      unregistered lock, so it cannot and does not demonstrate that
 *      running with no synchronization at all is safe. On a real target,
 *      a no-op critical section still lets every task touching the
 *      affected shared state run fully concurrently and UNLOCKED against
 *      it -- a genuine data-race/memory-corruption hazard, not a safe
 *      one. The actual fix for a real allocation failure is main/
 *      app_main.c's own mtek_enter_safe_failure_state (a mandatory-mutex
 *      failure now halts boot before any task that could race anything
 *      ever starts) -- not host-testable directly (no FreeRTOS on host,
 *      and this file cannot make xSemaphoreCreateMutex itself fail), so
 *      this test's only honest claim remains "does not crash when
 *      unregistered", nothing broader. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include <stdlib.h>
#include <time.h>

/* ---- Lock domains, mirroring main/app_main.c's own real wiring: every
 * dedicated mutex below is genuinely distinct from every other one, since
 * a sink emit call reachable from inside the publish guard (or the new
 * BLE lock domain) must never re-enter a mutex an outer caller already
 * holds. ---- */
static pthread_mutex_t s_router_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_router_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_router_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_router_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_router_mutex); }

static pthread_mutex_t s_pub_mutex = PTHREAD_MUTEX_INITIALIZER;
static void pub_lock_fn(void) { pthread_mutex_lock(&s_pub_mutex); }
static void pub_unlock_fn(void) { pthread_mutex_unlock(&s_pub_mutex); }

static pthread_mutex_t s_cap_mutex = PTHREAD_MUTEX_INITIALIZER;
static void cap_lock_fn(void) { pthread_mutex_lock(&s_cap_mutex); }
static void cap_unlock_fn(void) { pthread_mutex_unlock(&s_cap_mutex); }

/* P0 correction (this round, requirement 4): the new lock domain for
 * s_sig/s_gatt (mtek_ble_service_set_lock) -- previously nothing at all. */
static pthread_mutex_t s_ble_mutex = PTHREAD_MUTEX_INITIALIZER;
static void ble_lock_fn(void) { pthread_mutex_lock(&s_ble_mutex); }
static void ble_unlock_fn(void) { pthread_mutex_unlock(&s_ble_mutex); }

/* This file's own deferred-worker accounting -- NOT a production concept.
 * Every ACCEPTED_ASYNC dispatch below runs on its own detached pthread
 * (matching a real target's own async runner); with many back-to-back
 * scenarios in one process, TSan's much heavier per-thread bookkeeping
 * can otherwise leave a PRIOR scenario's own worker still winding down
 * while the NEXT scenario's mtk_fake_wifi_reset()/mtk_fake_ble_reset()
 * already clobbers the shared fake-HAL state it is still touching -- a
 * genuine cross-scenario race in this test harness, not in the
 * production code under test. Tracked with a counter/condvar so each
 * scenario can wait for full quiescence before it starts. */
static pthread_mutex_t s_worker_count_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_worker_count_cv = PTHREAD_COND_INITIALIZER;
static int s_worker_count = 0;

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    ta->fn(ta->arg);
    free(ta);
    pthread_mutex_lock(&s_worker_count_mutex);
    s_worker_count--;
    pthread_cond_broadcast(&s_worker_count_cv);
    pthread_mutex_unlock(&s_worker_count_mutex);
    return NULL;
}
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    pthread_mutex_lock(&s_worker_count_mutex);
    s_worker_count++;
    pthread_mutex_unlock(&s_worker_count_mutex);
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) {
        free(ta);
        pthread_mutex_lock(&s_worker_count_mutex);
        s_worker_count--;
        pthread_cond_broadcast(&s_worker_count_cv);
        pthread_mutex_unlock(&s_worker_count_mutex);
        return -1;
    }
    pthread_detach(t);
    return 0;
}
/* Blocks (bounded) until every deferred worker spawned so far has fully
 * returned -- called at the start of each scenario so it never races a
 * PRIOR scenario's still-finishing worker. */
static int wait_for_workers_idle(void) {
    pthread_mutex_lock(&s_worker_count_mutex);
    int idle = 1;
    for (int i = 0; i < 20000 && s_worker_count > 0; i++) {
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 500000; if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
        pthread_cond_timedwait(&s_worker_count_cv, &s_worker_count_mutex, &ts);
    }
    idle = (s_worker_count == 0);
    pthread_mutex_unlock(&s_worker_count_mutex);
    return idle;
}

/* ---- won-hook pause rendezvous: pauses whichever producer thread is
 * currently INSIDE mtk_op_begin_publish_guard (still holding the guard's
 * own lock), so a concurrent real HELLO's own attempt to bump the session
 * generation can be proven to genuinely block on the SAME lock -- exactly
 * round 5's own test_won_session_reset_race seam (mtk_op_set_won_hook),
 * reused here for every OTHER producer this round adds guarding to. ---- */
typedef struct {
    pthread_mutex_t m; pthread_cond_t cv;
    int arrived; int release;
} pause_rv_t;
static pause_rv_t s_pause;
static void pause_reset(void) {
    memset(&s_pause, 0, sizeof(s_pause));
    pthread_mutex_init(&s_pause.m, NULL);
    pthread_cond_init(&s_pause.cv, NULL);
}
static void won_hook_pause(uint32_t generation) {
    (void)generation;
    pthread_mutex_lock(&s_pause.m);
    s_pause.arrived = 1;
    pthread_cond_broadcast(&s_pause.cv);
    while (!s_pause.release) pthread_cond_wait(&s_pause.cv, &s_pause.m);
    pthread_mutex_unlock(&s_pause.m);
}
static void pause_wait_arrived(void) {
    pthread_mutex_lock(&s_pause.m);
    while (!s_pause.arrived) pthread_cond_wait(&s_pause.cv, &s_pause.m);
    pthread_mutex_unlock(&s_pause.m);
}
static void pause_release(void) {
    pthread_mutex_lock(&s_pause.m);
    s_pause.release = 1;
    pthread_cond_broadcast(&s_pause.cv);
    pthread_mutex_unlock(&s_pause.m);
}

/* Runs an arbitrary zero-arg trigger function on its own thread -- the
 * generic "deliver a frame" / "call a periodic tick" / "issue a STOP
 * request" producer action whose own internal mtk_op_begin_publish_guard
 * call this file wants to pause via the won-hook above. */
typedef struct { void (*trigger)(void); } trigger_arg_t;
static void *trigger_thread_fn(void *arg) {
    trigger_arg_t *t = (trigger_arg_t *)arg;
    t->trigger();
    return NULL;
}
static pthread_t spawn_trigger(void (*trigger)(void)) {
    static trigger_arg_t ta; /* single in-flight trigger at a time, matching this file's own fully sequential scenarios */
    ta.trigger = trigger;
    pthread_t t;
    MTK_CHECK(pthread_create(&t, NULL, trigger_thread_fn, &ta) == 0);
    return t;
}

/* ---- Real native SPI v1 cell plumbing, mirroring test_p0_concurrency_
 * closure_round5.c's own base_req_hdr_b/hello_hdr_b/hello_thread_fn
 * exactly -- only a real changed-epoch HELLO through this exact path
 * actually stamps mtk_request_ctx_t.session_generation and drives
 * mtk_core_bump_session_generation(), which is what this round's own
 * requirement ("issue a real changed-epoch HELLO concurrently") demands,
 * not a direct call to an internal helper. ---- */
static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id,
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
static mtk_spi_native_header_t hello_hdr(uint32_t peer_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_HELLO;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.boot_epoch = peer_epoch;
    return h;
}

typedef struct {
    mtk_spi_native_dispatch_ctx_t *dctx;
    uint32_t peer_epoch;
    volatile int done;
} hello_thread_arg_t;
static void *hello_thread_fn(void *arg) {
    hello_thread_arg_t *a = (hello_thread_arg_t *)arg;
    mtk_spi_native_header_t h = hello_hdr(a->peer_epoch);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(a->dctx, &h, NULL, 3, &resp_hdr, resp_payload, &resp_len);
    a->done = 1;
    return NULL;
}

/* A single dispatch context, reused (with a fresh initial HELLO) across
 * every scenario below -- each scenario is fully self-contained (its own
 * arbiter class, its own service), so nothing carries over between them
 * except the transport-level dctx plumbing itself. */
static mtk_spi_native_dispatch_ctx_t dctx;
static uint32_t s_next_peer_epoch = 0x10000000u;
static uint32_t s_next_request_id = 100;

/* Feeds a fresh initial HELLO (a new peer epoch every call -- this dctx's
 * own epoch-change detection is what actually resets/re-arms it between
 * scenarios) and returns the peer epoch just established, for that
 * scenario's own subsequent REQUEST cells to carry. */
static uint32_t fresh_session(void) {
    uint32_t peer_epoch = s_next_peer_epoch++;
    mtk_spi_native_header_t h = hello_hdr(peer_epoch);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 1, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    return peer_epoch;
}

/* Feeds one REQUEST cell for `op` with the given encoded body, under
 * `peer_epoch`'s own session -- fire-and-forget for ACCEPTED_ASYNC ops
 * (the response/IDLE distinction does not matter to this file's own
 * scenarios, matching round 5's own test_won_session_reset_race, which
 * never inspects it either); the caller separately polls arbiter/HAL
 * state or joins a trigger thread to know when real work has happened. */
static void feed_request(const mtk_opcode_entry_t *op, const void *req, uint32_t peer_epoch) {
    uint8_t buf[MTK_SPI_NATIVE_MAX_PAYLOAD]; size_t blen = 0;
    if (req) mtk_encode(op->req_desc, req, buf, sizeof(buf), &blen);
    mtk_spi_native_header_t hdr = base_req_hdr(op->service_id, op->opcode, s_next_request_id++, (uint16_t)blen, peer_epoch);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, blen ? buf : NULL, blen, &resp_hdr, resp_payload, &resp_len);
}

/* Blocks (bounded) until the arbiter reports `cls` active -- the deferred
 * ACCEPTED_ASYNC worker for whatever START request was just fed has
 * genuinely started running and acquired its own arbiter class. */
static int wait_for_arbiter_class(mtk_arbiter_class_t cls) {
    for (int i = 0; i < 20000; i++) {
        if (mtk_arbiter_active_class() == cls) return 1;
        usleep(500);
    }
    return 0;
}

/* GATT_CONNECT's own arbiter grant happens BEFORE the (fake, but still a
 * real call-through) gatt_connect() HAL call and the guarded s_gatt commit
 * that follows it -- wait_for_arbiter_class alone only proves the connect
 * attempt started, not that s_gatt.connected is actually 1 yet. Polls
 * GATT_STATUS (a MTK_LC_SYNCHRONOUS opcode, answered inline) until it
 * reports connected, so a subsequent GATT_SUBSCRIBE in the same scenario
 * never races the still-in-flight connect. */
static int wait_for_gatt_connected(uint32_t connection_token, uint32_t peer_epoch) {
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GATT_STATUS");
    mtk_gatt_status_req_t req = {0}; req.connection_token = connection_token;
    /* RC12 hardening round verification: this polls for a DEFERRED GATT_
     * CONNECT worker's own s_gatt.connected commit. Under ThreadSanitizer's
     * 10-30x instrumentation slowdown the former tight poll (20000 x 500us,
     * no yield) could STARVE that worker thread -- the main thread's own
     * back-to-back feed_cell dispatches monopolised the shared locks the
     * worker also needs, so the worker made little progress and the wait
     * intermittently gave up (~15-20% flake, SOLO, TSan-only; ASan 0/25,
     * and TSan reports zero data races here -- a scheduling-fairness flake,
     * not a correctness or race bug). The fix gives the worker a genuine
     * uncontended window every poll: yield THEN sleep a few ms with no lock
     * held, so a merely-slow worker reliably reaches its commit. The happy
     * path still returns within the first poll or two (a few ms). */
    for (int i = 0; i < 20000; i++) {
        uint8_t buf[MTK_SPI_NATIVE_MAX_PAYLOAD]; size_t blen = 0;
        mtk_encode(status_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(status_op->service_id, status_op->opcode, s_next_request_id++, (uint16_t)blen, peer_epoch);
        mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, blen, &resp_hdr, resp_payload, &resp_len);
        if (resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE && resp_hdr.status == MTK_STATUS_OK) {
            mtk_gatt_status_resp_t sr = {0};
            mtk_decode(status_op->resp_desc, &sr, resp_payload, resp_len, NULL);
            if (sr.connected) return 1;
        }
        sched_yield();
        usleep(3000); /* uncontended window for the starved worker (no lock held here) */
    }
    return 0;
}

/* Blocks (bounded) until the fake Wi-Fi HAL's own promisc_start has
 * genuinely been called. mtk_arbiter_acquire (hence wait_for_arbiter_
 * class returning true) happens BEFORE the deferred worker's own locked
 * session-struct init (s_hs/s_cap) AND before promisc_start -- under
 * TSan's own much heavier per-access instrumentation, that window is wide
 * enough for this test's own main thread to win the race and act on a
 * still-uninitialized (or, worse, a PRIOR scenario's stale) struct. Since
 * every handler that calls promisc_start does so strictly AFTER its own
 * locked struct init (same thread, real happens-before), this is a
 * reliable readiness signal for handshake/capture specifically -- not a
 * production concern, purely this test harness's own synchronization. */
static int wait_for_promisc_start(void) {
    for (int i = 0; i < 20000; i++) {
        fake_wifi_lock();
        unsigned n = g_fake_wifi.promisc_start_count;
        fake_wifi_unlock();
        if (n >= 1) return 1;
        usleep(500);
    }
    return 0;
}

/* Same readiness hazard as wait_for_promisc_start, for SIGNAL_METER_START
 * specifically: handle_signal_meter_start's own locked s_sig init runs
 * strictly before its own inline first-sample mtek_ble_signal_meter_tick()
 * call (same thread, real happens-before) -- waiting for that first
 * sample's own SIGNAL_METER_UPDATE to actually land in the queue is a
 * reliable proxy for "s_sig is now fully committed", unlike the arbiter
 * class alone (granted even earlier, before s_sig is touched at all).
 *
 * P0 correction (this round, TSan-caught test-harness bug): the ACCEPTED
 * response for SIGNAL_METER_START itself lands in this SAME queue (a
 * plain frame count, regardless of kind) strictly BEFORE the inline first
 * sample's own now_ms() read -- an earlier version of this helper waited
 * for "count >= 1" alone, which the response frame alone could already
 * satisfy, so a caller proceeding to mutate the test clock right
 * afterward could still race that in-flight read (TSan: "data race ...
 * s_mtk_test_now_ms"). Drains frames until it actually sees one of KIND
 * EVENT specifically -- a real happens-before edge (via the queue's own
 * mutex) from the tick's own push back to this observation. */
static int wait_for_queue_count_at_least(unsigned n) {
    (void)n; /* always exactly one EVENT -- every call site here waits for the one first-sample SIGNAL_METER_UPDATE */
    for (int i = 0; i < 20000; i++) {
        mtk_async_frame_t f;
        if (mtk_async_queue_pop(&dctx.event_queue, &f)) {
            if (f.kind == MTK_ASYNC_FRAME_EVENT) return 1;
            continue; /* a RESPONSE frame (or, in principle, a STREAM) ahead of it -- keep draining */
        }
        usleep(500);
    }
    return 0;
}

/* GATT_SUBSCRIBE refuses (PROTOCOL_ERROR) any handle not covered by a real
 * discovered service range from an actual GATT_DISCOVER call on this same
 * connection (gatt_find_containing_end_handle) -- never an arbitrary
 * window. g_fake_ble.gatt_services[] must already be populated by the
 * caller before this runs. */
static void discover_gatt(uint32_t connection_token, uint32_t peer_epoch) {
    const mtk_opcode_entry_t *disc_op = mtk_test_find_op("GATT_DISCOVER");
    mtk_gatt_discover_req_t req = {0}; req.connection_token = connection_token; req.max_items = 16;
    feed_request(disc_op, &req, peer_epoch);
}

static int sta_query_always_ready(void) { return 1; }

static void one_time_setup(void) {
    mtk_core_init(MTK_TEST_BOOT_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_arbiter_set_lock(router_lock, router_unlock);
    mtk_router_set_lock(router_lock, router_unlock);
    mtk_op_set_publish_lock(pub_lock_fn, pub_unlock_fn);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtek_capture_set_lock(cap_lock_fn, cap_unlock_fn);
    mtek_ble_service_set_lock(ble_lock_fn, ble_unlock_fn);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_ble_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    static const mtk_system_build_info_t info = {1,0,0,"t7",0,0,0,0,"h7","n"};
    mtek_system_service_init(&info, mtk_test_now_ms);
    mtek_wifi_service_init(mtk_test_now_ms);
    mtek_ble_service_init(mtk_test_now_ms);
    mtek_capture_service_init(mtk_test_now_ms);
    mtk_fake_wifi_reset();
    mtk_fake_ble_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_ble_set_hal(&g_fake_ble_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_ble_service_register();
    mtek_capture_service_register();
    mtek_system_set_sta_query(sta_query_always_ready);

    mtek_spi_native_dispatch_init(&dctx, MTK_TEST_BOOT_EPOCH);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);
    mtk_router_set_async_runner(pthread_runner);
}

/* ==== Shared Pattern-A driver: pause `trigger` inside the publish guard,
 * prove a concurrent real changed-epoch HELLO genuinely blocks (session
 * generation unchanged, HELLO thread not yet done), release, and prove
 * the HELLO then completes strictly AFTER the trigger's own publish. ==== */
static void run_pattern_a(void (*trigger)(void), uint32_t current_peer_epoch) {
    pause_reset();
    mtk_op_set_won_hook(won_hook_pause);
    pthread_t trig_tid = spawn_trigger(trigger);
    pause_wait_arrived();

    uint32_t generation_before = mtk_core_session_generation();
    hello_thread_arg_t hello_arg = { &dctx, s_next_peer_epoch++, 0 };
    (void)current_peer_epoch;
    pthread_t hello_tid;
    MTK_CHECK(pthread_create(&hello_tid, NULL, hello_thread_fn, &hello_arg) == 0);

    usleep(200000);
    MTK_CHECK(!hello_arg.done); /* genuinely still blocked -- cannot bump while the guard is open */
    MTK_CHECK_EQ(mtk_core_session_generation(), generation_before);

    pause_release();
    pthread_join(trig_tid, NULL); /* the trigger's own publish (emit_event/emit_stream) has now completed */

    usleep(300000);
    pthread_join(hello_tid, NULL);
    MTK_CHECK(hello_arg.done);
    MTK_CHECK(mtk_core_session_generation() != generation_before); /* the reset now genuinely completed, strictly after the publish */

    mtk_op_set_won_hook(NULL);
}

/* ==== 1. Deauth completion (natural loop-exit tail, deauth_finalize). ===
 * A count=1 BROADCAST deauth completes after exactly one send, on its own
 * deferred worker -- that worker's own guard-wrapped DEAUTH_STOPPED emit
 * is the pause point. ==== */
static uint32_t s_deauth_peer_epoch;
static void trigger_deauth(void) {
    const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");
    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    req.target_mode = 2; /* BROADCAST */
    memset(req.ap_bssid.b, 0x10, 6); /* even first byte -- mac_is_multicast checks bit0 */
    req.channel = 6; req.count = 1;
    feed_request(op, &req, s_deauth_peer_epoch);
}
static void test_deauth_completion_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();
    s_deauth_peer_epoch = fresh_session();
    run_pattern_a(trigger_deauth, s_deauth_peer_epoch);
    /* The reset's own cleanup found deauth already terminal (finalize won
     * the race before the reset could even bump the generation) -- a
     * no-op for deauth specifically, but the arbiter is confirmed free
     * and the send genuinely happened exactly once. */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    fake_wifi_lock();
    unsigned sent = g_fake_wifi.deauth_sent_count;
    fake_wifi_unlock();
    MTK_CHECK_EQ(sent, 1u);
}

/* ==== 2. Handshake progress (hs_frame_cb's own guarded HANDSHAKE_EVENT
 * emit) + completion (handshake_finish via HANDSHAKE_STOP). ============= */
static uint32_t s_hs_peer_epoch;
static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install) {
    memset(buf, 0, 200);
    static const uint8_t bssid[6] = {9,9,9,9,9,9};
    memcpy(buf + 4, bssid, 6); memcpy(buf + 10, bssid, 6); memcpy(buf + 16, bssid, 6);
    buf[0] = 0x88; buf[1] = 0x02;
    unsigned off = 24 + 2;
    static const uint8_t llc[8] = {0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E};
    memcpy(buf + off, llc, 8); off += 8;
    buf[off + 0] = 2; buf[off + 1] = 3; buf[off + 2] = 0; buf[off + 3] = 95;
    unsigned eapol = off;
    buf[eapol + 4] = 2;
    uint16_t key_info = (uint16_t)((ack << 7) | (mic << 8) | (secure << 9) | (install << 6));
    buf[eapol + 5] = (uint8_t)(key_info >> 8);
    buf[eapol + 6] = (uint8_t)(key_info & 0xFF);
    off = eapol + 4 + 1 + 2 + 96;
    return (uint16_t)off;
}
static void trigger_hs_deliver(void) {
    mtk_fake_wifi_deliver_frames(); /* runs hs_frame_cb synchronously on THIS thread */
}
static void test_handshake_progress_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();
    g_fake_wifi.defer_frames = 1;
    uint16_t len = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
    g_fake_wifi.frames[0].len = len; g_fake_wifi.frames[0].channel = 6;
    g_fake_wifi.frame_count = 1;

    s_hs_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("HANDSHAKE_START");
    mtk_handshake_start_req_t req = {0};
    memcpy(req.target_bssid.b, (uint8_t[]){9,9,9,9,9,9}, 6);
    req.channel = 6; req.deauth_count = 0;
    feed_request(start_op, &req, s_hs_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_H));
    MTK_CHECK(wait_for_promisc_start()); /* real promisc_start already ran before the pause -- never held under the guard */

    run_pattern_a(trigger_hs_deliver, s_hs_peer_epoch);

    /* Only M1 was ever observed (not the full 4-way) -- the handshake
     * operation was still genuinely RUNNING when the reset's own
     * mtek_wifi_cancel_active_for_peer_reset ran (strictly after the
     * paused M1 event's own legitimate publish completed) and correctly
     * tore it down. */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
}

/* ==== 3. MonstaShark capture: frame_cb's own guarded STREAM emit, the
 * channel-hop tick's own guarded CAPTURE_CHANNEL_EVENT emit, and
 * capture_teardown's own guarded CAPTURE_STOPPED emit (via CAPTURE_STOP,
 * a MTK_LC_SYNCHRONOUS opcode -- fed on its own thread since feed_cell
 * itself blocks for the whole synchronous handler, guard included). ===== */
static uint32_t s_cap_peer_epoch;
static void trigger_cap_deliver(void) {
    mtk_fake_wifi_deliver_frames(); /* runs frame_cb synchronously on THIS thread */
}
static void test_capture_stream_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();
    g_fake_wifi.defer_frames = 1;
    uint8_t frame[64]; memset(frame, 0, sizeof(frame));
    frame[0] = 0x08; frame[1] = 0x00; /* plain data frame -- capture has no filter set, matches unconditionally */
    memcpy(g_fake_wifi.frames[0].data, frame, sizeof(frame));
    g_fake_wifi.frames[0].len = sizeof(frame); g_fake_wifi.frames[0].channel = 1;
    g_fake_wifi.frame_count = 1;

    s_cap_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    mtk_capture_start_req_t req = {0};
    req.mode = 0 /* PUSH */; req.snap_len = 64;
    req.channel_plan.mode = 0; req.channel_plan.channel = 1;
    feed_request(start_op, &req, s_cap_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start());

    run_pattern_a(trigger_cap_deliver, s_cap_peer_epoch);

    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* torn down by the reset's own cancel path (still RUNNING) */
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
}

static uint32_t s_hop_peer_epoch;
static void trigger_hop_tick(void) {
    mtek_capture_channel_hop_tick(mtk_test_now_ms() + 50); /* dwell (10ms) already elapsed */
}
static void test_capture_channel_hop_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();
    s_hop_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    mtk_capture_start_req_t req = {0};
    req.mode = 1 /* POLL */; req.snap_len = 64;
    req.channel_plan.mode = 1 /* HOP */; req.channel_plan.hop_dwell_ms = 10;
    feed_request(start_op, &req, s_hop_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start()); /* s_cap's own locked init (hop_active/hop_current_channel/session_generation) is committed strictly before promisc_start */

    run_pattern_a(trigger_hop_tick, s_hop_peer_epoch);

    /* The real HAL channel switch itself (never held under the guard,
     * requirement 5) already happened before the pause -- the paused
     * portion was only the state-commit + CAPTURE_CHANNEL_EVENT emit. */
    fake_wifi_lock();
    unsigned switches = g_fake_wifi.set_channel_call_count;
    fake_wifi_unlock();
    MTK_CHECK_EQ(switches, 1u);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* still RUNNING (unbounded duration) -- torn down by the reset's own cancel path */
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
}

static uint32_t s_teardown_peer_epoch;
static void trigger_capture_stop(void) {
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");
    /* This capture session's own real operation_token is not tracked by
     * this file (Pattern A never needed one elsewhere) -- CAPTURE_STOP
     * looks up s_cap by whatever token GET_OPERATION_STATUS-independent
     * path applies; simplest correct token here is read directly off the
     * arbiter (the one currently MTK_ARB_M-active operation). */
    mtk_capture_stop_req_t req = {0}; req.operation_token = mtk_arbiter_active_token();
    feed_request(stop_op, &req, s_teardown_peer_epoch);
}
static void test_capture_teardown_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();
    s_teardown_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    mtk_capture_start_req_t req = {0};
    req.mode = 1; req.snap_len = 64;
    req.channel_plan.mode = 0; req.channel_plan.channel = 1;
    feed_request(start_op, &req, s_teardown_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start());

    run_pattern_a(trigger_capture_stop, s_teardown_peer_epoch);

    /* CAPTURE_STOP's own USER_REQUEST teardown already fully released/
     * restored before the reset's own (redundant, no-op) cancel path ran. */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
}

/* ==== 4. BLE signal meter tick (mtek_ble_signal_meter_tick's own guarded
 * SIGNAL_METER_UPDATE emit). ============================================ */
static uint32_t s_sig_peer_epoch;
static void trigger_sig_tick(void) {
    mtek_ble_signal_meter_tick();
}
static void test_signal_meter_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.signal_rc = 0; g_fake_ble.signal_rssi = -50; g_fake_ble.signal_is_random = 0;
    s_sig_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("SIGNAL_METER_START");
    mtk_signal_meter_start_req_t req = {0};
    memcpy(req.target.addr.b, (uint8_t[]){3,3,3,3,3,3}, 6); req.target.addr_type = 0;
    feed_request(start_op, &req, s_sig_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_SM));
    MTK_CHECK(wait_for_queue_count_at_least(1)); /* the START handler's own inline first sample -- proves s_sig is fully committed */
    mtk_async_queue_reset(&dctx.event_queue);

    s_mtk_test_now_ms += 5001; /* past the ~5s sampling interval -- the next tick genuinely samples */
    run_pattern_a(trigger_sig_tick, s_sig_peer_epoch);

    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* still RUNNING -- torn down by the reset's own cancel path */
}

/* ==== 5. GATT notify (mtek_ble_gatt_tick's own guarded GATT_VALUE_EVENT
 * emit). ================================================================= */
static uint32_t s_gatt_peer_epoch;
static void trigger_gatt_tick(void) {
    mtek_ble_gatt_tick();
}
static void test_gatt_notify_guard(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 77;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 10;
    s_gatt_peer_epoch = fresh_session();
    const mtk_opcode_entry_t *connect_op = mtk_test_find_op("GATT_CONNECT");
    mtk_gatt_connect_req_t creq = {0}; memcpy(creq.target.addr.b, (uint8_t[]){4,4,4,4,4,4}, 6);
    feed_request(connect_op, &creq, s_gatt_peer_epoch);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_GC));
    uint32_t conn_tok = mtk_arbiter_active_token();
    MTK_CHECK(wait_for_gatt_connected(conn_tok, s_gatt_peer_epoch));
    discover_gatt(conn_tok, s_gatt_peer_epoch);

    const mtk_opcode_entry_t *sub_op = mtk_test_find_op("GATT_SUBSCRIBE");
    mtk_gatt_subscribe_req_t sreq = {0}; sreq.connection_token = conn_tok; sreq.handle = 7; sreq.mode = 0;
    feed_request(sub_op, &sreq, s_gatt_peer_epoch);

    g_fake_ble.notify_pending = 1; g_fake_ble.notify_handle = 7; g_fake_ble.notify_len = 2;
    memcpy(g_fake_ble.notify_data, "hi", 2);

    run_pattern_a(trigger_gatt_tick, s_gatt_peer_epoch);

    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* still connected -- torn down (gatt_disconnect observed) by the reset's own cancel path */
    fake_ble_lock();
    unsigned disc = g_fake_ble.gatt_disconnect_call_count;
    fake_ble_unlock();
    MTK_CHECK(disc >= 1);
}

/* ==== Pattern B: a producer whose captured session_generation is already
 * stale (a real HELLO reset already ran) publishes nothing at all, and a
 * genuinely NEW session of the same kind then works normally end-to-end.
 * Demonstrated directly against signal-meter and GATT notify -- the same
 * mechanism (mtk_op_begin_publish_guard's own generation check) applies
 * identically to every other producer above; round 5/6 already proved the
 * equivalent for STA_CONNECT. ============================================ */
static void test_stale_producer_publishes_nothing(void) {
    MTK_CHECK(wait_for_workers_idle());
    /* -- Signal meter: start, let the reset invalidate it, then a stale
     * tick must not touch anything (no crash, no event, no arbiter
     * change), and a brand-new SIGNAL_METER_START immediately afterward
     * must work cleanly under the new generation. -- */
    mtk_fake_ble_reset();
    g_fake_ble.signal_rc = 0; g_fake_ble.signal_rssi = -50;
    uint32_t peer1 = fresh_session();
    const mtk_opcode_entry_t *sig_start = mtk_test_find_op("SIGNAL_METER_START");
    mtk_signal_meter_start_req_t sreq = {0};
    memcpy(sreq.target.addr.b, (uint8_t[]){5,5,5,5,5,5}, 6);
    feed_request(sig_start, &sreq, peer1);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_SM));
    MTK_CHECK(wait_for_queue_count_at_least(1)); /* s_sig fully committed -- see wait_for_queue_count_at_least's own doc comment */
    mtk_async_queue_reset(&dctx.event_queue);

    uint32_t generation_before = mtk_core_session_generation();
    uint32_t peer2 = fresh_session(); /* a real HELLO with a NEW peer epoch -- genuinely bumps the generation and cancels the still-active SIGNAL_METER session */
    MTK_CHECK(mtk_core_session_generation() != generation_before);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* torn down by the reset's own cancel path */

    unsigned queued_before = mtk_async_queue_count(&dctx.event_queue);
    MTK_CHECK_EQ(queued_before, 0u); /* the reset's own queue wipe already ran -- nothing stale survives it */
    s_mtk_test_now_ms += 5001;
    mtek_ble_signal_meter_tick(); /* the now-stale tick: s_sig.session_generation still names the OLD generation */
    MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0u); /* published nothing -- the guard itself refused before any sink call */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* the stale tick did not resurrect/acquire anything */

    /* A genuinely NEW signal-meter session, under the new generation,
     * works normally -- no leftover state from the torn-down old one. */
    mtk_signal_meter_start_req_t sreq2 = {0};
    memcpy(sreq2.target.addr.b, (uint8_t[]){6,6,6,6,6,6}, 6);
    feed_request(sig_start, &sreq2, peer2);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_SM));
    MTK_CHECK(wait_for_queue_count_at_least(1));
    mtk_async_queue_reset(&dctx.event_queue);
    s_mtk_test_now_ms += 5001;
    mtek_ble_signal_meter_tick(); /* legitimate -- the new session's own generation matches current */
    MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) >= 1u); /* the new session's own SIGNAL_METER_UPDATE genuinely landed */
    mtk_async_queue_reset(&dctx.event_queue);
    const mtk_opcode_entry_t *sig_stop = mtk_test_find_op("SIGNAL_METER_STOP");
    mtk_signal_meter_stop_req_t stopreq = {0}; stopreq.operation_token = mtk_arbiter_active_token();
    feed_request(sig_stop, &stopreq, peer2);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* -- GATT: connect, subscribe, reset (stale), stale notify tick
     * publishes nothing, then a genuinely new connect+subscribe+notify
     * cycle works cleanly under the new generation, proving the old
     * connection's own s_gatt fields (vendor_handle/connection_token/
     * subs[]) never leak into the new one. -- */
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 11;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 10;
    uint32_t gpeer1 = fresh_session();
    const mtk_opcode_entry_t *gc_op = mtk_test_find_op("GATT_CONNECT");
    mtk_gatt_connect_req_t creq = {0}; memcpy(creq.target.addr.b, (uint8_t[]){7,7,7,7,7,7}, 6);
    feed_request(gc_op, &creq, gpeer1);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_GC));
    uint32_t conn_tok1 = mtk_arbiter_active_token();
    MTK_CHECK(wait_for_gatt_connected(conn_tok1, gpeer1));
    discover_gatt(conn_tok1, gpeer1);
    const mtk_opcode_entry_t *sub_op = mtk_test_find_op("GATT_SUBSCRIBE");
    mtk_gatt_subscribe_req_t sub1 = {0}; sub1.connection_token = conn_tok1; sub1.handle = 9;
    feed_request(sub_op, &sub1, gpeer1);

    uint32_t gpeer2 = fresh_session(); /* real HELLO -- tears the still-connected GATT session down */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    fake_ble_lock();
    unsigned disc_after_reset = g_fake_ble.gatt_disconnect_call_count;
    fake_ble_unlock();
    MTK_CHECK(disc_after_reset >= 1);

    mtk_async_queue_reset(&dctx.event_queue);
    g_fake_ble.notify_pending = 1; g_fake_ble.notify_handle = 9; g_fake_ble.notify_len = 2;
    memcpy(g_fake_ble.notify_data, "xx", 2);
    mtek_ble_gatt_tick(); /* stale -- s_gatt no longer names this (now-disconnected) vendor_handle at all */
    MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0u);

    g_fake_ble.gatt_vendor_handle = 22;
    mtk_gatt_connect_req_t creq2 = {0}; memcpy(creq2.target.addr.b, (uint8_t[]){8,8,8,8,8,8}, 6);
    feed_request(gc_op, &creq2, gpeer2);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_GC));
    uint32_t new_conn_tok = mtk_arbiter_active_token();
    MTK_CHECK(new_conn_tok != 0);
    MTK_CHECK(wait_for_gatt_connected(new_conn_tok, gpeer2));
    discover_gatt(new_conn_tok, gpeer2);
    mtk_gatt_subscribe_req_t sub2 = {0}; sub2.connection_token = new_conn_tok; sub2.handle = 9;
    feed_request(sub_op, &sub2, gpeer2);
    g_fake_ble.notify_pending = 1; g_fake_ble.notify_handle = 9; g_fake_ble.notify_len = 2;
    memcpy(g_fake_ble.notify_data, "hi", 2);
    mtk_async_queue_reset(&dctx.event_queue);
    mtek_ble_gatt_tick(); /* legitimate -- the NEW connection's own generation matches current */
    MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) >= 1u);
}

/* ==== C. Non-native adapters (session_generation==0) are never fenced,
 * regardless of how many real peer-session resets have happened. ======== */
static void test_generation_zero_unaffected(void) {
    fresh_session();
    fresh_session();
    fresh_session(); /* several real resets -- current generation is now well past 1 */
    MTK_CHECK(mtk_core_session_generation() > 1u);
    MTK_CHECK_EQ(mtk_op_begin_publish_guard(0), 1);
    mtk_op_end_publish_guard();
    /* Sanity: a genuinely mismatched nonzero generation is correctly
     * refused, proving this isn't merely a guard that always returns 1. */
    MTK_CHECK_EQ(mtk_op_begin_publish_guard(1), 0);
}

/* ==== D. Unregistered publish/BLE lock hooks never CRASH -- a full
 * SIGNAL_METER_START/tick/STOP cycle with both explicitly cleared. This
 * proves NULL-call tolerance only, not that running unlocked is safe on a
 * real target (single-threaded here, so nothing genuinely races s_sig) --
 * see this file's own top-of-file doc comment, item D, for the Round 8
 * correction of an earlier claim that conflated the two. */
static void test_unregistered_locks_do_not_crash(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_op_set_publish_lock(NULL, NULL);
    mtek_ble_service_set_lock(NULL, NULL);
    mtk_fake_ble_reset();
    g_fake_ble.signal_rc = 0; g_fake_ble.signal_rssi = -60;
    uint32_t peer = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("SIGNAL_METER_START");
    mtk_signal_meter_start_req_t req = {0};
    memcpy(req.target.addr.b, (uint8_t[]){12,12,12,12,12,12}, 6);
    feed_request(start_op, &req, peer);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_SM));
    MTK_CHECK(wait_for_queue_count_at_least(1));
    mtk_async_queue_reset(&dctx.event_queue);
    s_mtk_test_now_ms += 5001;
    mtek_ble_signal_meter_tick(); /* no lock registered on either domain -- must not crash, must still publish (guard(0-registered) degrades to always-pass) */
    MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) >= 1u);
    mtk_async_queue_reset(&dctx.event_queue);
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("SIGNAL_METER_STOP");
    mtk_signal_meter_stop_req_t stopreq = {0}; stopreq.operation_token = mtk_arbiter_active_token();
    feed_request(stop_op, &stopreq, peer);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* Restore the real dedicated locks for anything that runs after this. */
    mtk_op_set_publish_lock(pub_lock_fn, pub_unlock_fn);
    mtek_ble_service_set_lock(ble_lock_fn, ble_unlock_fn);
}

MTK_TEST_MAIN_BEGIN

    one_time_setup();

    test_deauth_completion_guard();
    test_handshake_progress_guard();
    test_capture_stream_guard();
    test_capture_channel_hop_guard();
    test_capture_teardown_guard();
    test_signal_meter_guard();
    test_gatt_notify_guard();
    test_stale_producer_publishes_nothing();
    test_generation_zero_unaffected();
    test_unregistered_locks_do_not_crash();

    mtk_router_set_async_runner(NULL);

MTK_TEST_MAIN_END
