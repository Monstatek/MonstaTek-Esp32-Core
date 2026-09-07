/* Release-tooling-round P0 correction, ROUND 10 (narrowly bounded closure
 * pass on four items past round 9's own final concurrency/resource-
 * failure closure):
 *
 *  1. mtek_capture_channel_hop_tick re-validated identity, released
 *     cap_lock, and only THEN called hal->set_channel() -- a real window
 *     in which a concurrent STOP followed by a brand-new CAPTURE_START
 *     could reinitialize s_cap for a replacement session before the OLD
 *     tick's own (already-validated-stale) HAL call actually ran. Round
 *     9's own post-HAL re-validation caught this before committing state
 *     or emitting an event, but could not prevent the physical HAL call
 *     itself from landing against a replacement session's own radio
 *     config. Fixed: a NEW, dedicated lease (mtek_capture_set_action_lock)
 *     is now held from the tick's own final identity re-validation,
 *     across the real hal->set_channel() call, through its own post-HAL
 *     commit -- and handle_capture_start's own reinit now blocks on the
 *     SAME lease before touching s_cap at all, so a replacement session's
 *     reinit cannot happen anywhere inside that window. capture_teardown
 *     (STOP) now also explicitly clears s_cap.hop_active under its own
 *     existing field-level lock (no new lease needed there -- teardown
 *     never touches the radio channel), closing the "the tick still
 *     thinks it's hopping" gap for a plain stop with no replacement too.
 *  2. Every GATT handler that issues a blocking HAL call (discover/
 *     discover_chars/discover_descs/read/write/subscribe/unsubscribe)
 *     snapshotted identity, ran the HAL call unlocked, then re-validated
 *     -- protecting LOCAL state (svc_ranges[]/subs[] merges, response
 *     content) from a disconnect-then-reconnect that reuses the same
 *     vendor_handle, but never protecting the PHYSICAL HAL call itself
 *     from landing against a replacement connection's own live peer.
 *     Fixed: a NEW, dedicated GATT-operation lease (mtek_ble_service_set_
 *     gatt_op_lease_lock) is now acquired BEFORE each handler's own final
 *     identity snapshot and held across the HAL call and post-HAL
 *     revalidation/commit; handle_gatt_connect's own successful-connect
 *     reinit acquires the SAME lease before touching s_gatt, so a
 *     replacement connection's reinit cannot happen while a prior
 *     operation for the connection it would replace is still mid-flight.
 *     Deliberately never acquired by handle_gatt_disconnect (a local
 *     disconnect) or by mtek_ble_gatt_tick's own remote-disconnect/
 *     notify-poll paths, so neither can ever block on (or deadlock
 *     against) an in-flight operation.
 *  3. main/app_main.c logged, but did not act on, a failed factory UART
 *     task creation, and never even recorded a failed SPI runtime task
 *     creation as a factor in its own startup-readiness decision --
 *     always reaching the normal "firmware up" announcement regardless.
 *     Fixed: explicit uart_task_ok/spi_task_ok accounting; a compiled-in
 *     UART task failure now enters the same deterministic safe-failure
 *     state a mandatory-mutex allocation failure does (factory UART is
 *     the required shipped-M1 transport in this universal build); if NO
 *     compiled-in adapter's own task ever became usable, startup also
 *     fails safely. Not host-testable (main/app_main.c is ESP-IDF-only)
 *     -- verified by inspection, the same posture round 8's own item 4
 *     and round 9's own item 4 took for the identical class of gap.
 *  4. mtek_ble_hal_esp32.c's five "discover all ..." wrappers (services,
 *     characteristics, descriptors, plus the two narrower internal CCCD-
 *     bound-characteristic/CCCD-descriptor searches) discarded the NimBLE
 *     "start" call's own return code and the semaphore wait's own
 *     timeout signal, and never checked whether the completion
 *     callback's own FINAL status was the expected "done" sentinel
 *     (BLE_HS_EDONE) versus a real, callback-reported error -- silently
 *     reporting a fabricated empty (or partial) success for an immediate
 *     start failure, a bounded timeout, or a genuine mid-procedure
 *     failure. Fixed: a new, shared, portable classification function
 *     (mtek_ble_disc_failed, mtek_ble_disc_status.h) is now the single
 *     source of truth all five call sites use, distinguishing all four
 *     failure modes from a genuine (possibly empty) success.
 *
 * This file proves items 1 and 2's own two remaining P0 race windows are
 * closed -- not merely their post-HAL state-commit halves (already proven
 * in round 9's own test file), but the PRE-HAL windows themselves: a
 * concurrent replacement session/connection's own reinit is proven to
 * genuinely BLOCK (not merely be detected-after-the-fact) while an old
 * tick/operation's own action is in flight, using real fake-HAL call-count/
 * parameter assertions, not only post-call local state. Item 3 is
 * documented as not host-testable (see its own doc comment above). Item 4's
 * own portable classification logic is proven separately, in test_ble_
 * disc_status.c (its real NimBLE integration is ESP-IDF-only). */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

/* ---- Lock domains, mirroring test_p0_round9_final_closure.c's own
 * established pattern -- PLUS the two new leases this round introduces,
 * each genuinely distinct from every other lock. ---- */
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

static pthread_mutex_t s_ble_mutex = PTHREAD_MUTEX_INITIALIZER;
static void ble_lock_fn(void) { pthread_mutex_lock(&s_ble_mutex); }
static void ble_unlock_fn(void) { pthread_mutex_unlock(&s_ble_mutex); }

/* P0 correction (RC11 round 10, item 1): the capture channel-hop action
 * lease -- genuinely distinct from s_cap_mutex above. */
static pthread_mutex_t s_cap_action_mutex = PTHREAD_MUTEX_INITIALIZER;
static void cap_action_lock_fn(void) { pthread_mutex_lock(&s_cap_action_mutex); }
static void cap_action_unlock_fn(void) { pthread_mutex_unlock(&s_cap_action_mutex); }

/* P0 correction (RC11 round 10, item 2): the GATT operation lease --
 * genuinely distinct from s_ble_mutex above. */
static pthread_mutex_t s_gatt_op_lease_mutex = PTHREAD_MUTEX_INITIALIZER;
static void gatt_op_lease_lock_fn(void) { pthread_mutex_lock(&s_gatt_op_lease_mutex); }
static void gatt_op_lease_unlock_fn(void) { pthread_mutex_unlock(&s_gatt_op_lease_mutex); }

/* This file's own deferred-worker accounting -- NOT a production concept,
 * see test_p0_session_publication_closure_round7.c's own doc comment on
 * the identical mechanism for the full rationale. */
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
static int wait_for_workers_idle(void) {
    pthread_mutex_lock(&s_worker_count_mutex);
    for (int i = 0; i < 20000 && s_worker_count > 0; i++) {
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 500000; if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
        pthread_cond_timedwait(&s_worker_count_cv, &s_worker_count_mutex, &ts);
    }
    int idle = (s_worker_count == 0);
    pthread_mutex_unlock(&s_worker_count_mutex);
    return idle;
}

/* ---- generic pause rendezvous -- identical mechanism to
 * test_p0_round9_final_closure.c's own. ---- */
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
static void pause_block(void) {
    pthread_mutex_lock(&s_pause.m);
    s_pause.arrived = 1;
    pthread_cond_broadcast(&s_pause.cv);
    while (!s_pause.release) pthread_cond_wait(&s_pause.cv, &s_pause.m);
    pthread_mutex_unlock(&s_pause.m);
}
static void hop_tick_pause_hook(void) { pause_block(); }
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

typedef struct { void (*trigger)(void); } trigger_arg_t;
static void *trigger_thread_fn(void *arg) {
    trigger_arg_t *t = (trigger_arg_t *)arg;
    t->trigger();
    return NULL;
}
static pthread_t spawn_trigger(void (*trigger)(void)) {
    static trigger_arg_t ta;
    ta.trigger = trigger;
    pthread_t t;
    MTK_CHECK(pthread_create(&t, NULL, trigger_thread_fn, &ta) == 0);
    return t;
}

/* ---- Real native SPI v1 cell plumbing -- identical to
 * test_p0_round9_final_closure.c's own. ---- */
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
static mtk_spi_native_dispatch_ctx_t dctx;
static uint32_t s_next_peer_epoch = 0x40000000u;
static uint32_t s_next_request_id = 100;

static uint32_t fresh_session(void) {
    uint32_t peer_epoch = s_next_peer_epoch++;
    mtk_spi_native_header_t h = hello_hdr(peer_epoch);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 1, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    return peer_epoch;
}
static void feed_request(const mtk_opcode_entry_t *op, const void *req, uint32_t peer_epoch) {
    uint8_t buf[MTK_SPI_NATIVE_MAX_PAYLOAD]; size_t blen = 0;
    if (req) mtk_encode(op->req_desc, req, buf, sizeof(buf), &blen);
    mtk_spi_native_header_t hdr = base_req_hdr(op->service_id, op->opcode, s_next_request_id++, (uint16_t)blen, peer_epoch);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, blen ? buf : NULL, blen, &resp_hdr, resp_payload, &resp_len);
}
static int wait_for_arbiter_class(mtk_arbiter_class_t cls) {
    for (int i = 0; i < 20000; i++) {
        if (mtk_arbiter_active_class() == cls) return 1;
        usleep(500);
    }
    return 0;
}
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

/* ---- Direct (mtk_test_call-based) GATT helpers -- identical rationale to
 * test_p0_round8/9_final_closure.c's own (native SPI dispatch is not
 * thread-safe against concurrent callers; mtk_router_dispatch is). ---- */
static uint32_t poll_for_op_token(mtk_fake_sink_state_t *sink) {
    uint32_t token = 0;
    for (int i = 0; i < 20000 && token == 0; i++) {
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
static int poll_for_gatt_connected_direct(uint32_t connection_token) {
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GATT_STATUS");
    for (int i = 0; i < 20000; i++) {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9999);
        mtk_gatt_status_req_t req = {0}; req.connection_token = connection_token;
        mtk_test_call(&ctx, status_op, &req);
        if (sink.response.status == MTK_STATUS_OK) {
            mtk_gatt_status_resp_t sr = {0};
            mtk_decode(status_op->resp_desc, &sr, sink.response.body, sink.response.body_len, NULL);
            if (sr.connected) return 1;
        }
        usleep(500);
    }
    return 0;
}
static void discover_gatt_direct(uint32_t connection_token) {
    const mtk_opcode_entry_t *disc_op = mtk_test_find_op("GATT_DISCOVER");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9998);
    mtk_gatt_discover_req_t req = {0}; req.connection_token = connection_token; req.max_items = 16;
    mtk_test_call(&ctx, disc_op, &req);
}
/* See notify_delivers below for why this is file-static, not stack-local. */
static mtk_fake_sink_state_t s_gatt_conn_sink;
static uint32_t connect_gatt_direct(mtk_mac6_t addr) {
    const mtk_opcode_entry_t *connect_op = mtk_test_find_op("GATT_CONNECT");
    mtk_fake_sink_reset(&s_gatt_conn_sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&s_gatt_conn_sink, 9997);
    mtk_gatt_connect_req_t req = {0}; req.target.addr = addr;
    mtk_test_call(&ctx, connect_op, &req);
    uint32_t tok = poll_for_op_token(&s_gatt_conn_sink);
    MTK_CHECK(tok != 0);
    MTK_CHECK(poll_for_gatt_connected_direct(tok));
    return tok;
}
/* Non-blocking variant: dispatches GATT_CONNECT and returns immediately
 * without waiting for it to complete -- used to prove a reconnect attempt
 * genuinely BLOCKS (on the new gatt_op_lease) rather than completing
 * freely while a prior operation for the connection it would replace is
 * still mid-flight. */
static mtk_fake_sink_state_t s_gatt_conn_bg_sink;
static mtk_mac6_t s_gatt_conn_bg_addr;
static void trigger_connect_gatt_bg(void) {
    const mtk_opcode_entry_t *connect_op = mtk_test_find_op("GATT_CONNECT");
    mtk_fake_sink_reset(&s_gatt_conn_bg_sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&s_gatt_conn_bg_sink, 9993);
    mtk_gatt_connect_req_t req = {0}; req.target.addr = s_gatt_conn_bg_addr;
    mtk_test_call(&ctx, connect_op, &req);
}
static void disconnect_gatt_direct(uint32_t connection_token) {
    const mtk_opcode_entry_t *disconnect_op = mtk_test_find_op("GATT_DISCONNECT");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9996);
    mtk_gatt_disconnect_req_t req = {0}; req.connection_token = connection_token;
    mtk_test_call(&ctx, disconnect_op, &req);
}
static int subscribe_gatt_direct(uint32_t connection_token, uint16_t handle, uint8_t mode) {
    const mtk_opcode_entry_t *sub_op = mtk_test_find_op("GATT_SUBSCRIBE");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9995);
    mtk_gatt_subscribe_req_t req = {0}; req.connection_token = connection_token; req.handle = handle; req.mode = mode;
    mtk_test_call(&ctx, sub_op, &req);
    return sink.response.status;
}
static int unsubscribe_gatt_direct(uint32_t connection_token, uint16_t handle) {
    const mtk_opcode_entry_t *unsub_op = mtk_test_find_op("GATT_UNSUBSCRIBE");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9994);
    mtk_gatt_unsubscribe_req_t req = {0}; req.connection_token = connection_token; req.handle = handle;
    mtk_test_call(&ctx, unsub_op, &req);
    return sink.response.status;
}
/* Delivers one notification for `handle` on the connection the fake HAL
 * currently believes is active, and reports whether mtek_ble_gatt_tick
 * actually published a GATT_VALUE_EVENT for it into `sink` -- whichever
 * FILE-STATIC sink the connection currently live in s_gatt.ctx.sink was
 * actually established with (connect_gatt_direct's own s_gatt_conn_sink,
 * or trigger_connect_gatt_bg's own s_gatt_conn_bg_sink) -- never a stack-
 * local, which would dangle the instant the connecting function returned;
 * see test_p0_round9_final_closure.c's own doc comment on this exact
 * stack-use-after-return hazard. */
static int notify_delivers(mtk_fake_sink_state_t *sink, uint16_t handle) {
    fake_ble_lock();
    g_fake_ble.notify_handle = handle;
    g_fake_ble.notify_data[0] = 0xAB;
    g_fake_ble.notify_len = 1;
    g_fake_ble.notify_pending = 1;
    fake_ble_unlock();
    mtk_fake_sink_lock();
    sink->event_count = 0;
    mtk_fake_sink_unlock();
    mtek_ble_gatt_tick();
    mtk_fake_sink_lock();
    int saw = mtk_fake_find_event(sink, "GATT_VALUE_EVENT") != NULL;
    mtk_fake_sink_unlock();
    return saw;
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
    mtek_capture_set_action_lock(cap_action_lock_fn, cap_action_unlock_fn);
    mtek_ble_service_set_lock(ble_lock_fn, ble_unlock_fn);
    mtek_ble_service_set_gatt_op_lease_lock(gatt_op_lease_lock_fn, gatt_op_lease_unlock_fn);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_ble_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    static const mtk_system_build_info_t info = {1,0,0,"t10",0,0,0,0,"h10","n"};
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

/* ==== A. Capture channel-hop PRE-HAL race, fully closed (item 1). ======= */

static uint64_t s_hop_tick_now;
static void trigger_hop_tick(void) {
    mtek_capture_channel_hop_tick(s_hop_tick_now);
}

/* Non-blocking replacement-START trigger: dispatches a brand-new hop-mode
 * CAPTURE_START and returns immediately without waiting for it to
 * complete -- used to prove this attempt genuinely BLOCKS (on the new
 * mtek_capture_set_action_lock lease) rather than completing freely while
 * a prior tick's own action is still in flight for the session it would
 * replace. handle_capture_start's own reinit runs BEFORE it ever emits
 * the ACCEPTED response, so the blocked state is directly observable as
 * "no response yet" on start_b_sink. */
static mtk_fake_sink_state_t s_start_b_sink;
static mtk_capture_start_req_t s_start_b_req;
static void trigger_capture_start_b(void) {
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    mtk_fake_sink_reset(&s_start_b_sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&s_start_b_sink, 8887);
    mtk_test_call(&ctx, start_op, &s_start_b_req);
}

static void test_capture_channel_hop_pre_hal_race_closed(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();

    uint32_t peer = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");

    mtk_capture_start_req_t req = {0};
    req.mode = 0; req.snap_len = 64;
    req.channel_plan.mode = 1; req.channel_plan.channel = 1; req.channel_plan.hop_dwell_ms = 100;
    feed_request(start_op, &req, peer);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start());
    uint32_t old_token = mtk_arbiter_active_token();

    s_mtk_test_now_ms += 200; /* past hop_dwell_ms -- deterministic, no real wall-clock wait */
    s_hop_tick_now = mtk_test_now_ms();

    pause_reset();
    mtek_capture_set_hop_tick_pause_hook(hop_tick_pause_hook);
    pthread_t trig_tid = spawn_trigger(trigger_hop_tick);
    /* This invocation has passed its own final identity re-validation
     * (token/hop_active, under cap_lock) and is now paused HOLDING
     * mtek_capture_set_action_lock's own lease -- exactly the point
     * immediately before the real hal->set_channel() call. */
    pause_wait_arrived();

    fake_wifi_lock();
    unsigned baseline_set_channel_calls = g_fake_wifi.set_channel_call_count;
    fake_wifi_unlock();

    /* STOP the OLD capture -- capture_teardown needs no lease of its own
     * (it never touches the radio channel), so this completes freely even
     * while the paused tick still holds the action lease. Also clears
     * s_cap.hop_active under its own existing field-level lock. */
    mtk_capture_stop_req_t stopreq = {0}; stopreq.operation_token = old_token;
    feed_request(stop_op, &stopreq, peer);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    mtk_async_queue_reset(&dctx.event_queue); /* drain STOP's own real CAPTURE_STOPPED event -- irrelevant to this test's own claim */

    /* "Attempt/start a replacement": a brand-new hop-mode CAPTURE_START,
     * dispatched on its own background thread since it will genuinely
     * BLOCK -- handle_capture_start's own reinit acquires the SAME action
     * lease the paused tick still holds, and cannot proceed until this
     * tick resumes and releases it. This is the actual PRE-HAL closure
     * this item's own fix adds: round 9's own equivalent test could only
     * prove the tick's post-HAL commit was refused; this proves the
     * REPLACEMENT itself cannot even begin to exist yet. */
    memset(&s_start_b_req, 0, sizeof(s_start_b_req));
    s_start_b_req.mode = 0; s_start_b_req.snap_len = 64;
    s_start_b_req.channel_plan.mode = 1; s_start_b_req.channel_plan.channel = 1; s_start_b_req.channel_plan.hop_dwell_ms = 60000;
    pthread_t start_b_tid = spawn_trigger(trigger_capture_start_b);

    usleep(100000); /* let the worker genuinely reach (and block on) the action lease */
    mtk_fake_sink_lock();
    int start_b_done_early = s_start_b_sink.response.set;
    mtk_fake_sink_unlock();
    MTK_CHECK(!start_b_done_early); /* genuinely still blocked -- cannot reinit while the paused tick holds the lease */

    pause_release(); /* the old tick resumes: calls hal->set_channel() for A (harmless -- A has no replacement yet), then its own post-HAL re-check sees hop_active==0 (cleared by STOP) and bails without commit/emit */
    pthread_join(trig_tid, NULL);
    mtek_capture_set_hop_tick_pause_hook(NULL);

    /* No stale CAPTURE_CHANNEL_EVENT was ever queued -- the tick's own
     * post-HAL re-check correctly refused to commit/emit once it saw
     * hop_active cleared. */
    MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0u);

    /* Now that the tick has released the action lease, B's own blocked
     * reinit finally proceeds. */
    pthread_join(start_b_tid, NULL);
    uint32_t new_token = poll_for_op_token(&s_start_b_sink);
    MTK_CHECK(new_token != 0);
    MTK_CHECK(new_token != old_token);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start());

    /* B's own state was never touched by the stale tick: its own hop
     * fires normally once ITS OWN dwell elapses, and produces exactly one
     * new set_channel call plus one CAPTURE_CHANNEL_EVENT carrying B's OWN
     * token, never A's. B was started via a direct (mtk_test_ctx/fake-
     * sink) dispatch, not native SPI framing, so its own events land in
     * s_start_b_sink (the sink its own GATT_CONNECT... err, CAPTURE_START
     * call captured into s_cap.ctx.sink), never dctx.event_queue. */
    mtk_fake_sink_lock();
    s_start_b_sink.event_count = 0;
    mtk_fake_sink_unlock();
    s_mtk_test_now_ms += 60001;
    mtek_capture_channel_hop_tick(mtk_test_now_ms());
    fake_wifi_lock();
    unsigned after_b_hop = g_fake_wifi.set_channel_call_count;
    fake_wifi_unlock();
    MTK_CHECK(after_b_hop > baseline_set_channel_calls);
    mtk_fake_sink_lock();
    const mtk_fake_event_t *chev = mtk_fake_find_event(&s_start_b_sink, "CAPTURE_CHANNEL_EVENT");
    MTK_CHECK(chev != NULL);
    mtk_capture_channel_event_ev_t ev = {0};
    if (chev) mtk_decode(&mtk_capture_channel_event_ev_t_desc, &ev, chev->body, chev->body_len, NULL);
    mtk_fake_sink_unlock();
    MTK_CHECK_EQ(ev.operation_token, new_token); /* never old_token */

    mtk_capture_stop_req_t stopreq2 = {0}; stopreq2.operation_token = new_token;
    feed_request(stop_op, &stopreq2, peer);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
}

/* ==== B. GATT PRE-HAL race, fully closed (item 2). =======================
 * Common shape for SUBSCRIBE/UNSUBSCRIBE/DISCOVER below: connect A, pause
 * an in-flight operation INSIDE its own real (delayed) HAL call via the
 * fake BLE HAL's existing gatt_op_delay_ms knob (it already holds gatt_
 * op_lease at that point -- acquired before the operation's own final
 * identity snapshot), disconnect A (completes freely -- handle_gatt_
 * disconnect needs no lease), attempt a reconnect that deliberately
 * reuses the exact same vendor_handle (dispatched on its own background
 * thread since it will genuinely BLOCK on gatt_op_lease), confirm that
 * attempt is still blocked, let the delayed operation finish (it re-
 * validates identity, sees s_gatt already disconnected, and correctly
 * refuses to commit/report success), confirm the reconnect NOW proceeds,
 * and prove a genuinely NEW operation on the replacement connection still
 * works normally. */

static uint32_t s_race_conn_tok;

static void trigger_subscribe_race(void) {
    (void)subscribe_gatt_direct(s_race_conn_tok, 105, 0);
}
static void trigger_unsubscribe_race(void) {
    (void)unsubscribe_gatt_direct(s_race_conn_tok, 105);
}
static void trigger_discover_race(void) {
    discover_gatt_direct(s_race_conn_tok);
}

/* Shared second half of every scenario below: a reconnect attempt is
 * already in flight (blocked on gatt_op_lease) when this is called;
 * confirms it is genuinely still blocked, waits for the delayed operation
 * thread to finish, confirms the reconnect NOW proceeds, and returns its
 * own new connection_token. */
static uint32_t finish_gatt_pre_hal_race(pthread_t op_tid, uint32_t old_conn_tok) {
    usleep(100000); /* let the delayed op thread genuinely enter the blocked HAL call, and the reconnect thread genuinely reach (and block on) gatt_op_lease */

    /* GATT_CONNECT is ACCEPTED_ASYNC: handle_gatt_connect sends its own
     * ACCEPTED response (carrying the new operation_token) BEFORE its own
     * blocking gatt_connect() HAL call even runs, let alone before the
     * reinit that follows a successful one -- so response.set becoming
     * true is not itself proof of anything about the reinit's own
     * progress. The token is therefore already known (and safe to poll
     * for immediately -- the router dispatched this whole handler onto a
     * background worker, but that worker starts running right away). What
     * proves this reconnect is genuinely still blocked (on gatt_op_lease,
     * held by the still in-flight delayed operation) is that s_gatt has
     * not yet been reinitialized for it: GATT_STATUS for this already-
     * known token must not yet report connected. */
    uint32_t new_conn_tok = poll_for_op_token(&s_gatt_conn_bg_sink);
    MTK_CHECK(new_conn_tok != 0);
    MTK_CHECK(new_conn_tok != old_conn_tok); /* tokens are never reused -- only the vendor_handle is */
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GATT_STATUS");
    mtk_fake_sink_state_t chk_sink; mtk_fake_sink_reset(&chk_sink);
    mtk_request_ctx_t chk_ctx = mtk_test_ctx(&chk_sink, 8886);
    mtk_gatt_status_req_t chk_req = {0}; chk_req.connection_token = new_conn_tok;
    mtk_test_call(&chk_ctx, status_op, &chk_req);
    mtk_gatt_status_resp_t chk_resp = {0};
    if (chk_sink.response.status == MTK_STATUS_OK) {
        mtk_decode(status_op->resp_desc, &chk_resp, chk_sink.response.body, chk_sink.response.body_len, NULL);
    }
    MTK_CHECK(!chk_resp.connected); /* genuinely still blocked -- s_gatt has not yet been reinitialized for this token */

    pthread_join(op_tid, NULL); /* the delayed op's own HAL call has now returned; its post-HAL revalidation sees s_gatt already disconnected and correctly refuses to commit/report success; it then releases gatt_op_lease */

    MTK_CHECK(poll_for_gatt_connected_direct(new_conn_tok)); /* NOW proceeds -- the reinit finally runs */
    return new_conn_tok;
}

static void test_gatt_subscribe_pre_hal_race_closed(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 231;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;

    mtk_mac6_t addr; memset(addr.b, 61, 6);
    uint32_t old_conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(old_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;

    /* Delay the real gatt_subscribe() HAL call -- the trigger thread will
     * be genuinely blocked INSIDE it, still holding gatt_op_lease (which
     * handle_gatt_subscribe now acquires BEFORE its own final identity
     * snapshot, per this round's own item 2 fix). */
    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 400;
    fake_ble_unlock();
    s_race_conn_tok = old_conn_tok;
    pthread_t op_tid = spawn_trigger(trigger_subscribe_race);
    usleep(100000); /* let the trigger thread genuinely enter the blocked HAL call */

    /* DISCONNECT the OLD connection -- completes freely (no lease needed). */
    disconnect_gatt_direct(old_conn_tok);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* Attempt a reconnect that deliberately reuses the exact same
     * vendor_handle, on its own background thread since it will
     * genuinely block on gatt_op_lease. */
    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 0; /* the reconnect's own gatt_connect() call must not itself be delayed */
    fake_ble_unlock();
    s_gatt_conn_bg_addr = addr;
    pthread_t reconnect_tid = spawn_trigger(trigger_connect_gatt_bg);

    uint32_t new_conn_tok = finish_gatt_pre_hal_race(op_tid, old_conn_tok);
    pthread_join(reconnect_tid, NULL);

    /* The replacement connection's own state was never touched by the
     * stale subscribe: a genuinely NEW discover+subscribe on it, for the
     * same attr_handle value the stale attempt used, still works
     * normally. */
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;
    discover_gatt_direct(new_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(new_conn_tok, 105, 0), MTK_STATUS_OK);

    disconnect_gatt_direct(new_conn_tok);
}

static void test_gatt_unsubscribe_pre_hal_race_closed(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 232;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;

    mtk_mac6_t addr; memset(addr.b, 62, 6);
    uint32_t old_conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(old_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(old_conn_tok, 105, 0), MTK_STATUS_OK);

    /* Delay the real gatt_unsubscribe() HAL call. */
    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 400;
    fake_ble_unlock();
    s_race_conn_tok = old_conn_tok;
    pthread_t op_tid = spawn_trigger(trigger_unsubscribe_race);
    usleep(100000);

    disconnect_gatt_direct(old_conn_tok);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 0;
    fake_ble_unlock();
    s_gatt_conn_bg_addr = addr;
    pthread_t reconnect_tid = spawn_trigger(trigger_connect_gatt_bg);

    uint32_t new_conn_tok = finish_gatt_pre_hal_race(op_tid, old_conn_tok);
    pthread_join(reconnect_tid, NULL);

    /* The replacement connection independently subscribes the SAME handle
     * value the stale unsubscribe targeted, and it still works normally
     * -- proving the stale unsubscribe never touched it. */
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;
    discover_gatt_direct(new_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(new_conn_tok, 105, 0), MTK_STATUS_OK);
    MTK_CHECK(notify_delivers(&s_gatt_conn_bg_sink, 105)); /* the replacement connection was established via trigger_connect_gatt_bg -- its own live sink is s_gatt_conn_bg_sink, not s_gatt_conn_sink */

    disconnect_gatt_direct(new_conn_tok);
}

static void test_gatt_discover_pre_hal_race_closed(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 233;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 10;

    mtk_mac6_t addr; memset(addr.b, 63, 6);
    uint32_t old_conn_tok = connect_gatt_direct(addr);

    /* Delay the real gatt_discover() HAL call. */
    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 400;
    fake_ble_unlock();
    s_race_conn_tok = old_conn_tok;
    pthread_t op_tid = spawn_trigger(trigger_discover_race);
    usleep(100000);

    disconnect_gatt_direct(old_conn_tok);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 0;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110; /* deliberately DIFFERENT range than the old connection's own */
    fake_ble_unlock();
    s_gatt_conn_bg_addr = addr;
    pthread_t reconnect_tid = spawn_trigger(trigger_connect_gatt_bg);

    uint32_t new_conn_tok = finish_gatt_pre_hal_race(op_tid, old_conn_tok);
    pthread_join(reconnect_tid, NULL);

    /* The stale discover's own (1,10) result must never have merged into
     * the NEW connection's own svc_ranges[] -- proven exactly as in round
     * 9's own equivalent test: a genuinely new discover on the new
     * connection, then a subscribe bound to ITS OWN (100,110) range. */
    discover_gatt_direct(new_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    mtk_fake_sink_state_t bad_sink; mtk_fake_sink_reset(&bad_sink);
    mtk_request_ctx_t bad_ctx = mtk_test_ctx(&bad_sink, 1);
    const mtk_opcode_entry_t *sub_op = mtk_test_find_op("GATT_SUBSCRIBE");
    mtk_gatt_subscribe_req_t bad_sub = {0}; bad_sub.connection_token = new_conn_tok; bad_sub.handle = 5; /* inside the STALE old range, outside the new one */
    mtk_test_call(&bad_ctx, sub_op, &bad_sub);
    MTK_CHECK_EQ(bad_sink.response.status, MTK_STATUS_PROTOCOL_ERROR);
    MTK_CHECK_EQ(subscribe_gatt_direct(new_conn_tok, 105, 0), MTK_STATUS_OK); /* inside the NEW connection's own real range */

    disconnect_gatt_direct(new_conn_tok);
}

MTK_TEST_MAIN_BEGIN

    one_time_setup();

    test_capture_channel_hop_pre_hal_race_closed();
    test_gatt_subscribe_pre_hal_race_closed();
    test_gatt_unsubscribe_pre_hal_race_closed();
    test_gatt_discover_pre_hal_race_closed();

MTK_TEST_MAIN_END
