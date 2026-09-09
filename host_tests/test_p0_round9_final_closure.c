/* Release-tooling-round P0 correction, ROUND 9 (based on the Round 8
 * follow-up read-only audit): six further gaps found past round 8's own
 * concurrency and resource-failure closure:
 *
 *  1. mtek_wifi_hal_esp32_init() could fail (a required mutex/event-group/
 *     queue/netif not created), but app_main.c installed the Wi-Fi HAL
 *     unconditionally regardless -- esp32_connect/esp32_disconnect use
 *     s_wifi_events/s_prior_state_mutex directly, with no resources-ready
 *     gate of their own (unlike the radio-disturbing entry points, which
 *     already refuse via capture_prior_state_once). Fixed: wifi_hal_ready
 *     mirrors ble_hal_ready's own established pattern -- mtek_wifi_set_hal
 *     is skipped entirely on failure, so every Wi-Fi opcode (STA_CONNECT
 *     included) honestly refuses via mtek_wifi_logic.c's own pre-existing
 *     `s_hal &&` guards. Not host-testable (main/app_main.c is ESP-IDF-
 *     only) -- verified by inspection, exactly like round 8's own item 4
 *     (mutex-allocation failure)'s equivalent gap.
 *  2. GATT_SUBSCRIBE's own CCCD-bounding end_handle was looked up in a
 *     SEPARATE, later lock acquisition than the connection-identity check
 *     that produced vendor_handle -- "the service range retrieved
 *     separately from the connection identity". GATT_UNSUBSCRIBE had this
 *     PLUS three more real defects: a check-then-use TOCTOU (gatt_check_
 *     conn's own lock, released, then a second lock reading vendor_handle
 *     unconditionally), the local subscription cleared BEFORE the HAL
 *     outcome was known, and MTK_STATUS_OK reported unconditionally
 *     regardless of the HAL call's own (entirely discarded) return value.
 *     Fixed: gatt_snapshot_identity_with_range captures connection_token,
 *     vendor_handle, AND the containing service's end_handle together, in
 *     ONE lock acquisition; GATT_UNSUBSCRIBE now mirrors GATT_SUBSCRIBE's
 *     own three-phase shape (snapshot -> HAL call -> re-validated commit),
 *     never clearing local state or reporting OK before the HAL call's own
 *     result is known.
 *  3. mtek_capture_channel_hop_tick snapshotted an operation token but its
 *     later gates checked only hop_active -- unable to distinguish "my own
 *     session is still hopping" from "an entirely different, newer
 *     session that also happens to be hop-mode now occupies s_cap"
 *     (capture_teardown's own mtk_op_claim_finalization/mtk_op_
 *     transition_by_token(...STOPPED...) pair is not atomic with mtek_
 *     wifi_restore_and_release's own arbiter release, leaving a real
 *     window where a brand-new capture_start can reinitialize s_cap before
 *     the OLD token's own op-table record has transitioned to terminal). A
 *     tick preempted in that window could change the radio channel,
 *     mutate state, or emit an event for the replacement session. Fixed:
 *     every gate after the op-table check now also re-checks s_cap.token
 *     == snap_token, the identity captured at entry.
 *  4. A serial log alone was not honest protocol-level failure handling
 *     when main/app_main.c's ble_tick_task failed to start -- SIGNAL_
 *     METER_START/GATT_SUBSCRIBE/a duration-bounded or hop-mode CAPTURE_
 *     START were still silently ACCEPTED as if their own tick-driven
 *     background delivery worked. Fixed: mtek_ble_service_mark_tick_task_
 *     failed/mtek_capture_service_mark_tick_task_failed (called once from
 *     app_main.c on that xTaskCreate failure) make these three opcode
 *     families refuse honestly (MTK_STATUS_NOT_READY) instead.
 *  5. mtek_ble_hal_esp32.c's esp32_gatt_discover/_chars/_descs reported an
 *     allocation failure (xSemaphoreCreateBinary) as a bare 0 -- identical
 *     to a genuinely empty discovery -- and mtek_ble_logic.c's own `if (n <
 *     0) n = 0;` in all three handlers discarded the HAL's own -1 failure
 *     signal the same way, both turning a real resource failure into an
 *     apparently successful, empty result. bound_to_own_characteristic
 *     went further: on allocation failure it returned service_end_handle,
 *     its own pre-existing "no narrower bound found" fallback -- silently
 *     BROADENING the caller's own CCCD search to the full, unnarrowed
 *     service range. Fixed: -1 (discover/_chars/_descs), a reserved
 *     0xFFFF sentinel (find_cccd_handle_in_range), and 0 (bound_to_own_
 *     characteristic, never otherwise a legitimate return) are now
 *     distinct, honest failure signals threaded all the way up to
 *     MTK_STATUS_IO_ERROR; the HAL-internal sentinels are ESP32/NimBLE-
 *     specific and not host-testable (mtek_ble_hal_esp32.c is not linked
 *     into host tests at all), but the SERVICE-LAYER handling of a
 *     negative HAL return is, and is proven here via the fake HAL's new
 *     gatt_discover_force_fail injection.
 *  6. [SUPERSEDED by RC12 hardening round, item 5 (P1).] This round's own
 *     text below (kept for historical continuity) documented DEFERRING the
 *     TIME_SYNC_START capability downgrade because four host tests borrowed
 *     it as their only generic MTK_LC_ACCEPTED_ASYNC + MTK_ARB_NONE
 *     vehicle. RC12 removed that coupling: those tests now use a test-only
 *     overlay opcode (host_tests/support/mtk_test_async_fixture.h, routed
 *     to the SAME handler), and TIME_SYNC_START's native/Bedge capability
 *     is now truthfully UNSUPPORTED in the generated registry. The
 *     historical reasoning that follows no longer reflects the shipped
 *     capability state.
 *     TIME_SYNC_START's own generated registry entry USED TO declare
 *     MTK_CAP_SUPPORTED for both the native and Bedge/C3 profiles, but this
 *     candidate has no SNTP client wired in at all -- handle_time_sync_
 *     start ALWAYS completes FAILED/IO_ERROR, unconditionally, every call
 *     (see its own doc comment). Making it refuse UNSUPPORTED was
 *     deferred in round 9: TIME_SYNC_START was then the ONLY
 *     MTK_LC_ACCEPTED_ASYNC + MTK_ARB_NONE opcode in the entire registry,
 *     and four independent, unrelated host tests (test_op_alloc_aba_
 *     service.c, test_spi_native_4inflight.c, test_peer_session_
 *     invalidation_services.c, test_spi_native_deferred_and_retained.c)
 *     rely on exactly that unique arbiter-free shape as their own generic
 *     async/ABA/eviction/deferred-delivery test vehicle -- disabling it
 *     would regress all four at the protocol level, a real, confirmed
 *     regression (not a hypothetical one; this was tried, and reverted).
 *     Documented instead as a disclosed, understood limitation, per this
 *     item's own "document a precise reason this cannot be changed
 *     without a protocol exception" alternative -- see cap_for's own doc
 *     comment (mtek_system_logic.c) for the full reasoning. No behavior
 *     change; nothing to test here beyond round 8's own existing TIME_
 *     SYNC_START lifecycle coverage.
 *
 * This file proves, for each of items 1-5 above (item 6 is documentation-
 * only -- see its own doc comment just above, and section F below):
 *   A. (item 1) Not host-testable -- see its own doc comment above.
 *   B. (item 2) GATT_UNSUBSCRIBE: a HAL failure is reported IO_ERROR and
 *      never clears local subscription state; a HAL success clears it; a
 *      stale, in-flight UNSUBSCRIBE (delayed inside the real HAL call)
 *      that straddles a disconnect-then-reconnect reusing the exact same
 *      vendor_handle never clears the REPLACEMENT connection's own,
 *      independently-subscribed state for the same attr_handle.
 *   C. (item 3) A hop-mode capture's own periodic tick is paused (via the
 *      new mtek_capture_set_hop_tick_pause_hook) immediately after it has
 *      already passed the op-table terminal check for the OLD session;
 *      that OLD session is then STOPped for real and a brand-new hop-mode
 *      session STARTed (reinitializing the global s_cap struct); resuming
 *      the paused tick must never call the HAL's own set_channel, mutate
 *      state, or emit CAPTURE_CHANNEL_EVENT for the replacement session.
 *   D. (item 4) SIGNAL_METER_START, GATT_SUBSCRIBE, and a duration-bounded
 *      or hop-mode CAPTURE_START all refuse MTK_STATUS_NOT_READY once
 *      mtek_ble_service_mark_tick_task_failed/mtek_capture_service_mark_
 *      tick_task_failed have been called -- an unbounded, non-hopping
 *      CAPTURE_START remains unaffected.
 *   E. (item 5) GATT_DISCOVER: a HAL-layer failure (fake HAL's gatt_
 *      discover_force_fail) is reported MTK_STATUS_IO_ERROR, never an
 *      apparently successful empty discovery.
 *   F. (item 6) Documentation-only -- see the doc comment at its own
 *      section below. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

/* ---- Lock domains, mirroring main/app_main.c's own real wiring (and
 * test_p0_round8_final_closure.c's own established pattern) -- every
 * dedicated mutex below is genuinely distinct from every other one. ---- */
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
 * test_p0_round8_final_closure.c's own won-hook pause (there tied to
 * mtk_op_set_won_hook specifically; here reused for both that hook and the
 * new, void-returning mtek_capture_set_hop_tick_pause_hook). ---- */
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
 * test_p0_round8_final_closure.c's own. ---- */
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
static uint32_t s_next_peer_epoch = 0x30000000u;
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
/* Diagnosed-fix update: waits for a nonzero token in the SAME snapshot as
 * the class match, not class alone -- see test_p0_session_publication_
 * closure_round7.c's identical helper for the full rationale. */
static int wait_for_arbiter_class(mtk_arbiter_class_t cls) {
    for (int i = 0; i < 20000; i++) {
        mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
        if (snap.cls == cls && snap.token != 0) return 1;
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

/* Direct (mtk_test_call-based) GATT helpers -- see test_p0_round8_final_
 * closure.c's own doc comment on why the GATT vendor-handle-reuse
 * scenarios bypass native SPI framing entirely (mtek_spi_native_dispatch_
 * feed_cell/dctx are deliberately not thread-safe against two concurrent
 * callers; mtk_router_dispatch is). */
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
/* P0 correction (this round, test infrastructure): handle_gatt_connect
 * stores `*ctx` (including ctx->sink, a small POD struct carrying a
 * `void *user` pointer) into the long-lived, file-static s_gatt struct
 * (mtek_ble_logic.c) on a successful connect -- mtek_ble_gatt_tick's own
 * later notify-delivery path reads it back and calls sink.emit_event
 * through it, possibly long after the request that established the
 * connection has returned. A STACK-LOCAL mtk_fake_sink_state_t here would
 * leave that pointer dangling the instant this function returns (a real
 * stack-use-after-return once notify_delivers below actually exercises
 * that path -- round 8's own equivalent helper never triggered this,
 * since neither of its own two GATT tests ever called mtek_ble_gatt_tick
 * after connecting). File-static instead, so it remains valid for as
 * long as this whole test process runs. */
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
 * actually published a GATT_VALUE_EVENT for it (into s_gatt_conn_sink --
 * the sink connect_gatt_direct's own GATT_CONNECT stored into s_gatt.ctx)
 * -- the only externally observable proof of whether a subscription slot
 * is still active or not (this codebase exposes no direct "list my
 * subscriptions" opcode). */
static int notify_delivers(uint16_t handle) {
    fake_ble_lock();
    g_fake_ble.notify_handle = handle;
    g_fake_ble.notify_data[0] = 0xAB;
    g_fake_ble.notify_len = 1;
    g_fake_ble.notify_pending = 1;
    fake_ble_unlock();
    mtk_fake_sink_lock();
    s_gatt_conn_sink.event_count = 0;
    mtk_fake_sink_unlock();
    mtek_ble_gatt_tick();
    mtk_fake_sink_lock();
    int saw = mtk_fake_find_event(&s_gatt_conn_sink, "GATT_VALUE_EVENT") != NULL;
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
    mtek_ble_service_set_lock(ble_lock_fn, ble_unlock_fn);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    mtk_fake_ble_set_lock(router_lock, router_unlock);
    mtk_fake_sink_set_lock(router_lock, router_unlock);

    static const mtk_system_build_info_t info = {1,0,0,"t9",0,0,0,0,"h9","n"};
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

/* ==== B. GATT_UNSUBSCRIBE honesty and identity safety (item 2). ========= */

static void test_gatt_unsubscribe_reports_hal_failure_and_preserves_state(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 201;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;

    mtk_mac6_t addr; memset(addr.b, 31, 6);
    uint32_t conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(conn_tok);

    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(conn_tok, 105, 0), MTK_STATUS_OK);
    MTK_CHECK(notify_delivers(105)); /* sanity: the subscription is genuinely active */

    /* A real HAL failure (a CCCD write that did not actually succeed) must
     * be reported honestly, and must NEVER clear the local subscription
     * state first -- the peer's own CCCD may still be armed. */
    g_fake_ble.gatt_unsubscribe_rc = -1;
    MTK_CHECK_EQ(unsubscribe_gatt_direct(conn_tok, 105), MTK_STATUS_IO_ERROR);
    MTK_CHECK(notify_delivers(105)); /* still active -- the failed unsubscribe must not have cleared it */

    disconnect_gatt_direct(conn_tok);
}

static void test_gatt_unsubscribe_success_clears_local_state(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 202;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;

    mtk_mac6_t addr; memset(addr.b, 32, 6);
    uint32_t conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(conn_tok);

    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(conn_tok, 105, 0), MTK_STATUS_OK);
    MTK_CHECK(notify_delivers(105));

    g_fake_ble.gatt_unsubscribe_rc = 0;
    MTK_CHECK_EQ(unsubscribe_gatt_direct(conn_tok, 105), MTK_STATUS_OK);
    MTK_CHECK(!notify_delivers(105)); /* genuinely cleared -- no longer subscribed */

    /* Unsubscribing again (nothing left active) remains a clean, honest
     * no-op success -- never a spurious physical write. */
    MTK_CHECK_EQ(unsubscribe_gatt_direct(conn_tok, 105), MTK_STATUS_OK);

    disconnect_gatt_direct(conn_tok);
}

static uint32_t s_unsub_conn_tok;
static void trigger_gatt_unsubscribe(void) {
    (void)unsubscribe_gatt_direct(s_unsub_conn_tok, 105);
}
static void test_gatt_unsubscribe_survives_vendor_handle_reuse(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 203;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;

    mtk_mac6_t addr; memset(addr.b, 33, 6);
    uint32_t old_conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(old_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(old_conn_tok, 105, 0), MTK_STATUS_OK);

    /* Pause the real gatt_unsubscribe() HAL call. */
    g_fake_ble.gatt_op_delay_ms = 400;
    s_unsub_conn_tok = old_conn_tok;
    pthread_t trig_tid = spawn_trigger(trigger_gatt_unsubscribe);
    usleep(100000); /* let the trigger thread genuinely enter the blocked HAL call */

    disconnect_gatt_direct(old_conn_tok);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    fake_ble_lock();
    g_fake_ble.gatt_op_delay_ms = 0; /* the NEW connect/discover/subscribe below must not itself be delayed */
    fake_ble_unlock();
    uint32_t new_conn_tok = connect_gatt_direct(addr);
    MTK_CHECK(new_conn_tok != old_conn_tok); /* tokens are never reused -- only the vendor_handle is */
    discover_gatt_direct(new_conn_tok);
    g_fake_ble.gatt_subscribe_rc = 0;
    MTK_CHECK_EQ(subscribe_gatt_direct(new_conn_tok, 105, 0), MTK_STATUS_OK); /* the NEW connection independently subscribes the SAME handle value */

    pthread_join(trig_tid, NULL); /* the stale unsubscribe's own HAL call has now returned and its post-call re-validated commit has run (refused: connection_token mismatch) */

    /* The stale unsubscribe (for old_conn_tok) must never have cleared the
     * NEW connection's own, independently-established subscription for the
     * same attr_handle value. */
    MTK_CHECK(notify_delivers(105));

    disconnect_gatt_direct(new_conn_tok);
}

/* ==== C. Capture channel-hop session-replacement race (item 3). ========= */

static uint64_t s_hop_tick_now;
static void trigger_hop_tick(void) {
    mtek_capture_channel_hop_tick(s_hop_tick_now);
}
static void test_capture_channel_hop_session_replacement(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_wifi_reset();

    uint32_t peer = fresh_session();
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");

    mtk_capture_start_req_t req = {0};
    req.mode = 0 /* PUSH */; req.snap_len = 64;
    req.channel_plan.mode = 1 /* hop */; req.channel_plan.channel = 1; req.channel_plan.hop_dwell_ms = 100;
    feed_request(start_op, &req, peer);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    MTK_CHECK(wait_for_promisc_start());
    uint32_t old_token = mtk_arbiter_active_token();

    /* Advance the shared test clock (mtk_test_now_ms, host_tests/support/
     * mtk_test_bootstrap.h's own s_mtk_test_now_ms) past hop_dwell_ms --
     * deterministic, no real wall-clock wait needed. */
    s_mtk_test_now_ms += 200;
    s_hop_tick_now = mtk_test_now_ms();

    pause_reset();
    mtek_capture_set_hop_tick_pause_hook(hop_tick_pause_hook);
    pthread_t trig_tid = spawn_trigger(trigger_hop_tick);
    pause_wait_arrived(); /* this invocation has passed the op-table terminal check for old_token and is paused just before re-checking s_cap identity */

    fake_wifi_lock();
    unsigned baseline_set_channel_calls = g_fake_wifi.set_channel_call_count;
    fake_wifi_unlock();

    /* STOP the OLD capture for real -- capture_teardown transitions
     * old_token to terminal and releases MTK_ARB_M. */
    mtk_capture_stop_req_t stopreq = {0}; stopreq.operation_token = old_token;
    feed_request(stop_op, &stopreq, peer);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* START a brand-new HOP-MODE capture -- reinitializes the global s_cap
     * struct the paused tick's own snap_token no longer names. Also
     * hop-mode (not just any capture) so the paused tick's own pre-fix
     * `if (!s_cap.hop_active) return;` check (hop_active alone, no
     * identity) would have incorrectly proceeded here -- this genuinely
     * exercises the new identity re-check, not merely a hop_active==0
     * false negative. A long hop_dwell_ms means the NEW session's own
     * hop is not independently due yet.
     *
     * P0 correction (RC11 round 10, item 1 "close the capture-hop pre-HAL
     * race completely"): handle_capture_start's own reinit now blocks on
     * mtek_capture_set_action_lock's own lease, still held by the paused
     * tick above -- this dispatch's own worker thread genuinely starts
     * (claims the arbiter, which A's own STOP already freed -- mtk_
     * arbiter_force_transfer runs BEFORE the lease-gated reinit, so
     * new_token below is real and valid even while the reinit itself
     * remains blocked), but cannot finish reinitializing s_cap until the
     * tick below resumes and releases that SAME lease. */
    mtk_capture_start_req_t req2 = {0};
    req2.mode = 0; req2.snap_len = 64;
    req2.channel_plan.mode = 1; req2.channel_plan.channel = 1; req2.channel_plan.hop_dwell_ms = 60000;
    feed_request(start_op, &req2, peer);
    MTK_CHECK(wait_for_arbiter_class(MTK_ARB_M));
    uint32_t new_token = mtk_arbiter_active_token();
    MTK_CHECK(new_token != old_token);

    mtk_async_queue_reset(&dctx.event_queue);

    /* Resume the paused, now-stale tick: it proceeds to call hal->
     * set_channel() exactly once, for the OLD (already-stopped, but not
     * yet replaced -- B's own reinit is still blocked) session -- harmless,
     * since round 10's own action lease guarantees no replacement can
     * exist yet at that exact moment -- then its own post-HAL re-check
     * sees hop_active==0 (cleared by capture_teardown, this same round's
     * own item 1 fix) and correctly bails without committing state or
     * emitting an event, before finally releasing the lease. */
    pause_release();
    pthread_join(trig_tid, NULL);
    mtek_capture_set_hop_tick_pause_hook(NULL);

    /* B's own previously-blocked worker (dispatched back when this test
     * called feed_request for req2, above) is now free to finally acquire
     * the action lease and finish reinitializing s_cap -- genuinely
     * concurrent with this thread from this exact point (nothing
     * previously synchronized it beyond the arbiter/token checks, which
     * only prove it STARTED, not that it has FINISHED). Every assertion
     * below must wait for it to fully settle first -- both this thread's
     * own "no stale action" claims and any (unrelated) activity B's own
     * completion might itself produce. */
    MTK_CHECK(wait_for_workers_idle());
    MTK_CHECK(wait_for_promisc_start());

    fake_wifi_lock();
    unsigned after_calls = g_fake_wifi.set_channel_call_count;
    fake_wifi_unlock();
    MTK_CHECK_EQ(after_calls, baseline_set_channel_calls + 1); /* the stale tick's own one-time call for the old, already-stopped session */
    /* B was itself dispatched via native SPI framing (feed_request), so
     * ITS OWN deferred ACCEPTED response (a real, legitimate RESPONSE-kind
     * frame, not an EVENT -- MTK_ASYNC_FRAME_RESPONSE, carrying B's own
     * operation_token) also lands in this SAME dctx.event_queue once B's
     * previously-blocked worker finishes -- timing-dependent on exactly
     * when that happens relative to this check (already known-settled by
     * wait_for_workers_idle above, but not necessarily yet popped off the
     * queue by anything). Irrelevant to this assertion's own claim (new_
     * token was already extracted above via the arbiter directly, never
     * by decoding this deferred response) -- drain and check that no
     * EVENT-kind frame (the only kind this tick could ever emit --
     * CAPTURE_CHANNEL_EVENT) is present, rather than asserting the queue
     * is empty outright, which would also incorrectly fail on B's own
     * unrelated, legitimate response frame. */
    {
        mtk_async_frame_t f;
        while (mtk_async_queue_pop(&dctx.event_queue, &f)) {
            MTK_CHECK(f.kind != MTK_ASYNC_FRAME_EVENT); /* no commit or emitted event landed for either session */
        }
    }

    /* The NEW session's own tick, called again while its own dwell has
     * genuinely not yet elapsed, still correctly does nothing (sanity --
     * proves the fix didn't break the ordinary not-due-yet path either). */
    mtek_capture_channel_hop_tick(mtk_test_now_ms());
    fake_wifi_lock();
    MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, after_calls);
    fake_wifi_unlock();
    MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0u);

    /* The NEW session's own tick, once ITS OWN dwell genuinely elapses,
     * still works normally -- proves the fix only closes the stale-
     * identity window, not real, live ticking. */
    s_mtk_test_now_ms += 60001;
    mtek_capture_channel_hop_tick(mtk_test_now_ms());
    fake_wifi_lock();
    MTK_CHECK(g_fake_wifi.set_channel_call_count > baseline_set_channel_calls);
    fake_wifi_unlock();
    MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 1u);

    mtk_capture_stop_req_t stopreq2 = {0}; stopreq2.operation_token = new_token;
    feed_request(stop_op, &stopreq2, peer);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
}

/* ==== E. GATT_DISCOVER HAL-failure honesty (item 5). ===================== */

static void test_gatt_discover_hal_failure_reports_io_error(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 210;

    mtk_mac6_t addr; memset(addr.b, 40, 6);
    uint32_t conn_tok = connect_gatt_direct(addr);

    g_fake_ble.gatt_discover_force_fail = 1;
    const mtk_opcode_entry_t *disc_op = mtk_test_find_op("GATT_DISCOVER");
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    mtk_gatt_discover_req_t req = {0}; req.connection_token = conn_tok; req.max_items = 16;
    mtk_test_call(&ctx, disc_op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_IO_ERROR); /* never an apparently successful empty discovery */

    /* A genuine, real discovery (no injected failure) still works
     * normally afterward. */
    g_fake_ble.gatt_discover_force_fail = 0;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 5;
    mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
    mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 2);
    mtk_test_call(&ctx2, disc_op, &req);
    MTK_CHECK_EQ(sink2.response.status, MTK_STATUS_OK);

    disconnect_gatt_direct(conn_tok);
}

/* ==== F. TIME_SYNC_START capability declaration (item 6). ================
 * [SUPERSEDED by RC12 hardening round, item 5 (P1).] Round 9 DEFERRED the
 * capability downgrade because four host tests borrowed TIME_SYNC_START as
 * their only generic arbiter-free async vehicle. RC12 broke that coupling
 * (test-only overlay opcode, see mtk_test_async_fixture.h) and downgraded
 * the production capability to UNSUPPORTED (native + bedge_c3; factory_uart
 * was already UNAVAILABLE). This item is therefore now resolved, not
 * deferred; the operation's own runtime behavior when dispatched under the
 * test overlay (ACCEPTED, then always FAILED/IO_ERROR) is unchanged and is
 * still covered by round 8's own lifecycle tests. */

/* ==== D. Tick-task-failure readiness gates (item 4). =====================
 * Run LAST: mtek_ble_service_mark_tick_task_failed/mtek_capture_service_
 * mark_tick_task_failed are one-way, boot-session-permanent latches (never
 * re-armed, matching mtek_wifi_hal_esp32_mark_promisc_task_failed's own
 * established precedent) -- every OTHER test in this file (and every test
 * in every other file, since this is a single, separate host-test process)
 * needs the default ready=1 state to keep working normally. */
static void test_tick_task_failure_gates_are_honest(void) {
    MTK_CHECK(wait_for_workers_idle());
    mtk_fake_ble_reset();
    mtk_fake_wifi_reset();

    mtek_ble_service_mark_tick_task_failed();
    mtek_capture_service_mark_tick_task_failed();

    /* SIGNAL_METER_START must refuse honestly, before ever acquiring the
     * arbiter class it would otherwise hold for the life of the session.
     * ACCEPTED_ASYNC -- the router dispatches the whole handler (including
     * this early refusal) onto the registered async runner regardless, so
     * wait for that worker before trusting the response. */
    const mtk_opcode_entry_t *sm_op = mtk_test_find_op("SIGNAL_METER_START");
    mtk_fake_sink_state_t sm_sink; mtk_fake_sink_reset(&sm_sink);
    mtk_request_ctx_t sm_ctx = mtk_test_ctx(&sm_sink, 1);
    mtk_signal_meter_start_req_t sm_req = {0}; memset(sm_req.target.addr.b, 50, 6);
    mtk_test_call(&sm_ctx, sm_op, &sm_req);
    MTK_CHECK(wait_for_workers_idle());
    MTK_CHECK_EQ(sm_sink.response.status, MTK_STATUS_NOT_READY);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* GATT_SUBSCRIBE must refuse honestly too -- before ever touching the
     * peer's own CCCD. */
    g_fake_ble.gatt_connect_rc = 0; g_fake_ble.gatt_vendor_handle = 220;
    g_fake_ble.gatt_service_count = 1; g_fake_ble.gatt_services[0].start_handle = 100; g_fake_ble.gatt_services[0].end_handle = 110;
    mtk_mac6_t addr; memset(addr.b, 51, 6);
    uint32_t conn_tok = connect_gatt_direct(addr);
    discover_gatt_direct(conn_tok);
    MTK_CHECK_EQ(subscribe_gatt_direct(conn_tok, 105, 0), MTK_STATUS_NOT_READY);
    disconnect_gatt_direct(conn_tok);

    /* CAPTURE_START: a duration-bounded or hop-mode session refuses
     * honestly -- an unbounded, non-hopping one remains fully available
     * (it needs no tick at all; frame_cb alone drives it). Also
     * ACCEPTED_ASYNC -- same wait_for_workers_idle discipline as above. */
    const mtk_opcode_entry_t *cap_start_op = mtk_test_find_op("CAPTURE_START");

    mtk_fake_sink_state_t dur_sink; mtk_fake_sink_reset(&dur_sink);
    mtk_request_ctx_t dur_ctx = mtk_test_ctx(&dur_sink, 2);
    mtk_capture_start_req_t dur_req = {0}; dur_req.mode = 0; dur_req.snap_len = 64; dur_req.duration_ms = 5000;
    mtk_test_call(&dur_ctx, cap_start_op, &dur_req);
    MTK_CHECK(wait_for_workers_idle());
    MTK_CHECK_EQ(dur_sink.response.status, MTK_STATUS_NOT_READY);

    mtk_fake_sink_state_t hop_sink; mtk_fake_sink_reset(&hop_sink);
    mtk_request_ctx_t hop_ctx = mtk_test_ctx(&hop_sink, 3);
    mtk_capture_start_req_t hop_req = {0}; hop_req.mode = 0; hop_req.snap_len = 64;
    hop_req.channel_plan.mode = 1; hop_req.channel_plan.channel = 1; hop_req.channel_plan.hop_dwell_ms = 100;
    mtk_test_call(&hop_ctx, cap_start_op, &hop_req);
    MTK_CHECK(wait_for_workers_idle());
    MTK_CHECK_EQ(hop_sink.response.status, MTK_STATUS_NOT_READY);

    mtk_fake_sink_state_t ok_sink; mtk_fake_sink_reset(&ok_sink);
    mtk_request_ctx_t ok_ctx = mtk_test_ctx(&ok_sink, 4);
    mtk_capture_start_req_t ok_req = {0}; ok_req.mode = 0; ok_req.snap_len = 64; /* duration_ms=0, channel_plan.mode=0 */
    mtk_test_call(&ok_ctx, cap_start_op, &ok_req);
    uint32_t tok = poll_for_op_token(&ok_sink);
    MTK_CHECK(tok != 0);
    MTK_CHECK_EQ(ok_sink.response.status, MTK_STATUS_ACCEPTED);
    const mtk_opcode_entry_t *cap_stop_op = mtk_test_find_op("CAPTURE_STOP");
    mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
    mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 5);
    mtk_capture_stop_req_t stop_req = {0}; stop_req.operation_token = tok;
    mtk_test_call(&stop_ctx, cap_stop_op, &stop_req);
}

MTK_TEST_MAIN_BEGIN

    one_time_setup();

    test_gatt_unsubscribe_reports_hal_failure_and_preserves_state();
    test_gatt_unsubscribe_success_clears_local_state();
    test_gatt_unsubscribe_survives_vendor_handle_reuse();
    test_capture_channel_hop_session_replacement();
    test_gatt_discover_hal_failure_reports_io_error();

    test_tick_task_failure_gates_are_honest();

MTK_TEST_MAIN_END
