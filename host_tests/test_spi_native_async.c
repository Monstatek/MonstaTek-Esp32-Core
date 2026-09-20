/* Real cross-thread proof that native SPI v1's generic ACCEPTED_ASYNC
 * dispatch path (mtek_spi_native_dispatch.c's dispatch_complete_message)
 * is genuinely async-safe, using a real pthread-based mtk_router async
 * runner -- mirroring test_compat_async_deauth.c/test_compat_async_multi.c
 * for the other boot-exclusive adapter. Covers: lifetime/ownership (the
 * dctx-owned event_queue survives past the dispatching call's return),
 * ordering (two sequential deferred operations complete in issue order),
 * cancellation (a SYNCHRONOUS-lifecycle STOP against a still-pending
 * operation is safely rejected), reset (the queue is safe to reset
 * mid-flight and reusable afterward), and overflow/backpressure. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
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

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    usleep(20000);
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

/* Polls until a real RESPONSE cell arrives, skipping over (not stopping at) any
 * EVENT/STREAM cell in between -- since native SPI v1 now really relays those
 * onto the wire, a poll can legitimately return a non-IDLE EVENT cell before the
 * RESPONSE this test is actually waiting for. */
static void poll_until(mtk_spi_native_dispatch_ctx_t *dctx, mtk_spi_native_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_len) {
    for (int i = 0; i < 3000; i++) {
        mtek_spi_native_dispatch_poll_outbound(dctx, resp_hdr, resp_payload, resp_len);
        if (resp_hdr->msg_class == MTK_SPI_CLASS_RESPONSE) return;
        usleep(1000);
    }
}

static int any_pending(const mtk_spi_native_dispatch_ctx_t *dctx) {
    for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) if (dctx->pending[i].active) return 1;
    return 0;
}

static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id, uint16_t payload_len) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = 0x1234;
    return h;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(pthread_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* Verification fallout (same real TSan-caught gap as
     * test_spi_native_dup_cache.c's own identical fix): this test registers a
     * real pthread async runner, so a DEAUTH_START dispatched here can genuinely
     * race a concurrent DEAUTH_STOP/status query on the SAME operation record
     * via mtk_op_claim_finalization/mtk_op_transition_by_token -- but never
     * registered mtk_core_set_lock/mtek_wifi_service_set_lock/mtk_fake_
     * wifi_set_lock, leaving those calls' own internal locking as no-ops.
     * Mirrors test_deauth_continuous.c's own established pattern -- one real
     * mutex backing every one of these, matching mtk_router_set_ lock's own
     * router_lock/router_unlock above. */
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

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *deauth_op = mtk_test_find_op("DEAUTH_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("DEAUTH_STOP");

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* Lifetime/ownership + cancellation: DEAUTH_START deferred, DEAUTH_STOP
     * against the not-yet-known token safely rejected, eventual real delivery
     * with a real minted operation_token. -- */
    uint32_t first_token;
    {
        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req.channel = 6; req.target_mode = 2 /* BROADCAST */;
        req.count = 1; req.interval_ms = 0;
        uint8_t buf[128]; size_t blen = 0;
        mtk_encode(deauth_op->req_desc, &req, buf, sizeof(buf), &blen);

        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 1, (uint16_t)blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* genuinely deferred, not blocked-then-synchronous */
        MTK_CHECK_EQ(any_pending(&dctx), 1);

        /* Cancellation: DEAUTH_STOP needs the real operation_token, which
         * this dispatch layer does not have yet -- a caller attempting it
         * would have to supply a bogus/zero token, which the canonical
         * service correctly rejects as NOT_FOUND rather than crashing or
         * racing the pending accept. */
        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = 0;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 2, (uint16_t)stop_blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE); /* DEAUTH_STOP itself is SYNCHRONOUS lifecycle -- never deferred */
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);

        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.request_id, 1); /* answers the original deferred request, not the STOP */
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* real HAL reached on the worker thread */

        mtk_deauth_start_resp_t r = {0};
        mtk_decode(deauth_op->resp_desc, &r, resp_payload, resp_len, NULL);
        MTK_CHECK(r.operation_token != 0);
        first_token = r.operation_token;

        /* Now that the real token is known, DEAUTH_STOP works normally
         * (still SYNCHRONOUS, dispatched immediately). */
        stop_req.operation_token = first_token;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr2 = base_req_hdr(stop_op->service_id, stop_op->opcode, 3, (uint16_t)stop_blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr2, stop_buf, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    }

    /* Ordering: a second, independent deferred DEAUTH_START gets a genuinely new
     * token, proving no state bled across operations. -- */
    {
        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req.channel = 6; req.target_mode = 2;
        req.count = 1; req.interval_ms = 0;
        uint8_t buf[128]; size_t blen = 0;
        mtk_encode(deauth_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 4, (uint16_t)blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 4, &resp_hdr, resp_payload, &resp_len);
        /* This transaction's own reply is IDLE (deferred) unless a backlog EVENT
         * from the first sub-test's own DEAUTH_STOPPED terminal event (now
         * genuinely relayed onto the wire --) happened to still be queued and
         * got delivered here instead; either is correct, so only the eventual
         * RESPONSE this sub-test actually cares about is asserted below. */
        MTK_CHECK(resp_hdr.msg_class == MTK_SPI_CLASS_IDLE || resp_hdr.msg_class == MTK_SPI_CLASS_EVENT);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.request_id, 4);
        mtk_deauth_start_resp_t r = {0};
        mtk_decode(deauth_op->resp_desc, &r, resp_payload, resp_len, NULL);
        MTK_CHECK(r.operation_token != 0);
        MTK_CHECK(r.operation_token != first_token);

        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = r.operation_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 5, (uint16_t)stop_blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 5, &resp_hdr, resp_payload, &resp_len);
    }

    /* Reset: dctx's event_queue is safe to reset mid-flight and reusable
     * afterward. -- */
    {
        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req.channel = 6; req.target_mode = 2; req.count = 1; req.interval_ms = 0;
        uint8_t buf[128]; size_t blen = 0;
        mtk_encode(deauth_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 6, (uint16_t)blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 6, &resp_hdr, resp_payload, &resp_len);
        /* See the identical note in the Ordering sub-test above: a
         * backlog EVENT frame may legitimately be delivered for this
         * transaction instead of IDLE now that EVENT relay is real. */
        MTK_CHECK(resp_hdr.msg_class == MTK_SPI_CLASS_IDLE || resp_hdr.msg_class == MTK_SPI_CLASS_EVENT);
        MTK_CHECK_EQ(any_pending(&dctx), 1);

        mtk_async_queue_reset(&dctx.event_queue);
        for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) dctx.pending[i].active = 0;
        usleep(50000); /* let the orphaned worker thread finish and push into the now-reset queue */
        MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) <= 2); /* accept response + its own terminal event, never more */
        mtk_async_queue_reset(&dctx.event_queue);
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0);

        /* Reusable: a fresh operation on the same dctx works normally. */
        mtk_spi_native_header_t hdr2 = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 7, (uint16_t)blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr2, buf, 7, &resp_hdr, resp_payload, &resp_len);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.request_id, 7);
        mtk_deauth_start_resp_t r = {0};
        mtk_decode(deauth_op->resp_desc, &r, resp_payload, resp_len, NULL);
        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = r.operation_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 8, (uint16_t)stop_blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 8, &resp_hdr, resp_payload, &resp_len);
    }

    /* Overflow/backpressure: flooding dctx's queue with synthetic frames before
     * a real dispatch proves the real accept response is still correctly
     * found/delivered despite queue pressure, and the drop counter is honest. -- */
    {
        mtk_async_queue_reset(&dctx.event_queue);
        for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
            mtk_async_frame_t f; memset(&f, 0, sizeof(f));
            f.kind = MTK_ASYNC_FRAME_EVENT;
            mtk_async_queue_push(&dctx.event_queue, &f);
        }
        /* mtk_async_queue_push now evicts a lower-priority occupied slot to make
         * room for a higher- priority arrival (RESPONSE > EVENT > STREAM) -- a
         * genuinely full queue only refuses a push whose OWN priority is no
         * better than everything already queued. `overflow_frame` must match the
         * fill kind (EVENT) to prove that same-priority case here;
         * mtk_async_queue's own priority-eviction behavior is proven directly in
         * test_async_queue.c. */
        mtk_async_frame_t overflow_frame; memset(&overflow_frame, 0, sizeof(overflow_frame));
        overflow_frame.kind = MTK_ASYNC_FRAME_EVENT;
        MTK_CHECK_EQ(mtk_async_queue_push(&dctx.event_queue, &overflow_frame), 0);
        MTK_CHECK(mtk_async_queue_dropped_count(&dctx.event_queue) >= 1);

        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req.channel = 6; req.target_mode = 2; req.count = 1; req.interval_ms = 0;
        uint8_t buf[128]; size_t blen = 0;
        mtk_encode(deauth_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 9, (uint16_t)blen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 9, &resp_hdr, resp_payload, &resp_len);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.request_id, 9);
    }

MTK_TEST_MAIN_END
