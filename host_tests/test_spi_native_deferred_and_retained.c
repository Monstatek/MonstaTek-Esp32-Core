/* ROUND 3 (follow-up read-only audit, "genuine peer-session generation
 * ownership"): two scenarios that specifically need the real native-SPI
 * wire/dctx layer (unlike test_peer_session_invalidation_services.c's own direct
 * per-service exercise) -- both close gaps a read-only re-audit found in the
 * prior round's own peer-session invalidation:
 *
 * 1. "Deferred-before-accept requests": a request already dispatched to the
 * router's own async pool -- genuinely queued, its worker thread not yet
 * actually running -- when a peer reboot is detected must never be allowed to
 * mint a brand-new operation once its worker finally does start,
 * indistinguishable from one the NEW peer session legitimately created.
 * mtek_router.c's own async_trampoline now checks
 * mtk_request_ctx_t.session_generation (stamped by mtek_spi_native_dispatch.c at
 * the moment of admission) against mtk_core_session_generation immediately
 * before invoking the handler; a stale request is answered NOT_FOUND and the
 * handler is never called at all -- no operation minted, no arbiter class ever
 * acquired, no HAL call ever made. 2. "Invalidate every old-session operation
 * token, including terminal retained tokens" + "Preserve idempotency for
 * repeated HELLO with the same epoch": several operations that already completed
 * (naturally terminal, still retained in the table) earlier in one peer session
 * must ALL become unusable the instant that peer reboots -- proven here through
 * a REAL HELLO, not a direct call to the underlying per-service/core primitives
 * -- while a REPEATED HELLO carrying the SAME (already-adopted) epoch must leave
 * them completely undisturbed. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtk_test_async_fixture.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
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

#define ESP_EPOCH      0x55556666u
#define PEER_EPOCH_A   0x77778888u
#define PEER_EPOCH_B   0x9999AAAAu

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

/* A deliberately SLOW trampoline: the worker thread sleeps a real,
 * generous delay BEFORE it ever starts running the deferred handler --
 * simulating a request genuinely still queued (dispatched, but its own
 * worker not yet actually invoked) at the moment a peer reboot is
 * detected on the main dispatch thread. */
typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *slow_pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    usleep(300000); /* 300ms -- well past when this test's own HELLO below is sent */
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
static int slow_pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    pthread_t t;
    if (pthread_create(&t, NULL, slow_pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

static int sta_query_always_ready(void) { return 1; }

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

static void encode_deauth(const mtk_opcode_entry_t *op, uint8_t channel, uint8_t *buf, size_t *blen) {
    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req.channel = channel; req.target_mode = 2 /* BROADCAST */;
    req.count = 1; req.interval_ms = 0;
    mtk_encode(op->req_desc, &req, buf, 128, blen);
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(ESP_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_lock(router_lock, router_unlock);
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
    mtek_system_set_sta_query(sta_query_always_ready);

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, ESP_EPOCH);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *deauth_op = mtk_test_find_op("DEAUTH_START");
    const mtk_opcode_entry_t *ts_start_op = mtk_test_async_fixture_install() /* Test-only overlay async op, was
                                                                              * TIME_SYNC_START */;
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GET_OPERATION_STATUS");
    MTK_CHECK(deauth_op && ts_start_op && status_op);

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* 1. Initial HELLO from the peer (PEER_EPOCH_A). -------- */
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    }

    /* ==== 2. Deferred-before-accept fencing: dispatch DEAUTH_START with a
     * deliberately SLOW async runner (its own worker thread will not
     * actually start running for 300ms), then IMMEDIATELY simulate a peer
     * reboot (a real HELLO with a genuinely different epoch) well before
     * that delay elapses. The worker, once it finally runs, must find its
     * own request's session_generation stale and refuse to mint anything
     * at all. ============================================================ */
    {
        mtk_router_set_async_runner(slow_pthread_runner);
        uint8_t buf[128]; size_t blen = 0;
        encode_deauth(deauth_op, 6, buf, &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)blen, PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* genuinely deferred -- the slow worker has not run yet */

        /* Peer reboot, well within the worker's own 300ms delay. */
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* nothing was ever minted for the stale request */

        /* Adversarial test: "Old deferred request ID N -> peer HELLO/reset ->
         * new request with the same ID N before the old worker wakes."
         * request_id=100's own dctx->pending[] slot was just wiped by the HELLO
         * reset above (mtek_spi_native_dispatch.c's own cancel-and-reset path),
         * so it is legitimately free for a brand-new, unrelated request to reuse
         * -- while the OLD stale worker (already a real, independent background
         * pthread, unaffected by changing s_async_runner from this point on) is
         * still asleep, not yet anywhere near touching anything. Switch to
         * synchronous dispatch (safe: it only affects requests dispatched from
         * here on, never a thread already created) so this new request's own
         * response is immediate and unambiguous. */
        mtk_router_set_async_runner(NULL);
        uint32_t new_token;
        {
            uint8_t tbuf[8]; size_t tblen = 0;
            mtk_time_sync_start_req_t treq = {0};
            mtk_encode(ts_start_op->req_desc, &treq, tbuf, sizeof(tbuf), &tblen);
            mtk_spi_native_header_t thdr = base_req_hdr(ts_start_op->service_id, ts_start_op->opcode, 100, (uint16_t)tblen, PEER_EPOCH_B);
            mtek_spi_native_dispatch_feed_cell(&dctx, &thdr, tbuf, 40, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
            MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED); /* clean response -- untouched by the dead old request_id=100 */
            mtk_time_sync_start_resp_t r = {0};
            mtk_decode(ts_start_op->resp_desc, &r, resp_payload, resp_len, NULL);
            new_token = r.operation_token;
            MTK_CHECK(new_token != 0);
        }
        /* TIME_SYNC_START's own handler (mtek_system_logic.c) legitimately
         * emits a TIME_SYNC_RESULT event right after its ACCEPTED
         * response, completing synchronously in this same call -- a
         * real, expected event belonging to the NEW operation. Drain it
         * now so it is never mistaken for stale-worker interference in
         * the empty-queue check below. */
        {
            mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_EVENT); /* the new op's own real TIME_SYNC_RESULT (request_id=0 on the wire; its token travels inside the payload, RC7's own accepted encoding) */
        }
        {
            uint8_t sbuf[8]; size_t sblen = 0;
            mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = new_token;
            mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
            mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 101, (uint16_t)sblen, PEER_EPOCH_B);
            mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 41, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK); /* the new operation is genuinely present and healthy */
        }

        /* Let the slow (OLD) worker actually run now. */
        usleep(500000);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* still nothing -- the fence refused it */
        fake_wifi_lock();
        unsigned sent = g_fake_wifi.deauth_sent_count;
        fake_wifi_unlock();
        MTK_CHECK_EQ(sent, 0u); /* the HAL was never even reached */

        /* The new request's own token, minted under request_id=100 AFTER
         * the reset, must still be completely unaffected by the old
         * stale worker finally waking up under the SAME request_id
         * number -- proving the two are genuinely never confused with
         * each other, in either direction. */
        {
            uint8_t sbuf[8]; size_t sblen = 0;
            mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = new_token;
            mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
            mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 102, (uint16_t)sblen, PEER_EPOCH_B);
            mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 42, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        }

        /* async_trampoline no longer calls the original sink's emit_response at
         * all for a stale request -- it releases its router pool slot in
         * silence. Nothing is ever pushed into dctx->event_queue for request_id
         * 100, so the queue must be genuinely EMPTY here, not merely "polled and
         * found empty after discarding one unmatched frame". */
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0u);
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
        /* Already synchronous (set above, before the same-request_id-N
         * proof) for the remainder of this test. */
    }

    /* ==== 3. Retained completed tokens via a REAL peer reboot: several
     * TIME_SYNC operations complete (synchronously, immediately terminal)
     * under PEER_EPOCH_B; a genuine peer reboot (HELLO with yet another
     * new epoch) must evict ALL of them. ================================= */
    uint32_t retained_tokens[3];
    for (int i = 0; i < 3; i++) {
        uint8_t buf[8]; size_t blen = 0;
        mtk_time_sync_start_req_t req = {0};
        mtk_encode(ts_start_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(ts_start_op->service_id, ts_start_op->opcode, (uint32_t)(200 + i), (uint16_t)blen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, (uint32_t)(4 + i), &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_time_sync_start_resp_t r = {0};
        mtk_decode(ts_start_op->resp_desc, &r, resp_payload, resp_len, NULL);
        MTK_CHECK(r.operation_token != 0);
        retained_tokens[i] = r.operation_token;
    }
    /* Confirm all three are still present (retained, terminal/FAILED)
     * before the reboot. */
    for (int i = 0; i < 3; i++) {
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = retained_tokens[i];
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, (uint32_t)(300 + i), (uint16_t)sblen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, (uint32_t)(10 + i), &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    }

    /* Genuine peer reboot: a HELLO with yet another new epoch. */
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_A); /* reusing PEER_EPOCH_A's own value is fine -- what matters is that it DIFFERS from the currently-adopted PEER_EPOCH_B */
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 20, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    }

    /* All three retained tokens are now genuinely gone. */
    for (int i = 0; i < 3; i++) {
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = retained_tokens[i];
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, (uint32_t)(400 + i), (uint16_t)sblen, PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, (uint32_t)(21 + i), &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
    }

    /* ==== 4. Idempotency: mint ONE more retained token under the CURRENT
     * epoch (PEER_EPOCH_A), then send a REPEATED HELLO carrying that SAME
     * epoch -- it must NOT disturb the token at all. ===================== */
    uint32_t idempotency_token;
    {
        uint8_t buf[8]; size_t blen = 0;
        mtk_time_sync_start_req_t req = {0};
        mtk_encode(ts_start_op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(ts_start_op->service_id, ts_start_op->opcode, 500, (uint16_t)blen, PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 30, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_time_sync_start_resp_t r = {0};
        mtk_decode(ts_start_op->resp_desc, &r, resp_payload, resp_len, NULL);
        idempotency_token = r.operation_token;
        MTK_CHECK(idempotency_token != 0);
    }
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_A); /* SAME epoch as currently adopted -- must be a no-op */
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 31, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    }
    {
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = idempotency_token;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 501, (uint16_t)sblen, PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 32, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK); /* still present -- the repeated same-epoch HELLO never touched it */
    }

MTK_TEST_MAIN_END
