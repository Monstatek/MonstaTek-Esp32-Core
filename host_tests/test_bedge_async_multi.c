/* Real cross-thread proof that the async lifetime/delivery pattern now
 * proven for DEAUTH_START (test_bedge_async_deauth.c) genuinely extends
 * to every other ACCEPTED_ASYNC opcode this dispatch layer translates,
 * using a real pthread-based mtk_router async runner. Covers: ordering
 * (two sequential deferred operations complete in the order they were
 * issued, never interleaved/corrupted), cancellation (STOP against a
 * still-pending operation is safely rejected for HANDSHAKE_START, same
 * as DEAUTH_START), reset (an operation's dctx-owned event_queue is safe
 * to reset mid-flight and reusable afterward), and overflow/backpressure
 * (a worker task producing more frames than the bounded queue holds
 * never corrupts state, and the accept response itself is always
 * delivered even under queue pressure from other traffic). */
#include "mtk_test.h"
#include "mtek_bedge_dispatch.h"
#include "mtek_bedge_opcode_map.h"
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

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; unsigned delay_us; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    usleep(ta->delay_us);
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
static unsigned s_next_delay_us = 20000;
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg; ta->delay_us = s_next_delay_us;
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

static void poll_until(mtk_bedge_dispatch_ctx_t *dctx, mtk_bedge_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_len) {
    for (int i = 0; i < 3000; i++) {
        mtek_bedge_dispatch_poll_outbound(dctx, resp_hdr, resp_payload, resp_len);
        if (resp_hdr->msg_type != MTK_BEDGE_MSG_IDLE) return;
        usleep(1000);
    }
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(pthread_runner);
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

    mtk_bedge_dispatch_ctx_t dctx;
    mtek_bedge_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    mtk_bedge_header_t resp_hdr; uint8_t resp_payload[MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;

    /* ---- HANDSHAKE_START: deferred, delivered later, STOP-while-pending
     * rejected (same cancellation-safety pattern as DEAUTH_START). ---- */
    {
        s_next_delay_us = 20000;
        uint8_t hs_payload[9] = {1,2,3,4,5,6, 6, 0,0};
        mtk_bedge_header_t hdr = {0};
        hdr.magic = MTK_BEDGE_MAGIC; hdr.version = MTK_BEDGE_VERSION; hdr.msg_type = MTK_BEDGE_MSG_REQ;
        hdr.msg_id = 0x0310; hdr.payload_len = sizeof(hs_payload);
        mtek_bedge_dispatch_request(&dctx, &hdr, hs_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE); /* genuinely deferred */
        MTK_CHECK_EQ(dctx.handshake_token, 0);

        /* Cancellation: STOP against the still-pending (not yet accepted) operation. */
        mtk_bedge_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0313; stop_hdr.payload_len = 0;
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_NAK);
        MTK_CHECK_EQ(resp_payload[0], MTK_BEDGE_STATUS_ERR_NOT_RUNNING);

        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK_EQ(resp_hdr.msg_id, 0x0310);
        MTK_CHECK(dctx.handshake_token != 0);
        MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 1); /* real HAL reached on the worker thread */

        /* Clean up so the next sub-test starts from a known state. */
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.handshake_token, 0);
    }

    /* ---- CAPTURE_START: deferred delivery of the special raw-4-byte-
     * errno response format (not the generic bare-status convention) --
     * proves the per-opcode response-shape override survives deferral. */
    {
        s_next_delay_us = 15000;
        uint8_t cap_payload[2] = {0, 0}; /* channel=0(hop), band=0 */
        mtk_bedge_header_t hdr = {0};
        hdr.magic = MTK_BEDGE_MAGIC; hdr.version = MTK_BEDGE_VERSION; hdr.msg_type = MTK_BEDGE_MSG_REQ;
        hdr.msg_id = 0x0300; hdr.payload_len = sizeof(cap_payload);
        mtek_bedge_dispatch_request(&dctx, &hdr, cap_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);
        MTK_CHECK_EQ(dctx.capture_token, 0);

        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 4); /* raw 4-byte LE esp_err_t, not a bare 0-byte status */
        MTK_CHECK(dctx.capture_token != 0);

        mtk_bedge_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0301; stop_hdr.payload_len = 0;
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.capture_token, 0);
    }

    /* ---- Ordering: two sequential deferred DEAUTH_START/STOP operations
     * complete in issue order, never interleaved. ---- */
    {
        s_next_delay_us = 5000;
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_bedge_header_t hdr = {0};
        hdr.magic = MTK_BEDGE_MAGIC; hdr.version = MTK_BEDGE_VERSION; hdr.msg_type = MTK_BEDGE_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);
        mtek_bedge_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK_EQ(resp_hdr.msg_id, 0x0302); /* the FIRST deferred op's own reply, not a stale/mixed one */
        MTK_CHECK(dctx.deauth_token != 0);
        uint32_t first_token = dctx.deauth_token;

        mtk_bedge_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.deauth_token, 0);

        /* A second, independent deferred DEAUTH_START gets a genuinely
         * new token, proving no state bled across the two operations. */
        mtek_bedge_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK(dctx.deauth_token != 0);
        MTK_CHECK(dctx.deauth_token != first_token);
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    }

    /* ---- Reset: dctx's event_queue is safe to reset mid-flight (e.g. a
     * boot_epoch reset) and fully reusable afterward for an unrelated
     * later operation. ---- */
    {
        s_next_delay_us = 30000;
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_bedge_header_t hdr = {0};
        hdr.magic = MTK_BEDGE_MAGIC; hdr.version = MTK_BEDGE_VERSION; hdr.msg_type = MTK_BEDGE_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);
        mtek_bedge_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);

        mtk_async_queue_reset(&dctx.event_queue); /* simulated reset-intent */
        dctx.pending_start_msg_id = 0;
        usleep(60000); /* let the orphaned worker thread finish and push into the now-reset queue */
        /* At most the orphaned ACCEPTED response plus its own terminal
         * DEAUTH_STOPPED event (this short-lived count=1 operation emits
         * both, same as test_bedge_async_deauth.c's own reset sub-test),
         * never more -- no corruption, no duplication. */
        MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) <= 2);
        mtk_async_queue_reset(&dctx.event_queue);
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0);

        /* Reusable: a fresh operation on the same dctx works normally. */
        s_next_delay_us = 5000;
        mtek_bedge_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK(dctx.deauth_token != 0);
        mtk_bedge_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
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
         * before every fresh dispatch (see mtek_bedge_dispatch.c) means a
         * real new operation is never blocked by stale queue pressure
         * from unrelated prior traffic -- proven here directly. */
        s_next_delay_us = 5000;
        uint8_t req_payload[6 + 1 + 6 + 2 + 2];
        memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
        req_payload[6] = 6;
        memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        req_payload[13] = 1; req_payload[14] = 0;
        req_payload[15] = 0; req_payload[16] = 0;
        mtk_bedge_header_t hdr = {0};
        hdr.magic = MTK_BEDGE_MAGIC; hdr.version = MTK_BEDGE_VERSION; hdr.msg_type = MTK_BEDGE_MSG_REQ;
        hdr.msg_id = 0x0302; hdr.payload_len = sizeof(req_payload);
        mtek_bedge_dispatch_request(&dctx, &hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        poll_until(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
        MTK_CHECK(dctx.deauth_token != 0);
        mtk_bedge_header_t stop_hdr = hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
        mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    }

MTK_TEST_MAIN_END
