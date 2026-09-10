/* RC7 independent audit P0 "Native CREDIT and CANCEL are not
 * implemented": mtek_spi_native_dispatch_feed_cell used to map every
 * CREDIT/CANCEL cell straight to IDLE, so a native MonstaShark PUSH
 * session could never receive protocol credit and nothing could abort an
 * in-progress reassembly or a still-pending deferred operation from the
 * link layer. Proves real CREDIT delivery (mtek_capture_grant_credit is
 * genuinely called, with an honest applied/not-applied wire status),
 * real CANCEL of both transport-level resources it can target (an
 * in-progress inbound reassembly, and a still-pending[] deferred
 * operation), boot-epoch rejection for both classes, and that the five
 * outbound-only classes are rejected (LINK_ERROR/PROTOCOL_ERROR) rather
 * than silently accepted as IDLE when a peer sends one inbound. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtk_test_async_fixture.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_system_service.h"
#include "mtek_wifi_service.h"
#include "mtek_capture_service.h"
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
    usleep(100000); /* genuinely deferred: still pending when the test checks it */
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

static mtk_spi_native_header_t base_hdr(uint16_t service, uint16_t opcode, uint8_t msg_class, uint32_t request_id, uint32_t boot_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = msg_class;
    h.service = service; h.opcode = opcode;
    h.request_id = request_id; h.boot_epoch = boot_epoch;
    return h;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    /* No async runner registered yet -- CAPTURE_START/CANCEL(reassembly)
     * below need genuinely synchronous completion. Registered later,
     * right before the section that specifically needs real deferral. */
    mtk_router_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtek_capture_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_capture_service_register();

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* ---- CREDIT: start a real PUSH-mode capture session, grant it
     * credit via a CREDIT cell, prove it was genuinely applied by
     * successfully delivering a frame that needed it. -------------------- */
    uint32_t cap_token;
    {
        g_fake_wifi.defer_frames = 1; /* promisc_start below must not immediately replay any (still-empty) canned frame */
        const mtk_opcode_entry_t *cap_op = mtk_test_find_op("CAPTURE_START");
        mtk_capture_start_req_t creq = {0};
        creq.mode = 0 /* PUSH */; creq.snap_len = 1000;
        creq.channel_plan.mode = 0; creq.channel_plan.channel = 1;
        uint8_t cbuf[32]; size_t cblen = 0;
        mtk_encode(cap_op->req_desc, &creq, cbuf, sizeof(cbuf), &cblen);
        mtk_spi_native_header_t chdr = base_hdr(cap_op->service_id, cap_op->opcode, MTK_SPI_CLASS_REQUEST, 1, 0x1234);
        chdr.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; chdr.payload_len = (uint16_t)cblen; chdr.message_len = (uint16_t)cblen;
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, cbuf, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t started = {0};
        mtk_decode(cap_op->resp_desc, &started, resp_payload, resp_len, NULL);
        cap_token = started.operation_token;
    }

    /* CREDIT for an unknown token: applied=0 -> NOT_FOUND, no crash. */
    {
        uint8_t credit_payload[4] = {0x00, 0x10, 0x00, 0x00}; /* 4096 bytes, LE */
        mtk_spi_native_header_t chdr = base_hdr(0, 0, MTK_SPI_CLASS_CREDIT, 0xDEADBEEF, 0x1234);
        chdr.payload_len = 4;
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, credit_payload, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(resp_hdr.request_id, 0xDEADBEEF); /* CREDIT/CANCEL: request_id echoes the target token */
    }

    /* CREDIT with a stale boot_epoch: rejected LINK_ERROR/PROTOCOL_ERROR,
     * regardless of whether the token would otherwise match. */
    {
        uint8_t credit_payload[4] = {0x00, 0x10, 0x00, 0x00};
        mtk_spi_native_header_t chdr = base_hdr(0, 0, MTK_SPI_CLASS_CREDIT, cap_token, 0x9999 /* wrong epoch */);
        chdr.payload_len = 4;
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, credit_payload, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* CREDIT for the real active session's token: applied -> OK, and the
     * credit is genuinely usable (a subsequent frame that needed it is
     * actually delivered as a STREAM chunk, not dropped for lack of
     * credit). */
    {
        uint8_t credit_payload[4] = {0x00, 0x10, 0x00, 0x00}; /* 4096 bytes */
        mtk_spi_native_header_t chdr = base_hdr(0, 0, MTK_SPI_CLASS_CREDIT, cap_token, 0x1234);
        chdr.payload_len = 4;
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, credit_payload, 4, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(resp_hdr.request_id, cap_token);

        uint8_t pattern[500];
        for (int i = 0; i < 500; i++) pattern[i] = (uint8_t)i;
        memcpy(g_fake_wifi.frames[0].data, pattern, 500);
        g_fake_wifi.frames[0].len = 500; g_fake_wifi.frames[0].channel = 1;
        g_fake_wifi.frame_count = 1;
        mtk_fake_wifi_deliver_frames();

        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_STREAM); /* delivered -- the credit was real, not a no-op */
    }

    /* Stop the capture session so it doesn't interfere below. */
    {
        const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");
        mtk_capture_stop_req_t sreq = {0}; sreq.operation_token = cap_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(stop_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_hdr(stop_op->service_id, stop_op->opcode, MTK_SPI_CLASS_REQUEST, 5, 0x1234);
        shdr.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; shdr.payload_len = (uint16_t)sblen; shdr.message_len = (uint16_t)sblen;
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 5, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        /* CAPTURE_STOP's own CAPTURE_STOPPED terminal event is emitted
         * through the capture session's retained (queue-backed) sink from
         * CAPTURE_START -- drain it now so it doesn't sit at the front of
         * dctx.event_queue and get delivered as an unrelated later
         * transaction's own reply (any EVENT/STREAM frame is always
         * deliverable, see try_deliver_frame's own doc comment). */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_EVENT);
    }

    /* ---- CANCEL: abort an in-progress inbound reassembly ---------------- */
    {
        uint8_t part1[4] = {1,2,3,4};
        mtk_spi_native_header_t f1 = base_hdr(0, 1, MTK_SPI_CLASS_REQUEST, 100, 0x1234);
        f1.flags = MTK_SPI_FLAG_FIRST; f1.payload_len = 4; f1.message_len = 8; f1.fragment_offset = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f1, part1, 10, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* still accumulating */
        MTK_CHECK_EQ(dctx.inbound.active, 1);
        MTK_CHECK_EQ(dctx.inbound.request_id, 100);

        mtk_spi_native_header_t cancel_hdr = base_hdr(0, 0, MTK_SPI_CLASS_CANCEL, 100, 0x1234);
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 11, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(dctx.inbound.active, 0); /* genuinely aborted, freeing the single serialized reassembly context */

        /* A DIFFERENT message's FIRST fragment is now accepted immediately
         * (not MTK_SPI_REASM_BUSY -- the cancelled context no longer
         * blocks it). */
        uint8_t part2[4] = {5,6,7,8};
        mtk_spi_native_header_t f2 = base_hdr(0, 1, MTK_SPI_CLASS_REQUEST, 101, 0x1234);
        f2.flags = MTK_SPI_FLAG_FIRST; f2.payload_len = 4; f2.message_len = 8; f2.fragment_offset = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f2, part2, 12, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* accepted (still accumulating), not LINK_ERROR/BUSY */
        MTK_CHECK_EQ(dctx.inbound.request_id, 101);
        mtk_spi_native_reassembly_reset(&dctx.inbound); /* tidy up before the next section */
    }

    /* CANCEL of an unknown token: NOT_FOUND. */
    {
        mtk_spi_native_header_t cancel_hdr = base_hdr(0, 0, MTK_SPI_CLASS_CANCEL, 0xFEEDFACEu, 0x1234);
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 13, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
    }

    /* CANCEL with a stale boot_epoch: rejected regardless of any match. */
    {
        mtk_spi_native_header_t cancel_hdr = base_hdr(0, 0, MTK_SPI_CLASS_CANCEL, 101, 0x0001 /* wrong epoch */);
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 14, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* ---- CANCEL: abort a still-pending[] deferred dispatched operation -- */
    {
        mtk_router_set_async_runner(pthread_runner); /* only this section needs genuine deferral */
        const mtk_opcode_entry_t *ts_op = mtk_test_async_fixture_install() /* RC12 item 5: test-only overlay async op, was TIME_SYNC_START */;
        mtk_time_sync_start_req_t treq = {0}; treq.server.len = 0; treq.timeout_ms = 5000;
        uint8_t tbuf[264]; size_t tblen = 0;
        mtk_encode(ts_op->req_desc, &treq, tbuf, sizeof(tbuf), &tblen);
        mtk_spi_native_header_t thdr = base_hdr(ts_op->service_id, ts_op->opcode, MTK_SPI_CLASS_REQUEST, 200, 0x1234);
        thdr.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; thdr.payload_len = (uint16_t)tblen; thdr.message_len = (uint16_t)tblen;
        mtek_spi_native_dispatch_feed_cell(&dctx, &thdr, tbuf, 20, &resp_hdr, resp_payload, &resp_len);
        /* Genuinely deferred (100ms pthread delay >> this call): IDLE now,
         * a live pending[] slot for request_id=200. */
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
        int slot = -1;
        for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) if (dctx.pending[i].active && dctx.pending[i].request_id == 200) slot = i;
        MTK_CHECK(slot >= 0);

        mtk_spi_native_header_t cancel_hdr = base_hdr(0, 0, MTK_SPI_CLASS_CANCEL, 200, 0x1234);
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 21, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(dctx.pending[slot].active, 0); /* this transport's own tracking forgot it */

        /* The real worker's eventual response (arriving after this test's
         * own 100ms delay) is safely discarded as unmatched, never
         * mistaken for a different request's reply -- proven by draining
         * the queue directly (not through poll_outbound, which would also
         * legitimately return IDLE for the same reason) once it lands. */
        usleep(300000);
        mtk_async_frame_t f;
        int drained_a_response = 0;
        while (mtk_async_queue_pop(&dctx.event_queue, &f)) {
            if (f.kind == MTK_ASYNC_FRAME_RESPONSE && f.correlation == 200) drained_a_response = 1;
        }
        MTK_CHECK(drained_a_response); /* it did arrive... */
        /* ...but is never delivered as anyone's reply: try_deliver_frame's
         * pending[] matching loop has nothing active for request_id=200
         * any more (cleared by CANCEL above), so any further poll can only
         * ever surface IDLE or an unrelated EVENT/STREAM -- never a
         * RESPONSE. */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK(resp_hdr.msg_class != MTK_SPI_CLASS_RESPONSE);
    }

    /* ---- Direction validation: the five outbound-only classes are
     * rejected (LINK_ERROR/PROTOCOL_ERROR), never silently IDLE. -------- */
    {
        uint8_t bad_classes[] = { MTK_SPI_CLASS_RESPONSE, MTK_SPI_CLASS_HELLO_ACK, MTK_SPI_CLASS_EVENT,
                                   MTK_SPI_CLASS_STREAM, MTK_SPI_CLASS_LINK_ERROR };
        for (unsigned i = 0; i < sizeof(bad_classes); i++) {
            mtk_spi_native_header_t bhdr = base_hdr(0, 0, bad_classes[i], 1, 0x1234);
            mtek_spi_native_dispatch_feed_cell(&dctx, &bhdr, NULL, 30 + i, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
            MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
        }
    }

MTK_TEST_MAIN_END
