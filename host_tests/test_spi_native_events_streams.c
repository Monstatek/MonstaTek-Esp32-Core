/* RC5 independent audit P0 "Native events and streams are not
 * implemented": native SPI v1's dispatch layer used to discard every
 * emit_event/emit_stream call for an ACCEPTED_ASYNC operation (cap_event/
 * cap_stream were no-ops) -- signal-meter updates, GATT notifications,
 * capture delivery, and handshake progress could never reach a native
 * SPI peer at all. This proves real, byte-exact delivery for both EVENT
 * and STREAM classes against this implementation's own disclosed wire
 * encoding (mtek_spi_native_dispatch.c's build_event_or_stream_payload
 * doc comment: EVENT = `[name_len:u8][name][canonically-encoded body]`,
 * header.request_id = operation token, STREAM = raw chunk bytes,
 * header.request_id = session token, header.packet_seq = sequence),
 * plus multi-frame ordering and queue-overflow backpressure. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtk_test_async_fixture.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include <string.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

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

/* RC7 independent audit P0 "Native EVENT/STREAM encoding contradicts the
 * accepted header contract": the operation/session token now travels as
 * the payload's own first 4 bytes (little-endian), not header.request_id
 * (which must be 0 for EVENT/STREAM per the accepted protocol). */
static uint32_t token_from_payload(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install) {
    memset(buf, 0, 200);
    /* RC7 independent audit item 11 "EAPOL frames are not filtered to
     * the requested AP/station": addr1/addr2/addr3 must name the target
     * BSSID {1,2,3,4,5,6} used by this file's own HANDSHAKE_START
     * requests, or mtek_wifi_logic.c's new frame_matches_target_bssid
     * filter would reject every synthetic frame this test builds. */
    static const uint8_t bssid[6] = {1,2,3,4,5,6};
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

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init(); /* no async runner registered: the queue-backed ACCEPTED_ASYNC sink is used unconditionally by dispatch_complete_message regardless, so EVENT relay is exercised the same way either way */
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);

    /* ---- EVENT: HANDSHAKE_START's real M1 frame produces a real
     * HANDSHAKE_EVENT the fake HAL delivers synchronously within
     * promisc_start -- proves the disclosed EVENT encoding byte-exactly. ---- */
    const mtk_opcode_entry_t *hs_op = mtk_test_find_op("HANDSHAKE_START");
    uint16_t len = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1: FOUND_EAPOL */
    g_fake_wifi.frames[0].len = len; g_fake_wifi.frames[0].channel = 6;
    g_fake_wifi.frame_count = 1;

    mtk_handshake_start_req_t req = {0};
    memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    req.channel = 6; req.deauth_count = 0;
    uint8_t buf[32]; size_t blen = 0;
    mtk_encode(hs_op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_spi_native_header_t hdr = base_req_hdr(hs_op->service_id, hs_op->opcode, 1, (uint16_t)blen);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 1, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE); /* the accept itself, delivered first */
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
    mtk_handshake_start_resp_t started = {0};
    mtk_decode(hs_op->resp_desc, &started, resp_payload, resp_len, NULL);
    uint32_t op_token = started.operation_token;
    MTK_CHECK(op_token != 0);

    /* The queued HANDSHAKE_EVENT is delivered on the next poll (the peer
     * would send an IDLE cell to ask "anything for me?"). */
    mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_EVENT);
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(resp_hdr.request_id, 0); /* RC7: request_id=0 for EVENT, per the accepted protocol */
    MTK_CHECK_EQ(resp_hdr.packet_seq, 0); /* not meaningful for EVENT */
    MTK_CHECK_EQ(token_from_payload(resp_payload), op_token); /* the operation token instead travels in the payload */

    /* Byte-exact disclosed payload: [token:4][name_len][name]["HANDSHAKE_EVENT"][canonically-encoded mtk_handshake_event_ev_t]. */
    static const char expected_name[] = "HANDSHAKE_EVENT";
    uint8_t name_len = resp_payload[4];
    MTK_CHECK_EQ(name_len, (uint8_t)strlen(expected_name));
    MTK_CHECK(memcmp(resp_payload + 5, expected_name, name_len) == 0);
    mtk_handshake_event_ev_t ev = {0};
    mtk_decode(&mtk_handshake_event_ev_t_desc, &ev, resp_payload + 5 + name_len, resp_len - 5 - name_len, NULL);
    MTK_CHECK_EQ(ev.operation_token, op_token);
    MTK_CHECK_EQ(ev.phase, 0); /* FOUND_EAPOL */
    MTK_CHECK_EQ(ev.key_frame, 1); /* M1 */

    /* Nothing else queued right now beyond this one EVENT (this specific
     * M1-only single-frame capture does not also complete/terminate). */
    mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);

    /* Clean up this session so it does not interfere with the STREAM test below. */
    {
        const mtk_opcode_entry_t *hs_stop = mtk_test_find_op("HANDSHAKE_STOP");
        mtk_handshake_stop_req_t sreq = {0}; sreq.operation_token = op_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(hs_stop->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(hs_stop->service_id, hs_stop->opcode, 2, (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        /* Drain the resulting HANDSHAKE_STOPPED event so it doesn't leak
         * into the next test section's assertions. */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
    }

    /* ---- STREAM: MonstaShark PUSH-mode capture delivers real STREAM
     * chunks -- proves the disclosed STREAM encoding byte-exactly,
     * including a chunk large enough (>512 bytes, the OLD
     * MTK_ASYNC_FRAME_MAX_BODY cap) to prove the widening this fix also
     * required (a smaller cap would have silently truncated captured
     * frame data). ---- */
    {
        mtek_capture_service_init(now_ms);
        mtek_capture_service_register();
        mtk_fake_wifi_reset();
        g_fake_wifi.defer_frames = 1;
        uint8_t pattern[1000];
        for (int i = 0; i < 1000; i++) pattern[i] = (uint8_t)(i & 0xFF);
        memcpy(g_fake_wifi.frames[0].data, pattern, 1000);
        g_fake_wifi.frames[0].len = 1000; g_fake_wifi.frames[0].channel = 1;
        g_fake_wifi.frame_count = 1;

        const mtk_opcode_entry_t *cap_op = mtk_test_find_op("CAPTURE_START");
        mtk_capture_start_req_t creq = {0};
        creq.mode = 0 /* PUSH */; creq.snap_len = 1000;
        creq.channel_plan.mode = 0; creq.channel_plan.channel = 1;
        uint8_t cbuf[32]; size_t cblen = 0;
        mtk_encode(cap_op->req_desc, &creq, cbuf, sizeof(cbuf), &cblen);
        mtk_spi_native_header_t chdr = base_req_hdr(cap_op->service_id, cap_op->opcode, 10, (uint16_t)cblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, cbuf, 10, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t cstarted = {0};
        mtk_decode(cap_op->resp_desc, &cstarted, resp_payload, resp_len, NULL);
        uint32_t cap_token = cstarted.operation_token;

        mtek_capture_grant_credit(cap_token, 4000);
        mtk_fake_wifi_deliver_frames(); /* frame_cb fires now, strictly after the dispatch call above already returned */

        /* First fragment: 25-byte capture-record header + 896 bytes of
         * frame data, delivered verbatim (no further wrapping -- the
         * disclosed STREAM encoding is the raw chunk bytes). */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_STREAM);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(resp_hdr.request_id, 0); /* RC7: request_id=0 for STREAM, per the accepted protocol */
        MTK_CHECK_EQ(token_from_payload(resp_payload), cap_token); /* the session token instead travels in the payload */
        MTK_CHECK_EQ(resp_len, 4 + 25 + 896);
        MTK_CHECK_EQ(resp_hdr.packet_seq, 0); /* first chunk's sequence */
        MTK_CHECK(memcmp(resp_payload + 4 + 25, pattern, 896) == 0); /* byte-exact captured frame data */

        /* Second (continuation) fragment: 8-byte header + remaining 104 bytes. */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_STREAM);
        MTK_CHECK_EQ(resp_hdr.request_id, 0);
        MTK_CHECK_EQ(token_from_payload(resp_payload), cap_token);
        MTK_CHECK_EQ(resp_len, 4 + 8 + 104);
        MTK_CHECK_EQ(resp_hdr.packet_seq, 1); /* second chunk's sequence, one past the first */
        MTK_CHECK(memcmp(resp_payload + 4 + 8, pattern + 896, 104) == 0);

        /* Multi-frame ordering + queue drain: nothing else queued now. */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
    }

    /* ---- Backpressure: a full event_queue never blocks a genuine
     * RESPONSE from eventually being delivered -- mixed RESPONSE/EVENT/
     * STREAM traffic under queue pressure still resolves correctly, one
     * frame drained per poll, none corrupted. ---- */
    {
        mtk_async_queue_reset(&dctx.event_queue);
        for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
            mtk_async_frame_t f; memset(&f, 0, sizeof(f));
            f.kind = (i % 2 == 0) ? MTK_ASYNC_FRAME_EVENT : MTK_ASYNC_FRAME_STREAM;
            f.correlation = 0xDEAD0000u + i;
            mtk_async_queue_push(&dctx.event_queue, &f);
        }
        MTK_CHECK_EQ(mtk_async_queue_dropped_count(&dctx.event_queue), 0); /* exactly filled, not yet overflowed */
        /* RC7 independent audit P0 "Native scheduling can starve or drop
         * control and terminal traffic": mtk_async_queue_push now evicts
         * a lower-priority occupied slot to make room for a higher-
         * priority arrival (RESPONSE > EVENT > STREAM) -- `overflow_frame`
         * must itself be the LOWEST priority (STREAM) to prove the
         * genuinely-full case here (nothing already queued is lower
         * priority than it, so it is the one dropped, not a victim);
         * priority eviction itself is proven directly in
         * test_async_queue.c. */
        mtk_async_frame_t overflow_frame; memset(&overflow_frame, 0, sizeof(overflow_frame));
        overflow_frame.kind = MTK_ASYNC_FRAME_STREAM;
        MTK_CHECK_EQ(mtk_async_queue_push(&dctx.event_queue, &overflow_frame), 0);
        MTK_CHECK(mtk_async_queue_dropped_count(&dctx.event_queue) >= 1);

        /* A generic SYNCHRONOUS token-addressed STOP against an unknown
         * token (never touches dctx->event_queue at all) answers
         * immediately regardless of the queue backlog above, proving the
         * transport layer itself is never blocked by a full backlog of
         * unrelated EVENT/STREAM traffic. RC12 item 1: uses the test-only
         * overlay STOP (0x00F1) instead of the real TIME_SYNC_STOP, which is
         * now UNSUPPORTED on native. */
        const mtk_opcode_entry_t *ts_stop = mtk_test_async_fixture_stop_install();
        mtk_time_sync_stop_req_t sreq = {0}; sreq.operation_token = 0;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(ts_stop->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(ts_stop->service_id, ts_stop->opcode, 20, (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 20, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);

        /* Drain the 8 backlogged synthetic EVENT/STREAM frames -- each
         * delivered as its own cell, never dropped or merged. RC7
         * independent audit P0 "Native scheduling can starve or drop
         * control and terminal traffic": delivery order is now priority-
         * first (all 4 EVENTs, i=0/2/4/6, before all 4 STREAMs, i=1/3/5/7),
         * not strict issue order -- exactly the fix this test now proves,
         * not a regression (the original bug this closes is "FIFO
         * delivery can also send a stream before a queued response/
         * event"). FIFO order is preserved WITHIN each priority tier. */
        static const unsigned expect_order[MTK_ASYNC_QUEUE_DEPTH] = {0, 2, 4, 6, 1, 3, 5, 7};
        for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
            mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
            uint8_t expect_class = (i < 4) ? MTK_SPI_CLASS_EVENT : MTK_SPI_CLASS_STREAM;
            MTK_CHECK_EQ(resp_hdr.msg_class, expect_class);
            MTK_CHECK_EQ(resp_hdr.request_id, 0); /* RC7: request_id=0 for EVENT/STREAM */
            MTK_CHECK_EQ(token_from_payload(resp_payload), 0xDEAD0000u + expect_order[i]);
        }
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
    }

MTK_TEST_MAIN_END
