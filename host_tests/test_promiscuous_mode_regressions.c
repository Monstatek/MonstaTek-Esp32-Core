/* RC11 promiscuous-mode audit follow-up regression tests (portable-layer
 * properties only -- the deferred-queue/generation/resource-allocation
 * fixes live entirely in mtek_wifi_hal_esp32.c, ESP-IDF-only and not
 * host-testable; see that file's own doc comments for those).
 *
 * Part 1/2: a long-lived handshake/capture session must key its own
 * later mtk_op_snapshot lookups on the operation record's REAL core
 * boot_epoch (captured once at start), not the transport ctx's own
 * link/peer epoch -- proven by starting the session under a ctx whose
 * boot_epoch deliberately does NOT match mtk_core's real epoch (mirroring
 * a native SPI peer HELLO resetting its own link epoch independently of
 * mtk_core's own) and confirming frames are still processed correctly
 * (the pre-fix code would have silently dropped every frame, since every
 * later lookup would fail to match rec->boot_epoch).
 *
 * Part 3/4: the promiscuous frame callback must reject a session in
 * MTK_OPS_STOPPING (a concurrent STOP/finalization already claimed this
 * operation and may be mid-cleanup), not just a fully terminal one --
 * proven by forcing STOPPING directly (mtk_op_claim_finalization, the
 * same call a real STOP path uses) before delivering a deferred frame,
 * and confirming it is silently dropped rather than processed. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_capture_service.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install, const uint8_t bssid[6]) {
    memset(buf, 0, 200);
    memcpy(buf + 4, bssid, 6); memcpy(buf + 10, bssid, 6); memcpy(buf + 16, bssid, 6);
    buf[0] = 0x88; buf[1] = 0x02; /* QoS Data, type=2 subtype=8 */
    unsigned off = 26;
    static const uint8_t llc[8] = {0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E};
    memcpy(buf + off, llc, 8); off += 8; /* off == 34 == eapol */
    buf[off + 0] = 2; buf[off + 1] = 3; buf[off + 2] = 0; buf[off + 3] = 95;
    uint16_t key_info = (uint16_t)((ack << 7) | (mic << 8) | (secure << 9) | (install << 6));
    buf[off + 5] = (uint8_t)(key_info >> 8);
    buf[off + 6] = (uint8_t)(key_info & 0xFF);
    return (uint16_t)(off + 4 + 1 + 2 + 96);
}

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtek_capture_service_init(mtk_test_now_ms);
    mtek_capture_service_register();
    static const uint8_t bssid[6] = {9,9,9,9,9,9};

    /* A ctx whose own boot_epoch deliberately diverges from mtk_core's
     * real epoch (MTK_TEST_BOOT_EPOCH, seeded by mtk_test_bootstrap
     * above) -- mirrors a transport whose own link epoch reset
     * independently of mtk_core's own (mtek_spi_native_dispatch.c's own
     * peer-HELLO epoch adoption). */
    #define DIVERGED_CTX_EPOCH 0x77777777u

    /* Part 1: handshake session survives ctx/core epoch divergence. -- */
    {
        mtk_fake_wifi_reset();
        uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0, bssid);
        g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
        uint16_t l2 = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0, bssid);
        g_fake_wifi.frames[1].len = l2; g_fake_wifi.frames[1].channel = 6;
        uint16_t l3 = build_eapol_frame(g_fake_wifi.frames[2].data, 1, 1, 1, 1, bssid);
        g_fake_wifi.frames[2].len = l3; g_fake_wifi.frames[2].channel = 6;
        uint16_t l4 = build_eapol_frame(g_fake_wifi.frames[3].data, 0, 1, 1, 0, bssid);
        g_fake_wifi.frames[3].len = l4; g_fake_wifi.frames[3].channel = 6;
        g_fake_wifi.frame_count = 4;

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        ctx.boot_epoch = DIVERGED_CTX_EPOCH; /* deliberately NOT mtk_core_boot_epoch */
        const mtk_opcode_entry_t *op = mtk_test_find_op("HANDSHAKE_START");
        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, bssid, 6);
        req.channel = 6; req.deauth_count = 0;

        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        /* Pre-fix (s_hs.boot_epoch = ctx->boot_epoch): every hs_frame_cb
         * lookup below would use DIVERGED_CTX_EPOCH, never matching the
         * record's real mtk_core epoch -- classify_eapol would still run,
         * but the mtk_op_snapshot gate at the top of hs_frame_cb would
         * fail every time, so NO event would ever fire and M4 would never
         * complete the operation. */
        unsigned hs_events = 0;
        for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "HANDSHAKE_EVENT") == 0) hs_events++;
        MTK_CHECK_EQ(hs_events, 4u);
        const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
        MTK_CHECK(stopped != NULL);
        if (stopped) {
            mtk_handshake_stopped_ev_t done = {0};
            mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
            MTK_CHECK_EQ(done.status, MTK_STATUS_OK);
        }
        mtk_arbiter_reset();
    }

    /* Part 2: MonstaShark capture session survives ctx/core epoch divergence. -- */
    {
        mtk_fake_wifi_reset();
        memset(g_fake_wifi.frames[0].data, 0xAB, 100); g_fake_wifi.frames[0].len = 100; g_fake_wifi.frames[0].channel = 6;
        g_fake_wifi.frame_count = 1;

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        ctx.boot_epoch = DIVERGED_CTX_EPOCH;
        const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
        mtk_capture_start_req_t req = {0};
        req.mode = 1 /* POLL */; req.snap_len = 1000; req.duration_ms = 0;
        req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t started = {0};
        mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
        MTK_CHECK(started.operation_token != 0);

        /* Pre-fix (s_cap.ctx.boot_epoch used as the epoch), frame_cb's own
         * mtk_op_snapshot would never match -- the frame delivered
         * synchronously during promisc_start above would be silently
         * dropped, and CAPTURE_STATUS would report frames_captured==0. */
        mtk_fake_sink_state_t stsink; mtk_fake_sink_reset(&stsink);
        mtk_request_ctx_t stctx = mtk_test_ctx(&stsink, 2);
        const mtk_opcode_entry_t *status_op = mtk_test_find_op("CAPTURE_STATUS");
        mtk_capture_status_req_t sreq = {0}; sreq.operation_token = started.operation_token;
        mtk_test_call(&stctx, status_op, &sreq);
        MTK_CHECK_EQ(stsink.response.status, MTK_STATUS_OK);
        mtk_capture_status_resp_t sr = {0};
        mtk_decode(&mtk_capture_status_resp_t_desc, &sr, stsink.response.body, stsink.response.body_len, NULL);
        MTK_CHECK_EQ(sr.frames_captured, 1u);

        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 3);
        const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");
        mtk_capture_stop_req_t stopreq = {0}; stopreq.operation_token = started.operation_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK_EQ(stopsink.response.status, MTK_STATUS_OK);
    }

    /* Part 3: handshake callback rejects STOPPING (not just terminal). -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.defer_frames = 1; /* promisc_start only records the callback; replay explicitly below */
        uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0, bssid); /* M1 */
        g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
        g_fake_wifi.frame_count = 1;

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("HANDSHAKE_START");
        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, bssid, 6);
        req.channel = 6; req.deauth_count = 0;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        mtk_handshake_start_resp_t started = {0};
        mtk_decode(op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);

        /* Force STOPPING directly -- the same claim a real concurrent
         * STOP/finalization path uses -- WITHOUT running the rest of
         * handshake_finish's own cleanup, so the operation is stuck
         * mid-teardown exactly like a real race would leave it. */
        MTK_CHECK(mtk_op_claim_finalization(started.operation_token, mtk_core_boot_epoch()));

        mtk_fake_wifi_deliver_frames(); /* deliver the deferred M1 frame now, while STOPPING */
        unsigned hs_events = 0;
        for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "HANDSHAKE_EVENT") == 0) hs_events++;
        MTK_CHECK_EQ(hs_events, 0u); /* silently dropped -- never processed into a STOPPING session */

        mtk_op_transition_by_token(started.operation_token, mtk_core_boot_epoch(), MTK_OPS_STOPPED, MTK_STATUS_OK, mtk_test_now_ms());
        mtk_arbiter_reset();
    }

    /* Part 4: capture callback rejects STOPPING (not just terminal). -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.defer_frames = 1;
        memset(g_fake_wifi.frames[0].data, 0xCD, 100); g_fake_wifi.frames[0].len = 100; g_fake_wifi.frames[0].channel = 6;
        g_fake_wifi.frame_count = 1;

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
        mtk_capture_start_req_t req = {0};
        req.mode = 1 /* POLL */; req.snap_len = 1000; req.duration_ms = 0;
        req.channel_plan.mode = 0; req.channel_plan.channel = 6;
        mtk_test_call(&ctx, start_op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t started = {0};
        mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);

        MTK_CHECK(mtk_op_claim_finalization(started.operation_token, mtk_core_boot_epoch()));
        mtk_fake_wifi_deliver_frames(); /* deliver the deferred frame now, while STOPPING */

        mtk_fake_sink_state_t stsink; mtk_fake_sink_reset(&stsink);
        mtk_request_ctx_t stctx = mtk_test_ctx(&stsink, 2);
        const mtk_opcode_entry_t *status_op = mtk_test_find_op("CAPTURE_STATUS");
        mtk_capture_status_req_t sreq = {0}; sreq.operation_token = started.operation_token;
        mtk_test_call(&stctx, status_op, &sreq);
        mtk_capture_status_resp_t sr = {0};
        mtk_decode(&mtk_capture_status_resp_t_desc, &sr, stsink.response.body, stsink.response.body_len, NULL);
        MTK_CHECK_EQ(sr.frames_captured, 0u); /* silently dropped -- never processed into a STOPPING session */

        mtk_op_transition_by_token(started.operation_token, mtk_core_boot_epoch(), MTK_OPS_STOPPED, MTK_STATUS_OK, mtk_test_now_ms());
    }

MTK_TEST_MAIN_END
