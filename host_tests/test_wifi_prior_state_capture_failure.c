/* mtek_wifi_hal_esp32.c's own capture_prior_state_once previously ignored
 * esp_wifi_get_mode's own failure (leaving `valid` set around unknown mode) and
 * silently substituted channel 0 on an esp_wifi_get_channel failure -- and never
 * captured promiscuous state at all. The real fix treats ANY required snapshot
 * read failing (mode, channel, or the new promiscuous read, which also serves as
 * an invariant check) as a genuine transactional- entry failure -- never
 * proceeding over unknown prior radio state. This test proves the pattern via
 * the fake HAL's own mirror (prior_state_capture_rc): every real "disturb the
 * radio" entry point (deauth, handshake/capture, station scan, raw TX's own
 * channel select) must fail cleanly -- no transmission, no false "0 results"
 * success, exactly one terminal event, and the arbiter released -- rather than
 * silently proceeding over a snapshot that was never actually taken. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    /* DEAUTH_START: capture failure means the very first send_deauth call itself
     * fails, and the operation must not report a false "completed" success. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.prior_state_capture_rc = -1;

        const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");
        mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
        req.target_mode = 2 /* BROADCAST */;
        memset(req.ap_bssid.b, 0xAA, 6);
        req.channel = 6; req.count = 3; req.interval_ms = 1;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0u); /* the send itself never happened */

        const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "DEAUTH_STOPPED");
        MTK_CHECK(stopped != NULL);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

        /* Once the snapshot can genuinely be taken again, the identical
         * request succeeds -- proves this is a real, recoverable
         * transactional failure, not a permanent lockout. */
        g_fake_wifi.prior_state_capture_rc = 0;
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 2);
        mtk_test_call(&ctx2, op, &req);
        MTK_CHECK_EQ(sink2.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK(g_fake_wifi.deauth_sent_count >= 1);
    }

    /* HANDSHAKE_START: capture failure means promisc_start itself fails --
     * exactly the monitor-entry failure path, never sends the initiating deauth
     * burst. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.prior_state_capture_rc = -1;

        const mtk_opcode_entry_t *op = mtk_test_find_op("HANDSHAKE_START");
        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        req.channel = 6; req.deauth_count = 5;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0u);
        MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 0u); /* capture failed before promisc_start's own body ever ran */

        const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
        MTK_CHECK(stopped != NULL);
        if (stopped) {
            mtk_handshake_stopped_ev_t done = {0};
            mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
            MTK_CHECK_EQ(done.status, MTK_STATUS_IO_ERROR);
        }
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

    /* STA_SCAN_START: capture failure surfaces as a real transactional-entry
     * failure (status=IO_ERROR), never a false "0 stations found" success. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.prior_state_capture_rc = -1;

        const mtk_opcode_entry_t *op = mtk_test_find_op("STA_SCAN_START");
        mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
        memset(req.target_bssid.b, 0xAA, 6); req.channel = 6; req.duration_ms = 10;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(sink.event_count, 1u);
        mtk_sta_scan_complete_ev_t ev = {0};
        mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &ev, sink.events[0].body, sink.events[0].body_len, NULL);
        MTK_CHECK_EQ(ev.status, MTK_STATUS_IO_ERROR); /* not OK with result_count=0 */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

    /* RAW_TX_SEND: capture failure at set_channel means the frame itself must
     * never transmit. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_fake_wifi_reset();
        g_fake_wifi.prior_state_capture_rc = -1;

        const mtk_opcode_entry_t *op = mtk_test_find_op("RAW_TX_SEND");
        mtk_raw_tx_send_req_t req; memset(&req, 0, sizeof(req));
        req.channel = 7; req.frame.len = 10; memset(req.frame.data, 0x11, 10);
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_IO_ERROR);
        MTK_CHECK_EQ(g_fake_wifi.raw_tx_count, 0u);
    }

MTK_TEST_MAIN_END
