/* "operation finalization is not exactly-once/truthful" + "if radio restoration
 * fails, retain the prior-state snapshot and keep the radio lease quarantined --
 * never report/expose it as healthy/free" + "make WIFI_STOP_ALL retry verified
 * recovery before releasing ownership".
 *
 * Part 1: a DEAUTH operation whose own post-completion restore fails must report
 * IO_ERROR (never a clean OK), call restore exactly once (never
 * double-restored), and leave the D-class lease HELD (quarantined) rather than
 * released -- mtek_wifi_logic.c's own deauth_finalize.
 *
 * Part 2: same property for a WPA handshake capture's natural M4 completion --
 * mtek_wifi_logic.c's own handshake_finish.
 *
 * Part 3: WIFI_STOP_ALL against a persistent restore failure must retry a
 * bounded number of times (never just once), never release the lease it cannot
 * confirm safe, and GET_WIFI_RECOVERY_STATE must honestly report "not restored"
 * (never healthy/free) afterward.
 *
 * Part 4: WIFI_STOP_ALL against a TRANSIENT restore failure (fails on its first
 * attempts, succeeds within the retry budget) must actually recover -- proving
 * the retry loop is not cosmetic -- and release the lease once a retry genuinely
 * confirms success. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    /* Part 1: deauth, one-shot BROADCAST, restore's own "stop" step fails. -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.restore_stop_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");
        mtk_deauth_start_req_t req = {0};
        req.target_mode = 2; /* BROADCAST */
        memcpy(req.ap_bssid.b, (uint8_t[]){2,2,3,4,5,6}, 6);
        req.channel = 6; req.count = 1; req.interval_ms = 0;

        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "DEAUTH_STOPPED");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_deauth_stopped_ev_t done = {0};
            mtk_decode(&mtk_deauth_stopped_ev_t_desc, &done, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(done.status, MTK_STATUS_IO_ERROR); /* truthful: never a clean OK */
        }
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u);  /* exactly once -- never double-restored */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_D); /* lease HELD (quarantined), not released */
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        mtk_arbiter_reset(); /* test-harness cleanup only -- see
                              * mtk_fake_wifi_reset's own established role */
    }

    /* Part 2: WPA handshake, natural M4 completion, restore's own "promiscuous
     * off" step fails. -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.restore_promisc_off_rc = -1;

        /* Minimal single-frame M1+M2+M4-in-one-pass isn't representable
         * (classify_eapol needs one frame per message); reuse the same
         * 4-frame canned M1..M4 sequence test_handshake_capture.c already
         * proves classify_eapol against, just for restore-failure
         * honesty this time. */
        uint8_t *f;
        static const uint8_t bssid[6] = {7,7,7,7,7,7};
        /* Layout matches classify_eapol's own parse exactly (802.11 header 24B +
         * QoS control 2B = hdr 26; LLC/SNAP 8B at hdr; EAPOL header starts at
         * hdr+8=34; eapol_type at eapol+1=35; key_info at eapol+5..6=39..40) --
         * mirrors test_handshake_capture.c's own build_eapol_frame helper,
         * inlined here as a macro so this file stays self-contained. */
        #define BUILD(idx, ack, mic, secure, install) \
            f = g_fake_wifi.frames[idx].data; memset(f, 0, 200); \
            memcpy(f + 4, bssid, 6); memcpy(f + 10, bssid, 6); memcpy(f + 16, bssid, 6); \
            f[0] = 0x88; f[1] = 0x02; \
            memcpy(f + 26, (uint8_t[]){0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E}, 8); \
            f[34] = 2; f[35] = 3; f[36] = 0; f[37] = 95; \
            f[38] = 2; \
            f[39] = (uint8_t)((((ack)<<7)|((mic)<<8)|((secure)<<9)|((install)<<6)) >> 8); \
            f[40] = (uint8_t)((((ack)<<7)|((mic)<<8)|((secure)<<9)|((install)<<6)) & 0xFF); \
            g_fake_wifi.frames[idx].len = 34 + 4 + 1 + 2 + 96; g_fake_wifi.frames[idx].channel = 6;
        BUILD(0, 1, 0, 0, 0) /* M1 */
        BUILD(1, 0, 1, 0, 0) /* M2 */
        BUILD(2, 1, 1, 1, 1) /* M3 */
        BUILD(3, 0, 1, 1, 0) /* M4 */
        #undef BUILD
        g_fake_wifi.frame_count = 4;

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("HANDSHAKE_START");
        mtk_handshake_start_req_t req = {0};
        memcpy(req.target_bssid.b, bssid, 6);
        req.channel = 6; req.deauth_count = 0;

        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
        MTK_CHECK(stopped != NULL);
        if (stopped) {
            mtk_handshake_stopped_ev_t done = {0};
            mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
            MTK_CHECK_EQ(done.status, MTK_STATUS_IO_ERROR); /* truthful: never a clean OK */
        }
        MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1u);
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u); /* exactly once */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_H); /* lease HELD (quarantined), not released */
        MTK_CHECK(mtek_wifi_radio_is_quarantined());
        mtk_arbiter_reset();
    }

    /* Part 3: WIFI_STOP_ALL against a PERSISTENT restore failure -- every retry
     * attempt fails, so the lease must stay quarantined, never released, and the
     * retry loop must actually retry (not just try once). -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.restore_stop_rc = -1; /* sticky: every attempt fails identically */
        mtk_arbiter_reset();
        mtk_arbiter_acquire(MTK_ARB_D, 4242); /* manufacture an active lease WIFI_STOP_ALL must act on */

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("WIFI_STOP_ALL");
        mtk_test_call(&ctx, op, NULL);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK); /* WIFI_STOP_ALL's own response is unconditionally OK-shaped -- honesty lives in GET_WIFI_RECOVERY_STATE */
        MTK_CHECK(g_fake_wifi.restore_count >= 2u); /* genuinely retried, not a single best-effort attempt */
        MTK_CHECK(mtk_arbiter_active_class() != MTK_ARB_NONE); /* never released over an unconfirmed radio state */
        MTK_CHECK(mtek_wifi_radio_is_quarantined());

        mtk_fake_sink_state_t rsink; mtk_fake_sink_reset(&rsink);
        mtk_request_ctx_t rctx = mtk_test_ctx(&rsink, 2);
        const mtk_opcode_entry_t *rop = mtk_test_find_op("GET_WIFI_RECOVERY_STATE");
        mtk_test_call(&rctx, rop, NULL);
        MTK_CHECK_EQ(rsink.response.status, MTK_STATUS_OK);
        mtk_get_wifi_recovery_state_resp_t rr = {0};
        mtk_decode(&mtk_get_wifi_recovery_state_resp_t_desc, &rr, rsink.response.body, rsink.response.body_len, NULL);
        MTK_CHECK_EQ(rr.sta_mode_restored, 0); /* never reported healthy/free after every retry failed */
        mtk_arbiter_reset();
    }

    /* Part 4: WIFI_STOP_ALL against a TRANSIENT restore failure that clears
     * within the retry budget -- proves recovery actually happens, not merely
     * that failure is handled safely. -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.restore_fail_countdown = 2; /* fails attempts 1-2, succeeds on attempt 3 -- within the 3-attempt budget */
        mtk_arbiter_reset();
        mtk_arbiter_acquire(MTK_ARB_D, 4343);

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("WIFI_STOP_ALL");
        mtk_test_call(&ctx, op, NULL);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 3u); /* two failures, then the attempt that finally succeeds */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* genuinely recovered -- lease released */
        MTK_CHECK(!mtek_wifi_radio_is_quarantined());

        mtk_fake_sink_state_t rsink; mtk_fake_sink_reset(&rsink);
        mtk_request_ctx_t rctx = mtk_test_ctx(&rsink, 2);
        const mtk_opcode_entry_t *rop = mtk_test_find_op("GET_WIFI_RECOVERY_STATE");
        mtk_test_call(&rctx, rop, NULL);
        mtk_get_wifi_recovery_state_resp_t rr = {0};
        mtk_decode(&mtk_get_wifi_recovery_state_resp_t_desc, &rr, rsink.response.body, rsink.response.body_len, NULL);
        MTK_CHECK_EQ(rr.sta_mode_restored, 1); /* honestly reported healthy/free once actually confirmed */
    }

MTK_TEST_MAIN_END
