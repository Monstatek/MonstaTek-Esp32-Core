/* WPA handshake capture lifecycle (002-wifi-service.md Sec 2.5): fixed
 * channel, EAPOL M1-M4 classification from raw 802.11 frames, HANDSHAKE_EVENT
 * progress phases, HANDSHAKE_READ byte range, and the terminal
 * HANDSHAKE_STOPPED event. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

/* Builds a minimal 802.11 QoS data frame carrying one EAPOL-Key message
 * with the given Key Information bits, matching classify_eapol()'s parse
 * in mtek_wifi_logic.c exactly (802.11-2020 Sec 12.7.2). */
static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install) {
    memset(buf, 0, 200);
    /* RC7 independent audit item 11 "EAPOL frames are not filtered to
     * the requested AP/station": addr1/addr2/addr3 must name the target
     * BSSID {1,2,3,4,5,6} used by this file's own HANDSHAKE_START
     * requests, or mtek_wifi_logic.c's new frame_matches_target_bssid
     * filter would reject every synthetic frame this test builds. */
    static const uint8_t bssid[6] = {1,2,3,4,5,6};
    memcpy(buf + 4, bssid, 6); memcpy(buf + 10, bssid, 6); memcpy(buf + 16, bssid, 6);
    buf[0] = 0x88; buf[1] = 0x02; /* QoS Data, type=2 subtype=8 */
    unsigned off = 24 + 2; /* 802.11 header + QoS control */
    static const uint8_t llc[8] = {0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E};
    memcpy(buf + off, llc, 8); off += 8;
    buf[off + 0] = 2; /* EAPOL version */
    buf[off + 1] = 3; /* EAPOL-Key */
    buf[off + 2] = 0; buf[off + 3] = 95; /* body length, arbitrary */
    unsigned eapol = off;
    buf[eapol + 4] = 2; /* descriptor type */
    uint16_t key_info = (uint16_t)((ack << 7) | (mic << 8) | (secure << 9) | (install << 6));
    buf[eapol + 5] = (uint8_t)(key_info >> 8);
    buf[eapol + 6] = (uint8_t)(key_info & 0xFF);
    off = eapol + 4 + 1 + 2 + 96; /* enough trailing bytes for a realistic frame */
    return (uint16_t)off;
}

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_wifi_reset();

    uint16_t len;
    len = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
    g_fake_wifi.frames[0].len = len; g_fake_wifi.frames[0].channel = 6; g_fake_wifi.frames[0].rssi = -40;
    len = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0); /* M2 */
    g_fake_wifi.frames[1].len = len; g_fake_wifi.frames[1].channel = 6;
    len = build_eapol_frame(g_fake_wifi.frames[2].data, 1, 1, 1, 1); /* M3 */
    g_fake_wifi.frames[2].len = len; g_fake_wifi.frames[2].channel = 6;
    len = build_eapol_frame(g_fake_wifi.frames[3].data, 0, 1, 1, 0); /* M4 */
    g_fake_wifi.frames[3].len = len; g_fake_wifi.frames[3].channel = 6;
    g_fake_wifi.frame_count = 4;

    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("HANDSHAKE_START");
    mtk_handshake_start_req_t req = {0};
    memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    req.channel = 6; req.deauth_count = 0;

    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 1);

    /* Four HANDSHAKE_EVENT progress notifications, phases FOUND_EAPOL(0),
     * CAPTURED(1), CAPTURED(1) for M3, SUCCESS(2) for M4. */
    unsigned hs_events = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "HANDSHAKE_EVENT") == 0) hs_events++;
    MTK_CHECK_EQ(hs_events, 4);

    /* Terminal event fires once the full 4-way is observed. */
    const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
    MTK_CHECK(stopped != NULL);
    if (stopped) {
        mtk_handshake_stopped_ev_t done = {0};
        mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
        MTK_CHECK_EQ(done.status, MTK_STATUS_OK);
        MTK_CHECK(done.captured_total_len > 0);
    }
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
    MTK_CHECK_EQ(g_fake_wifi.restore_count, 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* HANDSHAKE_READ returns the accumulated raw bytes in the requested range. */
    mtk_fake_sink_state_t rsink; mtk_fake_sink_reset(&rsink);
    mtk_request_ctx_t rctx = mtk_test_ctx(&rsink, 2);
    const mtk_opcode_entry_t *read_op = mtk_test_find_op("HANDSHAKE_READ");
    mtk_handshake_read_req_t rreq = {0};
    /* the operation token from the ACCEPTED response */
    mtk_handshake_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
    rreq.operation_token = started.operation_token; rreq.offset = 0; rreq.max_len = 512;
    mtk_test_call(&rctx, read_op, &rreq);
    MTK_CHECK_EQ(rsink.response.status, MTK_STATUS_OK);
    mtk_handshake_read_resp_t rd = {0};
    mtk_decode(read_op->resp_desc, &rd, rsink.response.body, rsink.response.body_len, NULL);
    MTK_CHECK(rd.total_len > 0);
    MTK_CHECK(rd.data.len > 0);

    /* Channel bound validation: HANDSHAKE_START with channel 0 is rejected. */
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink, 3);
    mtk_handshake_start_req_t bad = {0};
    bad.channel = 0;
    mtk_test_call(&ctx2, op, &bad);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_INVALID_ARGUMENT);

    /* RC5 independent audit P1 "Handshake capture is unsafe and
     * semantically incomplete": "completion check accepts message 4 plus
     * either message 1 or 2 rather than proving the declared capture
     * criteria." M1 + M4 with NO M2 observed must NOT be reported as a
     * successful capture -- an offline crack attempt is mathematically
     * impossible without the station's SNonce+MIC (M2), regardless of
     * M4 having arrived. */
    {
        mtk_fake_wifi_reset();
        uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
        g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
        uint16_t l4 = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 1, 0); /* M4 -- no M2 in between */
        g_fake_wifi.frames[1].len = l4; g_fake_wifi.frames[1].channel = 6;
        g_fake_wifi.frame_count = 2;

        mtk_fake_sink_state_t psink; mtk_fake_sink_reset(&psink);
        mtk_request_ctx_t pctx = mtk_test_ctx(&psink, 4);
        mtk_handshake_start_req_t preq = {0};
        memcpy(preq.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        preq.channel = 6; preq.deauth_count = 0;
        mtk_test_call(&pctx, op, &preq);
        MTK_CHECK_EQ(psink.response.status, MTK_STATUS_ACCEPTED);

        /* Both HANDSHAKE_EVENT frames still fire (real progress is
         * always reported), but no HANDSHAKE_STOPPED -- the operation is
         * correctly still RUNNING, not falsely declared complete. */
        unsigned p_events = 0;
        for (unsigned i = 0; i < psink.event_count; i++) if (strcmp(psink.events[i].name, "HANDSHAKE_EVENT") == 0) p_events++;
        MTK_CHECK_EQ(p_events, 2);
        MTK_CHECK(mtk_fake_find_event(&psink, "HANDSHAKE_STOPPED") == NULL);

        mtk_handshake_start_resp_t pstarted = {0};
        mtk_decode(op->resp_desc, &pstarted, psink.response.body, psink.response.body_len, NULL);
        const mtk_opcode_entry_t *hstatus_op = mtk_test_find_op("HANDSHAKE_STATUS");
        mtk_handshake_status_req_t hsreq = {0}; hsreq.operation_token = pstarted.operation_token;
        mtk_fake_sink_state_t hssink; mtk_fake_sink_reset(&hssink);
        mtk_request_ctx_t hsctx = mtk_test_ctx(&hssink, 5);
        mtk_test_call(&hsctx, hstatus_op, &hsreq);
        mtk_handshake_status_resp_t hs = {0};
        mtk_decode(hstatus_op->resp_desc, &hs, hssink.response.body, hssink.response.body_len, NULL);
        MTK_CHECK_EQ(hs.state, MTK_OPS_RUNNING); /* not falsely COMPLETED -- the exact bug this fix closes */

        /* Clean up: stop the still-RUNNING session. */
        const mtk_opcode_entry_t *hstop_op = mtk_test_find_op("HANDSHAKE_STOP");
        mtk_handshake_stop_req_t hstopreq = {0}; hstopreq.operation_token = pstarted.operation_token;
        mtk_fake_sink_state_t hstopsink; mtk_fake_sink_reset(&hstopsink);
        mtk_request_ctx_t hstopctx = mtk_test_ctx(&hstopsink, 6);
        mtk_test_call(&hstopctx, hstop_op, &hstopreq);
        MTK_CHECK_EQ(hstopsink.response.status, MTK_STATUS_OK);
    }

    /* ---- RC7 independent audit P0 "Shared operation/session state
     * remains data-racy" + item 11 "natural success ... normally does
     * not stop promiscuous mode, release the arbiter, restore STA, or
     * emit the terminal stopped event": with frames delivered AFTER
     * HANDSHAKE_START already returned (matching a real target's
     * asynchronous Wi-Fi-driver-task delivery, not the synchronous-
     * within-promisc_start default every earlier section above relies
     * on), natural M4 completion must still perform full cleanup exactly
     * once, and an explicit STOP arriving afterward must be a safe,
     * non-double-cleaning no-op. -------------------------------------- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.defer_frames = 1; /* promisc_start only records the callback; frames replay later */
        uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
        g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
        uint16_t l2 = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0); /* M2 */
        g_fake_wifi.frames[1].len = l2; g_fake_wifi.frames[1].channel = 6;
        uint16_t l4 = build_eapol_frame(g_fake_wifi.frames[2].data, 0, 1, 1, 0); /* M4 */
        g_fake_wifi.frames[2].len = l4; g_fake_wifi.frames[2].channel = 6;
        g_fake_wifi.frame_count = 3;

        mtk_fake_sink_state_t asink; mtk_fake_sink_reset(&asink);
        mtk_request_ctx_t actx = mtk_test_ctx(&asink, 7);
        mtk_handshake_start_req_t areq = {0};
        /* Must match build_eapol_frame's own hardcoded {1,2,3,4,5,6}
         * address fields (RC7 independent audit item 11's new
         * frame_matches_target_bssid filter). */
        memcpy(areq.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        areq.channel = 6; areq.deauth_count = 0;
        mtk_test_call(&actx, op, &areq);
        MTK_CHECK_EQ(asink.response.status, MTK_STATUS_ACCEPTED);

        /* Nothing has completed yet -- promisc is still armed, the
         * arbiter lease is still held, no terminal event. */
        MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 0);
        MTK_CHECK(mtk_arbiter_active_class() != MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&asink, "HANDSHAKE_STOPPED") == NULL);

        mtk_handshake_start_resp_t astarted = {0};
        mtk_decode(op->resp_desc, &astarted, asink.response.body, asink.response.body_len, NULL);

        /* Frames arrive now, strictly after the call above already
         * returned -- hs_frame_cb runs "later", exactly matching a real
         * target's Wi-Fi-driver-task delivery timing. M4 completion must
         * itself perform full cleanup (handshake_finish) since nothing
         * else will. */
        mtk_fake_wifi_deliver_frames();
        MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *astopped = mtk_fake_find_event(&asink, "HANDSHAKE_STOPPED");
        MTK_CHECK(astopped != NULL);

        /* An explicit STOP arriving afterward (a real, if narrow, race
         * against natural completion) must be a safe no-op -- NOT a
         * second promisc_stop/restore/arbiter_release/HANDSHAKE_STOPPED.
         * This is the exact defect class this round closes: before this
         * fix, hs_frame_cb's own M4 completion never did this cleanup at
         * all, so only a REAL race even existed once handshake_finish was
         * introduced -- this proves the introduced fix doesn't ALSO
         * introduce a double-cleanup bug of its own. */
        const mtk_opcode_entry_t *stop_op2 = mtk_test_find_op("HANDSHAKE_STOP");
        mtk_handshake_stop_req_t stopreq2 = {0}; stopreq2.operation_token = astarted.operation_token;
        mtk_fake_sink_state_t stopsink2; mtk_fake_sink_reset(&stopsink2);
        mtk_request_ctx_t stopctx2 = mtk_test_ctx(&stopsink2, 8);
        mtk_test_call(&stopctx2, stop_op2, &stopreq2);
        MTK_CHECK_EQ(stopsink2.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1); /* unchanged -- STOP lost the race, correctly did nothing further */
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1);
        MTK_CHECK(mtk_fake_find_event(&stopsink2, "HANDSHAKE_STOPPED") == NULL); /* STOP's own response carries final state; it does not re-emit the terminal event */
    }

    /* ---- RC7 independent audit item 11 "EAPOL frames are not filtered
     * to the requested AP/station": a real EAPOL-Key frame from a
     * DIFFERENT BSSID than the one requested must be ignored entirely --
     * no HANDSHAKE_EVENT, no progress, no false completion -- even
     * though it is otherwise a perfectly well-formed M1/M2/M4 sequence
     * classify_eapol would happily accept from the right network. ------ */
    {
        mtk_fake_wifi_reset();
        uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
        uint16_t l2 = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0); /* M2 */
        uint16_t l4 = build_eapol_frame(g_fake_wifi.frames[2].data, 0, 1, 1, 0); /* M4 */
        /* Overwrite the addr1/addr2/addr3 fields build_eapol_frame set to
         * the "right" BSSID -- these frames belong to a DIFFERENT
         * network the caller never asked about. */
        static const uint8_t wrong_bssid[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
        for (int i = 0; i < 3; i++) {
            uint8_t *f = g_fake_wifi.frames[i].data;
            memcpy(f + 4, wrong_bssid, 6); memcpy(f + 10, wrong_bssid, 6); memcpy(f + 16, wrong_bssid, 6);
        }
        g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
        g_fake_wifi.frames[1].len = l2; g_fake_wifi.frames[1].channel = 6;
        g_fake_wifi.frames[2].len = l4; g_fake_wifi.frames[2].channel = 6;
        g_fake_wifi.frame_count = 3;

        mtk_fake_sink_state_t fsink; mtk_fake_sink_reset(&fsink);
        mtk_request_ctx_t fctx = mtk_test_ctx(&fsink, 9);
        mtk_handshake_start_req_t freq = {0};
        memcpy(freq.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6); /* the RIGHT bssid -- frames above are the WRONG one */
        freq.channel = 6; freq.deauth_count = 0;
        mtk_test_call(&fctx, op, &freq);
        MTK_CHECK_EQ(fsink.response.status, MTK_STATUS_ACCEPTED);

        /* Every frame delivered was for the wrong network -- none of them
         * should have produced any progress at all. */
        MTK_CHECK_EQ(fsink.event_count, 0);
        MTK_CHECK(mtk_fake_find_event(&fsink, "HANDSHAKE_EVENT") == NULL);
        MTK_CHECK(mtk_fake_find_event(&fsink, "HANDSHAKE_STOPPED") == NULL);

        mtk_handshake_start_resp_t fstarted = {0};
        mtk_decode(op->resp_desc, &fstarted, fsink.response.body, fsink.response.body_len, NULL);
        const mtk_opcode_entry_t *fstop_op = mtk_test_find_op("HANDSHAKE_STOP");
        mtk_handshake_stop_req_t fstopreq = {0}; fstopreq.operation_token = fstarted.operation_token;
        mtk_fake_sink_state_t fstopsink; mtk_fake_sink_reset(&fstopsink);
        mtk_request_ctx_t fstopctx = mtk_test_ctx(&fstopsink, 10);
        mtk_test_call(&fstopctx, fstop_op, &fstopreq);
        MTK_CHECK_EQ(fstopsink.response.status, MTK_STATUS_OK);
    }

MTK_TEST_MAIN_END
