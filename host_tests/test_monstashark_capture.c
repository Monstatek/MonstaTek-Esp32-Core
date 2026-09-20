/* MonstaShark capture: poll-mode lifecycle (start/poll/stop, tagged EMPTY/RECORD
 * response), push-mode credit gating and stream fragmentation, and
 * counters/drops. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_capture_service.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

static void make_frame(uint8_t *buf, uint16_t len, uint8_t fill) { memset(buf, fill, len); }

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtek_capture_service_init(mtk_test_now_ms);
    mtek_capture_service_register();

    /* Poll mode -- */
    mtk_fake_wifi_reset();
    make_frame(g_fake_wifi.frames[0].data, 100, 0xAB); g_fake_wifi.frames[0].len = 100; g_fake_wifi.frames[0].channel = 6; g_fake_wifi.frames[0].rssi = -30;
    make_frame(g_fake_wifi.frames[1].data, 200, 0xCD); g_fake_wifi.frames[1].len = 200; g_fake_wifi.frames[1].channel = 6;
    g_fake_wifi.frame_count = 2;

    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *start_op = mtk_test_find_op("CAPTURE_START");
    mtk_capture_start_req_t req = {0};
    req.mode = 1; /* POLL */
    req.snap_len = 1000;
    req.duration_ms = 0;
    req.channel_plan.mode = 0; req.channel_plan.channel = 6;
    mtk_test_call(&ctx, start_op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    mtk_capture_start_resp_t started = {0};
    mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);
    MTK_CHECK(started.operation_token != 0);

    const mtk_opcode_entry_t *poll_op = mtk_test_find_op("CAPTURE_POLL_READ");
    mtk_capture_poll_read_req_t preq = {0}; preq.operation_token = started.operation_token;

    mtk_fake_sink_state_t psink1; mtk_fake_sink_reset(&psink1);
    mtk_request_ctx_t pctx1 = mtk_test_ctx(&psink1, 2);
    mtk_test_call(&pctx1, poll_op, &preq);
    MTK_CHECK_EQ(psink1.response.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(psink1.response.body_len >= 1, 1);
    MTK_CHECK_EQ(psink1.response.body[0], 1); /* tag=RECORD */

    mtk_fake_sink_state_t psink2; mtk_fake_sink_reset(&psink2);
    mtk_request_ctx_t pctx2 = mtk_test_ctx(&psink2, 3);
    mtk_test_call(&pctx2, poll_op, &preq);
    MTK_CHECK_EQ(psink2.response.body[0], 1); /* second buffered frame */

    mtk_fake_sink_state_t psink3; mtk_fake_sink_reset(&psink3);
    mtk_request_ctx_t pctx3 = mtk_test_ctx(&psink3, 4);
    mtk_test_call(&pctx3, poll_op, &preq);
    MTK_CHECK_EQ(psink3.response.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(psink3.response.body_len, 1);
    MTK_CHECK_EQ(psink3.response.body[0], 0); /* tag=EMPTY: exactly one byte, no fake trailing record */

    /* CAPTURE_STOP: idempotent, no duplicate terminal event. */
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("CAPTURE_STOP");
    mtk_capture_stop_req_t sreq = {0}; sreq.operation_token = started.operation_token;
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t sctx = mtk_test_ctx(&sink, 5);
    mtk_test_call(&sctx, stop_op, &sreq);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
    unsigned stopped_count = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "CAPTURE_STOPPED") == 0) stopped_count++;
    MTK_CHECK_EQ(stopped_count, 1);
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t sctx2 = mtk_test_ctx(&sink, 6);
    mtk_test_call(&sctx2, stop_op, &sreq); /* repeat STOP */
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
    unsigned stopped_count2 = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "CAPTURE_STOPPED") == 0) stopped_count2++;
    MTK_CHECK_EQ(stopped_count2, 0); /* repeat_stop_emits_terminal_event: false */

    /* Push mode: credit gating and fragmentation -- */
    /* No credit granted before the frame arrives -> dropped, credit_stalls++. */
    mtk_fake_wifi_reset();
    make_frame(g_fake_wifi.frames[0].data, 1000, 0x11); g_fake_wifi.frames[0].len = 1000; g_fake_wifi.frames[0].channel = 1; /* forces 2 fragments */
    g_fake_wifi.frame_count = 1;

    mtk_fake_sink_state_t psh; mtk_fake_sink_reset(&psh);
    mtk_request_ctx_t pctx = mtk_test_ctx(&psh, 7);
    mtk_capture_start_req_t preq2 = {0};
    preq2.mode = 0; /* PUSH */
    preq2.snap_len = 1000;
    preq2.channel_plan.mode = 0; preq2.channel_plan.channel = 1;
    mtk_test_call(&pctx, start_op, &preq2);
    MTK_CHECK_EQ(psh.stream_count, 0);
    const mtk_opcode_entry_t *stats_op2 = mtk_test_find_op("CAPTURE_STATS");
    {
        mtk_capture_start_resp_t s1 = {0};
        mtk_decode(start_op->resp_desc, &s1, psh.response.body, psh.response.body_len, NULL);
        mtk_capture_stats_req_t sr = {0}; sr.operation_token = s1.operation_token;
        mtk_fake_sink_state_t stsink0; mtk_fake_sink_reset(&stsink0);
        mtk_request_ctx_t stctx0 = mtk_test_ctx(&stsink0, 71);
        mtk_test_call(&stctx0, stats_op2, &sr);
        mtk_capture_stats_resp_t stats0 = {0};
        mtk_decode(stats_op2->resp_desc, &stats0, stsink0.response.body, stsink0.response.body_len, NULL);
        MTK_CHECK_EQ(stats0.credit_stalls, 1);
        /* The frame is no longer actually LOST when PUSH mode lacks credit -- it
         * still lands in the shared bounded ring (frame_cb,
         * mtek_capture_logic.c), just not delivered as a STREAM chunk this
         * instant. dropped_frames only increments on a genuine ring-overflow now
         * ('s very first frame, with an empty ring, can never overflow it) --
         * credit_stalls alone reflects "STREAM delivery was skipped for lack of
         * credit", a real, distinct condition from "this frame is gone". */
        MTK_CHECK_EQ(stats0.dropped_frames, 0);

        /* Release the M-class lease before starting a second session. */
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 72);
        mtk_capture_stop_req_t stopreq = {0}; stopreq.operation_token = s1.operation_token;
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK_EQ(stopsink.response.status, MTK_STATUS_OK);
    }

    /* Grant ample credit before frames arrive (deferred-delivery fake) ->
     * both fragments (25B+896B header/chunk, then 8B+104B) are streamed. */
    mtk_fake_wifi_reset();
    g_fake_wifi.defer_frames = 1;
    make_frame(g_fake_wifi.frames[0].data, 1000, 0x22); g_fake_wifi.frames[0].len = 1000; g_fake_wifi.frames[0].channel = 1;
    g_fake_wifi.frame_count = 1;
    mtk_fake_sink_state_t psh2; mtk_fake_sink_reset(&psh2);
    mtk_request_ctx_t pctx2b = mtk_test_ctx(&psh2, 8);
    mtk_test_call(&pctx2b, start_op, &preq2);
    mtk_capture_start_resp_t s2 = {0};
    mtk_decode(start_op->resp_desc, &s2, psh2.response.body, psh2.response.body_len, NULL);
    mtek_capture_grant_credit(s2.operation_token, 2000);
    mtk_fake_wifi_deliver_frames();
    MTK_CHECK_EQ(psh2.stream_count, 2); /* first fragment (25+896) + continuation (8+104) */
    MTK_CHECK_EQ(psh2.streams[0].len, 25 + 896);
    MTK_CHECK_EQ(psh2.streams[1].len, 8 + 104);
    MTK_CHECK_EQ(psh2.streams[0].session_token, s2.operation_token);
    MTK_CHECK_EQ(psh2.streams[1].sequence, psh2.streams[0].sequence + 1);
    /* Byte 24 of the 25-byte first-fragment header (the reserved/padding byte
     * between the fixed fields ending at offset 23 and the data payload starting
     * at offset 25) used to be emitted straight off an uninitialized stack array
     * -- a real information leak of whatever happened to be on the stack. The
     * header is now zero-initialized, so this reserved byte is always a defined
     * 0. */
    MTK_CHECK_EQ(psh2.streams[0].data[24], 0);

    const mtk_opcode_entry_t *stats_op = mtk_test_find_op("CAPTURE_STATS");
    mtk_capture_stats_req_t streq = {0}; streq.operation_token = started.operation_token;
    mtk_fake_sink_state_t stsink; mtk_fake_sink_reset(&stsink);
    mtk_request_ctx_t stctx = mtk_test_ctx(&stsink, 9);
    mtk_test_call(&stctx, stats_op, &streq);
    MTK_CHECK_EQ(stsink.response.status, MTK_STATUS_OK);

    /* Release the M-class lease before the next section. */
    {
        mtk_capture_start_resp_t s2r = {0};
        mtk_decode(start_op->resp_desc, &s2r, psh2.response.body, psh2.response.body_len, NULL);
        mtk_capture_stop_req_t stopreq = {0}; stopreq.operation_token = s2r.operation_token;
        mtk_fake_sink_state_t stopsink; mtk_fake_sink_reset(&stopsink);
        mtk_request_ctx_t stopctx = mtk_test_ctx(&stopsink, 10);
        mtk_test_call(&stopctx, stop_op, &stopreq);
        MTK_CHECK_EQ(stopsink.response.status, MTK_STATUS_OK);
    }

    /* BSSID filter: "capture filters... are stored but not applied". A set
     * filter_bssid must exclude frames whose addr1/2/3 don't match it, and never
     * count them as captured/dropped/ truncated (a real, distinct "filtered out"
     * outcome). -- */
    {
        mtk_fake_wifi_reset();
        static const uint8_t target_bssid[6] = {0x10,0x20,0x30,0x40,0x50,0x60};
        static const uint8_t other_bssid[6]  = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
        static const uint8_t station[6]      = {0x02,0x02,0x02,0x02,0x02,0x02};
        /* Frame 0: matches filter (addr2 = target BSSID, a real 24-byte
         * 802.11 header, no payload beyond it). */
        memset(g_fake_wifi.frames[0].data, 0, 24);
        memcpy(g_fake_wifi.frames[0].data + 4, station, 6);
        memcpy(g_fake_wifi.frames[0].data + 10, target_bssid, 6);
        memcpy(g_fake_wifi.frames[0].data + 16, target_bssid, 6);
        g_fake_wifi.frames[0].len = 24; g_fake_wifi.frames[0].channel = 6;
        /* Frame 1: does not match (a different BSSID throughout). */
        memset(g_fake_wifi.frames[1].data, 0, 24);
        memcpy(g_fake_wifi.frames[1].data + 4, station, 6);
        memcpy(g_fake_wifi.frames[1].data + 10, other_bssid, 6);
        memcpy(g_fake_wifi.frames[1].data + 16, other_bssid, 6);
        g_fake_wifi.frames[1].len = 24; g_fake_wifi.frames[1].channel = 6;
        g_fake_wifi.frame_count = 2;

        mtk_fake_sink_state_t fsink; mtk_fake_sink_reset(&fsink);
        mtk_request_ctx_t fctx = mtk_test_ctx(&fsink, 11);
        mtk_capture_start_req_t freq = {0};
        freq.mode = 1 /* POLL */; freq.snap_len = 100;
        freq.channel_plan.mode = 0; freq.channel_plan.channel = 6;
        memcpy(freq.filter.filter_bssid.b, target_bssid, 6);
        mtk_test_call(&fctx, start_op, &freq);
        MTK_CHECK_EQ(fsink.response.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t fstarted = {0};
        mtk_decode(start_op->resp_desc, &fstarted, fsink.response.body, fsink.response.body_len, NULL);

        /* Only the matching frame was captured. */
        const mtk_opcode_entry_t *fstats_op = mtk_test_find_op("CAPTURE_STATS");
        mtk_capture_stats_req_t fstreq = {0}; fstreq.operation_token = fstarted.operation_token;
        mtk_fake_sink_state_t fstsink; mtk_fake_sink_reset(&fstsink);
        mtk_request_ctx_t fstctx = mtk_test_ctx(&fstsink, 12);
        mtk_test_call(&fstctx, fstats_op, &fstreq);
        mtk_capture_stats_resp_t fstats = {0};
        mtk_decode(fstats_op->resp_desc, &fstats, fstsink.response.body, fstsink.response.body_len, NULL);
        MTK_CHECK_EQ(fstats.total_frames, 1); /* the non-matching frame was never counted at all */

        mtk_capture_poll_read_req_t fpreq = {0}; fpreq.operation_token = fstarted.operation_token;
        mtk_fake_sink_state_t fpsink; mtk_fake_sink_reset(&fpsink);
        mtk_request_ctx_t fpctx = mtk_test_ctx(&fpsink, 13);
        mtk_test_call(&fpctx, poll_op, &fpreq);
        MTK_CHECK_EQ(fpsink.response.body[0], 1); /* RECORD -- the matching frame */
        mtk_fake_sink_state_t fpsink2; mtk_fake_sink_reset(&fpsink2);
        mtk_request_ctx_t fpctx2 = mtk_test_ctx(&fpsink2, 14);
        mtk_test_call(&fpctx2, poll_op, &fpreq);
        MTK_CHECK_EQ(fpsink2.response.body[0], 0); /* EMPTY -- nothing else buffered */

        mtk_capture_stop_req_t fstopreq = {0}; fstopreq.operation_token = fstarted.operation_token;
        mtk_fake_sink_state_t fstopsink; mtk_fake_sink_reset(&fstopsink);
        mtk_request_ctx_t fstopctx = mtk_test_ctx(&fstopsink, 15);
        mtk_test_call(&fstopctx, stop_op, &fstopreq);
    }

    /* Channel hop: a hop-mode (channel_plan.mode==1) session must actually
     * switch channel via the HAL once hop_dwell_ms elapses, driven by
     * mtek_capture_channel_hop_tick. -- */
    {
        mtk_fake_wifi_reset();
        mtk_fake_sink_state_t hsink; mtk_fake_sink_reset(&hsink);
        mtk_request_ctx_t hctx = mtk_test_ctx(&hsink, 16);
        mtk_capture_start_req_t hreq = {0};
        hreq.mode = 1 /* POLL */; hreq.snap_len = 100;
        hreq.channel_plan.mode = 1 /* HOP */; hreq.channel_plan.hop_dwell_ms = 10;
        mtk_test_call(&hctx, start_op, &hreq);
        MTK_CHECK_EQ(hsink.response.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t hstarted = {0};
        mtk_decode(start_op->resp_desc, &hstarted, hsink.response.body, hsink.response.body_len, NULL);

        MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, 0); /* not yet -- dwell hasn't elapsed */
        mtek_capture_channel_hop_tick(mtk_test_now_ms() + 5); /* still within dwell */
        MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, 0);
        mtek_capture_channel_hop_tick(mtk_test_now_ms() + 15); /* dwell elapsed */
        MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.current_channel, 2); /* round-robin from the starting channel 1 */

        mtk_capture_stop_req_t hstopreq = {0}; hstopreq.operation_token = hstarted.operation_token;
        mtk_fake_sink_state_t hstopsink; mtk_fake_sink_reset(&hstopsink);
        mtk_request_ctx_t hstopctx = mtk_test_ctx(&hstopsink, 17);
        mtk_test_call(&hstopctx, stop_op, &hstopreq);

        /* No further hopping once stopped, even if ticked again. */
        unsigned before = g_fake_wifi.set_channel_call_count;
        mtek_capture_channel_hop_tick(mtk_test_now_ms() + 1000);
        MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, before);
    }

    /* a session with a real requested duration must automatically stop once that
     * much wall-clock time has genuinely elapsed, driven by the same periodic
     * tick as channel hopping above. ---------- */
    {
        mtk_fake_wifi_reset();
        mtk_fake_sink_state_t dsink; mtk_fake_sink_reset(&dsink);
        mtk_request_ctx_t dctx = mtk_test_ctx(&dsink, 18);
        mtk_capture_start_req_t dreq = {0};
        dreq.mode = 1 /* POLL */; dreq.snap_len = 100; dreq.duration_ms = 1000;
        dreq.channel_plan.mode = 0; dreq.channel_plan.channel = 1;
        mtk_test_call(&dctx, start_op, &dreq);
        MTK_CHECK_EQ(dsink.response.status, MTK_STATUS_ACCEPTED);
        mtk_capture_start_resp_t dstarted = {0};
        mtk_decode(start_op->resp_desc, &dstarted, dsink.response.body, dsink.response.body_len, NULL);

        uint64_t start = mtk_test_now_ms();
        mtek_capture_channel_hop_tick(start + 500); /* before duration elapses -- still running */
        MTK_CHECK(mtk_fake_find_event(&dsink, "CAPTURE_STOPPED") == NULL);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_M);

        mtek_capture_channel_hop_tick(start + 1001); /* duration elapsed -- auto-stop */
        const mtk_fake_event_t *dstopped = mtk_fake_find_event(&dsink, "CAPTURE_STOPPED");
        MTK_CHECK(dstopped != NULL);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

        /* Idempotent: a further tick well past duration is a safe no-op,
         * not a second CAPTURE_STOPPED. */
        unsigned stopped_events = 0;
        for (unsigned i = 0; i < dsink.event_count; i++) if (strcmp(dsink.events[i].name, "CAPTURE_STOPPED") == 0) stopped_events++;
        mtek_capture_channel_hop_tick(start + 5000);
        unsigned stopped_events2 = 0;
        for (unsigned i = 0; i < dsink.event_count; i++) if (strcmp(dsink.events[i].name, "CAPTURE_STOPPED") == 0) stopped_events2++;
        MTK_CHECK_EQ(stopped_events2, stopped_events);

        /* duration_ms=0 (the existing session-info default across this
         * whole test file's earlier sections) means "run until stopped"
         * -- never auto-stopped regardless of how much time passes. */
        mtk_fake_sink_state_t nsink; mtk_fake_sink_reset(&nsink);
        mtk_request_ctx_t nctx = mtk_test_ctx(&nsink, 19);
        mtk_capture_start_req_t nreq = {0};
        nreq.mode = 1; nreq.snap_len = 100; nreq.duration_ms = 0;
        nreq.channel_plan.mode = 0; nreq.channel_plan.channel = 1;
        mtk_test_call(&nctx, start_op, &nreq);
        MTK_CHECK_EQ(nsink.response.status, MTK_STATUS_ACCEPTED);
        mtek_capture_channel_hop_tick(mtk_test_now_ms() + 1000000);
        MTK_CHECK(mtk_fake_find_event(&nsink, "CAPTURE_STOPPED") == NULL);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_M);

        mtk_capture_start_resp_t nstarted = {0};
        mtk_decode(start_op->resp_desc, &nstarted, nsink.response.body, nsink.response.body_len, NULL);
        mtk_capture_stop_req_t nstopreq = {0}; nstopreq.operation_token = nstarted.operation_token;
        mtk_fake_sink_state_t nstopsink; mtk_fake_sink_reset(&nstopsink);
        mtk_request_ctx_t nstopctx = mtk_test_ctx(&nstopsink, 20);
        mtk_test_call(&nstopctx, stop_op, &nstopreq);
    }

    /* "Make monitor entry transactional: on any step failure, restore the prior
     * mode/session and return an error." A hal->promisc_start failure must not
     * leave a capture session marked RUNNING forever with no real frame delivery
     * ever possible -- torn down immediately (arbiter released, a real
     * CAPTURE_STOPPED terminal event with an honest failure status), exactly as
     * if it had failed moments after a successful start. -- */
    {
        mtk_fake_wifi_reset();
        g_fake_wifi.promisc_start_rc = -1;
        mtk_fake_sink_state_t fsink; mtk_fake_sink_reset(&fsink);
        mtk_request_ctx_t fctx = mtk_test_ctx(&fsink, 21);
        mtk_capture_start_req_t freq = {0};
        freq.mode = 1; freq.snap_len = 100; freq.duration_ms = 0;
        freq.channel_plan.mode = 0; freq.channel_plan.channel = 1;
        mtk_test_call(&fctx, start_op, &freq);
        /* The accept response is already sent by the time the HAL call
         * even runs (an ACCEPTED_ASYNC operation's own established
         * shape) -- it cannot retroactively become an error. */
        MTK_CHECK_EQ(fsink.response.status, MTK_STATUS_ACCEPTED);
        const mtk_fake_event_t *stopped = mtk_fake_find_event(&fsink, "CAPTURE_STOPPED");
        MTK_CHECK(stopped != NULL);
        if (stopped != NULL) {
            mtk_capture_stopped_ev_t sev = {0};
            mtk_decode(&mtk_capture_stopped_ev_t_desc, &sev, stopped->body, stopped->body_len, NULL);
            MTK_CHECK_EQ(sev.status, MTK_STATUS_IO_ERROR);
            MTK_CHECK_EQ(sev.reason, 2); /* START_FAILED -- distinct from USER_REQUEST=0/DURATION_ELAPSED=1 */
        }
        /* The arbiter lease took is genuinely released, not leaked -- a
         * subsequent capture/deauth/handshake session (any MTK_ARB_M-class op)
         * must be able to acquire it. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        g_fake_wifi.promisc_start_rc = 0;
    }

MTK_TEST_MAIN_END
