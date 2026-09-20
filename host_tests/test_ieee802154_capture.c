/* Core-managed IEEE 802.15.4 capture and the versioned host contract.
 *
 * Capture reuses the raw service's bounded ring and single teardown path, so
 * these tests focus on what capture adds: channel-plan validation,
 * deterministic bounded hopping, and the fact that a capture session owns the
 * radio exactly like every other 802.15.4 session.
 *
 * Also guards the two defect classes found during development:
 *   - capability reporting claiming an opcode is unavailable while dispatch
 *     still serves it (or the reverse), across every registered opcode;
 *   - the API identity drifting from what the image actually serves.
 */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include "mtek_schema_constants.h"
#include "mtek_arbiter.h"
#include <string.h>

static uint32_t cap_start(mtk_fake_sink_state_t *sink, uint8_t mode, uint8_t channel,
                           uint32_t mask, uint16_t dwell, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("IEEE154_CAPTURE_START");
    mtk_ieee154_capture_start_req_t req; memset(&req, 0, sizeof(req));
    req.mode = mode; req.channel = channel; req.channel_mask = mask; req.hop_dwell_ms = dwell;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
    if (sink->response.status != MTK_STATUS_ACCEPTED) return 0;
    mtk_ieee154_capture_start_resp_t r; memset(&r, 0, sizeof(r));
    mtk_decode(&mtk_ieee154_capture_start_resp_t_desc, &r, sink->response.body, sink->response.body_len, NULL);
    return r.operation_token;
}

static void cap_stop(mtk_fake_sink_state_t *sink, uint32_t token, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 2);
    mtk_ieee154_capture_stop_req_t req; memset(&req, 0, sizeof(req));
    req.operation_token = token;
    mtk_test_call(&ctx, mtk_test_find_op("IEEE154_CAPTURE_STOP"), &req);
    if (status_out) *status_out = sink->response.status;
}

static void cap_status(mtk_ieee154_capture_status_resp_t *out) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 3);
    mtk_test_call(&ctx, mtk_test_find_op("IEEE154_CAPTURE_STATUS"), NULL);
    memset(out, 0, sizeof(*out));
    mtk_decode(&mtk_ieee154_capture_status_resp_t_desc, out, sink.response.body, sink.response.body_len, NULL);
}

MTK_TEST_MAIN_BEGIN

    /* ---- Fixed-channel capture across the whole 11..26 page. */
    {
        for (unsigned ch = 11; ch <= 26; ch++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            uint32_t t = cap_start(&sink, 0, (uint8_t)ch, 0, 0, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
            MTK_CHECK_EQ(g_fake_154.last_channel, ch);
            /* A capture always sees every frame on the channel. */
            MTK_CHECK_EQ(g_fake_154.last_promiscuous, 1);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_IEEE154);
            cap_stop(&sink, t, NULL);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- Channel-plan validation, before any resource is taken. */
    {
        struct { uint8_t mode; uint8_t ch; uint32_t mask; uint16_t dwell; } bad[] = {
            { 0, 10, 0, 0 },                      /* fixed: below the page */
            { 0, 27, 0, 0 },                      /* fixed: above the page */
            { 1, 0, 0u, 50 },                     /* hop: empty mask */
            { 1, 0, (1u << 10), 50 },             /* hop: channel below the page */
            { 1, 0, (1u << 27), 50 },             /* hop: channel above the page */
            { 1, 0, 0xFFFFFFFFu, 50 },            /* hop: mask includes illegal channels */
            { 1, 0, (1u << 15), 0 },              /* hop: zero dwell is not deterministic */
            { 2, 15, (1u << 15), 50 },            /* unknown mode */
        };
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            MTK_CHECK_EQ(cap_start(&sink, bad[i].mode, bad[i].ch, bad[i].mask, bad[i].dwell, &st), 0);
            MTK_CHECK_EQ(st, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(g_fake_154.start_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- Hopping is deterministic, ascending, wrapping and dwell-bounded. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t mask = (1u << 11) | (1u << 15) | (1u << 26);
        uint32_t token = cap_start(&sink, 1, 0, mask, 10, NULL);
        MTK_CHECK(token != 0);
        /* Starts on the lowest selected channel. */
        MTK_CHECK_EQ(g_fake_154.last_channel, 11);

        /* Before the dwell elapses nothing moves. */
        mtek_ieee802154_service_tick();
        MTK_CHECK_EQ(g_fake_154.last_channel, 11);

        /* Each elapsed dwell advances exactly one selected channel, in order,
         * and wraps back to the lowest. */
        const uint8_t expect[] = { 15, 26, 11, 15, 26, 11 };
        for (unsigned i = 0; i < sizeof(expect); i++) {
            mtk_test_advance_ms(10);
            mtek_ieee802154_service_tick();
            MTK_CHECK_EQ(g_fake_154.last_channel, expect[i]);
        }
        mtk_ieee154_capture_status_resp_t st;
        cap_status(&st);
        MTK_CHECK_EQ(st.mode, 1);
        MTK_CHECK_EQ(st.hop_count, 6);
        MTK_CHECK_EQ(st.channel, 11);
        cap_stop(&sink, token, NULL);
    }

    /* ---- A single-channel mask must not churn the radio. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 1, 0, (1u << 20), 5, NULL);
        MTK_CHECK(token != 0);
        unsigned retunes_before = g_fake_154.set_channel_count;
        for (unsigned i = 0; i < 5; i++) { mtk_test_advance_ms(5); mtek_ieee802154_service_tick(); }
        MTK_CHECK_EQ(g_fake_154.set_channel_count, retunes_before);
        MTK_CHECK_EQ(g_fake_154.last_channel, 20);
        cap_stop(&sink, token, NULL);
    }

    /* ---- Hopping stops dead once the session ends: no tick may retune a
     *      radio this session no longer owns. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 1, 0, (1u << 11) | (1u << 20), 10, NULL);
        cap_stop(&sink, token, NULL);
        unsigned retunes_after_stop = g_fake_154.set_channel_count;
        for (unsigned i = 0; i < 10; i++) { mtk_test_advance_ms(10); mtek_ieee802154_service_tick(); }
        MTK_CHECK_EQ(g_fake_154.set_channel_count, retunes_after_stop);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

    /* ---- Captured frames keep full MPDUs and PCAP metadata, drained through
     *      the same POLL_RECV the raw session uses. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 0, 15, 0, 0, NULL);
        uint8_t mpdu[MTK_154_MAX_PHY_LEN];
        for (unsigned i = 0; i < sizeof(mpdu); i++) mpdu[i] = (uint8_t)(i ^ 0x5A);
        mtk_fake_154_deliver(0xAABBCCDDEEFF0011ull, 15, -70, 180, mpdu, MTK_154_MAX_PHY_LEN);
        mtek_ieee802154_service_tick();

        mtk_fake_sink_state_t ps; mtk_fake_sink_reset(&ps);
        mtk_request_ctx_t pc = mtk_test_ctx(&ps, 4);
        mtk_test_call(&pc, mtk_test_find_op("IEEE154_POLL_RECV"), NULL);
        MTK_CHECK_EQ(ps.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(ps.response.body[0], 1); /* RECORD */
        /* Maximum-size MPDU survives intact and is not flagged truncated. */
        MTK_CHECK_EQ(ps.response.body[12], 0);
        MTK_CHECK_EQ(ps.response.body[15], MTK_154_MAX_PHY_LEN);
        MTK_CHECK(memcmp(ps.response.body + 16, mpdu, MTK_154_MAX_PHY_LEN) == 0);
        cap_stop(&sink, token, NULL);
    }

    /* ---- Ring overflow during capture drops oldest and is counted. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 0, 15, 0, 0, NULL);
        uint8_t f[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
        for (unsigned round = 0; round < 3; round++) {
            for (unsigned i = 0; i < 32; i++) mtk_fake_154_deliver(i, 15, -50, 100, f, sizeof(f));
            mtek_ieee802154_service_tick();
        }
        mtk_ieee154_capture_status_resp_t st;
        cap_status(&st);
        MTK_CHECK_EQ(st.frames_received, 96);
        MTK_CHECK_EQ(st.frames_dropped, 80);   /* 96 delivered into a 16-slot ring */
        cap_stop(&sink, token, NULL);
    }

    /* ---- Capture contends for the one radio exactly like any other session,
     *      and restart/peer-reset behave. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 0, 15, 0, 0, NULL);
        MTK_CHECK(token != 0);

        /* A raw session cannot start while capture owns the radio. */
        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 5);
        mtk_ieee154_start_req_t raw; memset(&raw, 0, sizeof(raw));
        raw.channel = 20;
        mtk_test_call(&c2, mtk_test_find_op("IEEE154_START"), &raw);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_BUSY);

        /* Nor can Wi-Fi. */
        mtk_fake_sink_state_t s3; mtk_fake_sink_reset(&s3);
        mtk_request_ctx_t c3 = mtk_test_ctx(&s3, 6);
        mtk_softap_start_req_t ap; memset(&ap, 0, sizeof(ap));
        ap.config.ssid.len = 2; memcpy(ap.config.ssid.data, "ap", 2);
        ap.config.channel = 6;
        mtk_test_call(&c3, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(s3.response.status, MTK_STATUS_BUSY);

        /* Peer reset reclaims the radio and emits the capture's own terminal event. */
        mtk_op_id_t cancelled = mtek_ieee802154_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, token);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&sink, "IEEE154_CAPTURE_STOPPED") != NULL);

        /* And the radio is genuinely reusable afterwards. */
        for (unsigned i = 0; i < 3; i++) {
            mtk_fake_sink_state_t s4; mtk_fake_sink_reset(&s4);
            uint8_t st = 0;
            uint32_t t = cap_start(&s4, 0, 15, 0, 0, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
            cap_stop(&s4, t, NULL);
        }
    }

    /* ---- A HAL start failure tears capture down and frees the lease. */
    {
        mtk_test_bootstrap();
        g_fake_154.start_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = cap_start(&sink, 0, 15, 0, 0, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);   /* already on the wire */
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "IEEE154_CAPTURE_STOPPED");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_ieee154_capture_stopped_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_ieee154_capture_stopped_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_IO_ERROR);
        }
    }

    /* ---- STOP is family-gated and idempotent. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = cap_start(&sink, 0, 15, 0, 0, NULL);
        uint8_t st = 0;
        cap_stop(&sink, token ^ 0x5A5A5A5Au, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);
        cap_stop(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
        unsigned stops = g_fake_154.stop_count;
        cap_stop(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_154.stop_count, stops);   /* not torn down twice */
    }

    /* ---- Versioned host contract. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 7);
        mtk_test_call(&ctx, mtk_test_find_op("GET_API_IDENTITY"), NULL);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        mtk_get_api_identity_resp_t r; memset(&r, 0, sizeof(r));
        mtk_decode(&mtk_get_api_identity_resp_t_desc, &r, sink.response.body, sink.response.body_len, NULL);
        /* The identity is the frozen contract version, not a build timestamp. */
        MTK_CHECK_EQ(r.api_major, MTK_CORE_API_MAJOR);
        MTK_CHECK_EQ(r.api_minor, MTK_CORE_API_MINOR);
        MTK_CHECK(r.api_major >= 1);
        MTK_CHECK(r.variant_name.len > 0);
        /* capability_count must describe what this image actually serves, so
         * it has to agree with a direct count over the live table. */
        unsigned expect = 0;
        for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {
            mtk_fake_sink_state_t cs; mtk_fake_sink_reset(&cs);
            mtk_request_ctx_t cc = mtk_test_ctx(&cs, 8);
            const mtk_opcode_entry_t *e = &mtk_opcode_table[i];
            uint8_t buf[512]; size_t blen = 0;
            static uint8_t zero[4096];
            if (e->req_desc) mtk_encode(e->req_desc, zero, buf, sizeof(buf), &blen);
            mtk_router_dispatch(&cc, e->service_id, e->opcode, buf, blen);
            if (cs.response.status != MTK_STATUS_UNSUPPORTED) expect++;
        }
        MTK_CHECK_EQ(r.capability_count, expect);
    }

MTK_TEST_MAIN_END
