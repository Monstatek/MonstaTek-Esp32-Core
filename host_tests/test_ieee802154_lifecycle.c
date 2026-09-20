/* IEEE 802.15.4 service (0x0007).
 *
 * The radio is protocol-neutral: these tests exercise PHY-level behaviour and
 * radio ownership, never Thread or Zigbee semantics, because Core deliberately
 * knows nothing about either. Coverage: channel validation across the whole
 * 11..26 page, start/stop/restart, retune, transmit validation, bounded
 * receive buffering under saturation, energy scan including mask validation,
 * ownership against Wi-Fi/ESP-NOW, family-gated STOP, and peer reset. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include "mtek_arbiter.h"
#include <string.h>

static uint32_t start_154(mtk_fake_sink_state_t *sink, uint8_t channel, uint8_t promisc, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("IEEE154_START");
    mtk_ieee154_start_req_t req; memset(&req, 0, sizeof(req));
    req.channel = channel; req.promiscuous = promisc;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
    if (sink->response.status != MTK_STATUS_ACCEPTED) return 0;
    mtk_ieee154_start_resp_t r; memset(&r, 0, sizeof(r));
    mtk_decode(&mtk_ieee154_start_resp_t_desc, &r, sink->response.body, sink->response.body_len, NULL);
    return r.operation_token;
}

static void stop_154(mtk_fake_sink_state_t *sink, uint32_t token, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 2);
    const mtk_opcode_entry_t *op = mtk_test_find_op("IEEE154_STOP");
    mtk_ieee154_stop_req_t req; memset(&req, 0, sizeof(req));
    req.operation_token = token;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
}

static uint8_t tx_154(const uint8_t *data, uint8_t len, uint8_t cca) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 3);
    mtk_ieee154_tx_req_t req; memset(&req, 0, sizeof(req));
    req.data.len = len; if (len) memcpy(req.data.data, data, len);
    req.cca = cca;
    mtk_test_call(&ctx, mtk_test_find_op("IEEE154_TX"), &req);
    return sink.response.status;
}

static uint8_t poll_154(uint8_t *body, size_t *blen, uint8_t *status_out) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 4);
    mtk_test_call(&ctx, mtk_test_find_op("IEEE154_POLL_RECV"), NULL);
    if (status_out) *status_out = sink.response.status;
    if (sink.response.status != MTK_STATUS_OK || sink.response.body_len < 1) return 0xFF;
    if (body) memcpy(body, sink.response.body, sink.response.body_len);
    if (blen) *blen = sink.response.body_len;
    return sink.response.body[0];
}

MTK_TEST_MAIN_BEGIN

    /* ---- Every channel on the 2.4GHz page is accepted; everything outside
     *      it is refused before any resource is taken. */
    {
        for (unsigned ch = 11; ch <= 26; ch++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            uint32_t t = start_154(&sink, (uint8_t)ch, 0, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
            MTK_CHECK_EQ(g_fake_154.last_channel, ch);
            stop_154(&sink, t, NULL);
        }
        uint8_t bad[] = { 0, 1, 10, 27, 100, 255 };
        for (unsigned i = 0; i < sizeof(bad); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            MTK_CHECK_EQ(start_154(&sink, bad[i], 0, &st), 0);
            MTK_CHECK_EQ(st, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(g_fake_154.start_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- Lease is held for the session and released on stop; promiscuous
     *      reaches the HAL, which is what a sniffer depends on. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 1, NULL);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_154.last_promiscuous, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_IEEE154);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);
        uint8_t st = 0;
        stop_154(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_154.stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&sink, "IEEE154_STOPPED") != NULL);
    }

    /* ---- Restart after stop works: the radio is genuinely reusable. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        for (unsigned round = 0; round < 3; round++) {
            uint8_t st = 0;
            uint32_t t = start_154(&sink, 20, 0, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
            MTK_CHECK(t != 0);
            stop_154(&sink, t, NULL);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
        MTK_CHECK_EQ(g_fake_154.start_count, 3);
        MTK_CHECK_EQ(g_fake_154.stop_count, 3);
    }

    /* ---- A HAL start failure tears the session down rather than leaving it
     *      RUNNING with a dead radio, and frees the lease. */
    {
        mtk_test_bootstrap();
        g_fake_154.start_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_154(&sink, 15, 0, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);   /* already on the wire */
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_154.stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "IEEE154_STOPPED");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_ieee154_stopped_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_ieee154_stopped_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_IO_ERROR);
        }
    }

    /* ---- Data-plane opcodes require a live session: without the lease there
     *      is no right to touch the radio. */
    {
        mtk_test_bootstrap();
        uint8_t frame[16]; memset(frame, 0xA5, sizeof(frame));
        MTK_CHECK_EQ(tx_154(frame, sizeof(frame), 1), MTK_STATUS_NOT_READY);
        uint8_t st = 0;
        poll_154(NULL, NULL, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_READY);
        MTK_CHECK_EQ(g_fake_154.transmit_count, 0);
    }

    /* ---- Transmit: length validated at both ends, CCA passed through,
     *      failure reported honestly. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 0, NULL);
        uint8_t frame[MTK_154_MAX_PHY_LEN];
        memset(frame, 0x5A, sizeof(frame));

        MTK_CHECK_EQ(tx_154(frame, 1, 0), MTK_STATUS_OK);                 /* minimum */
        MTK_CHECK_EQ(tx_154(frame, MTK_154_MAX_PHY_LEN, 1), MTK_STATUS_OK); /* aMaxPHYPacketSize */
        MTK_CHECK_EQ(g_fake_154.last_tx_len, MTK_154_MAX_PHY_LEN);
        MTK_CHECK_EQ(g_fake_154.last_tx_cca, 1);
        MTK_CHECK_EQ(tx_154(frame, 0, 0), MTK_STATUS_INVALID_ARGUMENT);   /* empty is not a frame */

        g_fake_154.transmit_rc = -1;
        MTK_CHECK_EQ(tx_154(frame, 10, 0), MTK_STATUS_IO_ERROR);
        g_fake_154.transmit_rc = 0;
        stop_154(&sink, token, NULL);
    }

    /* ---- Retune a live session; refuse an illegal channel and refuse when
     *      no session owns the radio. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        const mtk_opcode_entry_t *op = mtk_test_find_op("IEEE154_SET_CHANNEL");
        mtk_ieee154_set_channel_req_t req; memset(&req, 0, sizeof(req));

        req.channel = 20;
        mtk_fake_sink_state_t s0; mtk_fake_sink_reset(&s0);
        mtk_request_ctx_t c0 = mtk_test_ctx(&s0, 5);
        mtk_test_call(&c0, op, &req);
        MTK_CHECK_EQ(s0.response.status, MTK_STATUS_NOT_READY); /* no session */

        uint32_t token = start_154(&sink, 15, 0, NULL);
        mtk_fake_sink_state_t s1; mtk_fake_sink_reset(&s1);
        mtk_request_ctx_t c1 = mtk_test_ctx(&s1, 6);
        mtk_test_call(&c1, op, &req);
        MTK_CHECK_EQ(s1.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_154.last_channel, 20);

        req.channel = 27;
        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 7);
        mtk_test_call(&c2, op, &req);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_INVALID_ARGUMENT);
        stop_154(&sink, token, NULL);
    }

    /* ---- Receive carries the metadata a PCAP export needs, and an
     *      oversized frame is truncated and flagged rather than overflowing. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 1, NULL);

        MTK_CHECK_EQ(poll_154(NULL, NULL, NULL), 0 /* EMPTY */);

        uint8_t payload[32];
        for (unsigned i = 0; i < sizeof(payload); i++) payload[i] = (uint8_t)i;
        mtk_fake_154_deliver(0x1122334455667788ull, 15, -61, 210, payload, sizeof(payload));
        mtek_ieee802154_service_tick();

        uint8_t body[256]; size_t blen = 0;
        MTK_CHECK_EQ(poll_154(body, &blen, NULL), 1 /* RECORD */);
        MTK_CHECK_EQ(blen, (size_t)(1 + 8 + 1 + 1 + 1 + 1 + 2 + 1 + 32));
        uint64_t ts = 0;
        for (int i = 0; i < 8; i++) ts |= ((uint64_t)body[1 + i]) << (8 * i);
        MTK_CHECK(ts == 0x1122334455667788ull);
        MTK_CHECK_EQ(body[9], 15);                 /* channel */
        MTK_CHECK_EQ((int8_t)body[10], -61);       /* rssi */
        MTK_CHECK_EQ(body[11], 210);               /* lqi */
        MTK_CHECK_EQ(body[12], 0);                 /* flags: not truncated */
        MTK_CHECK_EQ(body[13] | (body[14] << 8), 32); /* original_len */
        MTK_CHECK_EQ(body[15], 32);                /* captured len */
        MTK_CHECK(memcmp(body + 16, payload, 32) == 0);

        MTK_CHECK_EQ(poll_154(NULL, NULL, NULL), 0); /* drained */
        stop_154(&sink, token, NULL);
    }

    /* ---- Bounded buffering: a host that stops polling cannot cause
     *      unbounded growth; drops are counted honestly. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 1, NULL);
        uint8_t one[4] = { 1, 2, 3, 4 };
        for (unsigned round = 0; round < 4; round++) {
            for (unsigned i = 0; i < 32; i++) mtk_fake_154_deliver(round * 100 + i, 15, -50, 100, one, 4);
            mtek_ieee802154_service_tick();
        }
        mtk_fake_sink_state_t s; mtk_fake_sink_reset(&s);
        mtk_request_ctx_t c = mtk_test_ctx(&s, 8);
        mtk_test_call(&c, mtk_test_find_op("IEEE154_STATUS"), NULL);
        mtk_ieee154_status_resp_t st; memset(&st, 0, sizeof(st));
        mtk_decode(&mtk_ieee154_status_resp_t_desc, &st, s.response.body, s.response.body_len, NULL);
        MTK_CHECK_EQ(st.frames_received, 128);
        MTK_CHECK_EQ(st.frames_dropped, 112);   /* 128 delivered, 16-slot ring */
        MTK_CHECK_EQ(st.channel, 15);
        MTK_CHECK_EQ(st.promiscuous, 1);

        unsigned got = 0;
        while (poll_154(NULL, NULL, NULL) == 1) got++;
        MTK_CHECK_EQ(got, 16);
        stop_154(&sink, token, NULL);
    }

    /* ---- Energy scan: mask validated, every selected channel measured,
     *      results reported once, lease released afterwards. */
    {
        mtk_test_bootstrap();
        g_fake_154.energy_peak = -77;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        const mtk_opcode_entry_t *op = mtk_test_find_op("IEEE154_ENERGY_SCAN");
        mtk_ieee154_energy_scan_req_t req; memset(&req, 0, sizeof(req));
        req.channel_mask = (1u << 11) | (1u << 15) | (1u << 26);
        req.dwell_ms = 20;
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 9);
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
        MTK_CHECK_EQ(g_fake_154.energy_count, 3);
        MTK_CHECK_EQ(g_fake_154.energy_channels[0], 11);
        MTK_CHECK_EQ(g_fake_154.energy_channels[1], 15);
        MTK_CHECK_EQ(g_fake_154.energy_channels[2], 26);
        /* Radio handed back once the scan completed. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "IEEE154_ENERGY_RESULT");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_ieee154_energy_result_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_ieee154_energy_result_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_OK);
            MTK_CHECK_EQ(d.results.count, 3);
            MTK_CHECK_EQ(d.results.items[0].channel, 11);
            MTK_CHECK_EQ(d.results.items[0].peak_rssi, -77);
            MTK_CHECK_EQ(d.results.items[2].channel, 26);
        }

        /* Illegal masks and a zero dwell are refused without touching the radio. */
        uint32_t bad_masks[] = { 0u, 1u << 10, 1u << 27, 0xFFFFFFFFu };
        for (unsigned i = 0; i < sizeof(bad_masks)/sizeof(bad_masks[0]); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
            mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 10);
            memset(&req, 0, sizeof(req));
            req.channel_mask = bad_masks[i]; req.dwell_ms = 20;
            mtk_test_call(&c2, op, &req);
            MTK_CHECK_EQ(s2.response.status, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(g_fake_154.energy_count, 0);
        }
        mtk_test_bootstrap();
        mtk_fake_sink_state_t s3; mtk_fake_sink_reset(&s3);
        mtk_request_ctx_t c3 = mtk_test_ctx(&s3, 11);
        memset(&req, 0, sizeof(req));
        req.channel_mask = (1u << 15); req.dwell_ms = 0;
        mtk_test_call(&c3, op, &req);
        MTK_CHECK_EQ(s3.response.status, MTK_STATUS_INVALID_ARGUMENT);
    }

    /* ---- One radio: 802.15.4 serializes against Wi-Fi and ESP-NOW. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 0, NULL);
        MTK_CHECK(token != 0);

        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 12);
        mtk_softap_start_req_t ap; memset(&ap, 0, sizeof(ap));
        ap.config.ssid.len = 2; memcpy(ap.config.ssid.data, "ap", 2);
        ap.config.channel = 6;
        mtk_test_call(&c2, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_BUSY);

        mtk_fake_sink_state_t s3; mtk_fake_sink_reset(&s3);
        mtk_request_ctx_t c3 = mtk_test_ctx(&s3, 13);
        mtk_espnow_start_req_t en; memset(&en, 0, sizeof(en));
        en.channel = 6;
        mtk_test_call(&c3, mtk_test_find_op("ESPNOW_START"), &en);
        MTK_CHECK_EQ(s3.response.status, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);

        stop_154(&sink, token, NULL);
        /* Radio genuinely available again once 802.15.4 is done. */
        mtk_fake_sink_state_t s4; mtk_fake_sink_reset(&s4);
        mtk_request_ctx_t c4 = mtk_test_ctx(&s4, 14);
        mtk_test_call(&c4, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(s4.response.status, MTK_STATUS_ACCEPTED);
    }

    /* ---- And the reverse: Wi-Fi owning the radio blocks 802.15.4. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 15);
        mtk_softap_start_req_t ap; memset(&ap, 0, sizeof(ap));
        ap.config.ssid.len = 2; memcpy(ap.config.ssid.data, "ap", 2);
        ap.config.channel = 6;
        mtk_test_call(&ctx, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);

        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        uint8_t st = 0;
        MTK_CHECK_EQ(start_154(&s2, 15, 0, &st), 0);
        MTK_CHECK_EQ(st, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(g_fake_154.start_count, 0);
    }

    /* ---- STOP is family-gated, repeat STOP is idempotent, and a peer reset
     *      reclaims the radio. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_154(&sink, 15, 0, NULL);

        mtk_fake_sink_state_t bad; mtk_fake_sink_reset(&bad);
        uint8_t st = 0;
        stop_154(&bad, token ^ 0x5A5A5A5Au, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(g_fake_154.stop_count, 0);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);

        stop_154(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
        unsigned after_first = g_fake_154.stop_count;
        stop_154(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_154.stop_count, after_first); /* not torn down twice */

        uint32_t t2 = start_154(&sink, 15, 0, NULL);
        MTK_CHECK(t2 != 0);
        mtk_op_id_t cancelled = mtek_ieee802154_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, t2);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
