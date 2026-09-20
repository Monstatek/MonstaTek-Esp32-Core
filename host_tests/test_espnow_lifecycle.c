/* ESP-NOW service (0x0006). ESP-NOW transmits on the 2.4GHz Wi-Fi radio, so
 * the properties that matter are the same ones every radio-owning operation in
 * this tree must hold -- one token, one lease, one teardown path -- plus two
 * specific to it: it must serialize against Wi-Fi operations rather than
 * running alongside them, and the receive ring must stay bounded when nobody
 * drains it. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include "mtek_arbiter.h"
#include <string.h>

static uint32_t start_espnow(mtk_fake_sink_state_t *sink, uint8_t channel, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("ESPNOW_START");
    mtk_espnow_start_req_t req; memset(&req, 0, sizeof(req));
    req.channel = channel;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
    if (sink->response.status != MTK_STATUS_ACCEPTED) return 0;
    mtk_espnow_start_resp_t r; memset(&r, 0, sizeof(r));
    mtk_decode(&mtk_espnow_start_resp_t_desc, &r, sink->response.body, sink->response.body_len, NULL);
    return r.operation_token;
}

static void stop_espnow(mtk_fake_sink_state_t *sink, uint32_t token, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 2);
    const mtk_opcode_entry_t *op = mtk_test_find_op("ESPNOW_STOP");
    mtk_espnow_stop_req_t req; memset(&req, 0, sizeof(req));
    req.operation_token = token;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
}

static uint8_t send_one(const uint8_t mac[6], const char *payload) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 3);
    const mtk_opcode_entry_t *op = mtk_test_find_op("ESPNOW_SEND");
    mtk_espnow_send_req_t req; memset(&req, 0, sizeof(req));
    memcpy(req.peer.b, mac, 6);
    req.data.len = (uint16_t)strlen(payload);
    memcpy(req.data.data, payload, strlen(payload));
    mtk_test_call(&ctx, op, &req);
    return sink.response.status;
}

/* Returns the tag byte (0 EMPTY / 1 RECORD), or 0xFF if the call was refused. */
static uint8_t poll_recv(uint8_t *body_out, size_t *len_out, uint8_t *status_out) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 4);
    const mtk_opcode_entry_t *op = mtk_test_find_op("ESPNOW_POLL_RECV");
    mtk_test_call(&ctx, op, NULL);
    if (status_out) *status_out = sink.response.status;
    if (sink.response.status != MTK_STATUS_OK || sink.response.body_len < 1) return 0xFF;
    if (body_out) memcpy(body_out, sink.response.body, sink.response.body_len);
    if (len_out) *len_out = sink.response.body_len;
    return sink.response.body[0];
}

MTK_TEST_MAIN_BEGIN

    static const uint8_t peer_mac[6] = { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 };

    /* ---- Happy path: start takes the lease, stop releases it and reports. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_espnow(&sink, 6, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_espnow.start_count, 1);
        MTK_CHECK_EQ(g_fake_espnow.last_channel, 6);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_ESPNOW);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);

        uint8_t stop_st = 0;
        stop_espnow(&sink, token, &stop_st);
        MTK_CHECK_EQ(stop_st, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_espnow.stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&sink, "ESPNOW_STOPPED") != NULL);
    }

    /* ---- Channel is validated before any resource is taken. */
    {
        uint8_t bad[] = { 0, 14 };
        for (unsigned i = 0; i < sizeof(bad); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            MTK_CHECK_EQ(start_espnow(&sink, bad[i], &st), 0);
            MTK_CHECK_EQ(st, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(g_fake_espnow.start_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- A HAL start failure tears the session down and frees the lease. */
    {
        mtk_test_bootstrap();
        g_fake_espnow.start_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_espnow(&sink, 6, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED); /* already on the wire */
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_espnow.stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "ESPNOW_STOPPED");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_espnow_stopped_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_espnow_stopped_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_IO_ERROR);
        }
    }

    /* ---- Data-plane opcodes require a live session: without the lease there
     *      is no right to touch the radio at all. */
    {
        mtk_test_bootstrap();
        MTK_CHECK_EQ(send_one(peer_mac, "hello"), MTK_STATUS_NOT_READY);
        uint8_t st = 0;
        poll_recv(NULL, NULL, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_READY);
        MTK_CHECK_EQ(g_fake_espnow.send_count, 0);
    }

    /* ---- Peer management: an encrypted peer needs a full 16-byte key, and a
     *      full peer table surfaces as OVERFLOW rather than a silent failure. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);
        MTK_CHECK(token != 0);
        const mtk_opcode_entry_t *op = mtk_test_find_op("ESPNOW_ADD_PEER");

        /* Unencrypted peer, no key: accepted. */
        mtk_fake_sink_state_t s1; mtk_fake_sink_reset(&s1);
        mtk_request_ctx_t c1 = mtk_test_ctx(&s1, 5);
        mtk_espnow_add_peer_req_t p; memset(&p, 0, sizeof(p));
        memcpy(p.peer.b, peer_mac, 6); p.channel = 6; p.encrypt = 0;
        mtk_test_call(&c1, op, &p);
        MTK_CHECK_EQ(s1.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_espnow.last_peer_encrypt, 0);

        /* Encrypted peer with a short key: refused, never downgraded to
         * plaintext. */
        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 6);
        memset(&p, 0, sizeof(p));
        memcpy(p.peer.b, peer_mac, 6); p.channel = 6; p.encrypt = 1; p.lmk.len = 8;
        mtk_test_call(&c2, op, &p);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_INVALID_ARGUMENT);

        /* Encrypted peer with a full key: accepted and passed through. */
        mtk_fake_sink_state_t s3; mtk_fake_sink_reset(&s3);
        mtk_request_ctx_t c3 = mtk_test_ctx(&s3, 7);
        memset(&p, 0, sizeof(p));
        memcpy(p.peer.b, peer_mac, 6); p.channel = 6; p.encrypt = 1; p.lmk.len = 16;
        memset(p.lmk.data, 0xA5, 16);
        mtk_test_call(&c3, op, &p);
        MTK_CHECK_EQ(s3.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_espnow.last_peer_encrypt, 1);
        MTK_CHECK_EQ(g_fake_espnow.last_peer_lmk_len, 16);

        /* A full peer table is reported as OVERFLOW. */
        g_fake_espnow.add_peer_rc = 1;
        mtk_fake_sink_state_t s4; mtk_fake_sink_reset(&s4);
        mtk_request_ctx_t c4 = mtk_test_ctx(&s4, 8);
        memset(&p, 0, sizeof(p));
        memcpy(p.peer.b, peer_mac, 6); p.channel = 6;
        mtk_test_call(&c4, op, &p);
        MTK_CHECK_EQ(s4.response.status, MTK_STATUS_OVERFLOW);

        g_fake_espnow.add_peer_rc = 0;
        stop_espnow(&sink, token, NULL);
    }

    /* ---- Send validates length and reports transmit failure honestly. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);
        MTK_CHECK_EQ(send_one(peer_mac, "hello"), MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_espnow.send_count, 1);
        MTK_CHECK_EQ(g_fake_espnow.last_send_len, 5);

        /* An empty payload is not a legal ESP-NOW frame. */
        mtk_fake_sink_state_t s0; mtk_fake_sink_reset(&s0);
        mtk_request_ctx_t c0 = mtk_test_ctx(&s0, 9);
        mtk_espnow_send_req_t req; memset(&req, 0, sizeof(req));
        memcpy(req.peer.b, peer_mac, 6); req.data.len = 0;
        mtk_test_call(&c0, mtk_test_find_op("ESPNOW_SEND"), &req);
        MTK_CHECK_EQ(s0.response.status, MTK_STATUS_INVALID_ARGUMENT);

        g_fake_espnow.send_rc = -1;
        MTK_CHECK_EQ(send_one(peer_mac, "nope"), MTK_STATUS_IO_ERROR);
        g_fake_espnow.send_rc = 0;
        stop_espnow(&sink, token, NULL);
    }

    /* ---- Receive: frames arrive via the deferred drain and are returned one
     *      per poll, EMPTY once exhausted. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);

        /* Nothing is captured until the tick drains the HAL. */
        MTK_CHECK_EQ(poll_recv(NULL, NULL, NULL), 0 /* EMPTY */);

        const uint8_t payload[4] = { 1, 2, 3, 4 };
        mtk_fake_espnow_deliver(peer_mac, -42, payload, 4);
        mtek_espnow_service_tick();

        uint8_t body[300]; size_t blen = 0;
        MTK_CHECK_EQ(poll_recv(body, &blen, NULL), 1 /* RECORD */);
        MTK_CHECK_EQ(blen, (size_t)(1 + 6 + 1 + 1 + 4));
        MTK_CHECK(memcmp(body + 1, peer_mac, 6) == 0);
        MTK_CHECK_EQ((int8_t)body[7], -42);
        MTK_CHECK_EQ(body[8], 4);
        MTK_CHECK(memcmp(body + 9, payload, 4) == 0);

        /* Drained: the next poll is EMPTY again. */
        MTK_CHECK_EQ(poll_recv(NULL, NULL, NULL), 0);
        stop_espnow(&sink, token, NULL);
    }

    /* ---- The receive ring is bounded: a peer that never polls cannot cause
     *      unbounded buffering, and the drops are counted honestly. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);
        const uint8_t payload[1] = { 0x7F };
        for (unsigned round = 0; round < 4; round++) {
            for (unsigned i = 0; i < 32; i++) mtk_fake_espnow_deliver(peer_mac, -10, payload, 1);
            mtek_espnow_service_tick();
        }
        /* 128 delivered into a 16-slot ring. */
        mtk_fake_sink_state_t s; mtk_fake_sink_reset(&s);
        mtk_request_ctx_t c = mtk_test_ctx(&s, 10);
        mtk_test_call(&c, mtk_test_find_op("ESPNOW_STATS"), NULL);
        mtk_espnow_stats_resp_t st; memset(&st, 0, sizeof(st));
        mtk_decode(&mtk_espnow_stats_resp_t_desc, &st, s.response.body, s.response.body_len, NULL);
        MTK_CHECK_EQ(s.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(st.received, 128);
        MTK_CHECK_EQ(st.dropped, 112); /* 128 received - 16 retained */

        /* Exactly the ring's worth is retrievable, then EMPTY. */
        unsigned got = 0;
        while (poll_recv(NULL, NULL, NULL) == 1) got++;
        MTK_CHECK_EQ(got, 16);
        stop_espnow(&sink, token, NULL);
    }

    /* ---- ESP-NOW shares the Wi-Fi radio: it must serialize against Wi-Fi
     *      operations, not run alongside them. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);
        MTK_CHECK(token != 0);

        /* A SoftAP cannot come up while ESP-NOW owns the radio. */
        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t ctx = mtk_test_ctx(&s2, 11);
        mtk_softap_start_req_t ap; memset(&ap, 0, sizeof(ap));
        ap.config.ssid.len = 2; memcpy(ap.config.ssid.data, "ap", 2);
        ap.config.channel = 6;
        mtk_test_call(&ctx, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);
        stop_espnow(&sink, token, NULL);

        /* And once ESP-NOW is done, the radio is genuinely available again. */
        mtk_fake_sink_state_t s3; mtk_fake_sink_reset(&s3);
        mtk_request_ctx_t c3 = mtk_test_ctx(&s3, 12);
        mtk_test_call(&c3, mtk_test_find_op("SOFTAP_START"), &ap);
        MTK_CHECK_EQ(s3.response.status, MTK_STATUS_ACCEPTED);
    }

    /* ---- STOP is family-gated, and a peer reset reclaims the radio. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_espnow(&sink, 6, NULL);
        mtk_fake_sink_state_t bad; mtk_fake_sink_reset(&bad);
        uint8_t st = 0;
        stop_espnow(&bad, token ^ 0x5A5A5A5Au, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(g_fake_espnow.stop_count, 0);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);

        mtk_op_id_t cancelled = mtek_espnow_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, token);
        MTK_CHECK_EQ(g_fake_espnow.stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
