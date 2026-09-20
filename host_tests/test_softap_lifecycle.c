/* SoftAP lifecycle (SOFTAP_START/STOP/STA_LIST, service 0x0001, arbiter class
 * SAP). Covers the properties the AP shares with every other long-lived radio
 * operation in this tree -- one token, one lease, one teardown path -- plus the
 * two that are specific to it: the passphrase is never retained, and a start
 * that fails at the HAL must not leave a session advertised as RUNNING. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include "mtek_arbiter.h"
#include <string.h>

static uint32_t start_ap(mtk_fake_sink_state_t *sink, const char *ssid,
                          const char *psk, uint8_t channel, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("SOFTAP_START");
    mtk_softap_start_req_t req; memset(&req, 0, sizeof(req));
    req.config.ssid.len = (uint16_t)strlen(ssid);
    memcpy(req.config.ssid.data, ssid, strlen(ssid));
    if (psk) { req.config.psk.len = (uint16_t)strlen(psk); memcpy(req.config.psk.data, psk, strlen(psk)); }
    req.config.channel = channel;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
    if (sink->response.status != MTK_STATUS_ACCEPTED) return 0;
    mtk_softap_start_resp_t r; memset(&r, 0, sizeof(r));
    mtk_decode(&mtk_softap_start_resp_t_desc, &r, sink->response.body, sink->response.body_len, NULL);
    return r.operation_token;
}

static void stop_ap(mtk_fake_sink_state_t *sink, uint32_t token, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 2);
    const mtk_opcode_entry_t *op = mtk_test_find_op("SOFTAP_STOP");
    mtk_softap_stop_req_t req; memset(&req, 0, sizeof(req));
    req.operation_token = token;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
}

MTK_TEST_MAIN_BEGIN

    /* ---- Happy path: start advertises, READY is emitted, STOP tears down. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_ap(&sink, "MonstaTek-Test", "supersecret", 6, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.softap_last_channel, 6);
        MTK_CHECK_EQ(g_fake_wifi.softap_last_ssid_len, 14);
        MTK_CHECK(memcmp(g_fake_wifi.softap_last_ssid, "MonstaTek-Test", 14) == 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_last_psk_len, 11);
        /* The AP holds the SAP lease for as long as it serves. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_SAP);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token);
        MTK_CHECK(mtk_fake_find_event(&sink, "SOFTAP_READY") != NULL);

        uint8_t stop_st = 0;
        stop_ap(&sink, token, &stop_st);
        MTK_CHECK_EQ(stop_st, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.softap_active, 0);
        /* Lease released and a terminal event emitted exactly once. */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&sink, "SOFTAP_STOPPED") != NULL);
    }

    /* ---- Invalid arguments are refused before any resource is taken. */
    {
        struct { const char *ssid; const char *psk; uint8_t ch; } bad[] = {
            { "",        "supersecret", 6  },   /* empty SSID */
            { "ok",      "short",       6  },   /* PSK below the WPA2 minimum */
            { "ok",      "supersecret", 0  },   /* channel below range */
            { "ok",      "supersecret", 14 },   /* channel above 2.4GHz range */
        };
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            uint32_t token = start_ap(&sink, bad[i].ssid, bad[i].psk, bad[i].ch, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(token, 0);
            /* Nothing was started and no lease was taken. */
            MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- An open network (no passphrase) is legal and reaches the HAL as such. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_ap(&sink, "OpenNet", NULL, 1, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_last_psk_len, 0);
        stop_ap(&sink, token, NULL);
    }

    /* ---- A HAL start failure tears the session down instead of leaving it
     *      RUNNING with nothing serving, and frees the lease. */
    {
        mtk_test_bootstrap();
        g_fake_wifi.softap_start_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_ap(&sink, "WillFail", "supersecret", 3, &st);
        /* ACCEPTED is already on the wire before the radio is touched. */
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, 1); /* teardown ran */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "SOFTAP_STOPPED");
        MTK_CHECK(ev != NULL);
        MTK_CHECK(mtk_fake_find_event(&sink, "SOFTAP_READY") == NULL);
        if (ev) {
            mtk_softap_stopped_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_softap_stopped_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_IO_ERROR);
        }
    }

    /* ---- Only one AP at a time: a second START is refused BUSY, and the
     *      refusal does not disturb the running session. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t first = start_ap(&sink, "First", "supersecret", 6, NULL);
        MTK_CHECK(first != 0);
        mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
        uint8_t st2 = 0;
        uint32_t second = start_ap(&sink2, "Second", "supersecret", 6, &st2);
        MTK_CHECK_EQ(st2, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(second, 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 1); /* the HAL was never called a second time */
        MTK_CHECK_EQ(mtk_arbiter_active_token(), first); /* first session untouched */
        stop_ap(&sink, first, NULL);
    }

    /* ---- STOP is token-addressed and family-gated: a foreign or unknown
     *      token can never finalize this operation or free its lease. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_ap(&sink, "Guarded", "supersecret", 6, NULL);
        MTK_CHECK(token != 0);

        mtk_fake_sink_state_t bad; mtk_fake_sink_reset(&bad);
        uint8_t st = 0;
        stop_ap(&bad, token ^ 0x5A5A5A5Au, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_NOT_FOUND);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, 0);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), token); /* still owned by the real session */

        stop_ap(&sink, token, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_OK);
    }

    /* ---- Repeat STOP is idempotent: it reports the already-recorded terminal
     *      result and does not tear down twice. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_ap(&sink, "Idem", "supersecret", 6, NULL);
        uint8_t st1 = 0, st2 = 0;
        stop_ap(&sink, token, &st1);
        unsigned stops_after_first = g_fake_wifi.softap_stop_count;
        stop_ap(&sink, token, &st2);
        MTK_CHECK_EQ(st1, MTK_STATUS_OK);
        MTK_CHECK_EQ(st2, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, stops_after_first);
    }

    /* ---- STA_LIST reflects real state, and an unknown count reports zero
     *      rather than a guess. */
    {
        mtk_test_bootstrap();
        const mtk_opcode_entry_t *list_op = mtk_test_find_op("SOFTAP_STA_LIST");

        /* Inactive before any AP exists. */
        mtk_fake_sink_state_t s0; mtk_fake_sink_reset(&s0);
        mtk_request_ctx_t c0 = mtk_test_ctx(&s0, 9);
        mtk_test_call(&c0, list_op, NULL);
        mtk_softap_sta_list_resp_t lr; memset(&lr, 0, sizeof(lr));
        mtk_decode(&mtk_softap_sta_list_resp_t_desc, &lr, s0.response.body, s0.response.body_len, NULL);
        MTK_CHECK_EQ(s0.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(lr.active, 0);
        MTK_CHECK_EQ(lr.connected_station_count, 0);

        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_ap(&sink, "Counted", "supersecret", 6, NULL);
        g_fake_wifi.softap_sta_count_value = 3;

        mtk_fake_sink_state_t s1; mtk_fake_sink_reset(&s1);
        mtk_request_ctx_t c1 = mtk_test_ctx(&s1, 10);
        mtk_test_call(&c1, list_op, NULL);
        memset(&lr, 0, sizeof(lr));
        mtk_decode(&mtk_softap_sta_list_resp_t_desc, &lr, s1.response.body, s1.response.body_len, NULL);
        MTK_CHECK_EQ(lr.active, 1);
        MTK_CHECK_EQ(lr.connected_station_count, 3);
        MTK_CHECK_EQ(lr.internet_shared, 0); /* never claimed without a real uplink */

        /* HAL cannot report the count: answer zero, not a stale or invented one. */
        g_fake_wifi.softap_sta_count_rc = -1;
        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&s2, 11);
        mtk_test_call(&c2, list_op, NULL);
        memset(&lr, 0, sizeof(lr));
        mtk_decode(&mtk_softap_sta_list_resp_t_desc, &lr, s2.response.body, s2.response.body_len, NULL);
        MTK_CHECK_EQ(lr.active, 1);
        MTK_CHECK_EQ(lr.connected_station_count, 0);

        g_fake_wifi.softap_sta_count_rc = 0;
        stop_ap(&sink, token, NULL);
    }

    /* ---- A peer reset tears down an AP the departed peer left serving. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_ap(&sink, "Orphan", "supersecret", 6, NULL);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_SAP);
        mtk_op_id_t cancelled = mtek_wifi_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, token);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.softap_active, 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
