/* Captive portal (CAPTIVE_PORTAL_START/STOP/GET_CREDENTIALS/GET_DIAGNOSTICS).
 *
 * The portal shares the SoftAP's arbiter class and teardown path, so the
 * lifecycle assertions here focus on what is specific to it: the AP it brings
 * up is open (a portal a client must already hold a passphrase for cannot
 * steer anyone to a sign-in page), a portal that fails to start does not leave
 * an unannounced open AP behind, captured credentials survive an ordinary stop
 * but never leak across sessions, and the store is bounded. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include "mtek_arbiter.h"
#include <string.h>

static uint32_t start_portal(mtk_fake_sink_state_t *sink, const char *ssid,
                              const char *title, uint8_t channel, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("CAPTIVE_PORTAL_START");
    mtk_captive_portal_start_req_t req; memset(&req, 0, sizeof(req));
    req.ssid.len = (uint16_t)strlen(ssid);
    memcpy(req.ssid.data, ssid, strlen(ssid));
    req.portal_title.len = (uint16_t)strlen(title);
    memcpy(req.portal_title.data, title, strlen(title));
    req.channel = channel;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
    if (sink->response.status != MTK_STATUS_ACCEPTED) return 0;
    mtk_captive_portal_start_resp_t r; memset(&r, 0, sizeof(r));
    mtk_decode(&mtk_captive_portal_start_resp_t_desc, &r, sink->response.body, sink->response.body_len, NULL);
    return r.operation_token;
}

static void stop_portal(mtk_fake_sink_state_t *sink, uint32_t token, uint8_t *status_out) {
    mtk_request_ctx_t ctx = mtk_test_ctx(sink, 2);
    const mtk_opcode_entry_t *op = mtk_test_find_op("CAPTIVE_PORTAL_STOP");
    mtk_captive_portal_stop_req_t req; memset(&req, 0, sizeof(req));
    req.operation_token = token;
    mtk_test_call(&ctx, op, &req);
    if (status_out) *status_out = sink->response.status;
}

static void get_creds(mtk_captive_portal_get_credentials_resp_t *out, uint8_t max_count) {
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 7);
    const mtk_opcode_entry_t *op = mtk_test_find_op("CAPTIVE_PORTAL_GET_CREDENTIALS");
    mtk_captive_portal_get_credentials_req_t req; memset(&req, 0, sizeof(req));
    req.max_count = max_count;
    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
    memset(out, 0, sizeof(*out));
    mtk_decode(&mtk_captive_portal_get_credentials_resp_t_desc, out,
               sink.response.body, sink.response.body_len, NULL);
}

MTK_TEST_MAIN_BEGIN

    /* ---- Happy path: AP is brought up OPEN, portal starts, READY emitted. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_portal(&sink, "Free-WiFi", "Sign in to continue", 6, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 1);
        /* Open by construction -- never a passphrase-protected portal. */
        MTK_CHECK_EQ(g_fake_wifi.softap_last_psk_len, 0);
        MTK_CHECK_EQ(g_fake_wifi.portal_start_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.portal_last_title_len, 19);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_SAP);
        MTK_CHECK(mtk_fake_find_event(&sink, "CAPTIVE_PORTAL_READY") != NULL);

        uint8_t stop_st = 0;
        stop_portal(&sink, token, &stop_st);
        MTK_CHECK_EQ(stop_st, MTK_STATUS_OK);
        /* Portal runtime stopped, and the interface under it too. */
        MTK_CHECK_EQ(g_fake_wifi.portal_stop_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.softap_stop_count, 1);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        /* The portal's own terminal event, not the SoftAP one. */
        MTK_CHECK(mtk_fake_find_event(&sink, "CAPTIVE_PORTAL_STOPPED") != NULL);
        MTK_CHECK(mtk_fake_find_event(&sink, "SOFTAP_STOPPED") == NULL);
    }

    /* ---- A portal that cannot start must not leave an open AP serving with
     *      no sign-in page. */
    {
        mtk_test_bootstrap();
        g_fake_wifi.portal_start_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint8_t st = 0;
        uint32_t token = start_portal(&sink, "Free-WiFi", "Sign in", 6, &st);
        MTK_CHECK_EQ(st, MTK_STATUS_ACCEPTED);
        MTK_CHECK(token != 0);
        MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 1); /* AP did come up ... */
        MTK_CHECK_EQ(g_fake_wifi.softap_active, 0);      /* ... and was taken back down */
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(mtk_fake_find_event(&sink, "CAPTIVE_PORTAL_READY") == NULL);
        const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "CAPTIVE_PORTAL_STOPPED");
        MTK_CHECK(ev != NULL);
        if (ev) {
            mtk_captive_portal_stopped_ev_t d; memset(&d, 0, sizeof(d));
            mtk_decode(&mtk_captive_portal_stopped_ev_t_desc, &d, ev->body, ev->body_len, NULL);
            MTK_CHECK_EQ(d.status, MTK_STATUS_IO_ERROR);
        }
    }

    /* ---- Invalid arguments refused before any resource is taken. */
    {
        struct { const char *ssid; uint8_t ch; } bad[] = {
            { "",   6  },   /* empty SSID */
            { "ok", 0  },   /* channel below range */
            { "ok", 14 },   /* above the 2.4GHz range */
        };
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            mtk_test_bootstrap();
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            uint8_t st = 0;
            uint32_t token = start_portal(&sink, bad[i].ssid, "t", bad[i].ch, &st);
            MTK_CHECK_EQ(st, MTK_STATUS_INVALID_ARGUMENT);
            MTK_CHECK_EQ(token, 0);
            MTK_CHECK_EQ(g_fake_wifi.softap_start_count, 0);
            MTK_CHECK_EQ(g_fake_wifi.portal_start_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

    /* ---- Submissions are captured via the deferred drain, and survive an
     *      ordinary stop (that is the point of the feature). */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_portal(&sink, "Free-WiFi", "Sign in", 6, NULL);
        MTK_CHECK(token != 0);

        mtk_fake_portal_submit("alice", "hunter2");
        mtk_fake_portal_submit("bob", "correct-horse");
        /* Nothing is captured until the service tick drains the HAL. */
        mtk_captive_portal_get_credentials_resp_t creds;
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 0);

        mtek_wifi_service_tick();
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 2);
        MTK_CHECK_EQ(creds.credentials.items[0].username.len, 5);
        MTK_CHECK(memcmp(creds.credentials.items[0].username.data, "alice", 5) == 0);
        MTK_CHECK_EQ(creds.credentials.items[0].password.len, 7);
        MTK_CHECK(memcmp(creds.credentials.items[0].password.data, "hunter2", 7) == 0);
        MTK_CHECK(memcmp(creds.credentials.items[1].username.data, "bob", 3) == 0);

        /* max_count narrows the reported set without losing the rest. */
        get_creds(&creds, 1);
        MTK_CHECK_EQ(creds.credentials.count, 1);
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 2);

        stop_portal(&sink, token, NULL);
        /* Still retrievable after the portal is down. */
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 2);
    }

    /* ---- A new session never inherits the previous engagement's captures. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t t1 = start_portal(&sink, "One", "Sign in", 6, NULL);
        mtk_fake_portal_submit("alice", "hunter2");
        mtek_wifi_service_tick();
        mtk_captive_portal_get_credentials_resp_t creds;
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 1);
        stop_portal(&sink, t1, NULL);

        uint32_t t2 = start_portal(&sink, "Two", "Sign in", 6, NULL);
        MTK_CHECK(t2 != 0);
        get_creds(&creds, 0);
        MTK_CHECK_EQ(creds.credentials.count, 0); /* zeroized for the new session */
        stop_portal(&sink, t2, NULL);
    }

    /* ---- The store is bounded: submissions past its capacity are dropped,
     *      never written past the end of the array. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_portal(&sink, "Flood", "Sign in", 6, NULL);
        for (unsigned round = 0; round < 8; round++) {   /* 8 x 8 = 64 submissions */
            for (unsigned i = 0; i < 8; i++) mtk_fake_portal_submit("u", "p");
            mtek_wifi_service_tick();
        }
        mtk_captive_portal_get_credentials_resp_t creds;
        get_creds(&creds, 0);
        /* Capped at the schema's own maximum, read from the generated array
         * itself so retuning the capacity in schemas.json cannot silently
         * leave this bound untested. */
        const uint32_t cap =
            (uint32_t)(sizeof(creds.credentials.items) / sizeof(creds.credentials.items[0]));
        MTK_CHECK_EQ(creds.credentials.count, cap);
        stop_portal(&sink, token, NULL);
    }

    /* ---- Diagnostics report real counters, and zeros when the HAL cannot. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_portal(&sink, "Diag", "Sign in", 6, NULL);
        g_fake_wifi.portal_dns_queries = 41;
        g_fake_wifi.portal_http_hits = 7;
        g_fake_wifi.softap_sta_count_value = 2;
        memcpy(g_fake_wifi.portal_last_post, "user=a&pass=b", 13);
        g_fake_wifi.portal_last_post_len = 13;

        const mtk_opcode_entry_t *op = mtk_test_find_op("CAPTIVE_PORTAL_GET_DIAGNOSTICS");
        mtk_fake_sink_state_t d1; mtk_fake_sink_reset(&d1);
        mtk_request_ctx_t c1 = mtk_test_ctx(&d1, 8);
        mtk_test_call(&c1, op, NULL);
        mtk_captive_portal_get_diagnostics_resp_t dr; memset(&dr, 0, sizeof(dr));
        mtk_decode(&mtk_captive_portal_get_diagnostics_resp_t_desc, &dr, d1.response.body, d1.response.body_len, NULL);
        MTK_CHECK_EQ(d1.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(dr.dns_queries, 41);
        MTK_CHECK_EQ(dr.http_hits, 7);
        MTK_CHECK_EQ(dr.connected_clients, 2);
        MTK_CHECK_EQ(dr.last_post_body.len, 13);

        /* HAL cannot report: zeros, not stale values. */
        g_fake_wifi.portal_stats_rc = -1;
        mtk_fake_sink_state_t d2; mtk_fake_sink_reset(&d2);
        mtk_request_ctx_t c2 = mtk_test_ctx(&d2, 9);
        mtk_test_call(&c2, op, NULL);
        memset(&dr, 0, sizeof(dr));
        mtk_decode(&mtk_captive_portal_get_diagnostics_resp_t_desc, &dr, d2.response.body, d2.response.body_len, NULL);
        MTK_CHECK_EQ(dr.dns_queries, 0);
        MTK_CHECK_EQ(dr.http_hits, 0);
        MTK_CHECK_EQ(dr.last_post_body.len, 0);

        g_fake_wifi.portal_stats_rc = 0;
        stop_portal(&sink, token, NULL);
    }

    /* ---- A portal and a plain SoftAP contend for the same class: whichever
     *      holds it, the other is refused BUSY. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t portal = start_portal(&sink, "Portal", "Sign in", 6, NULL);
        MTK_CHECK(portal != 0);

        mtk_fake_sink_state_t s2; mtk_fake_sink_reset(&s2);
        mtk_request_ctx_t ctx = mtk_test_ctx(&s2, 5);
        const mtk_opcode_entry_t *ap_op = mtk_test_find_op("SOFTAP_START");
        mtk_softap_start_req_t ap; memset(&ap, 0, sizeof(ap));
        ap.config.ssid.len = 2; memcpy(ap.config.ssid.data, "ap", 2);
        ap.config.channel = 6;
        mtk_test_call(&ctx, ap_op, &ap);
        MTK_CHECK_EQ(s2.response.status, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(mtk_arbiter_active_token(), portal);
        stop_portal(&sink, portal, NULL);
    }

    /* ---- A peer reset takes the portal down with the session that started it. */
    {
        mtk_test_bootstrap();
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        uint32_t token = start_portal(&sink, "Orphan", "Sign in", 6, NULL);
        MTK_CHECK(token != 0);
        mtk_op_id_t cancelled = mtek_wifi_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, token);
        MTK_CHECK_EQ(g_fake_wifi.portal_stop_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.softap_active, 0);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
