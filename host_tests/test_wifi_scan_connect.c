/* Wi-Fi AP scan and STA connect/status/disconnect (List A parity surface,
 * 002-wifi-service.md Sec 2.1/2.6): scan result paging, connect
 * accept/timeout/failure outcomes, idempotent status, and factory-UART's
 * UNAVAILABLE gate on STA_CONNECT. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    /* AP scan -> COMPLETE event -> RESULTS_PAGE round trip. */
    mtk_fake_wifi_reset();
    g_fake_wifi.ap_count = 3;
    for (int i = 0; i < 3; i++) {
        g_fake_wifi.ap_results[i].channel = (uint8_t)(i + 1);
        g_fake_wifi.ap_results[i].rssi = (int8_t)(-40 - i);
        g_fake_wifi.ap_results[i].ssid_len = 4;
        memcpy(g_fake_wifi.ap_results[i].ssid, "APxx", 4);
    }
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *scan_op = mtk_test_find_op("AP_SCAN_START");
    mtk_ap_scan_start_req_t req = {0}; req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;
    mtk_test_call(&ctx, scan_op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    const mtk_fake_event_t *done_ev = mtk_fake_find_event(&sink, "AP_SCAN_COMPLETE");
    MTK_CHECK(done_ev != NULL);
    mtk_ap_scan_complete_ev_t done = {0};
    mtk_decode(&mtk_ap_scan_complete_ev_t_desc, &done, done_ev->body, done_ev->body_len, NULL);
    MTK_CHECK_EQ(done.result_count, 3);

    const mtk_opcode_entry_t *page_op = mtk_test_find_op("AP_SCAN_RESULTS_PAGE");
    mtk_ap_scan_results_page_req_t preq = {0}; preq.result_generation = done.result_generation; preq.max_items = 2;
    mtk_fake_sink_state_t psink; mtk_fake_sink_reset(&psink);
    mtk_request_ctx_t pctx = mtk_test_ctx(&psink, 2);
    mtk_test_call(&pctx, page_op, &preq);
    mtk_ap_scan_results_page_resp_t page = {0};
    mtk_decode(page_op->resp_desc, &page, psink.response.body, psink.response.body_len, NULL);
    MTK_CHECK_EQ(page.items.count, 2);
    MTK_CHECK(page.next_index != 0); /* one more page remains */

    /* A stale result_generation is NOT_FOUND, never mixed-generation data. */
    mtk_fake_sink_reset(&psink);
    mtk_request_ctx_t pctx2 = mtk_test_ctx(&psink, 3);
    preq.result_generation = 0xFFFFFFFF;
    mtk_test_call(&pctx2, page_op, &preq);
    MTK_CHECK_EQ(psink.response.status, MTK_STATUS_NOT_FOUND);

    /* band=BAND_5GHZ alone is UNSUPPORTED on this 2.4GHz-only target. */
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx5g = mtk_test_ctx(&sink, 4);
    mtk_ap_scan_start_req_t req5g = {0}; req5g.band = 1;
    mtk_test_call(&ctx5g, scan_op, &req5g);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_UNSUPPORTED);

    /* STA_CONNECT: successful connect. */
    mtk_fake_wifi_reset();
    g_fake_wifi.connect_rc = 0;
    g_fake_wifi.connect_result.connected = 1;
    memcpy(g_fake_wifi.connect_result.bssid.b, (uint8_t[]){1,1,1,1,1,1}, 6);
    g_fake_wifi.connect_result.channel = 6;
    g_fake_wifi.connect_result.ip_present = 1;
    memcpy(g_fake_wifi.connect_result.ip.b, (uint8_t[]){10,0,0,5}, 4);

    const mtk_opcode_entry_t *connect_op = mtk_test_find_op("STA_CONNECT");
    mtk_sta_connect_req_t creq = {0};
    creq.ssid.len = 4; memcpy(creq.ssid.data, "Home", 4);
    creq.auth_mode = 3; creq.credential.kind = 1;
    creq.credential.psk.len = 8; memcpy(creq.credential.psk.data, "password", 8);
    creq.connect_timeout_ms = 15000;
    mtk_fake_sink_state_t csink; mtk_fake_sink_reset(&csink);
    mtk_request_ctx_t cctx = mtk_test_ctx(&csink, 5);
    mtk_test_call(&cctx, connect_op, &creq);
    MTK_CHECK_EQ(csink.response.status, MTK_STATUS_ACCEPTED);
    const mtk_fake_event_t *conn_done = mtk_fake_find_event(&csink, "STA_CONNECT_COMPLETE");
    MTK_CHECK(conn_done != NULL);
    mtk_sta_connect_complete_ev_t cd = {0};
    mtk_decode(&mtk_sta_connect_complete_ev_t_desc, &cd, conn_done->body, conn_done->body_len, NULL);
    MTK_CHECK_EQ(cd.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(cd.ip_present, 1);
    MTK_CHECK(mtek_wifi_is_sta_connected());

    /* STA_STATUS is always callable, never radio-gated: acquire WS first,
     * then confirm STA_STATUS still succeeds. */
    mtk_arbiter_reset();
    mtk_arbiter_acquire(MTK_ARB_WS, 999);
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("STA_STATUS");
    mtk_fake_sink_state_t ssink; mtk_fake_sink_reset(&ssink);
    mtk_request_ctx_t sctx = mtk_test_ctx(&ssink, 6);
    mtk_router_dispatch(&sctx, status_op->service_id, status_op->opcode, NULL, 0);
    MTK_CHECK_EQ(ssink.response.status, MTK_STATUS_OK);
    mtk_arbiter_release(MTK_ARB_WS);

    /* STA_DISCONNECT. */
    const mtk_opcode_entry_t *disc_op = mtk_test_find_op("STA_DISCONNECT");
    mtk_fake_sink_reset(&ssink);
    mtk_request_ctx_t dctx = mtk_test_ctx(&ssink, 7);
    mtk_router_dispatch(&dctx, disc_op->service_id, disc_op->opcode, NULL, 0);
    MTK_CHECK_EQ(ssink.response.status, MTK_STATUS_OK);
    MTK_CHECK(!mtek_wifi_is_sta_connected());

    /* Connect timeout path. */
    mtk_fake_wifi_reset();
    g_fake_wifi.connect_rc = -1;
    g_fake_wifi.connect_result.timed_out = 1;
    mtk_fake_sink_reset(&csink);
    mtk_request_ctx_t tctx = mtk_test_ctx(&csink, 8);
    mtk_test_call(&tctx, connect_op, &creq);
    const mtk_fake_event_t *timeout_ev = mtk_fake_find_event(&csink, "STA_CONNECT_COMPLETE");
    MTK_CHECK(timeout_ev != NULL);
    mtk_sta_connect_complete_ev_t td = {0};
    mtk_decode(&mtk_sta_connect_complete_ev_t_desc, &td, timeout_ev->body, timeout_ev->body_len, NULL);
    MTK_CHECK_EQ(td.status, MTK_STATUS_TIMEOUT);

    /* factory-UART: STA_CONNECT is UNAVAILABLE -> wire UNSUPPORTED. */
    mtk_fake_sink_reset(&csink);
    mtk_request_ctx_t uctx = mtk_test_ctx(&csink, 9);
    uctx.profile = MTK_PROFILE_FACTORY_UART;
    uctx.dispatch_mode = MTK_DISPATCH_INLINE;
    mtk_router_dispatch(&uctx, connect_op->service_id, connect_op->opcode, NULL, 0);
    MTK_CHECK_EQ(csink.response.status, MTK_STATUS_UNSUPPORTED);

MTK_TEST_MAIN_END
