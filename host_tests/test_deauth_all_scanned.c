/* Deauth: ALL_SCANNED target mode -- every station from one retained, completed
 * STA_SCAN_START result snapshot. Covers the stale/wrong-generation NOT_FOUND
 * and empty-scan NOT_READY failure paths too. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    /* A stale/never-issued station_scan_token is NOT_FOUND. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");
        mtk_deauth_start_req_t req = {0};
        req.target_mode = 1; req.station_scan_token = 12345;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_NOT_FOUND);
    }

    /* Run a real STA_SCAN_START first to produce a retained snapshot. */
    mtk_fake_wifi_reset();
    g_fake_wifi.sta_count = 3;
    for (int i = 0; i < 3; i++) { memset(g_fake_wifi.sta_results[i].mac.b, 0x30 + i, 6); g_fake_wifi.sta_results[i].rssi = -50 - i; }

    mtk_fake_sink_state_t scan_sink; mtk_fake_sink_reset(&scan_sink);
    mtk_request_ctx_t scan_ctx = mtk_test_ctx(&scan_sink, 2);
    const mtk_opcode_entry_t *sta_scan_op = mtk_test_find_op("STA_SCAN_START");
    mtk_sta_scan_start_req_t sreq = {0};
    memcpy(sreq.target_bssid.b, (uint8_t[]){9,9,9,9,9,9}, 6);
    sreq.channel = 9; sreq.duration_ms = 5000;
    mtk_test_call(&scan_ctx, sta_scan_op, &sreq);
    MTK_CHECK_EQ(scan_sink.response.status, MTK_STATUS_ACCEPTED);
    const mtk_fake_event_t *scan_done = mtk_fake_find_event(&scan_sink, "STA_SCAN_COMPLETE");
    MTK_CHECK(scan_done != NULL);
    mtk_sta_scan_complete_ev_t done = {0};
    mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &done, scan_done->body, scan_done->body_len, NULL);
    MTK_CHECK_EQ(done.result_count, 3);

    /* Now ALL_SCANNED against that snapshot deauths every retained station. */
    mtk_fake_wifi_reset(); /* clear deauth_sent_count without disturbing s_sta */
    mtk_fake_sink_state_t sink2; mtk_fake_sink_reset(&sink2);
    mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink2, 3);
    const mtk_opcode_entry_t *deauth_op = mtk_test_find_op("DEAUTH_START");
    mtk_deauth_start_req_t req2 = {0};
    req2.target_mode = 1; req2.station_scan_token = done.result_generation;
    mtk_test_call(&ctx2, deauth_op, &req2);
    MTK_CHECK_EQ(sink2.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 3); /* one pass over all 3 retained stations */

    /* A completed-but-empty scan is NOT_READY, not silently zero targets. */
    mtk_fake_wifi_reset();
    g_fake_wifi.sta_count = 0;
    mtk_fake_sink_reset(&scan_sink);
    mtk_request_ctx_t scan_ctx2 = mtk_test_ctx(&scan_sink, 4);
    mtk_test_call(&scan_ctx2, sta_scan_op, &sreq);
    mtk_sta_scan_complete_ev_t done2 = {0};
    const mtk_fake_event_t *ev2 = mtk_fake_find_event(&scan_sink, "STA_SCAN_COMPLETE");
    mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &done2, ev2->body, ev2->body_len, NULL);
    mtk_fake_sink_state_t sink3; mtk_fake_sink_reset(&sink3);
    mtk_request_ctx_t ctx3 = mtk_test_ctx(&sink3, 5);
    mtk_deauth_start_req_t req3 = {0};
    req3.target_mode = 1; req3.station_scan_token = done2.result_generation;
    mtk_test_call(&ctx3, deauth_op, &req3);
    MTK_CHECK_EQ(sink3.response.status, MTK_STATUS_NOT_READY);

MTK_TEST_MAIN_END
