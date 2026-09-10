/* BLE scan and advertising (002-ble-gatt-service.md Sec 2.1/2.2): scan
 * result paging, advertising start/status/stop, and the shared BA-class
 * single-advertising-slot exclusion. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    mtk_fake_ble_reset();
    g_fake_ble.scan_count = 2;
    memcpy(g_fake_ble.scan_results[0].addr.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    g_fake_ble.scan_results[0].rssi = -50;
    g_fake_ble.scan_results[0].name_len = 3; memcpy(g_fake_ble.scan_results[0].name, "Tag", 3);
    g_fake_ble.scan_results[1].rssi = -80;

    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *scan_op = mtk_test_find_op("BLE_SCAN_START");
    mtk_ble_scan_start_req_t req = {0}; req.mode = 0; req.duration_ms = 5000;
    mtk_test_call(&ctx, scan_op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    unsigned found_events = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "BLE_DEVICE_FOUND") == 0) found_events++;
    MTK_CHECK_EQ(found_events, 2);
    const mtk_fake_event_t *done_ev = mtk_fake_find_event(&sink, "BLE_SCAN_COMPLETE");
    MTK_CHECK(done_ev != NULL);
    mtk_ble_scan_complete_ev_t done = {0};
    mtk_decode(&mtk_ble_scan_complete_ev_t_desc, &done, done_ev->body, done_ev->body_len, NULL);
    MTK_CHECK_EQ(done.result_count, 2);

    const mtk_opcode_entry_t *page_op = mtk_test_find_op("BLE_SCAN_RESULTS_PAGE");
    mtk_ble_scan_results_page_req_t preq = {0}; preq.result_generation = done.result_generation; preq.max_items = 10;
    mtk_fake_sink_state_t psink; mtk_fake_sink_reset(&psink);
    mtk_request_ctx_t pctx = mtk_test_ctx(&psink, 2);
    mtk_test_call(&pctx, page_op, &preq);
    mtk_ble_scan_results_page_resp_t page = {0};
    mtk_decode(page_op->resp_desc, &page, psink.response.body, psink.response.body_len, NULL);
    MTK_CHECK_EQ(page.items.count, 2);
    MTK_CHECK_EQ(page.items.items[0].name.len, 3);

    /* Advertising: start, status, single-slot exclusion, stop. */
    mtk_fake_ble_reset();
    g_fake_ble.adv_start_rc = 0;
    const mtk_opcode_entry_t *adv_start = mtk_test_find_op("BLE_ADV_START");
    mtk_ble_adv_start_req_t areq = {0};
    areq.name.len = 4; memcpy(areq.name.data, "M1BT", 4);
    mtk_fake_sink_state_t asink; mtk_fake_sink_reset(&asink);
    mtk_request_ctx_t actx = mtk_test_ctx(&asink, 3);
    mtk_test_call(&actx, adv_start, &areq);
    MTK_CHECK_EQ(asink.response.status, MTK_STATUS_ACCEPTED);
    mtk_ble_adv_start_resp_t started = {0};
    mtk_decode(adv_start->resp_desc, &started, asink.response.body, asink.response.body_len, NULL);

    /* Second BLE_ADV_START while the first is still RUNNING: BUSY (BA self-pair). */
    mtk_fake_sink_reset(&asink);
    mtk_request_ctx_t actx2 = mtk_test_ctx(&asink, 4);
    mtk_test_call(&actx2, adv_start, &areq);
    MTK_CHECK_EQ(asink.response.status, MTK_STATUS_BUSY);

    const mtk_opcode_entry_t *adv_status = mtk_test_find_op("BLE_ADV_STATUS");
    mtk_ble_adv_status_req_t streq = {0}; streq.operation_token = started.operation_token;
    mtk_fake_sink_reset(&asink);
    mtk_request_ctx_t sctx = mtk_test_ctx(&asink, 5);
    mtk_test_call(&sctx, adv_status, &streq);
    mtk_ble_adv_status_resp_t st = {0};
    mtk_decode(adv_status->resp_desc, &st, asink.response.body, asink.response.body_len, NULL);
    MTK_CHECK_EQ(st.name.len, 4);
    /* RC5 independent audit P0 "BLE advertising breaks the frozen shipped
     * interface": the frozen List A parity record requires non-connectable
     * advertising with no scan response -- must agree with the real HAL's
     * esp32_ble_adv_start (BLE_GAP_CONN_MODE_NON, no adv-rsp fields set). */
    MTK_CHECK_EQ(st.connectable, 0);
    MTK_CHECK_EQ(st.has_scan_response, 0);

    const mtk_opcode_entry_t *adv_stop = mtk_test_find_op("BLE_ADV_STOP");
    mtk_ble_adv_stop_req_t spreq = {0}; spreq.operation_token = started.operation_token;
    mtk_fake_sink_reset(&asink);
    mtk_request_ctx_t spctx = mtk_test_ctx(&asink, 6);
    mtk_test_call(&spctx, adv_stop, &spreq);
    MTK_CHECK_EQ(asink.response.status, MTK_STATUS_OK);
    MTK_CHECK(mtk_fake_find_event(&asink, "BLE_ADV_STOPPED") != NULL);

    /* Now a new BLE_ADV_START succeeds, since the slot was released. */
    mtk_fake_sink_reset(&asink);
    mtk_request_ctx_t actx3 = mtk_test_ctx(&asink, 7);
    mtk_test_call(&actx3, adv_start, &areq);
    MTK_CHECK_EQ(asink.response.status, MTK_STATUS_ACCEPTED);

MTK_TEST_MAIN_END
