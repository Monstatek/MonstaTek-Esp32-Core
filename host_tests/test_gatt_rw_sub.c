/* GATT client: connect/discover/read/write/subscribe/unsubscribe/disconnect
 * (002-ble-gatt-service.md Sec 3), including the drop/notify tick path. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_ble_reset();
    g_fake_ble.gatt_connect_rc = 0;
    g_fake_ble.gatt_vendor_handle = 42;
    g_fake_ble.gatt_service_count = 1;
    g_fake_ble.gatt_services[0].uuid_width = 0;
    /* RC8 independent audit P0-5 "Stop GATT UUID stack-data leakage":
     * poison the tail a 16-bit UUID never legitimately uses (bytes
     * [2..15]) to 0xEE -- simulating exactly the real bug
     * (mtek_ble_hal_esp32.c's disc_svc_cb once left these as whatever
     * uninitialized/stale memory preceded them) via this fake HAL's own
     * data instead. The wire response must show these as a defined 0
     * regardless, proving mtek_ble_logic.c's own defensive truncation
     * (not merely trusting the HAL) actually works. */
    memset(g_fake_ble.gatt_services[0].uuid_value, 0xEE, sizeof(g_fake_ble.gatt_services[0].uuid_value));
    g_fake_ble.gatt_services[0].uuid_value[0] = 0x0F; g_fake_ble.gatt_services[0].uuid_value[1] = 0x18;
    g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 10;
    g_fake_ble.gatt_read_len = 4;
    memcpy(g_fake_ble.gatt_read_data, "data", 4);
    g_fake_ble.gatt_write_rc = 0;
    g_fake_ble.gatt_subscribe_rc = 0;

    const mtk_opcode_entry_t *connect_op = mtk_test_find_op("GATT_CONNECT");
    mtk_gatt_connect_req_t creq = {0};
    memcpy(creq.target.addr.b, (uint8_t[]){1,1,1,1,1,1}, 6);
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    mtk_test_call(&ctx, connect_op, &creq);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    const mtk_fake_event_t *cev = mtk_fake_find_event(&sink, "GATT_CONNECT_COMPLETE");
    MTK_CHECK(cev != NULL);
    mtk_gatt_connect_complete_ev_t cd = {0};
    mtk_decode(&mtk_gatt_connect_complete_ev_t_desc, &cd, cev->body, cev->body_len, NULL);
    MTK_CHECK_EQ(cd.status, MTK_STATUS_OK);
    uint32_t conn_tok = cd.connection_token;
    MTK_CHECK(conn_tok != 0);

    /* GATT_DISCOVER */
    const mtk_opcode_entry_t *disc_op = mtk_test_find_op("GATT_DISCOVER");
    mtk_gatt_discover_req_t dreq = {0}; dreq.connection_token = conn_tok; dreq.max_items = 16;
    mtk_fake_sink_state_t dsink; mtk_fake_sink_reset(&dsink);
    mtk_request_ctx_t dctx = mtk_test_ctx(&dsink, 2);
    mtk_test_call(&dctx, disc_op, &dreq);
    MTK_CHECK_EQ(dsink.response.status, MTK_STATUS_OK);
    mtk_gatt_discover_resp_t dr = {0};
    mtk_decode(disc_op->resp_desc, &dr, dsink.response.body, dsink.response.body_len, NULL);
    MTK_CHECK_EQ(dr.services.count, 1);
    MTK_CHECK_EQ(dr.services.items[0].start_handle, 1);
    MTK_CHECK_EQ(dr.services.items[0].end_handle, 10);
    /* RC8 independent audit P0-5: uuid.value[2..15] must never leak the
     * poisoned 0xEE tail above -- a 16-bit UUID's unused bytes are a
     * defined 0 on the wire, not whatever memory preceded them. */
    MTK_CHECK_EQ(dr.services.items[0].uuid.width, 0);
    MTK_CHECK_EQ(dr.services.items[0].uuid.value[0], 0x0F);
    MTK_CHECK_EQ(dr.services.items[0].uuid.value[1], 0x18);
    for (int u = 2; u < 16; u++) MTK_CHECK_EQ(dr.services.items[0].uuid.value[u], 0);

    /* GATT_READ */
    const mtk_opcode_entry_t *read_op = mtk_test_find_op("GATT_READ");
    mtk_gatt_read_req_t rreq = {0}; rreq.connection_token = conn_tok; rreq.handle = 5;
    mtk_fake_sink_state_t rsink; mtk_fake_sink_reset(&rsink);
    mtk_request_ctx_t rctx = mtk_test_ctx(&rsink, 3);
    mtk_test_call(&rctx, read_op, &rreq);
    MTK_CHECK_EQ(rsink.response.status, MTK_STATUS_OK);
    mtk_gatt_read_resp_t rr = {0};
    mtk_decode(read_op->resp_desc, &rr, rsink.response.body, rsink.response.body_len, NULL);
    MTK_CHECK_EQ(rr.data.len, 4);
    MTK_CHECK(memcmp(rr.data.data, "data", 4) == 0);

    /* GATT_WRITE */
    const mtk_opcode_entry_t *write_op = mtk_test_find_op("GATT_WRITE");
    mtk_gatt_write_req_t wreq = {0}; wreq.connection_token = conn_tok; wreq.handle = 5;
    wreq.data.len = 3; memcpy(wreq.data.data, "abc", 3); wreq.with_response = 1;
    mtk_fake_sink_state_t wsink; mtk_fake_sink_reset(&wsink);
    mtk_request_ctx_t wctx = mtk_test_ctx(&wsink, 4);
    mtk_test_call(&wctx, write_op, &wreq);
    MTK_CHECK_EQ(wsink.response.status, MTK_STATUS_OK);

    /* GATT_SUBSCRIBE, then a delivered notification via mtek_ble_gatt_tick(). */
    const mtk_opcode_entry_t *sub_op = mtk_test_find_op("GATT_SUBSCRIBE");
    mtk_gatt_subscribe_req_t sreq = {0}; sreq.connection_token = conn_tok; sreq.handle = 7; sreq.mode = 0;
    mtk_fake_sink_state_t subsink; mtk_fake_sink_reset(&subsink);
    mtk_request_ctx_t subctx = mtk_test_ctx(&subsink, 5);
    mtk_test_call(&subctx, sub_op, &sreq);
    MTK_CHECK_EQ(subsink.response.status, MTK_STATUS_OK);
    /* The real discovered service end_handle (10, from GATT_DISCOVER
     * above) was threaded through to the HAL's CCCD search -- not a
     * guessed window and not attr_handle+1 (8). */
    MTK_CHECK_EQ(g_fake_ble.last_subscribe_attr_handle, 7);
    MTK_CHECK_EQ(g_fake_ble.last_subscribe_end_handle, 10);

    /* RC8 independent audit P0-6 "Preserve exact shipped UART behavior":
     * the accepted baseline requires a DISTINCT "no CCCD" error, never
     * conflated with an ordinary write failure. MTK_HAL_GATT_SUBSCRIBE_
     * NO_CCCD from the HAL must surface as MTK_STATUS_NOT_FOUND here
     * (mtek_ble_logic.c's own new distinction), not the generic IO_ERROR
     * an unrelated write failure would produce. */
    {
        g_fake_ble.gatt_subscribe_rc = MTK_HAL_GATT_SUBSCRIBE_NO_CCCD;
        mtk_gatt_subscribe_req_t nocccd_req = {0}; nocccd_req.connection_token = conn_tok; nocccd_req.handle = 7; nocccd_req.mode = 0;
        mtk_fake_sink_state_t nsink; mtk_fake_sink_reset(&nsink);
        mtk_request_ctx_t nctx = mtk_test_ctx(&nsink, 50);
        mtk_test_call(&nctx, sub_op, &nocccd_req);
        MTK_CHECK_EQ(nsink.response.status, MTK_STATUS_NOT_FOUND);
        g_fake_ble.gatt_subscribe_rc = 0; /* restore for the real subscribe below */
    }

    g_fake_ble.notify_pending = 1;
    g_fake_ble.notify_handle = 7;
    g_fake_ble.notify_len = 2;
    memcpy(g_fake_ble.notify_data, "hi", 2);
    mtek_ble_gatt_tick();
    /* The tick delivers into the connection's own captured ctx sink, which
     * is `subctx`'s underlying sink -- but subscribe itself used a fresh
     * sink; GATT_CONNECT's own ctx is what the service retains
     * (mtek_ble_logic.c s_gatt.ctx = *ctx at connect time), so assert
     * against the connect-time sink instead. */
    MTK_CHECK(mtk_fake_find_event(&sink, "GATT_VALUE_EVENT") != NULL);

    /* GATT_UNSUBSCRIBE */
    const mtk_opcode_entry_t *unsub_op = mtk_test_find_op("GATT_UNSUBSCRIBE");
    mtk_gatt_unsubscribe_req_t ureq = {0}; ureq.connection_token = conn_tok; ureq.handle = 7;
    mtk_fake_sink_state_t usink; mtk_fake_sink_reset(&usink);
    mtk_request_ctx_t uctx = mtk_test_ctx(&usink, 6);
    mtk_test_call(&uctx, unsub_op, &ureq);
    MTK_CHECK_EQ(usink.response.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(g_fake_ble.last_unsubscribe_attr_handle, 7);
    MTK_CHECK_EQ(g_fake_ble.last_unsubscribe_end_handle, 10);

    /* SUBSCRIBE against a handle outside every discovered service range:
     * fails cleanly (PROTOCOL_ERROR), never guesses a window. */
    {
        mtk_gatt_subscribe_req_t bad_sreq = {0}; bad_sreq.connection_token = conn_tok; bad_sreq.handle = 999; bad_sreq.mode = 0;
        mtk_fake_sink_state_t bsink; mtk_fake_sink_reset(&bsink);
        mtk_request_ctx_t bctx = mtk_test_ctx(&bsink, 50);
        mtk_test_call(&bctx, sub_op, &bad_sreq);
        MTK_CHECK_EQ(bsink.response.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* GATT_STATUS then GATT_DISCONNECT. */
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GATT_STATUS");
    mtk_gatt_status_req_t stq = {0}; stq.connection_token = conn_tok;
    mtk_fake_sink_state_t stsink; mtk_fake_sink_reset(&stsink);
    mtk_request_ctx_t stctx = mtk_test_ctx(&stsink, 7);
    mtk_test_call(&stctx, status_op, &stq);
    mtk_gatt_status_resp_t sr = {0};
    mtk_decode(status_op->resp_desc, &sr, stsink.response.body, stsink.response.body_len, NULL);
    MTK_CHECK_EQ(sr.connected, 1);
    MTK_CHECK_EQ(sr.dropped_notification_count, 0); /* nothing dropped yet */

    /* RC7 independent audit item 9 "notification queue overflow is
     * silent and never increments the public dropped counter": GATT_
     * STATUS's own dropped_notification_count now reads live from the
     * HAL (previously a service-layer field nothing ever incremented --
     * always reported 0 regardless of real overflow). */
    g_fake_ble.notify_dropped_count = 7;
    mtk_fake_sink_state_t stsink2; mtk_fake_sink_reset(&stsink2);
    mtk_request_ctx_t stctx2 = mtk_test_ctx(&stsink2, 12);
    mtk_test_call(&stctx2, status_op, &stq);
    mtk_gatt_status_resp_t sr0 = {0};
    mtk_decode(status_op->resp_desc, &sr0, stsink2.response.body, stsink2.response.body_len, NULL);
    MTK_CHECK_EQ(sr0.dropped_notification_count, 7);
    g_fake_ble.notify_dropped_count = 0; /* reset before the rest of this test's own flow */

    const mtk_opcode_entry_t *disconnect_op = mtk_test_find_op("GATT_DISCONNECT");
    mtk_gatt_disconnect_req_t dcreq = {0}; dcreq.connection_token = conn_tok;
    mtk_fake_sink_state_t dcsink; mtk_fake_sink_reset(&dcsink);
    mtk_request_ctx_t dcctx = mtk_test_ctx(&dcsink, 8);
    mtk_test_call(&dcctx, disconnect_op, &dcreq);
    MTK_CHECK_EQ(dcsink.response.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* Operations against a stale connection_token after disconnect: NOT_FOUND. */
    mtk_fake_sink_reset(&rsink);
    mtk_request_ctx_t rctx2 = mtk_test_ctx(&rsink, 9);
    mtk_test_call(&rctx2, read_op, &rreq);
    MTK_CHECK_EQ(rsink.response.status, MTK_STATUS_NOT_FOUND);

    /* RC5 independent audit P1 "BLE/GATT implementation is not yet
     * parity-complete": "Remote disconnect is ignored... leaving
     * connection/arbiter state stale." A genuine peer-initiated
     * disconnect (never a caller-issued GATT_DISCONNECT) must still
     * release the GC arbiter lease and clear connected-state bookkeeping
     * via mtek_ble_gatt_tick's own poll, so a subsequent GATT_STATUS
     * correctly reports disconnected rather than staying stale. */
    {
        mtk_fake_sink_state_t csink; mtk_fake_sink_reset(&csink);
        mtk_request_ctx_t cctx = mtk_test_ctx(&csink, 10);
        mtk_test_call(&cctx, connect_op, &creq);
        MTK_CHECK_EQ(csink.response.status, MTK_STATUS_ACCEPTED);
        const mtk_fake_event_t *cev2 = mtk_fake_find_event(&csink, "GATT_CONNECT_COMPLETE");
        mtk_gatt_connect_complete_ev_t cd2 = {0};
        mtk_decode(&mtk_gatt_connect_complete_ev_t_desc, &cd2, cev2->body, cev2->body_len, NULL);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_GC);

        /* Not yet disconnected: a tick is a no-op for connection state. */
        mtek_ble_gatt_tick();
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_GC);

        /* Simulate the peer tearing down the link (BLE_GAP_EVENT_DISCONNECT
         * on a real target -- see mtek_ble_hal_esp32.c's conn_gap_cb). */
        g_fake_ble.remote_disconnect_pending = 1;
        mtek_ble_gatt_tick();
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* lease released without any caller-issued GATT_DISCONNECT */

        const mtk_opcode_entry_t *status_op2 = mtk_test_find_op("GATT_STATUS");
        mtk_gatt_status_req_t sq2 = {0}; sq2.connection_token = cd2.connection_token;
        mtk_fake_sink_state_t ssink2; mtk_fake_sink_reset(&ssink2);
        mtk_request_ctx_t sctx2 = mtk_test_ctx(&ssink2, 11);
        mtk_test_call(&sctx2, status_op2, &sq2);
        mtk_gatt_status_resp_t sr2 = {0};
        mtk_decode(status_op2->resp_desc, &sr2, ssink2.response.body, ssink2.response.body_len, NULL);
        MTK_CHECK_EQ(sr2.connected, 0); /* state is correct, not stale, with no caller ever calling GATT_DISCONNECT */
    }

MTK_TEST_MAIN_END
