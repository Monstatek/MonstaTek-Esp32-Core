/* Deauth: SELECTED target mode (one or more explicitly named stations
 * against one AP) -- 002-wifi-service.md Sec 2.4. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");

    mtk_deauth_start_req_t req = {0};
    req.target_mode = 0; /* SELECTED */
    memcpy(req.ap_bssid.b, (uint8_t[]){2,2,3,4,5,6}, 6); /* first octet even: unicast, not group/multicast */
    req.channel = 6;
    req.targets.count = 1;
    memcpy(req.targets.items[0].b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req.count = 1; req.interval_ms = 0;

    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_ap.b, req.ap_bssid.b, 6) == 0);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, req.targets.items[0].b, 6) == 0);
    const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "DEAUTH_STOPPED");
    MTK_CHECK(ev != NULL);
    if (ev) {
        mtk_deauth_stopped_ev_t done = {0};
        mtk_decode(&mtk_deauth_stopped_ev_t_desc, &done, ev->body, ev->body_len, NULL);
        MTK_CHECK_EQ(done.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(done.total_sent, 1);
    }
    MTK_CHECK_EQ(g_fake_wifi.restore_count, 1); /* STA mode restored on completion */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* Self-pair: a second DEAUTH_START while the D class is still active
     * (before this one's synchronous run completes conceptually) is BUSY
     * -- exercised directly against the arbiter, since this session's
     * synchronous HAL model completes DEAUTH_START within one call. */
    mtk_arbiter_reset();
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_D, 555), MTK_ARB_GRANT_OK);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_D, 556), MTK_ARB_GRANT_BUSY);

    /* Invalid selections rejected before any radio acquisition. */
    {
        mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink, 2);
        mtk_deauth_start_req_t bad = {0};
        bad.target_mode = 0;
        /* zero targets */
        mtk_test_call(&ctx2, op, &bad);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_INVALID_ARGUMENT);
    }
    {
        mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx3 = mtk_test_ctx(&sink, 3);
        mtk_deauth_start_req_t bad = {0};
        bad.target_mode = 0;
        memcpy(bad.ap_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        bad.channel = 6;
        bad.targets.count = 2;
        memcpy(bad.targets.items[0].b, (uint8_t[]){9,9,9,9,9,9}, 6);
        memcpy(bad.targets.items[1].b, (uint8_t[]){9,9,9,9,9,9}, 6); /* duplicate */
        mtk_test_call(&ctx3, op, &bad);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_INVALID_ARGUMENT);
    }

MTK_TEST_MAIN_END
