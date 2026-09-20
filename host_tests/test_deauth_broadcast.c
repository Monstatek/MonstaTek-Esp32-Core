/* Deauth: BROADCAST target mode -- transmits to FF:FF:FF:FF:FF:FF for the named
 * AP, distinct from ALL_SCANNED even though both may affect every client. */
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
    req.target_mode = 2; /* BROADCAST */
    memcpy(req.ap_bssid.b, (uint8_t[]){0x44,0x44,0x44,0x44,0x44,0x44}, 6);
    req.channel = 3;

    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1);
    static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, bcast, 6) == 0);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_ap.b, req.ap_bssid.b, 6) == 0);

    /* BROADCAST requires a valid AP BSSID and channel; all-zero BSSID is rejected. */
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink, 2);
    mtk_deauth_start_req_t bad = {0};
    bad.target_mode = 2; bad.channel = 3;
    mtk_test_call(&ctx2, op, &bad);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_INVALID_ARGUMENT);

MTK_TEST_MAIN_END
