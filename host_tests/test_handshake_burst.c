#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();mtk_fake_sink_state_t sink;mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx=mtk_test_ctx(&sink,1);ctx.profile=MTK_PROFILE_FACTORY_UART;
    mtk_handshake_start_req_t req={0};req.target_bssid.b[0]=2;req.channel=6;req.deauth_count=2;
    const mtk_opcode_entry_t *start=mtk_test_find_op("HANDSHAKE_START");
    mtk_test_call(&ctx,start,&req);MTK_CHECK_EQ(sink.response.status,MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,0);
    mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,1);
    mtk_test_advance_ms(19);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,1);
    mtk_test_advance_ms(1);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,2);
    mtk_test_advance_ms(9999);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,2);
    mtk_test_advance_ms(1);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,3);
    mtk_test_call(&ctx,mtk_test_find_op("WIFI_STOP_ALL"),NULL);
    mtk_test_advance_ms(20000);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,3);
    MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
    ctx.profile=MTK_PROFILE_NATIVE_SPI;req.deauth_count=2;
    mtk_test_call(&ctx,start,&req);mtek_wifi_service_tick();mtk_test_advance_ms(20);mtek_wifi_service_tick();
    unsigned count=g_fake_wifi.deauth_sent_count;mtk_test_advance_ms(20000);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,count);
    mtek_wifi_cancel_active_for_peer_reset();mtk_test_advance_ms(20);mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count,count);
MTK_TEST_MAIN_END
