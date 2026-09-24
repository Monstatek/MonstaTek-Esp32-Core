#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>
static uint8_t last_frame[128];static unsigned last_len;
static int record_tx(const uint8_t *p,uint16_t n) { memcpy(last_frame,p,n);last_len=n;g_fake_wifi.raw_tx_count++;return g_fake_wifi.raw_tx_rc; }
MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();
    mtk_wifi_hal_t hal=*mtek_wifi_get_hal();hal.raw_tx=record_tx;mtek_wifi_set_hal(&hal);
    mtk_fake_sink_state_t sink;mtk_fake_sink_reset(&sink);mtk_request_ctx_t ctx=mtk_test_ctx(&sink,1);
    const mtk_opcode_entry_t *start=mtk_test_find_op("BEACON_START");
    mtk_beacon_start_req_t req={0};
    mtk_test_call(&ctx,start,&req);MTK_CHECK_EQ(sink.response.status,MTK_STATUS_INVALID_ARGUMENT);
    req.ssids.count=1;req.ssids.items[0].len=4;memcpy(req.ssids.items[0].data,"test",4);
    mtk_test_call(&ctx,start,&req);MTK_CHECK_EQ(sink.response.status,MTK_STATUS_ACCEPTED);
    mtk_beacon_start_resp_t resp={0};mtk_decode(start->resp_desc,&resp,sink.response.body,sink.response.body_len,NULL);
    mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.raw_tx_count,1);MTK_CHECK_EQ(last_frame[0],0x80);
    MTK_CHECK_EQ(last_frame[37],4);MTK_CHECK(!memcmp(last_frame+38,"test",4));MTK_CHECK_EQ(last_frame[last_len-1],1);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_BS,999),MTK_ARB_GRANT_BUSY);
    mtk_beacon_stop_req_t stop={resp.operation_token};
    mtk_test_call(&ctx,mtk_test_find_op("BEACON_STOP"),&stop);MTK_CHECK_EQ(sink.response.status,MTK_STATUS_OK);
    mtek_wifi_service_tick();MTK_CHECK_EQ(g_fake_wifi.raw_tx_count,1);MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
    mtk_test_call(&ctx,start,&req);mtek_wifi_cancel_active_for_peer_reset();
    MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
    mtk_test_call(&ctx,start,&req);g_fake_wifi.raw_tx_rc=-1;mtek_wifi_service_tick();
    MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
MTK_TEST_MAIN_END
