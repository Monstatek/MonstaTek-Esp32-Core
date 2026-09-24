#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include <string.h>
static int live_start(const mtk_hal_ble_adv_t *p,unsigned n){(void)p;(void)n;return 0;}
static void live_stop(void){}
static int snapshot(mtk_hal_ble_adv_t *p,unsigned max){if(!max)return 0;memset(p,0,sizeof(*p));p->addr.b[0]=2;p->rssi=-50;p->name_len=3;memcpy(p->name,"Tag",3);return 1;}
static int sample(mtk_hal_mac6_t a,uint8_t t,int8_t *r,uint8_t *random){(void)a;(void)t;*r=-54;*random=0;return 0;}
MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();mtk_ble_hal_t ble=g_fake_ble_hal;
    ble.live_start=live_start;ble.live_stop=live_stop;ble.live_snapshot=snapshot;ble.signal_sample=sample;mtek_ble_set_hal(&ble);
    g_fake_wifi.ap_count=2;
    for(unsigned i=0;i<2;i++){g_fake_wifi.ap_results[i].bssid.b[0]=2+i*2;g_fake_wifi.ap_results[i].channel=1+i*5;g_fake_wifi.ap_results[i].rssi=-40-i*10;}
    g_fake_wifi.sta_count=1;g_fake_wifi.sta_results[0].mac.b[0]=6;g_fake_wifi.sta_results[0].rssi=-60;
    mtk_uart_adapter_state_t st;mtek_uart_adapter_init(&st,MTK_TEST_BOOT_EPOCH);char out[4096];
    for(unsigned cycle=0;cycle<3;cycle++) {
        mtek_uart_process_line(&st,"mode -b",out,sizeof(out));
        mtek_uart_process_line(&st,"scan live",out,sizeof(out));MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_BS);
        mtek_uart_process_line(&st,"list",out,sizeof(out));MTK_CHECK(strstr(out,"Tag")!=NULL);
        mtek_uart_process_line(&st,"signal 02:00:00:00:00:00",out,sizeof(out));MTK_CHECK(st.ble_signal_running);
        mtek_uart_process_line(&st,"mode -w",out,sizeof(out));
        mtek_uart_process_line(&st,"scan -a",out,sizeof(out));MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_SM);
        mtek_uart_process_line(&st,"stop",out,sizeof(out));MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
        mtek_uart_process_line(&st,"scan -a",out,sizeof(out));MTK_CHECK_EQ(st.ap_count,2);MTK_CHECK_EQ(st.ap_table[1].channel,6);
        mtek_uart_process_line(&st,"select -a 0",out,sizeof(out));mtek_uart_process_line(&st,"scan -s",out,sizeof(out));MTK_CHECK_EQ(st.sta_count,1);
        mtek_uart_process_line(&st,"select -s 0",out,sizeof(out));mtek_uart_process_line(&st,"deauth",out,sizeof(out));
        mtek_uart_process_line(&st,"stop",out,sizeof(out));
        mtek_uart_process_line(&st,"beacon \"test\"",out,sizeof(out));MTK_CHECK(strstr(out,"Beacon started")!=NULL);mtek_wifi_service_tick();
        mtek_uart_process_line(&st,"stop",out,sizeof(out));MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
        mtek_uart_process_line(&st,"handshake",out,sizeof(out));MTK_CHECK(st.handshake_running);
        mtek_uart_process_line(&st,"stop",out,sizeof(out));MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
        mtek_uart_process_line(&st,"list -h",out,sizeof(out));
        while(mtek_uart_adapter_poll_background(&st,out,sizeof(out))){}
    }
MTK_TEST_MAIN_END
