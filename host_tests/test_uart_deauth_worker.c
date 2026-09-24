#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include "mtek_async_sink.h"
#include "mtek_schema_message_descs.h"
#include <pthread.h>
#include <unistd.h>
#include <stdatomic.h>
static pthread_mutex_t mu=PTHREAD_MUTEX_INITIALIZER;
static void lock(void){pthread_mutex_lock(&mu);}static void unlock(void){pthread_mutex_unlock(&mu);}
static void qlock(void *p){(void)p;lock();}static void qunlock(void *p){(void)p;unlock();}
static pthread_t thread;static void (*work)(void *);static void *work_arg;
static void *run(void *p){(void)p;usleep(20000);work(work_arg);return NULL;}
static int runner(void (*fn)(void *),void *p){work=fn;work_arg=p;return pthread_create(&thread,NULL,run,NULL);}
static atomic_uint sends;
static int send_frame(mtk_hal_mac6_t ap,mtk_hal_mac6_t sta,uint8_t c){(void)ap;(void)sta;(void)c;atomic_fetch_add(&sends,1);return 0;}
static void delay(uint32_t ms){(void)ms;usleep(1000);}
MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();mtk_core_set_lock(lock,unlock);mtk_router_set_lock(lock,unlock);
    mtek_wifi_service_set_lock(lock,unlock);mtk_router_set_async_runner(runner);
    mtk_wifi_hal_t hal=g_fake_wifi_hal;hal.send_deauth=send_frame;hal.pace_delay_ms=delay;mtek_wifi_set_hal(&hal);
    mtk_uart_adapter_state_t st;mtek_uart_adapter_init(&st,MTK_TEST_BOOT_EPOCH);
    mtk_async_queue_set_lock(&st.session_queue,qlock,qunlock,NULL);
    st.ap_selected=0;st.ap_count=1;st.ap_table[0].bssid.b[0]=2;st.ap_table[0].channel=6;
    st.sta_selected=0;st.sta_count=1;st.sta_table[0].mac.b[0]=4;
    char out[512];mtek_uart_process_line(&st,"deauth",out,sizeof(out));MTK_CHECK(st.deauth_pending!=0);
    for(unsigned i=0;i<1000 && st.deauth_pending;i++){usleep(1000);mtek_uart_adapter_poll_background(&st,out,sizeof(out));}
    MTK_CHECK(st.deauth_running);MTK_CHECK(strstr(out,"Deauth started")!=NULL);
    usleep(10000);MTK_CHECK(atomic_load(&sends)>1);
    mtk_deauth_progress_ev_t ev={st.deauth_token,10,0};
    mtk_async_sink_event(&st.session_queue,st.deauth_token,"DEAUTH_PROGRESS",&ev,&mtk_deauth_progress_ev_t_desc);
    mtek_uart_adapter_poll_background(&st,out,sizeof(out));MTK_CHECK(strstr(out,"10 pkts/s")!=NULL);
    mtek_uart_process_line(&st,"stop",out,sizeof(out));pthread_join(thread,NULL);
    unsigned ended=atomic_load(&sends);usleep(5000);MTK_CHECK_EQ(ended,atomic_load(&sends));
    MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
    while(mtek_uart_adapter_poll_background(&st,out,sizeof(out))) {}
    mtek_uart_process_line(&st,"deauth",out,sizeof(out));
    mtek_uart_process_line(&st,"stop",out,sizeof(out));MTK_CHECK(st.deauth_stop_pending);
    for(unsigned i=0;i<1000 && st.deauth_pending;i++){usleep(1000);mtek_uart_adapter_poll_background(&st,out,sizeof(out));}
    pthread_join(thread,NULL);MTK_CHECK(!st.deauth_running);MTK_CHECK_EQ(mtk_arbiter_snapshot().cls,MTK_ARB_NONE);
MTK_TEST_MAIN_END
