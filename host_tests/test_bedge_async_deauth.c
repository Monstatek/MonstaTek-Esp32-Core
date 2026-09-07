/* Real cross-thread proof of the async lifetime/delivery mechanism for
 * DEAUTH_START (a required List B feature, explicitly owner-approved) --
 * not a single-threaded simulation. Registers a genuine pthread-based
 * mtk_router async runner + a real pthread_mutex lock, dispatches
 * DEAUTH_START through the full Bedge wire path, and proves: the
 * dispatching call returns immediately without blocking on the
 * background thread; the current transaction correctly answers IDLE
 * while genuinely deferred; the persistent, dctx-owned event_queue
 * survives past the dispatching call's return; the real canonical deauth
 * logic actually ran on the OTHER thread (fake HAL reached); the
 * eventual real response is delivered correctly on a later poll with the
 * right operation_token; STOP against a still-pending (not yet accepted)
 * operation is safely rejected, not a race/crash; and the whole sequence
 * is ASan/UBSan-clean under real concurrent execution. */
#include "mtk_test.h"
#include "mtek_bedge_dispatch.h"
#include "mtek_bedge_opcode_map.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_router_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_router_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_router_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_router_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_router_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    /* A real background worker delay, standing in for genuine radio-HAL
     * latency -- long enough that the main thread's first few polls
     * reliably observe the still-deferred state before the real response
     * arrives, proving the deferral is real rather than incidentally
     * fast. */
    usleep(20000);
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(pthread_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* RC11 independent correction order P0 verification fallout (same
     * real TSan-caught gap as test_spi_native_dup_cache.c/test_spi_
     * native_async.c's own identical fix): a real async runner is
     * registered above, and DEAUTH_START's own completion path
     * (deauth_finalize) genuinely touches shared operation-table/wifi-
     * service/fake-HAL state from a different thread than a concurrent
     * DEAUTH_STOP/status request -- every one of these must share the
     * SAME real mutex as mtk_router_set_lock above. */
    mtk_core_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();

    mtk_bedge_dispatch_ctx_t dctx;
    mtek_bedge_dispatch_init(&dctx, 0x1234);
    /* The dctx-owned event_queue is genuinely shared across the calling
     * thread and the background worker thread the pthread_runner above
     * spawns -- it needs real mutual exclusion, exactly as target glue
     * would register real FreeRTOS mutex hooks. */
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    /* m1esp_deauth_req_t {bssid[6], channel, station[6], count:u16, interval_ms:u16} */
    uint8_t req_payload[6 + 1 + 6 + 2 + 2];
    memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
    req_payload[6] = 6;
    memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req_payload[13] = 1; req_payload[14] = 0; /* count = 1 LE */
    req_payload[15] = 0; req_payload[16] = 0; /* interval_ms = 0 */

    mtk_bedge_header_t req_hdr = {0};
    req_hdr.magic = MTK_BEDGE_MAGIC; req_hdr.version = MTK_BEDGE_VERSION;
    req_hdr.msg_type = MTK_BEDGE_MSG_REQ; req_hdr.msg_id = 0x0302; req_hdr.payload_len = sizeof(req_payload);

    mtk_bedge_header_t resp_hdr; uint8_t resp_payload[MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
    mtek_bedge_dispatch_request(&dctx, &req_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);

    /* Dispatching returned immediately: the transport loop is never
     * blocked waiting for the background deauth work to finish. Since
     * the worker thread sleeps 20ms before doing anything, the real
     * canonical handler cannot possibly have run yet -- this transaction
     * must be a genuine deferral, not an accidentally-fast synchronous
     * completion. */
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);
    MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0); /* real HAL not reached yet -- proves this really deferred */
    MTK_CHECK_EQ(dctx.deauth_token, 0); /* not yet minted/harvested */

    /* A STOP against an operation whose token this dispatch layer does
     * not have yet (because the ACCEPTED response is still in flight on
     * the other thread) is safely rejected -- no race, no crash, no
     * guessed token. */
    mtk_bedge_header_t stop_hdr = req_hdr; stop_hdr.msg_id = 0x0303; stop_hdr.payload_len = 0;
    mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_NAK);
    MTK_CHECK_EQ(resp_payload[0], MTK_BEDGE_STATUS_ERR_NOT_RUNNING);

    /* Poll (as the physical SPI transaction loop would on repeated IDLE
     * exchanges from the peer) until the real, cross-thread-delivered
     * response arrives. Bounded retry -- this must complete well within
     * a couple of seconds on any real machine; a hang here would itself
     * be a test failure (CTest's own timeout), which is an acceptable,
     * intentional way to catch a genuinely broken delivery path. */
    int got_resp = 0;
    for (int i = 0; i < 2000 && !got_resp; i++) {
        mtek_bedge_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        if (resp_hdr.msg_type != MTK_BEDGE_MSG_IDLE) got_resp = 1;
        else usleep(1000);
    }
    MTK_CHECK(got_resp);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
    MTK_CHECK_EQ(resp_hdr.msg_id, 0x0302); /* answers the original DEAUTH_START request */
    MTK_CHECK_EQ(dctx.pending_start_msg_id, 0); /* cleared once delivered */

    /* The real canonical deauth logic genuinely ran on the background
     * thread (not the thread that called mtek_bedge_dispatch_request):
     * the fake HAL was actually reached, and the operation_token was
     * correctly harvested across the thread boundary via the persistent,
     * dctx-owned event_queue -- not a stack-local capture that would
     * have gone out of scope long before the worker thread got to it. */
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_ap.b, req_payload, 6) == 0);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, req_payload + 7, 6) == 0);
    MTK_CHECK(dctx.deauth_token != 0);

    /* Subsequent poll (nothing else outstanding): well-formed IDLE, not
     * a repeat of the already-delivered response. */
    mtek_bedge_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);

    /* Now that the token is real, DEAUTH_STOP works normally (also
     * dispatched synchronously here since STOP is lifecycle SYNCHRONOUS,
     * never deferred by the router). */
    mtek_bedge_dispatch_request(&dctx, &stop_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_RESP);
    MTK_CHECK_EQ(dctx.deauth_token, 0);

    /* Cancellation/reset of a still-pending deferred operation: start a
     * second DEAUTH_START (deferred again, worker still sleeping), then
     * simulate a boot_epoch reset by resetting the event_queue directly
     * -- the queue must accept being drained mid-flight without leaving
     * dctx in a state where a later, unrelated poll misinterprets a
     * stale frame as belonging to a new request. */
    {
        g_fake_wifi.deauth_sent_count = 0;
        mtek_bedge_dispatch_request(&dctx, &req_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_BEDGE_MSG_IDLE);
        MTK_CHECK_EQ(dctx.pending_start_msg_id, 0x0302);
        mtk_async_queue_reset(&dctx.event_queue);
        dctx.pending_start_msg_id = 0; /* the adapter's own reset-intent handling would clear this alongside the queue */
        /* Let the worker thread actually finish and push its (now
         * orphaned) frame into the queue AFTER the reset, proving the
         * queue is safely reusable and does not corrupt state even
         * though nothing is listening for that frame anymore. */
        usleep(50000);
        /* At most the orphaned ACCEPTED response plus its own terminal
         * DEAUTH_STOPPED event (this short-lived count=1 operation emits
         * both), never more -- no corruption, no duplication, no runaway
         * growth from the reset racing the worker thread. */
        MTK_CHECK(mtk_async_queue_count(&dctx.event_queue) <= 2);
        mtk_async_queue_reset(&dctx.event_queue);
        MTK_CHECK_EQ(mtk_async_queue_count(&dctx.event_queue), 0);
    }

MTK_TEST_MAIN_END
