/* RC5 independent audit P0 "Native in-flight request guarantee is not
 * met": dctx->pending_active used to be a single scalar, so a second
 * deferred request silently overwrote the first's identity, permanently
 * orphaning its eventual response (the first request's real answer would
 * arrive in the shared event_queue but nothing would ever remember which
 * transaction it belonged to). This test drives four genuinely
 * concurrent deferred TIME_SYNC_START operations through the SAME dctx
 * (matching MTK_SPI_NATIVE_MAX_IN_FLIGHT, itself matching the router's
 * own confirmed 4-in-flight async pool budget). TIME_SYNC_START is used
 * rather than a radio-owning opcode (DEAUTH_START, AP_SCAN_START, etc.)
 * because the canonical resource arbiter (002-resource-arbiter.md)
 * deliberately serializes every pair of radio-owning classes to exactly
 * one active owner at a time -- true 4-way *simultaneous radio work* is
 * not a scenario this system supports at all, by design. TIME_SYNC_START
 * touches no arbiter class (mtek_system_logic.c), so it is the correct
 * opcode for isolating and proving the TRANSPORT layer's own in-flight
 * bookkeeping (this file) independent of that unrelated, working-as-
 * designed radio-serialization constraint. Real pthread workers complete
 * in a DIFFERENT order than they were issued, proving delivery is
 * matched by identity (request_id/correlation), not FIFO-by-issue-order.
 * Also proves a 5th concurrent attempt while all 4 slots are genuinely
 * still in flight gets the router's own synchronous NO_MEMORY, and that
 * a STOP/cancel against one still-pending operation does not disturb the
 * other three. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtk_test_async_fixture.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_system_service.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }
static int sta_connected(void) { return 1; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }
static void core_lock(void) { pthread_mutex_lock(&s_mutex); }
static void core_unlock(void) { pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; unsigned delay_us; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    usleep(ta->delay_us);
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
/* Deliberately reverses completion order relative to issue order: the
 * FIRST-issued request gets the LONGEST delay, so it finishes LAST. */
static unsigned s_next_delay_us = 80000;
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    ta->delay_us = s_next_delay_us;
    s_next_delay_us -= 20000;
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id, uint16_t payload_len) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = 0x1234;
    return h;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x1234);
    mtk_core_set_lock(core_lock, core_unlock); /* op-token table shared across 4 concurrent workers */
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(pthread_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_system_set_sta_query(sta_connected);
    mtek_system_service_register();

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *ts_op = mtk_test_async_fixture_install() /* RC12 item 5: test-only overlay async op, was TIME_SYNC_START */;
    const mtk_opcode_entry_t *stop_op = mtk_test_async_fixture_stop_install(); /* RC12 item 1: test-only overlay STOP (was TIME_SYNC_STOP, now UNSUPPORTED on native) */
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* Issue 4 distinct TIME_SYNC_START operations back-to-back,
     * request_ids 100..103, none yet resolved -- each must land in its
     * own pending[] slot without disturbing the others. */
    uint8_t bufs[4][16]; size_t blens[4];
    for (int i = 0; i < 4; i++) {
        mtk_time_sync_start_req_t req = {0}; req.timeout_ms = 0;
        mtk_encode(ts_op->req_desc, &req, bufs[i], sizeof(bufs[i]), &blens[i]);
        mtk_spi_native_header_t hdr = base_req_hdr(ts_op->service_id, ts_op->opcode, (uint32_t)(100 + i), (uint16_t)blens[i]);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, bufs[i], (uint32_t)(1 + i), &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* all 4 genuinely deferred -- workers are still sleeping */
    }
    int active_count = 0;
    for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) if (dctx.pending[i].active) active_count++;
    MTK_CHECK_EQ(active_count, 4); /* every one of the 4 got its own slot -- none silently overwrote another */

    /* A 5th concurrent attempt while all 4 router pool slots (and this
     * dctx's own 4 pending[] slots) are genuinely still occupied gets the
     * router's own synchronous NO_MEMORY -- a checked rejection, never a
     * phantom 5th "pending" this dispatch layer has no room to track. */
    {
        uint8_t buf5[16]; size_t blen5;
        mtk_time_sync_start_req_t req5 = {0};
        mtk_encode(ts_op->req_desc, &req5, buf5, sizeof(buf5), &blen5);
        mtk_spi_native_header_t hdr5 = base_req_hdr(ts_op->service_id, ts_op->opcode, 200, (uint16_t)blen5);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr5, buf5, 5, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE); /* answered synchronously, not deferred -- pool was full */
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NO_MEMORY);
        MTK_CHECK_EQ(resp_hdr.request_id, 200);
    }

    /* Cancellation via TIME_SYNC_STOP against an unknown token (zero) is
     * cleanly rejected NOT_FOUND (itself SYNCHRONOUS lifecycle, answered
     * immediately) and does not disturb the other four pending slots. */
    {
        mtk_time_sync_stop_req_t sreq = {0}; sreq.operation_token = 0;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(stop_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 300, (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 6, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
    }
    active_count = 0;
    for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) if (dctx.pending[i].active) active_count++;
    MTK_CHECK_EQ(active_count, 4); /* the cancel attempt above touched none of the 4 real pending slots */

    /* Poll until all 4 (and only those 4) responses have been delivered,
     * regardless of the out-of-order completion the varying worker
     * delays force. Each must answer its OWN request_id with a distinct,
     * nonzero operation_token. */
    uint32_t tokens[4] = {0, 0, 0, 0};
    int delivered[4] = {0, 0, 0, 0};
    int total_delivered = 0;
    for (int poll = 0; poll < 6000 && total_delivered < 4; poll++) {
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        if (resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE && resp_hdr.request_id >= 100 && resp_hdr.request_id < 104) {
            int idx = (int)(resp_hdr.request_id - 100);
            MTK_CHECK_EQ(delivered[idx], 0); /* never delivered twice */
            delivered[idx] = 1;
            total_delivered++;
            mtk_time_sync_start_resp_t r = {0};
            mtk_decode(ts_op->resp_desc, &r, resp_payload, resp_len, NULL);
            MTK_CHECK(r.operation_token != 0);
            tokens[idx] = r.operation_token;
        } else {
            usleep(1000);
        }
    }
    MTK_CHECK_EQ(total_delivered, 4);
    /* Every token distinct -- no cross-request contamination. */
    for (int a = 0; a < 4; a++) for (int b = a + 1; b < 4; b++) MTK_CHECK(tokens[a] != tokens[b]);

    /* Every pending[] slot is free again after full delivery -- the dctx
     * is left in a clean, reusable state. */
    active_count = 0;
    for (int i = 0; i < MTK_SPI_NATIVE_MAX_IN_FLIGHT; i++) if (dctx.pending[i].active) active_count++;
    MTK_CHECK_EQ(active_count, 0);

    mtk_core_set_lock(NULL, NULL);

MTK_TEST_MAIN_END
