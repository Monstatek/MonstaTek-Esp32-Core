/* Count=0 ("run until stopped") used to convert into exactly one round-robin
 * pass and complete immediately. Proves, with a real pthread-based mtk_router
 * async runner (not a single-threaded simulation) driving
 * DEAUTH_START/STOP/STATUS through native SPI v1's own dispatch layer (whose
 * persistent, queue-backed sink is the established safe pattern for a genuinely
 * deferred operation -- see mtek_spi_native_dispatch.c), that count=0 now
 * genuinely keeps sending across many rounds until a concurrent DEAUTH_STOP on
 * this thread ends it -- with live progress visible via DEAUTH_STATUS while it
 * runs, and a real HAL-observed total that keeps growing well past a single
 * target-set pass. The synchronous (no real background execution context)
 * fallback is already covered by every existing single-threaded test_deauth_*.c
 * that uses count=0 without registering a runner -- proving that path still
 * completes deterministically rather than hanging is exactly the regression a
 * naive "just loop until terminal" fix would have reintroduced (see
 * mtek_router.c's mtk_router_running_on_worker and mtek_wifi_logic.c's own doc
 * comment on this). */
#include "mtk_test.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
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

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
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

static const mtk_opcode_entry_t *find_op(const char *name) {
    for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) if (strcmp(mtk_opcode_table[i].name, name) == 0) return &mtk_opcode_table[i];
    return NULL;
}

static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id, uint16_t payload_len) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = 0x2222;
    return h;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0x2222);
    /* This test's own real pthread worker (pthread_runner below) calls
     * mtk_op_transition concurrently with the main thread's own deauth-
     * status/-stop reads -- mtk_core_set_lock must be registered for that
     * module's own already-proven locking (test_core_concurrency.c) to actually
     * activate here too. */
    mtk_core_set_lock(router_lock, router_unlock);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_async_runner(pthread_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* A real ThreadSanitizer run against this exact test (which genuinely races
     * handle_deauth_start's own worker-thread progress writes against
     * handle_deauth_status's own concurrent reads of s_deauth) confirmed the
     * data race this registration closes -- see mtek_wifi_service.h's own doc
     * comment on mtek_wifi_service_set_lock. */
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    /* 's own TSan verification pass caught a real race on g_fake_wifi's own
     * prior-state/channel/mode fields between this test's worker thread
     * (fake_wifi_send_deauth) and the main thread's concurrent
     * fake_wifi_restore_sta_mode call -- see mtk_fake_wifi_hal.h's own doc
     * comment on mtk_fake_wifi_set_lock. */
    mtk_fake_wifi_set_lock(router_lock, router_unlock);
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x2222);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *start_op = find_op("DEAUTH_START");
    const mtk_opcode_entry_t *stop_op = find_op("DEAUTH_STOP");
    const mtk_opcode_entry_t *status_op = find_op("DEAUTH_STATUS");
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req.channel = 6; req.target_mode = 2 /* BROADCAST */; req.count = 0 /* run until stopped */; req.interval_ms = 0;
    uint8_t buf[128]; size_t blen = 0;
    mtk_encode(start_op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_spi_native_header_t hdr = base_req_hdr(start_op->service_id, start_op->opcode, 1, (uint16_t)blen);
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 1, &resp_hdr, resp_payload, &resp_len);

    /* Mtek_spi_
     * native_dispatch.c's own dispatch_complete_message (via its shared
     * try_deliver_frame helper) explicitly permits the just-dispatched
     * request to already be answered by the time THIS call returns -- "the
     * just-dispatched request already answered here -- fast/synchronous
     * completion" is that function's own documented, legitimate outcome,
     * not an implementation accident. Requiring IDLE unconditionally here
     * was therefore flaky by design (a fast worker thread can race its own
     * accept-response push ahead of this call's own final delivery check),
     * not merely rare under contention -- the follow-up audit reproduced it
     * directly (one full TSan-suite failure, 1 failure in 10 isolated repetitions).
     * Both protocol-legal outcomes are accepted below; every other
     * class/status is rejected, and if no valid token is obtained this
     * test fails cleanly with a single explicit early exit -- never by
     * falling through into the rest of the test body, which would spend
     * thousands of iterations re-asserting against an invalid all-zero
     * token (exactly the follow-up audit's own "cascading assertions" symptom: every
     * DEAUTH_STATUS call below would get NOT_FOUND instead of OK, in a
     * loop bounded at 4000 iterations). */
    uint32_t op_token = 0;
    if (resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE) {
        /* Fast path: the worker already finished and its own accept
         * response rode back on this same call -- decode it directly,
         * never poll for something that was already delivered here (and
         * therefore can never be found again via poll_outbound: try_
         * deliver_frame pops the queue entry exactly once). */
        if (resp_hdr.status == MTK_STATUS_ACCEPTED) {
            mtk_deauth_start_resp_t started = {0};
            mtk_decode(start_op->resp_desc, &started, resp_payload, resp_len, NULL);
            op_token = started.operation_token;
        } else {
            fprintf(stderr, "test_deauth_continuous: immediate RESPONSE has status %u, expected "
                            "MTK_STATUS_ACCEPTED (%u)\n", resp_hdr.status, MTK_STATUS_ACCEPTED);
            mtk_test_failures++;
        }
    } else if (resp_hdr.msg_class == MTK_SPI_CLASS_IDLE) {
        /* Genuinely deferred -- poll for the ACCEPTED accept response
         * (real cross-thread delivery, the already-proven pattern). */
        for (int i = 0; i < 3000 && op_token == 0; i++) {
            mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
            if (resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE) {
                if (resp_hdr.status != MTK_STATUS_ACCEPTED) {
                    fprintf(stderr, "test_deauth_continuous: polled RESPONSE has status %u, expected "
                                    "MTK_STATUS_ACCEPTED (%u)\n", resp_hdr.status, MTK_STATUS_ACCEPTED);
                    mtk_test_failures++;
                    break;
                }
                mtk_deauth_start_resp_t started = {0};
                mtk_decode(start_op->resp_desc, &started, resp_payload, resp_len, NULL);
                op_token = started.operation_token;
            } else {
                usleep(500);
            }
        }
    } else {
        fprintf(stderr, "test_deauth_continuous: initial dispatch returned msg_class %u, expected "
                        "IDLE (%u) or RESPONSE (%u)\n", resp_hdr.msg_class, MTK_SPI_CLASS_IDLE,
                        MTK_SPI_CLASS_RESPONSE);
        mtk_test_failures++;
    }

    if (op_token == 0) {
        fprintf(stderr, "test_deauth_continuous: no valid operation token obtained from DEAUTH_START -- "
                        "failing cleanly here instead of continuing with an invalid token\n");
        fprintf(stderr, "%d check(s) FAILED\n", mtk_test_failures ? mtk_test_failures : 1);
        return 1;
    }

    /* Let it genuinely run across several rounds -- broadcast mode has
     * exactly 1 target per round, so real elapsed sends here prove more
     * than the old "one round-robin pass" (which would have been exactly
     * 1 send total, already complete and STOPPED by this point). Poll
     * DEAUTH_STATUS (SYNCHRONOUS lifecycle -- answered directly, never
     * deferred) until it reports real, growing progress. */
    uint32_t sent_seen = 0;
    for (int i = 0; i < 4000 && sent_seen < 20; i++) {
        mtk_deauth_status_req_t sreq = {0}; sreq.operation_token = op_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, (uint32_t)(100 + i), (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, (uint32_t)(2 + i), &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_deauth_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        sent_seen = st.sent;
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING); /* still running on its own -- genuinely open-ended */
        if (sent_seen < 20) usleep(500);
    }
    MTK_CHECK(sent_seen >= 20); /* real progress well past a single-target-set pass (1 send) */

    /* STOP it from this thread while the worker is still looping. */
    mtk_deauth_stop_req_t stopreq = {0}; stopreq.operation_token = op_token;
    uint8_t stopbuf[8]; size_t stopblen = 0;
    mtk_encode(stop_op->req_desc, &stopreq, stopbuf, sizeof(stopbuf), &stopblen);
    mtk_spi_native_header_t stophdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 900, (uint16_t)stopblen);
    mtek_spi_native_dispatch_feed_cell(&dctx, &stophdr, stopbuf, 900, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE); /* SYNCHRONOUS -- answered immediately regardless of the worker */
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);

    /* The worker thread's loop must actually observe the STOP and exit --
     * give it a bounded window, then confirm the send count stops
     * growing. */
    usleep(80000);
    uint32_t count_after_stop_1;
    {
        mtk_deauth_status_req_t sreq = {0}; sreq.operation_token = op_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 901, (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 901, &resp_hdr, resp_payload, &resp_len);
        mtk_deauth_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        count_after_stop_1 = st.sent;
        MTK_CHECK(mtk_op_state_is_terminal((mtk_op_state_t)st.state));
    }
    usleep(50000);
    {
        mtk_deauth_status_req_t sreq = {0}; sreq.operation_token = op_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 902, (uint16_t)sblen);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 902, &resp_hdr, resp_payload, &resp_len);
        mtk_deauth_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(st.sent, count_after_stop_1); /* no further sends after STOP was observed */
    }

    /* Radio restoration ran (deterministic mode restore, RC5 P1) and the
     * arbiter's D-class lease was released. */
    MTK_CHECK(g_fake_wifi.restore_count >= 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

MTK_TEST_MAIN_END
