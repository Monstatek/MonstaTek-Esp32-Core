/* And "epoch reset" (SPI_PROTOCOL_V1.md "Correlation and duplicate safety" /
 * "Reset and resynchronization"): proves the real three-stage duplicate- safety
 * lookup dispatch_complete_message performs for side-effecting (non-idempotent)
 * opcodes -- a still-active pending[] retry answers IDLE without re-dispatching,
 * a completed-response cache hit replays the cached response (same
 * operation_token, no fresh mtk_router_dispatch), a
 * same-request-ID/different-content collision is a protocol error, and a genuine
 * HELLO boot_epoch change invalidates the cache/pending table so a stale-epoch
 * request_id collision cannot resurrect old state. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
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

/* Deliberately DISTINCT from every peer/dctx epoch value this file uses (0x1234,
 * 0x5678) -- proves the ESP's own canonical epoch (mtk_core_boot_epoch) is used
 * for operation-token dispatch/lookup and ESP-originated wire stamping
 * regardless of what the peer's own HELLO epoch is or later changes to, closing
 * the exact gap the audit found: every host test previously used the SAME value
 * for the peer, the dctx seed, and mtk_core, which could never have caught this
 * class of bug. */
#define TEST_ESP_BOOT_EPOCH 0x99998888u

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }
static void queue_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void queue_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    usleep(100000); /* genuinely deferred: still pending when the test checks it */
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

static void poll_until_response(mtk_spi_native_dispatch_ctx_t *dctx, mtk_spi_native_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_len) {
    for (int i = 0; i < 3000; i++) {
        mtek_spi_native_dispatch_poll_outbound(dctx, resp_hdr, resp_payload, resp_len);
        if (resp_hdr->msg_class == MTK_SPI_CLASS_RESPONSE) return;
        usleep(1000);
    }
}

/* The worker thread's own ACCEPTED response is pushed BEFORE its send
 * loop runs (handle_deauth_start's own real shape: accept first, then
 * perform the actual sends) -- seeing ACCEPTED via poll_until_response
 * above proves the request was dispatched, never that the send loop
 * (running concurrently on that same worker thread, immediately
 * afterward) has already reached its own HAL call by the instant this
 * test's own polling thread notices the queued accept. Waits for the
 * real, observable side effect instead of assuming a timing coincidence
 * that happened to hold under normal/ASan scheduling but not under
 * heavier instrumentation (a real TSan run surfaced this exact gap). */
static void wait_for_deauth_count(unsigned expected) {
    /* Fallout: reads deauth_sent_count under the same real mutex
     * fake_wifi_send_deauth's own write now goes through
     * (mtk_fake_wifi_set_lock, registered above) -- a plain unsynchronized read
     * here would still be a genuine TSan-flagged race even after the write side
     * is locked, since nothing would establish a happens-before edge between
     * them otherwise. */
    for (int i = 0; i < 3000; i++) {
        fake_wifi_lock();
        unsigned count = g_fake_wifi.deauth_sent_count;
        fake_wifi_unlock();
        if (count >= expected) break;
        usleep(1000);
    }
}

static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id, uint16_t payload_len,
                                             uint32_t boot_epoch, uint8_t retry) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST | (retry ? MTK_SPI_FLAG_RETRY : 0);
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = boot_epoch;
    return h;
}

static mtk_spi_native_header_t hello_hdr(uint32_t boot_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_HELLO;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.boot_epoch = boot_epoch;
    return h;
}

static void encode_deauth(const mtk_opcode_entry_t *op, uint8_t channel, uint8_t *buf, size_t *blen) {
    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req.channel = channel; req.target_mode = 2 /* BROADCAST */;
    req.count = 1; req.interval_ms = 0;
    mtk_encode(op->req_desc, &req, buf, 128, blen);
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(TEST_ESP_BOOT_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_lock(router_lock, router_unlock);
    /* Verification fallout: a real TSan run against THIS test's own genuine
     * concurrency (the DEAUTH_STOP at the bottom of this block can race the
     * pthread_ runner-driven worker's own natural completion, both touching the
     * SAME operation record via mtk_op_claim_finalization/mtk_op_
     * transition_by_token) found this test never registered mtk_core_set_
     * lock/mtek_wifi_service_set_lock/mtk_fake_wifi_set_lock at all -- a real,
     * previously-undetected gap (this file predates deauth's own exactly-once
     * finalization fix using a shared claim/cleanup/ transition sequence that
     * made the SAME slot's state field a genuine cross-thread read+write target
     * here, not merely a single atomic mtk_op_transition call as before).
     * Mirrors test_deauth_continuous.c's own established pattern -- one real
     * mutex backing every one of these, matching mtk_router_set_lock's own
     * router_lock/router_unlock above. */
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

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);
    mtk_transport_counters_reset();

    const mtk_opcode_entry_t *deauth_op = mtk_test_find_op("DEAUTH_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("DEAUTH_STOP");
    MTK_CHECK_EQ(deauth_op->idempotent, 0); /* the whole test rests on DEAUTH_START being side-effecting */

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* Still-active pending[] retry: dispatched with a real async runner
     * registered (genuinely deferred, not yet delivered), a RETRY of the SAME
     * request_id/content while still pending answers IDLE -- never a fresh
     * dispatch (no second radio-side-effect, no second operation_token minted).
     * -------------------- */
    uint32_t real_token;
    {
        mtk_router_set_async_runner(pthread_runner);
        uint8_t buf[128]; size_t blen = 0;
        encode_deauth(deauth_op, 6, buf, &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)blen, 0x1234, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* genuinely deferred */
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0); /* worker has not run yet */

        /* RETRY of the identical request while still in flight: must NOT
         * re-dispatch (deauth_sent_count stays 0) -- answered IDLE, the
         * honest "still working on it, nothing new yet" reply. */
        mtk_spi_native_header_t retry_hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)blen, 0x1234, 1);
        mtek_spi_native_dispatch_feed_cell(&dctx, &retry_hdr, buf, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0); /* still not re-executed */

        mtk_transport_counters_t c1; mtk_transport_counters_get(&c1);
        MTK_CHECK_EQ(c1.retries_observed, 1);

        /* A DIFFERENT content under the SAME still-pending request_id is a
         * genuine protocol violation -- explicit LINK_ERROR, never
         * silently accepted or merged into the in-flight request. */
        uint8_t bad_buf[128]; size_t bad_blen = 0;
        encode_deauth(deauth_op, 9 /* different channel */, bad_buf, &bad_blen);
        mtk_spi_native_header_t mismatch_hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)bad_blen, 0x1234, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &mismatch_hdr, bad_buf, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0); /* the violation itself was never executed either */

        poll_until_response(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_deauth_start_resp_t r = {0};
        mtk_decode(deauth_op->resp_desc, &r, resp_payload, resp_len, NULL);
        MTK_CHECK(r.operation_token != 0);
        real_token = r.operation_token;
        wait_for_deauth_count(1);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* executed exactly once, by the ORIGINAL request */
    }

    /* Completed-response cache: a retry AFTER delivery is answered from the
     * cache -- SAME operation_token, no second radio send, and
     * duplicate_responses_served increments. -------------- */
    {
        uint8_t buf[128]; size_t blen = 0;
        encode_deauth(deauth_op, 6, buf, &blen);
        mtk_spi_native_header_t retry_hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)blen, 0x1234, 1);
        mtek_spi_native_dispatch_feed_cell(&dctx, &retry_hdr, buf, 4, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_deauth_start_resp_t r2 = {0};
        mtk_decode(deauth_op->resp_desc, &r2, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(r2.operation_token, real_token); /* the SAME operation, not a fresh one */
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* still exactly once -- no re-execution */

        mtk_transport_counters_t c2; mtk_transport_counters_get(&c2);
        MTK_CHECK_EQ(c2.duplicate_responses_served, 1);
        MTK_CHECK_EQ(c2.retries_observed, 2); /* this retry, plus the earlier still-pending one */

        /* Same request_id, different content, against the CACHE (not an
         * active pending[] slot this time): still a protocol error. */
        uint8_t bad_buf[128]; size_t bad_blen = 0;
        encode_deauth(deauth_op, 11, bad_buf, &bad_blen);
        mtk_spi_native_header_t mismatch_hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)bad_blen, 0x1234, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &mismatch_hdr, bad_buf, 5, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);

        /* Clean up: stop the real operation so it doesn't interfere below. */
        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = real_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 200, (uint16_t)stop_blen, 0x1234, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 6, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    }

    /* packet-sequence diagnostics: a skipped now_seq/packet_seq is counted as a
     * gap, a contiguous one is not. ------------ */
    {
        /* Directly exercise the packet_seq tracker embedded in dctx via a
         * CANCEL cell (any class reaching feed_cell notes packet_seq).
         * Every earlier REQUEST/HELLO in this test left packet_seq at its
         * struct-default 0, so establish a known-fresh baseline (c1) first
         * -- whether or not c1 itself registers as a gap relative to
         * whatever came before is not the property under test here. */
        mtk_spi_native_header_t c1; memset(&c1, 0, sizeof(c1));
        c1.magic = MTK_SPI_NATIVE_MAGIC; c1.major = MTK_SPI_NATIVE_MAJOR; c1.minor = MTK_SPI_NATIVE_MINOR;
        c1.msg_class = MTK_SPI_CLASS_CANCEL; c1.boot_epoch = 0x1234; c1.request_id = 0xABCDEF; c1.packet_seq = 500;
        mtek_spi_native_dispatch_feed_cell(&dctx, &c1, NULL, 7, &resp_hdr, resp_payload, &resp_len);
        mtk_transport_counters_t baseline; mtk_transport_counters_get(&baseline);

        mtk_spi_native_header_t c2 = c1;
        c2.packet_seq = 550; /* a real gap: not 501 */
        mtek_spi_native_dispatch_feed_cell(&dctx, &c2, NULL, 8, &resp_hdr, resp_payload, &resp_len);

        mtk_transport_counters_t after; mtk_transport_counters_get(&after);
        MTK_CHECK_EQ(after.packet_seq_gaps, baseline.packet_seq_gaps + 1);

        mtk_spi_native_header_t c_next = c1;
        c_next.packet_seq = 551; /* contiguous with c2 -- no new gap */
        mtek_spi_native_dispatch_feed_cell(&dctx, &c_next, NULL, 9, &resp_hdr, resp_payload, &resp_len);
        mtk_transport_counters_t after2; mtk_transport_counters_get(&after2);
        MTK_CHECK_EQ(after2.packet_seq_gaps, after.packet_seq_gaps);
    }

    /* epoch reset: a HELLO carrying a genuinely different boot_epoch invalidates
     * the duplicate cache -- request_id 100 (already retired into the cache
     * above) is no longer recognized as a duplicate under the NEW epoch, so an
     * identical-looking request now dispatches as a brand-new operation (a
     * different operation_token) instead of incorrectly replaying stale
     * cross-epoch cached data. ------- */
    {
        mtk_spi_native_header_t h = hello_hdr(0x5678);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 10, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
        /* HELLO_ACK always stamps the ESP's OWN fixed epoch (mtk_core_
         * boot_epoch, unaffected by mtk_core_reset since nothing here calls it)
         * -- NEVER the peer's own epoch, even immediately after adopting a
         * genuinely new PEER epoch (0x1234 -> 0x5678) above. Proves the two
         * epoch concepts stay independent. */
        MTK_CHECK_EQ(resp_hdr.boot_epoch, TEST_ESP_BOOT_EPOCH);

        uint8_t buf[128]; size_t blen = 0;
        encode_deauth(deauth_op, 6, buf, &blen);
        mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, 100, (uint16_t)blen, 0x5678, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, buf, 11, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* genuinely re-dispatched and deferred, not answered from the invalidated cache */

        poll_until_response(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
        mtk_deauth_start_resp_t r3 = {0};
        mtk_decode(deauth_op->resp_desc, &r3, resp_payload, resp_len, NULL);
        MTK_CHECK(r3.operation_token != real_token); /* a genuinely NEW operation, proving no stale-epoch cache hit occurred */
        wait_for_deauth_count(2);
        MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 2); /* the new epoch's request really executed */

        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = r3.operation_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 400, (uint16_t)stop_blen, 0x5678, 0);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 12, &resp_hdr, resp_payload, &resp_len);
    }

MTK_TEST_MAIN_END
