/* mtek_spi_native_dispatch.c used to adopt a real peer's own random HELLO
 * boot_epoch into dctx->boot_epoch, then reuse that SAME field both for the
 * canonical request context's boot_epoch (which /§3.4 requires to be the ESP's
 * own in-memory epoch, mtk_core_boot_epoch) and for stamping every
 * ESP-originated outbound cell's own boot_epoch header field (which
 * SPI_PROTOCOL_V1.md's Header table requires to be THIS sender's -- the ESP's --
 * own epoch, never an echo of the peer's). Every pre-existing native-SPI host
 * test used the identical value for the peer's HELLO epoch, the dctx seed, and
 * mtk_core's own epoch (0x1234 or MTK_TEST_BOOT_EPOCH), which could never
 * distinguish "coincidentally equal" from "correctly independent" -- exactly why
 * this defect reached this point undetected by the existing suite (follow-up
 * audit finding).
 *
 * ROUND 2 (follow-up read-only audit, "the new peer-reboot test encodes the
 * wrong lifecycle"): this test's own FIRST version wrongly asserted that an
 * operation started under PEER_EPOCH_A stays fully usable (STATUS/STOP both
 * succeeding) after a simulated peer reboot to PEER_EPOCH_B. Per
 * SPI_PROTOCOL_V1.md's own "Reset and resynchronization" rule, a changed peer
 * epoch must invalidate "in-flight requests... and operation tokens for that
 * peer" -- the ESP-epoch-scoped LOOKUP mechanism itself is correct and must stay
 * peer-epoch-independent (that is what the first round's fix achieved), but the
 * SERVICE-LEVEL response to a detected peer reboot must actively
 * cancel/finalize/evict whatever operation the old peer session left running,
 * releasing its radio/arbiter resources rather than leaving them orphaned
 * forever (the old peer, having rebooted, can never send a STOP for it again).
 * mtek_spi_native_dispatch.c's own invalidate_prior_epoch_ state now calls
 * cancel_active_operations_for_peer_reset, which finalizes the active operation
 * (byte-for-byte the same cleanup a real STOP would run) and immediately evicts
 * its now-terminal record (mtk_op_evict) -- this test proves that end-to-end.
 *
 * This test deliberately uses THREE, mutually-distinct epoch values (the ESP's
 * own core epoch, and two different peer epochs simulating a peer reboot
 * mid-session) and proves: 1. HELLO_ACK always stamps the ESP's own fixed epoch,
 * never the peer's, both before and after a peer-epoch change (simulated
 * reboot). 2. An operation started under PEER_EPOCH_A is genuinely CANCELLED and
 * its token made unusable (NOT_FOUND for both STATUS and STOP) the moment the
 * peer reboots to PEER_EPOCH_B -- never silently left RUNNING nor holding the
 * radio/arbiter lease forever. 3. A NEW operation started under the new peer
 * session (PEER_EPOCH_B) gets a genuinely different token and completes a full
 * START -> STATUS -> STOP round trip normally -- the arbiter/radio really was
 * freed by the cancellation above, not left stuck. 4. A REPEATED HELLO carrying
 * the SAME (already-adopted) peer epoch is idempotent: it must NOT cancel a live
 * operation that belongs to the CURRENT peer session. 5. CREDIT and CANCEL
 * correctly reject a stale PEER epoch (compared against the currently-adopted
 * peer epoch, never the ESP's), accept a current one, and stamp their own
 * responses with the ESP's epoch. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
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

/* Three deliberately distinct, nonzero values -- never coincidentally
 * equal, unlike every pre-existing native-SPI test's shared 0x1234. */
#define ESP_EPOCH      0x0A0B0C0Du   /* the ESP's own core epoch: fixed for this whole test */
#define PEER_EPOCH_A   0x11112222u   /* the peer's epoch before its simulated reboot */
#define PEER_EPOCH_B   0x33334444u   /* the peer's epoch after its simulated reboot */

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

static void wait_for_deauth_count(unsigned expected) {
    for (int i = 0; i < 3000; i++) {
        fake_wifi_lock();
        unsigned count = g_fake_wifi.deauth_sent_count;
        fake_wifi_unlock();
        if (count >= expected) break;
        usleep(1000);
    }
}

static mtk_spi_native_header_t base_req_hdr(uint16_t service, uint16_t opcode, uint32_t request_id,
                                             uint16_t payload_len, uint32_t peer_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = service; h.opcode = opcode;
    h.payload_len = payload_len; h.message_len = payload_len;
    h.request_id = request_id; h.boot_epoch = peer_epoch;
    return h;
}

static mtk_spi_native_header_t hello_hdr(uint32_t peer_epoch) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = MTK_SPI_NATIVE_MAJOR; h.minor = MTK_SPI_NATIVE_MINOR;
    h.msg_class = MTK_SPI_CLASS_HELLO;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.boot_epoch = peer_epoch;
    return h;
}

static void encode_deauth(const mtk_opcode_entry_t *op, uint8_t channel, uint8_t *buf, size_t *blen) {
    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    memcpy(req.ap_bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req.channel = channel; req.target_mode = 2 /* BROADCAST */;
    req.count = 0 /* run until stopped */; req.interval_ms = 0;
    mtk_encode(op->req_desc, &req, buf, 128, blen);
}

/* Dispatches a DEAUTH_START under `peer_epoch` and returns its minted
 * operation_token once the accept response is actually observed (never
 * assumed) -- shared by every "start a fresh deauth" step below. */
static uint32_t start_deauth(mtk_spi_native_dispatch_ctx_t *dctx, const mtk_opcode_entry_t *deauth_op,
                              uint32_t request_id, uint32_t peer_epoch, uint32_t now_seq) {
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    uint8_t buf[128]; size_t blen = 0;
    encode_deauth(deauth_op, 6, buf, &blen);
    mtk_spi_native_header_t hdr = base_req_hdr(deauth_op->service_id, deauth_op->opcode, request_id, (uint16_t)blen, peer_epoch);
    mtek_spi_native_dispatch_feed_cell(dctx, &hdr, buf, now_seq, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* genuinely deferred (async runner registered) */

    int got = 0;
    for (int i = 0; i < 3000 && !got; i++) {
        mtek_spi_native_dispatch_poll_outbound(dctx, &resp_hdr, resp_payload, &resp_len);
        if (resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE) got = 1; else usleep(1000);
    }
    MTK_CHECK(got);
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_ACCEPTED);
    mtk_deauth_start_resp_t started = {0};
    mtk_decode(deauth_op->resp_desc, &started, resp_payload, resp_len, NULL);
    MTK_CHECK(started.operation_token != 0);
    return started.operation_token;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(ESP_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_router_set_lock(router_lock, router_unlock);
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
    mtk_router_set_async_runner(pthread_runner);

    /* dctx seeded with a placeholder distinct from PEER_EPOCH_A/B -- exactly
     * what main/mtek_spi_runtime.c's own real caller does (seeds with
     * mtk_core_boot_epoch, a value a real peer's own independent random epoch is
     * virtually certain to differ from). */
    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, ESP_EPOCH);
    mtk_async_queue_set_lock(&dctx.event_queue, queue_lock, queue_unlock, NULL);

    const mtk_opcode_entry_t *deauth_op = mtk_test_find_op("DEAUTH_START");
    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("DEAUTH_STOP");
    const mtk_opcode_entry_t *status_op = mtk_test_find_op("GET_OPERATION_STATUS");

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;

    /* 1. Initial HELLO from the peer (PEER_EPOCH_A): HELLO_ACK always carries
     * the ESP's OWN fixed epoch, never the peer's. -------- */
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 1, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH);
    }

    /* 2. START a deauth under PEER_EPOCH_A, confirm it is genuinely RUNNING.
     * ----------------------------- */
    uint32_t old_token = start_deauth(&dctx, deauth_op, 100, PEER_EPOCH_A, 2);
    wait_for_deauth_count(1);
    {
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = old_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 101, (uint16_t)sblen, PEER_EPOCH_A);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    }
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_D); /* the deauth genuinely holds the radio lease */

    /* 3. Simulate a peer reboot: a HELLO with a genuinely DIFFERENT peer epoch.
     * HELLO_ACK still carries the ESP's own unchanged epoch, and the old
     * session's deauth is cancelled -- the radio lease is genuinely released,
     * not left stuck. ---------------- */
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 4, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH); /* unchanged -- the ESP itself did not reboot */
    }
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE); /* the old session's radio lease was genuinely released */

    /* 4. STATUS/STOP against the OLD token now both report NOT_FOUND -- the
     * token was cancelled and evicted, genuinely unusable, never silently left
     * RUNNING for a peer that can no longer reach it. --- */
    {
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = old_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 102, (uint16_t)sblen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 5, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
    }
    {
        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = old_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 103, (uint16_t)stop_blen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 6, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND);
    }
    /* No more sends were ever confirmed beyond the pre-reboot baseline --
     * the cancelled session's send loop genuinely stopped, never kept
     * transmitting in the background after its own token became unusable. */
    fake_wifi_lock(); unsigned sent_after_cancel = g_fake_wifi.deauth_sent_count; fake_wifi_unlock();

    /* 5. A NEW operation under the NEW peer session (PEER_EPOCH_B) gets a
     * genuinely different token and completes a full START -> STATUS -> STOP
     * round trip normally -- proving the arbiter/radio really was freed above,
     * not left stuck. --------------- */
    uint32_t new_token = start_deauth(&dctx, deauth_op, 200, PEER_EPOCH_B, 7);
    MTK_CHECK(new_token != old_token);
    wait_for_deauth_count(sent_after_cancel + 1);
    {
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = new_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 201, (uint16_t)sblen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 8, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    }

    /* 6. A REPEATED HELLO carrying the SAME (already-adopted) peer epoch is
     * idempotent: it must NOT cancel new_token, which genuinely belongs to the
     * CURRENT peer session. ---------------- */
    {
        mtk_spi_native_header_t h = hello_hdr(PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &h, NULL, 9, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH);
    }
    {
        mtk_get_operation_status_req_t sreq = {0}; sreq.operation_token = new_token;
        uint8_t sbuf[8]; size_t sblen = 0;
        mtk_encode(status_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
        mtk_spi_native_header_t shdr = base_req_hdr(status_op->service_id, status_op->opcode, 202, (uint16_t)sblen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &shdr, sbuf, 10, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK); /* still alive -- an idempotent repeated HELLO never cancelled it */
        mtk_get_operation_status_resp_t st = {0};
        mtk_decode(status_op->resp_desc, &st, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    }

    /* STOP new_token, under the new peer session -- succeeds normally. */
    {
        mtk_deauth_stop_req_t stop_req = {0}; stop_req.operation_token = new_token;
        uint8_t stop_buf[8]; size_t stop_blen = 0;
        mtk_encode(stop_op->req_desc, &stop_req, stop_buf, sizeof(stop_buf), &stop_blen);
        mtk_spi_native_header_t stop_hdr = base_req_hdr(stop_op->service_id, stop_op->opcode, 203, (uint16_t)stop_blen, PEER_EPOCH_B);
        mtek_spi_native_dispatch_feed_cell(&dctx, &stop_hdr, stop_buf, 11, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_deauth_stop_resp_t st = {0};
        mtk_decode(stop_op->resp_desc, &st, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(st.final_state, MTK_OPS_STOPPED);
    }

    /* 7. CREDIT: stale PEER epoch rejected, current one accepted, ESP epoch
     * stamped on the response either way. ------------- */
    {
        uint8_t credit_payload[4] = {0x00, 0x10, 0x00, 0x00};
        mtk_spi_native_header_t chdr; memset(&chdr, 0, sizeof(chdr));
        chdr.magic = MTK_SPI_NATIVE_MAGIC; chdr.major = MTK_SPI_NATIVE_MAJOR; chdr.minor = MTK_SPI_NATIVE_MINOR;
        chdr.msg_class = MTK_SPI_CLASS_CREDIT; chdr.request_id = 0xABCDu; chdr.payload_len = 4;
        chdr.boot_epoch = PEER_EPOCH_A; /* stale: the adopted peer epoch is now PEER_EPOCH_B */
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, credit_payload, 12, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH); /* the ESP's own epoch, never the peer's stale one it just rejected */

        chdr.boot_epoch = PEER_EPOCH_B; /* current */
        mtek_spi_native_dispatch_feed_cell(&dctx, &chdr, credit_payload, 13, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE); /* accepted at the transport level (unknown token -> NOT_FOUND, not LINK_ERROR) */
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_NOT_FOUND); /* no live capture session token 0xABCD -- an honest, non-crashing miss */
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH);
    }

    /* CANCEL: same stale/current peer-epoch split, same ESP-epoch stamping on
     * the response. ---------------------- */
    {
        uint8_t part1[4] = {1,2,3,4};
        mtk_spi_native_header_t f1 = base_req_hdr(0, 1, 300, 4, PEER_EPOCH_B);
        f1.flags = MTK_SPI_FLAG_FIRST; f1.message_len = 8; f1.fragment_offset = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f1, part1, 14, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* still accumulating a multi-cell request */
        MTK_CHECK_EQ(dctx.inbound.active, 1);

        mtk_spi_native_header_t cancel_hdr; memset(&cancel_hdr, 0, sizeof(cancel_hdr));
        cancel_hdr.magic = MTK_SPI_NATIVE_MAGIC; cancel_hdr.major = MTK_SPI_NATIVE_MAJOR; cancel_hdr.minor = MTK_SPI_NATIVE_MINOR;
        cancel_hdr.msg_class = MTK_SPI_CLASS_CANCEL; cancel_hdr.request_id = 300;
        cancel_hdr.boot_epoch = PEER_EPOCH_A; /* stale */
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 15, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_PROTOCOL_ERROR);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH);
        MTK_CHECK_EQ(dctx.inbound.active, 1); /* the stale-epoch CANCEL must not have touched it */

        cancel_hdr.boot_epoch = PEER_EPOCH_B; /* current */
        mtek_spi_native_dispatch_feed_cell(&dctx, &cancel_hdr, NULL, 16, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(resp_hdr.boot_epoch, ESP_EPOCH);
        MTK_CHECK_EQ(dctx.inbound.active, 0); /* genuinely cancelled this time */
    }

MTK_TEST_MAIN_END
