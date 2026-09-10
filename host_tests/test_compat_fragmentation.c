/* Legacy SPI Compatibility dispatch-layer OUTBOUND fragmentation: a response too large
 * for one 502-byte cell is staged and drained as FRAG cells followed by
 * one terminal RESP/NAK cell (mtek_compat_dispatch_poll_outbound), mirroring
 * mtk_compat_reassembly_feed's own inbound convention exactly. Exercises
 * the exact boundaries the accepted contract cares about: 0, 1, 502
 * (fits in one cell, no fragmentation), 503 (one byte over -> one FRAG +
 * one 1-byte terminal cell), and 4082 (the full reassembly ceiling). Also
 * covers the "malformed sequence" cases: polling with nothing staged,
 * and polling again after the terminal cell already drained. */
#include "mtk_test.h"
#include "mtek_compat_dispatch.h"
#include <string.h>

static void stage(mtk_compat_dispatch_ctx_t *dctx, uint16_t msg_id, uint8_t final_type, const uint8_t *data, uint16_t len) {
    memset(&dctx->outbound, 0, sizeof(dctx->outbound));
    dctx->outbound.active = 1;
    dctx->outbound.msg_id = msg_id;
    dctx->outbound.final_msg_type = final_type;
    dctx->outbound.total_len = len;
    dctx->outbound.sent_offset = 0;
    if (len) memcpy(dctx->outbound.data, data, len);
}

/* Drains dctx->outbound fully, reconstructing the transmitted bytes and
 * counting FRAG vs terminal cells; asserts every intermediate cell is
 * exactly the 502-byte ceiling (never a short FRAG). */
static void drain_and_check(mtk_compat_dispatch_ctx_t *dctx, uint16_t expect_msg_id, uint8_t expect_final_type,
                             const uint8_t *expect_data, uint16_t expect_len) {
    uint8_t reassembled[MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD];
    uint16_t got = 0;
    unsigned frag_cells = 0, terminal_cells = 0;
    for (unsigned guard = 0; guard < 32; guard++) {
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_poll_outbound(dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.magic, MTK_COMPAT_MAGIC);
        MTK_CHECK_EQ(resp_hdr.msg_id, expect_msg_id);
        if (resp_hdr.msg_type == MTK_COMPAT_MSG_FRAG) {
            frag_cells++;
            MTK_CHECK_EQ(resp_len, MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX); /* never a short FRAG */
            memcpy(reassembled + got, resp_payload, resp_len);
            got = (uint16_t)(got + resp_len);
        } else {
            MTK_CHECK_EQ(resp_hdr.msg_type, expect_final_type);
            terminal_cells++;
            memcpy(reassembled + got, resp_payload, resp_len);
            got = (uint16_t)(got + resp_len);
            break;
        }
    }
    MTK_CHECK_EQ(terminal_cells, 1);
    MTK_CHECK_EQ(got, expect_len);
    MTK_CHECK(memcmp(reassembled, expect_data, expect_len) == 0);
    MTK_CHECK_EQ(dctx->outbound.active, 0);

    unsigned expect_frags = expect_len > MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX
        ? (expect_len - 1) / MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX
        : 0;
    MTK_CHECK_EQ(frag_cells, expect_frags);
}

MTK_TEST_MAIN_BEGIN

    mtk_compat_dispatch_ctx_t dctx;
    memset(&dctx, 0, sizeof(dctx));

    static uint8_t pattern[MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD];
    for (unsigned i = 0; i < sizeof(pattern); i++) pattern[i] = (uint8_t)(i * 7 + 3);

    /* Polling with nothing staged: well-formed IDLE, not garbage. */
    {
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0xFFFF;
        mtek_compat_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(resp_hdr.payload_len, 0);
        MTK_CHECK_EQ(resp_len, 0);
    }

    /* 0 bytes: single terminal cell, zero-length payload -- no FRAG. */
    stage(&dctx, 0x1001, MTK_COMPAT_MSG_RESP, pattern, 0);
    drain_and_check(&dctx, 0x1001, MTK_COMPAT_MSG_RESP, pattern, 0);

    /* 1 byte: single terminal cell. */
    stage(&dctx, 0x1002, MTK_COMPAT_MSG_RESP, pattern, 1);
    drain_and_check(&dctx, 0x1002, MTK_COMPAT_MSG_RESP, pattern, 1);

    /* Exactly 502 bytes: still fits in one terminal cell, no FRAG at all. */
    stage(&dctx, 0x1003, MTK_COMPAT_MSG_NAK, pattern, MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX);
    drain_and_check(&dctx, 0x1003, MTK_COMPAT_MSG_NAK, pattern, MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX);

    /* 503 bytes: one full 502-byte FRAG cell + one 1-byte terminal cell. */
    stage(&dctx, 0x1004, MTK_COMPAT_MSG_RESP, pattern, MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX + 1);
    drain_and_check(&dctx, 0x1004, MTK_COMPAT_MSG_RESP, pattern, MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX + 1);

    /* Full 4082-byte reassembly ceiling: 8 full FRAG cells + one terminal
     * cell carrying the 4082 - 8*502 = 66-byte remainder. */
    stage(&dctx, 0x1005, MTK_COMPAT_MSG_RESP, pattern, MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD);
    drain_and_check(&dctx, 0x1005, MTK_COMPAT_MSG_RESP, pattern, MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD);
    MTK_CHECK_EQ((MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD - 1) / MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX, 8);

    /* Malformed sequence: polling again after the terminal cell already
     * drained must not repeat it or read past the buffer -- IDLE. */
    {
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0xFFFF;
        mtek_compat_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_IDLE);
        MTK_CHECK_EQ(resp_len, 0);
    }

    /* Malformed sequence: a fresh dispatch_request implicitly cancels any
     * still-active prior outbound (a real REQ always takes priority over
     * draining a stale response) -- proven via the public entry point at
     * the framing layer already (test_compat_framing.c); here we prove the
     * outbound struct itself never lets sent_offset exceed total_len even
     * under a pathological repeated-poll sequence. */
    stage(&dctx, 0x1006, MTK_COMPAT_MSG_RESP, pattern, 600);
    for (unsigned i = 0; i < 10; i++) {
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK(dctx.outbound.sent_offset <= dctx.outbound.total_len);
        if (!dctx.outbound.active) { MTK_CHECK_EQ(resp_hdr.msg_type, i == 1 ? MTK_COMPAT_MSG_RESP : MTK_COMPAT_MSG_IDLE); }
    }

MTK_TEST_MAIN_END
