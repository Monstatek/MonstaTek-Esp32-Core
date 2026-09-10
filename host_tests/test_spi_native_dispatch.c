/* Native SPI v1 dispatch (SPI_PROTOCOL_V1.md): near-direct passthrough
 * into the canonical router, since the native wire header carries
 * service/opcode/status directly and the payload IS the canonical wire
 * encoding -- proven end-to-end against a real opcode (PING), the
 * IDLE/HELLO edge cases, a genuinely multi-cell (fragmented) PING
 * request reaching the real canonical handler through real reassembly,
 * a multi-cell response drained via poll_outbound, and a malformed
 * fragment sequence correctly rejected with LINK_ERROR. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, MTK_TEST_BOOT_EPOCH);

    const mtk_opcode_entry_t *ping_op = mtk_test_find_op("PING");
    mtk_ping_req_t preq = {0}; preq.nonce = 0xCAFEBABE;
    uint8_t req_payload[8]; size_t req_len = 0;
    mtk_encode(ping_op->req_desc, &preq, req_payload, sizeof(req_payload), &req_len);

    mtk_spi_native_header_t hdr; memset(&hdr, 0, sizeof(hdr));
    hdr.magic = MTK_SPI_NATIVE_MAGIC; hdr.major = MTK_SPI_NATIVE_MAJOR; hdr.minor = MTK_SPI_NATIVE_MINOR;
    hdr.msg_class = MTK_SPI_CLASS_REQUEST;
    hdr.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    hdr.service = ping_op->service_id; hdr.opcode = ping_op->opcode;
    hdr.payload_len = (uint16_t)req_len; hdr.message_len = (uint32_t)req_len;
    hdr.request_id = 42; hdr.packet_seq = 1; hdr.boot_epoch = MTK_TEST_BOOT_EPOCH;

    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &hdr, req_payload, 1, &resp_hdr, resp_payload, &resp_len);

    MTK_CHECK_EQ(resp_hdr.magic, MTK_SPI_NATIVE_MAGIC);
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(resp_hdr.request_id, 42); /* echoed, correlating request/response */
    mtk_ping_resp_t presp = {0};
    mtk_decode(ping_op->resp_desc, &presp, resp_payload, resp_len, NULL);
    MTK_CHECK_EQ(presp.nonce, 0xCAFEBABE); /* the real canonical PING handler ran, not a stub */

    /* IDLE class in -> IDLE class out, no crash, no fabricated response. */
    {
        mtk_spi_native_header_t idle_hdr = hdr; idle_hdr.msg_class = MTK_SPI_CLASS_IDLE; idle_hdr.payload_len = 0; idle_hdr.message_len = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &idle_hdr, NULL, 2, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
        MTK_CHECK_EQ(resp_len, 0);
    }

    /* HELLO -> HELLO_ACK. */
    {
        mtk_spi_native_header_t hello_hdr = hdr; hello_hdr.msg_class = MTK_SPI_CLASS_HELLO; hello_hdr.payload_len = 0; hello_hdr.message_len = 0;
        mtek_spi_native_dispatch_feed_cell(&dctx, &hello_hdr, NULL, 3, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    }

    /* A genuinely multi-cell (fragmented) PING request: split into two
     * physical cells, reassembled for real, and dispatched to the real
     * canonical handler -- not merely accepted at the framing layer. */
    {
        uint8_t part1[4], part2[4];
        memcpy(part1, req_payload, 4);
        memcpy(part2, req_payload + 4, 4);

        mtk_spi_native_header_t f1 = hdr;
        f1.flags = MTK_SPI_FLAG_FIRST; f1.payload_len = 4; f1.message_len = 8; f1.fragment_offset = 0; f1.request_id = 77;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f1, part1, 10, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE); /* still accumulating */

        mtk_spi_native_header_t f2 = hdr;
        f2.flags = MTK_SPI_FLAG_LAST; f2.payload_len = 4; f2.message_len = 8; f2.fragment_offset = 4; f2.request_id = 77;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f2, part2, 11, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_ping_resp_t presp2 = {0};
        mtk_decode(ping_op->resp_desc, &presp2, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(presp2.nonce, 0xCAFEBABE); /* the real handler ran against the fully reassembled payload */
    }

    /* A malformed fragment sequence (a gap) is rejected with LINK_ERROR,
     * never a silent partial decode. */
    {
        mtk_spi_native_header_t f1 = hdr;
        f1.flags = MTK_SPI_FLAG_FIRST; f1.payload_len = 4; f1.message_len = 8; f1.fragment_offset = 0; f1.request_id = 88;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f1, req_payload, 20, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);

        mtk_spi_native_header_t f2 = hdr;
        f2.flags = MTK_SPI_FLAG_LAST; f2.payload_len = 4; f2.message_len = 8; f2.fragment_offset = 6 /* gap: expected 4 */; f2.request_id = 88;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f2, req_payload, 21, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_len, 0);
    }

    /* RC6 independent audit P0 "Native reassembly and duplicate safety are
     * materially incomplete": a second logical message's FIRST fragment
     * arriving while another is still mid-reassembly is rejected
     * (LINK_ERROR/BUSY) at the transport layer, and -- the actual
     * regression this test reproduces -- the first message's in-progress
     * reassembly survives completely intact and dispatches correctly
     * afterward. Without this fix, the second FIRST fragment would
     * silently reset dctx->inbound to itself, permanently losing the
     * first message's already-received bytes with no error to either
     * side. */
    {
        uint8_t part1[4]; memcpy(part1, req_payload, 4);
        mtk_spi_native_header_t f1 = hdr;
        f1.flags = MTK_SPI_FLAG_FIRST; f1.payload_len = 4; f1.message_len = 8; f1.fragment_offset = 0; f1.request_id = 200;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f1, part1, 30, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);

        uint8_t intruder[4] = {1, 2, 3, 4};
        mtk_spi_native_header_t f_other = hdr;
        f_other.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; f_other.payload_len = 4; f_other.message_len = 4;
        f_other.fragment_offset = 0; f_other.request_id = 201;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f_other, intruder, 31, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_LINK_ERROR);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_BUSY);
        MTK_CHECK_EQ(dctx.inbound.active, 1);
        MTK_CHECK_EQ(dctx.inbound.request_id, 200);
        MTK_CHECK_EQ(dctx.inbound.received_len, 4);

        /* request_id=200 still completes correctly, dispatching against
         * its own (uncorrupted) reassembled bytes. */
        uint8_t part2[4]; memcpy(part2, req_payload + 4, 4);
        mtk_spi_native_header_t f2 = hdr;
        f2.flags = MTK_SPI_FLAG_LAST; f2.payload_len = 4; f2.message_len = 8; f2.fragment_offset = 4; f2.request_id = 200;
        mtek_spi_native_dispatch_feed_cell(&dctx, &f2, part2, 32, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_RESPONSE);
        MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
        mtk_ping_resp_t presp3 = {0};
        mtk_decode(ping_op->resp_desc, &presp3, resp_payload, resp_len, NULL);
        MTK_CHECK_EQ(presp3.nonce, 0xCAFEBABE);
    }

    /* A response too large for one cell is delivered as a real multi-cell
     * sequence, drained via poll_outbound -- not a LINK_ERROR/OVERFLOW. */
    {
        /* No opcode in this session's implemented canonical set has a
         * response that both (a) genuinely exceeds 984 bytes and (b) is
         * reachable through a real fake-HAL-driven service call (the one
         * opcode whose facts describe an over-one-cell response,
         * CAPTIVE_PORTAL_GET_CREDENTIALS, is not implemented at the
         * canonical service layer this session -- docs/PROVENANCE.md).
         * To still exercise the real multi-cell drain sequence
         * deterministically, this drives a large response synthetically
         * through the same outbound primitive dispatch_complete_message
         * itself uses, proving mtek_spi_native_dispatch_poll_outbound's
         * own physical drain behavior end-to-end. */
        static uint8_t big[3000];
        for (unsigned i = 0; i < sizeof(big); i++) big[i] = (uint8_t)(i * 7 + 5);
        mtk_spi_native_header_t tmpl; memset(&tmpl, 0, sizeof(tmpl));
        tmpl.magic = MTK_SPI_NATIVE_MAGIC; tmpl.major = MTK_SPI_NATIVE_MAJOR; tmpl.minor = MTK_SPI_NATIVE_MINOR;
        tmpl.msg_class = MTK_SPI_CLASS_RESPONSE; tmpl.service = 99; tmpl.opcode = 1; tmpl.request_id = 55; tmpl.status = MTK_STATUS_OK;
        mtk_spi_native_outbound_start(&dctx.outbound, &tmpl, big, sizeof(big));

        uint8_t reassembled[3000]; uint32_t got = 0;
        int more = 1; unsigned cells = 0;
        while (more) {
            mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK(resp_hdr.msg_class == MTK_SPI_CLASS_RESPONSE);
            memcpy(reassembled + got, resp_payload, resp_len);
            got += resp_len;
            more = dctx.outbound.active;
            cells++;
            MTK_CHECK(cells < 10); /* guard against a runaway loop */
        }
        MTK_CHECK_EQ(got, sizeof(big));
        MTK_CHECK(memcmp(reassembled, big, sizeof(big)) == 0);

        /* Polling again once fully drained: well-formed IDLE. */
        mtek_spi_native_dispatch_poll_outbound(&dctx, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_IDLE);
    }

MTK_TEST_MAIN_END
