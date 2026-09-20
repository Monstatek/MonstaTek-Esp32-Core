/* Native SPI v1 multi-cell fragmentation/reassembly (SPI_PROTOCOL_V1.md):
 * bounded memory, sequence validation (gaps/duplicates/orphan
 * fragments), overflow, timeout, and round-trip outbound fragmentation
 * at exact cell-count boundaries. */
#include "mtk_test.h"
#include "mtek_spi_native_frame.h"
#include <string.h>

static mtk_spi_native_header_t base_hdr(uint32_t request_id) {
    mtk_spi_native_header_t h; memset(&h, 0, sizeof(h));
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = 1; h.minor = 0;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.service = 1; h.opcode = 1; h.request_id = request_id; h.boot_epoch = 0xABCD;
    return h;
}

MTK_TEST_MAIN_BEGIN

    /* Basic multi-fragment reassembly, in order ------------ */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t part1[900], part2[900], part3[200];
        for (int i = 0; i < 900; i++) part1[i] = (uint8_t)i;
        for (int i = 0; i < 900; i++) part2[i] = (uint8_t)(i + 1);
        for (int i = 0; i < 200; i++) part3[i] = (uint8_t)(i + 2);
        uint32_t total = 900 + 900 + 200;

        mtk_spi_native_header_t h1 = base_hdr(42);
        h1.flags = MTK_SPI_FLAG_FIRST; h1.payload_len = 900; h1.message_len = total; h1.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1, part1, 1), MTK_SPI_REASM_IN_PROGRESS);
        MTK_CHECK_EQ(ctx.received_len, 900);

        mtk_spi_native_header_t h2 = base_hdr(42);
        h2.flags = 0; h2.payload_len = 900; h2.message_len = total; h2.fragment_offset = 900;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, part2, 2), MTK_SPI_REASM_IN_PROGRESS);
        MTK_CHECK_EQ(ctx.received_len, 1800);

        mtk_spi_native_header_t h3 = base_hdr(42);
        h3.flags = MTK_SPI_FLAG_LAST; h3.payload_len = 200; h3.message_len = total; h3.fragment_offset = 1800;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h3, part3, 3), MTK_SPI_REASM_COMPLETE);
        MTK_CHECK_EQ(ctx.received_len, total);
        MTK_CHECK(memcmp(ctx.data, part1, 900) == 0);
        MTK_CHECK(memcmp(ctx.data + 900, part2, 900) == 0);
        MTK_CHECK(memcmp(ctx.data + 1800, part3, 200) == 0);
        MTK_CHECK_EQ(ctx.service, 1); MTK_CHECK_EQ(ctx.opcode, 1);
    }

    /* Gap: a fragment_offset ahead of received_len is rejected ---- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[100]; memset(buf, 0xAA, sizeof(buf));
        mtk_spi_native_header_t h1 = base_hdr(1);
        h1.flags = MTK_SPI_FLAG_FIRST; h1.payload_len = 100; h1.message_len = 500; h1.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1, buf, 1), MTK_SPI_REASM_IN_PROGRESS);

        mtk_spi_native_header_t h2 = base_hdr(1);
        h2.flags = 0; h2.payload_len = 100; h2.message_len = 500; h2.fragment_offset = 300; /* skipped 100..300 */
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, buf, 2), MTK_SPI_REASM_GAP);
        MTK_CHECK_EQ(ctx.active, 0); /* abandoned, not left in a corrupt in-progress state */
    }

    /* Duplicate: a fragment_offset behind received_len is rejected -- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[100]; memset(buf, 0xAA, sizeof(buf));
        mtk_spi_native_header_t h1 = base_hdr(1);
        h1.flags = MTK_SPI_FLAG_FIRST; h1.payload_len = 100; h1.message_len = 500; h1.fragment_offset = 0;
        mtk_spi_native_reassembly_feed(&ctx, &h1, buf, 1);

        mtk_spi_native_header_t h2 = base_hdr(1);
        h2.flags = 0; h2.payload_len = 100; h2.message_len = 500; h2.fragment_offset = 50; /* re-sent overlapping range */
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, buf, 2), MTK_SPI_REASM_DUPLICATE);
        MTK_CHECK_EQ(ctx.active, 0);
    }

    /* Orphan fragment: a continuation with no matching start ------ */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[10]; memset(buf, 0, sizeof(buf));
        mtk_spi_native_header_t h = base_hdr(99);
        h.flags = 0; h.payload_len = 10; h.message_len = 100; h.fragment_offset = 50; /* no prior FIRST for request_id=99 */
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h, buf, 1), MTK_SPI_REASM_ORPHAN_FRAGMENT);

        /* A continuation for a DIFFERENT request_id than the one actually
         * in progress is equally an orphan (never silently merged into
         * the wrong message). */
        mtk_spi_native_header_t hs = base_hdr(1);
        hs.flags = MTK_SPI_FLAG_FIRST; hs.payload_len = 10; hs.message_len = 100; hs.fragment_offset = 0;
        mtk_spi_native_reassembly_feed(&ctx, &hs, buf, 1);
        mtk_spi_native_header_t wrong_id = base_hdr(2);
        wrong_id.flags = 0; wrong_id.payload_len = 10; wrong_id.message_len = 100; wrong_id.fragment_offset = 10;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &wrong_id, buf, 2), MTK_SPI_REASM_ORPHAN_FRAGMENT);
        MTK_CHECK_EQ(ctx.active, 1); /* the genuinely in-progress request_id=1 context is untouched by the orphan */
        MTK_CHECK_EQ(ctx.request_id, 1);
    }

    /* a FIRST fragment for a genuinely DIFFERENT, concurrent request_id must NOT
     * silently supersede/overwrite an actively in-progress reassembly -- that
     * was the original defect (two concurrent fragmented requests corrupting
     * each other with no signal to either peer). Regression test reproducing the
     * exact original failure mode: without this fix, feeding request_id=2's
     * FIRST fragment here would silently reset ctx to request_id=2, permanently
     * losing request_id=1's already- received first fragment. */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf1[10]; memset(buf1, 0x11, sizeof(buf1));
        mtk_spi_native_header_t h1 = base_hdr(1);
        h1.flags = MTK_SPI_FLAG_FIRST; h1.payload_len = 10; h1.message_len = 100; h1.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1, buf1, 1), MTK_SPI_REASM_IN_PROGRESS);
        MTK_CHECK_EQ(ctx.request_id, 1);
        MTK_CHECK_EQ(ctx.received_len, 10);

        uint8_t buf2[5]; memset(buf2, 0x22, sizeof(buf2));
        mtk_spi_native_header_t h2 = base_hdr(2);
        h2.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; h2.payload_len = 5; h2.message_len = 5; h2.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, buf2, 2), MTK_SPI_REASM_BUSY);
        /* request_id=1's in-progress reassembly is completely untouched:
         * still active, still the right id, still the right received_len,
         * still the right bytes. */
        MTK_CHECK_EQ(ctx.active, 1);
        MTK_CHECK_EQ(ctx.request_id, 1);
        MTK_CHECK_EQ(ctx.received_len, 10);
        MTK_CHECK(memcmp(ctx.data, buf1, 10) == 0);

        /* request_id=1 can still complete normally afterward -- BUSY did
         * not corrupt or abandon it. */
        uint8_t buf1b[90]; memset(buf1b, 0x33, sizeof(buf1b));
        mtk_spi_native_header_t h1b = base_hdr(1);
        h1b.flags = MTK_SPI_FLAG_LAST; h1b.payload_len = 90; h1b.message_len = 100; h1b.fragment_offset = 10;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1b, buf1b, 3), MTK_SPI_REASM_COMPLETE);
        MTK_CHECK_EQ(ctx.received_len, 100);

        /* COMPLETE does not itself clear ctx->active -- the real caller
         * (mtek_spi_native_dispatch_feed_cell) dispatches the completed
         * message out of ctx->data first, then explicitly resets, exactly
         * mirrored here. Once reset, request_id=2's FIRST fragment is
         * accepted normally. */
        mtk_spi_native_reassembly_reset(&ctx);
        MTK_CHECK_EQ(ctx.active, 0);
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, buf2, 4), MTK_SPI_REASM_COMPLETE);
        MTK_CHECK_EQ(ctx.request_id, 2);
    }

    /* A repeated FIRST fragment for the SAME request_id already in progress (a
     * legitimate resend/retry) still restarts that context -- only a DIFFERENT
     * concurrent request_id is rejected BUSY. ----- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[10]; memset(buf, 0x44, sizeof(buf));
        mtk_spi_native_header_t h1 = base_hdr(9);
        h1.flags = MTK_SPI_FLAG_FIRST; h1.payload_len = 10; h1.message_len = 100; h1.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1, buf, 1), MTK_SPI_REASM_IN_PROGRESS);

        mtk_spi_native_header_t h1_retry = base_hdr(9);
        h1_retry.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_RETRY; h1_retry.payload_len = 10; h1_retry.message_len = 100; h1_retry.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h1_retry, buf, 2), MTK_SPI_REASM_IN_PROGRESS);
        MTK_CHECK_EQ(ctx.request_id, 9);
        MTK_CHECK_EQ(ctx.received_len, 10);
    }

    /* Overflow: message_len beyond the reassembly ceiling ------- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[10]; memset(buf, 0, sizeof(buf));
        mtk_spi_native_header_t h = base_hdr(1);
        h.flags = MTK_SPI_FLAG_FIRST; h.payload_len = 10; h.message_len = MTK_SPI_NATIVE_MAX_MESSAGE + 1; h.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h, buf, 1), MTK_SPI_REASM_OVERFLOW);
        MTK_CHECK_EQ(ctx.active, 0);
    }

    /* Timeout: an abandoned in-flight reassembly is detectable ---- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[10]; memset(buf, 0, sizeof(buf));
        mtk_spi_native_header_t h = base_hdr(1);
        h.flags = MTK_SPI_FLAG_FIRST; h.payload_len = 10; h.message_len = 100; h.fragment_offset = 0;
        mtk_spi_native_reassembly_feed(&ctx, &h, buf, 100 /* now_seq */);
        MTK_CHECK_EQ(mtk_spi_native_reassembly_timed_out(&ctx, 150, 100), 0); /* only 50 elapsed, under the 100 threshold */
        MTK_CHECK_EQ(mtk_spi_native_reassembly_timed_out(&ctx, 250, 100), 1); /* 150 elapsed, over threshold */
        mtk_spi_native_reassembly_reset(&ctx);
        MTK_CHECK_EQ(mtk_spi_native_reassembly_timed_out(&ctx, 999999, 100), 0); /* inactive context is never "timed out" */
    }

    /* Cancellation/reset mid-flight is safe and reusable --------- */
    {
        mtk_spi_native_reassembly_t ctx; mtk_spi_native_reassembly_reset(&ctx);
        uint8_t buf[10]; memset(buf, 0, sizeof(buf));
        mtk_spi_native_header_t h = base_hdr(1);
        h.flags = MTK_SPI_FLAG_FIRST; h.payload_len = 10; h.message_len = 100; h.fragment_offset = 0;
        mtk_spi_native_reassembly_feed(&ctx, &h, buf, 1);
        MTK_CHECK_EQ(ctx.active, 1);
        mtk_spi_native_reassembly_reset(&ctx); /* e.g. a boot_epoch reset invalidating everything in flight */
        MTK_CHECK_EQ(ctx.active, 0);
        /* Reusable for a fresh message afterward. */
        mtk_spi_native_header_t h2 = base_hdr(7);
        h2.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; h2.payload_len = 3; h2.message_len = 3; h2.fragment_offset = 0;
        MTK_CHECK_EQ(mtk_spi_native_reassembly_feed(&ctx, &h2, buf, 5), MTK_SPI_REASM_COMPLETE);
    }

    /* Outbound fragmentation: exact cell-count boundaries ------- */
    {
        static uint8_t pattern[3000];
        for (unsigned i = 0; i < sizeof(pattern); i++) pattern[i] = (uint8_t)(i * 3 + 1);

        const uint32_t sizes[] = { 0, 1, MTK_SPI_NATIVE_MAX_PAYLOAD, MTK_SPI_NATIVE_MAX_PAYLOAD + 1, 3000 };
        for (unsigned s = 0; s < sizeof(sizes)/sizeof(sizes[0]); s++) {
            uint32_t body_len = sizes[s];
            mtk_spi_native_header_t tmpl = base_hdr(0x55);
            tmpl.msg_class = MTK_SPI_CLASS_RESPONSE;
            mtk_spi_native_outbound_t ob;
            mtk_spi_native_outbound_start(&ob, &tmpl, pattern, body_len);

            uint32_t got = 0;
            unsigned cells = 0;
            mtk_spi_native_reassembly_t rctx; mtk_spi_native_reassembly_reset(&rctx);
            int more = 1;
            while (more) {
                uint8_t cell[MTK_SPI_NATIVE_CELL_SIZE];
                more = mtk_spi_native_outbound_next(&ob, cell);
                cells++;
                mtk_spi_native_header_t phdr; const uint8_t *ppayload;
                MTK_CHECK_EQ(mtk_spi_native_parse_cell(cell, &phdr, &ppayload), MTK_SPI_PARSE_OK);
                MTK_CHECK_EQ(phdr.request_id, 0x55);
                MTK_CHECK_EQ(phdr.message_len, body_len);
                mtk_spi_reasm_result_t rr = mtk_spi_native_reassembly_feed(&rctx, &phdr, ppayload, cells);
                if (phdr.flags & MTK_SPI_FLAG_LAST) {
                    MTK_CHECK_EQ(rr, MTK_SPI_REASM_COMPLETE);
                    got = rctx.received_len;
                } else {
                    MTK_CHECK_EQ(rr, MTK_SPI_REASM_IN_PROGRESS);
                }
            }
            MTK_CHECK_EQ(ob.active, 0);
            MTK_CHECK_EQ(got, body_len);
            if (body_len) MTK_CHECK(memcmp(rctx.data, pattern, body_len) == 0);

            unsigned expect_cells = body_len == 0 ? 1 : (body_len + MTK_SPI_NATIVE_MAX_PAYLOAD - 1) / MTK_SPI_NATIVE_MAX_PAYLOAD;
            MTK_CHECK_EQ(cells, expect_cells);
        }
    }

MTK_TEST_MAIN_END
