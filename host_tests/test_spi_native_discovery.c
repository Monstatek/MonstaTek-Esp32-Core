/* RC5 independent audit P0 "Native SPI discovery drops the first
 * request": main/mtek_spi_runtime.c's physical loop bumped the
 * transaction cell size to the negotiated 1024-byte steady state in the
 * SAME iteration that recognizes the discovery HELLO, before the
 * HELLO_ACK answering that HELLO has actually gone out to the peer --
 * which the master, having only just sent a 512-byte discovery HELLO and
 * not yet seen any acknowledgement, still expects at the 512-byte
 * cadence. The fix (deferring the cell-size upgrade by one further
 * transaction) lives in main/mtek_spi_runtime.c, which is ESP-IDF-target-
 * only and cannot be host-built/tested (see docs/TEST_MATRIX.md's own
 * disclosed gap) -- the physical spi_slave transaction timing itself is
 * therefore not something this suite can exercise end-to-end without
 * hardware.
 *
 * What IS host-testable, and proven here byte-exact, is the content-
 * correctness half of the fix: mtek_spi_native_dispatch_feed_cell (the
 * portable dispatch layer the runtime loop calls into for that very
 * HELLO, in the same physical transaction it was received in -- see the
 * runtime loop's own comment) correctly recognizes and answers a HELLO
 * cell with a real HELLO_ACK, even when the cell it is fed lives in a
 * buffer sized to the 512-byte discovery transaction the master actually
 * used to send it -- exactly the runtime's own call shape for the first
 * cold-boot HELLO. */
#include "mtk_test.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_compat_frame.h"
#include "mtek_schema_constants.h"
#include "mtek_core.h"
#include <string.h>

#define MTK_DISCOVERY_CELL_SIZE MTK_COMPAT_CELL_SIZE /* 512: shared discovery-phase transaction size */

/* Release-tooling-round P0 correction (independent audit, "Native SPI
 * confuses the STM32 and ESP boot epochs"): deliberately DISTINCT from the
 * peer's own HELLO epoch (0x1234) below -- proves HELLO_ACK stamps the
 * ESP's own canonical epoch (mtk_core_boot_epoch()), never the peer's. */
#define TEST_ESP_BOOT_EPOCH 0x77776666u

MTK_TEST_MAIN_BEGIN

    mtk_core_init(TEST_ESP_BOOT_EPOCH);

    /* Build a HELLO cell the way the physical loop's own rxbuf would
     * hold it after a 512-byte discovery transaction: a
     * MTK_SPI_NATIVE_CELL_SIZE (1024)-byte buffer whose first 512 bytes
     * were actually clocked by the master (a real, CRC-valid, zero-
     * payload HELLO), with the trailing 512 bytes left as whatever was
     * there before (zero, matching the runtime's own static buffer's
     * initial state) -- since the master's own transaction only clocked
     * 512 bytes, dispatch must not depend on anything beyond that. */
    uint8_t rxbuf[MTK_SPI_NATIVE_CELL_SIZE];
    memset(rxbuf, 0, sizeof(rxbuf));
    mtk_spi_native_header_t hello_hdr; memset(&hello_hdr, 0, sizeof(hello_hdr));
    hello_hdr.magic = MTK_SPI_NATIVE_MAGIC; hello_hdr.major = MTK_SPI_NATIVE_MAJOR; hello_hdr.minor = MTK_SPI_NATIVE_MINOR;
    hello_hdr.msg_class = MTK_SPI_CLASS_HELLO;
    hello_hdr.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    hello_hdr.boot_epoch = 0x1234;
    uint8_t full_cell[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&hello_hdr, NULL, full_cell);
    /* Only the first 512 bytes are "real" (what the master actually
     * clocked at the discovery cell size); the HELLO's own meaningful
     * content (40-byte header, CRC over bytes 0..35 since payload_len=0)
     * is entirely within that window. */
    memcpy(rxbuf, full_cell, MTK_DISCOVERY_CELL_SIZE);

    /* Recognition (mtk_transport_select.c's own job, already covered by
     * test_transport_select.c) confirms this buffer is a valid native
     * HELLO at the 512-byte discovery bound -- included here so this
     * test stands alone as the exact end-to-end shape the runtime loop's
     * AUTO block exercises. */
    mtk_spi_native_header_t bounded_hdr; const uint8_t *bounded_payload;
    MTK_CHECK_EQ(mtk_spi_native_parse_bounded(rxbuf, MTK_DISCOVERY_CELL_SIZE, &bounded_hdr, &bounded_payload), MTK_SPI_PARSE_OK);
    MTK_CHECK_EQ(bounded_hdr.msg_class, MTK_SPI_CLASS_HELLO);

    /* The runtime loop, having just recognized this exact rxbuf as a
     * native HELLO, immediately (same physical transaction, same
     * iteration -- see main/mtek_spi_runtime.c's own comment on this)
     * re-parses it with the full/unbounded parser and dispatches it --
     * proving that reparse+dispatch produces a correct, byte-exact
     * HELLO_ACK despite `rxbuf` only having 512 real bytes behind it. */
    mtk_spi_native_header_t full_hdr; const uint8_t *full_payload;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(rxbuf, &full_hdr, &full_payload), MTK_SPI_PARSE_OK);
    MTK_CHECK_EQ(full_hdr.msg_class, MTK_SPI_CLASS_HELLO);

    mtk_spi_native_dispatch_ctx_t dctx;
    mtek_spi_native_dispatch_init(&dctx, 0x1234);
    mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
    mtek_spi_native_dispatch_feed_cell(&dctx, &full_hdr, full_payload, 1, &resp_hdr, resp_payload, &resp_len);

    /* Exact expected first-two-transaction MISO content: transaction A's
     * MISO (whatever was queued before the HELLO arrived) is unchanged by
     * this call; the ACK produced here is what the runtime loop places
     * into txbuf for transaction B, still at the discovery cell size per
     * the deferred-upgrade fix. */
    MTK_CHECK_EQ(resp_hdr.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    MTK_CHECK_EQ(resp_hdr.status, MTK_STATUS_OK);
    MTK_CHECK_EQ(resp_len, 0); /* disclosed minimal HELLO_ACK: empty payload */
    /* HELLO_ACK stamps the ESP's OWN canonical epoch (mtk_core_boot_
     * epoch()), never the peer's HELLO epoch (0x1234, deliberately
     * different from TEST_ESP_BOOT_EPOCH here). */
    MTK_CHECK_EQ(resp_hdr.boot_epoch, TEST_ESP_BOOT_EPOCH);

    /* Byte-exact: building the ACK cell and re-parsing it round-trips
     * (CRC-valid, magic/version correct) -- the literal bytes that would
     * be clocked out to the master on transaction B. */
    uint8_t ack_cell[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&resp_hdr, resp_payload, ack_cell);
    mtk_spi_native_header_t reparsed; const uint8_t *reparsed_payload;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(ack_cell, &reparsed, &reparsed_payload), MTK_SPI_PARSE_OK);
    MTK_CHECK_EQ(reparsed.msg_class, MTK_SPI_CLASS_HELLO_ACK);
    /* And that ACK cell is itself valid even when only its first 512
     * bytes are actually clocked to the master at the still-512-byte
     * discovery cadence (the deferred-upgrade fix's whole point): the
     * bounded parser accepts the same first-512-byte prefix. */
    mtk_spi_native_header_t ack_bounded; const uint8_t *ack_bounded_payload;
    MTK_CHECK_EQ(mtk_spi_native_parse_bounded(ack_cell, MTK_DISCOVERY_CELL_SIZE, &ack_bounded, &ack_bounded_payload), MTK_SPI_PARSE_OK);
    MTK_CHECK_EQ(ack_bounded.msg_class, MTK_SPI_CLASS_HELLO_ACK);

MTK_TEST_MAIN_END
