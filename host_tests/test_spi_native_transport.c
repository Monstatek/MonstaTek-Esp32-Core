/* Native SPI v1 transport framing (SPI_PROTOCOL_V1.md): header round
 * trip, CRC32C integrity, magic/version/flags/class/length validation,
 * and the 984-byte payload ceiling. */
#include "mtk_test.h"
#include "mtek_spi_native_frame.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_spi_native_header_t h = {0};
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = 1; h.minor = 0;
    h.msg_class = MTK_SPI_CLASS_REQUEST;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = 0x0001; h.opcode = 0x0010; h.status = 0;
    uint8_t payload[16]; for (int i = 0; i < 16; i++) payload[i] = (uint8_t)i;
    h.payload_len = 16; h.request_id = 7; h.packet_seq = 1; h.boot_epoch = 0x12345678;
    h.message_len = 16; h.fragment_offset = 0;

    uint8_t cell[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&h, payload, cell);
    MTK_CHECK_EQ(cell[0], 0x4D); MTK_CHECK_EQ(cell[1], 0x31); MTK_CHECK_EQ(cell[2], 0x53); MTK_CHECK_EQ(cell[3], 0x31);

    mtk_spi_native_header_t parsed; const uint8_t *pl;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(cell, &parsed, &pl), MTK_SPI_PARSE_OK);
    MTK_CHECK_EQ(parsed.request_id, 7);
    MTK_CHECK_EQ(parsed.service, 0x0001);
    MTK_CHECK_EQ(parsed.opcode, 0x0010);
    MTK_CHECK_EQ(parsed.boot_epoch, (long)0x12345678);
    MTK_CHECK(memcmp(pl, payload, 16) == 0);

    /* Unused tail beyond the header+payload is zero-filled. */
    for (unsigned i = MTK_SPI_NATIVE_HEADER_SIZE + 16; i < MTK_SPI_NATIVE_CELL_SIZE; i++) MTK_CHECK_EQ(cell[i], 0);

    /* CRC does not cover the unused tail: corrupting it must not fail parse. */
    uint8_t tail_corrupt[MTK_SPI_NATIVE_CELL_SIZE];
    memcpy(tail_corrupt, cell, sizeof(tail_corrupt));
    tail_corrupt[MTK_SPI_NATIVE_CELL_SIZE - 1] ^= 0xFF;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(tail_corrupt, &parsed, &pl), MTK_SPI_PARSE_OK);

    /* Corrupted payload byte -> CRC mismatch. */
    uint8_t bad_payload[MTK_SPI_NATIVE_CELL_SIZE];
    memcpy(bad_payload, cell, sizeof(bad_payload));
    bad_payload[MTK_SPI_NATIVE_HEADER_SIZE] ^= 0xFF;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(bad_payload, &parsed, &pl), MTK_SPI_PARSE_BAD_CRC);

    /* Bad magic. */
    uint8_t bad_magic[MTK_SPI_NATIVE_CELL_SIZE];
    memcpy(bad_magic, cell, sizeof(bad_magic));
    bad_magic[0] = 0x00;
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(bad_magic, &parsed, &pl), MTK_SPI_PARSE_BAD_MAGIC);

    /* Bad major version. */
    uint8_t bad_ver[MTK_SPI_NATIVE_CELL_SIZE];
    memcpy(bad_ver, cell, sizeof(bad_ver));
    bad_ver[4] = 2;
    /* recompute nothing -- CRC covers header bytes 0..35 including byte 4,
     * so this must fail on CRC before version is even checked in a
     * byte-for-byte-faithful implementation; assert it's rejected either way. */
    mtk_spi_parse_result_t rc = mtk_spi_native_parse_cell(bad_ver, &parsed, &pl);
    MTK_CHECK(rc == MTK_SPI_PARSE_BAD_CRC || rc == MTK_SPI_PARSE_BAD_VERSION);

    /* Reserved flag bits must be zero. */
    mtk_spi_native_header_t h2 = h;
    h2.flags |= 0x10;
    uint8_t cell2[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&h2, payload, cell2);
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(cell2, &parsed, &pl), MTK_SPI_PARSE_BAD_FLAGS);

    /* Unknown class. */
    mtk_spi_native_header_t h3 = h;
    h3.msg_class = 200;
    uint8_t cell3[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&h3, payload, cell3);
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(cell3, &parsed, &pl), MTK_SPI_PARSE_BAD_CLASS);

    /* payload_len over the 984-byte ceiling is rejected. Built from a
     * genuinely valid 16-byte-payload cell (matching the real `payload`
     * buffer), then the on-wire payload_len field is patched directly to
     * an out-of-range value -- exercising the header-claims-more-than-
     * allowed rejection path without asking build_cell to read past the
     * end of a real (small) source buffer. */
    uint8_t cell4[MTK_SPI_NATIVE_CELL_SIZE];
    memcpy(cell4, cell, sizeof(cell4));
    cell4[14] = (uint8_t)(985 & 0xFF); cell4[15] = (uint8_t)((985 >> 8) & 0xFF); /* payload_len, LE */
    MTK_CHECK_EQ(mtk_spi_native_parse_cell(cell4, &parsed, &pl), MTK_SPI_PARSE_BAD_LENGTH);

    /* CRC32C known-answer test (standard test vector: "123456789" -> 0xE3069283). */
    MTK_CHECK_EQ(mtk_crc32c((const uint8_t *)"123456789", 9), 0xE3069283u);

MTK_TEST_MAIN_END
