/* Clean-room implementation from MonstaTek contract
 * (001-profile-bootstrap-feasibility.md Sec 1, evidence extracted from the
 * pinned Bedge `m1_link`/`m1_rpc` transport's own header/behavior, and an
 * independently-written frame codec -- no Bedge source is transcribed).
 *
 * Bedge/C3 `m1_link` transport: fixed 512-byte full-duplex SPI cells, an
 * 8-byte header, payload, and a 2-byte CRC16-CCITT trailer. Every cell is
 * always a complete, parseable, magic-bearing frame (IDLE when nothing is
 * queued) -- never silence or garbage. */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MTK_BEDGE_CELL_SIZE 512
#define MTK_BEDGE_HEADER_SIZE 8
#define MTK_BEDGE_CRC_SIZE 2
#define MTK_BEDGE_MAGIC 0x4D31u /* u16, little-endian on the wire (bytes 0x31, 0x4D) */
#define MTK_BEDGE_VERSION 0x01u
/* Single-cell payload ceiling: 512 - 8 (header) - 2 (CRC) = 502. */
#define MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX (MTK_BEDGE_CELL_SIZE - MTK_BEDGE_HEADER_SIZE - MTK_BEDGE_CRC_SIZE)
/* Reassembled logical message ceiling: 4092 - 8 - 2 = 4082 (matches the
 * underlying SPI-DMA transport's own frame-size figure per the facts). */
#define MTK_BEDGE_MAX_REASSEMBLY_PAYLOAD (4092 - MTK_BEDGE_HEADER_SIZE - MTK_BEDGE_CRC_SIZE)
#define MTK_BEDGE_MAX_INFLIGHT 1 /* the pinned reference dispatch loop is strictly sequential */

typedef enum {
    MTK_BEDGE_MSG_IDLE = 0x00,
    MTK_BEDGE_MSG_REQ = 0x01,
    MTK_BEDGE_MSG_RESP = 0x02,
    MTK_BEDGE_MSG_EVENT = 0x03,
    MTK_BEDGE_MSG_FRAG = 0x04,
    MTK_BEDGE_MSG_NAK = 0x05,
} mtk_bedge_msg_type_t;

/* m1esp_status_t (facts Sec 1.2 "Status codes"). ERR_PENDING (0xFF) means
 * "accepted, result arrives later as an EVENT" -- distinct from this
 * task's own ACCEPTED_ASYNC model, translated at the adapter boundary. */
typedef enum {
    MTK_BEDGE_STATUS_OK = 0x00,
    MTK_BEDGE_STATUS_ERR_UNKNOWN = 0x01,
    MTK_BEDGE_STATUS_ERR_INVALID_ARGS = 0x02,
    MTK_BEDGE_STATUS_ERR_BUSY = 0x03,
    MTK_BEDGE_STATUS_ERR_TIMEOUT = 0x04,
    MTK_BEDGE_STATUS_ERR_NO_MEM = 0x05,
    MTK_BEDGE_STATUS_ERR_NOT_INIT = 0x06,
    MTK_BEDGE_STATUS_ERR_ALREADY_RUN = 0x07,
    MTK_BEDGE_STATUS_ERR_NOT_RUNNING = 0x08,
    MTK_BEDGE_STATUS_ERR_HARDWARE = 0x09,
    MTK_BEDGE_STATUS_ERR_UNSUPPORTED = 0x0A,
    MTK_BEDGE_STATUS_ERR_BAD_CRC = 0x0B,
    MTK_BEDGE_STATUS_ERR_PENDING = 0xFF,
} mtk_bedge_status_t;

typedef struct {
    uint16_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint16_t msg_id;
    uint16_t payload_len; /* 0..MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX for one cell */
} mtk_bedge_header_t;

typedef enum {
    MTK_BEDGE_PARSE_OK = 0,
    MTK_BEDGE_PARSE_TRUNCATED,
    MTK_BEDGE_PARSE_BAD_MAGIC,
    MTK_BEDGE_PARSE_BAD_VERSION,
    MTK_BEDGE_PARSE_BAD_MSG_TYPE,
    MTK_BEDGE_PARSE_BAD_LENGTH,
    MTK_BEDGE_PARSE_BAD_CRC,
} mtk_bedge_parse_result_t;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, not reflected, no output
 * XOR (known-answer: mtk_bedge_crc16("123456789", 9) == 0x29B1). */
uint16_t mtk_bedge_crc16(const uint8_t *data, size_t len);

/* Builds one fixed 512-byte cell (header + payload + zero-padded
 * remainder + trailing CRC over header+payload only). `payload_len` must
 * be <= MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX. */
void mtk_bedge_build_cell(const mtk_bedge_header_t *hdr, const uint8_t *payload, uint8_t out[MTK_BEDGE_CELL_SIZE]);

/* Validates and parses exactly one 512-byte cell. Never partially
 * executes an invalid message: on any result other than
 * MTK_BEDGE_PARSE_OK, `*hdr` is not a trustworthy frame to act on. */
mtk_bedge_parse_result_t mtk_bedge_parse_cell(const uint8_t cell[MTK_BEDGE_CELL_SIZE], mtk_bedge_header_t *hdr,
                                               const uint8_t **payload_out);

/* Discovery-phase variant: validates a Bedge frame within a buffer that
 * may be shorter than the full 512-byte steady-state cell (used only
 * during the non-dispatching AUTO discovery window,
 * SPI_PROTOCOL_V1.md "Runtime transport selection"). Requires the full
 * header+payload+CRC to fit within `len`; does not require `len==512` or
 * that trailing bytes be present/zero. */
mtk_bedge_parse_result_t mtk_bedge_parse_bounded(const uint8_t *buf, size_t len, mtk_bedge_header_t *hdr,
                                                  const uint8_t **payload_out);

/* ---- Bounded reassembly (Sec "Fragmentation") -------------------------- */

typedef struct {
    uint8_t active;
    uint16_t msg_id;
    uint8_t final_msg_type; /* the REQ/RESP/EVENT type that terminated reassembly */
    uint32_t len;
    uint8_t data[MTK_BEDGE_MAX_REASSEMBLY_PAYLOAD];
} mtk_bedge_reassembly_t;

typedef enum {
    MTK_BEDGE_REASM_IN_PROGRESS = 0,
    MTK_BEDGE_REASM_COMPLETE,
    MTK_BEDGE_REASM_OVERFLOW,
    MTK_BEDGE_REASM_MSG_ID_MISMATCH,
} mtk_bedge_reasm_result_t;

void mtk_bedge_reassembly_reset(mtk_bedge_reassembly_t *ctx);
/* Feeds one parsed, already-CRC-valid frame into the reassembly context.
 * A FRAG frame continues accumulation; a non-FRAG (REQ/RESP/EVENT) frame
 * appends its own payload and completes the message. A FRAG/terminal
 * frame whose msg_id doesn't match an in-progress context starts a new
 * one (matching the single-outstanding-request model: only one logical
 * message is ever being reassembled at a time). */
mtk_bedge_reasm_result_t mtk_bedge_reassembly_feed(mtk_bedge_reassembly_t *ctx, const mtk_bedge_header_t *hdr,
                                                    const uint8_t *payload);

#ifdef __cplusplus
}
#endif
