/* Clean-room implementation from MonstaTek contract
 * (001-profile-bootstrap-feasibility.md Sec 1). Explicit little-endian
 * load/store throughout -- no native-structure casting onto the wire
 * buffer, matching this task's general codec discipline
 * (SPI_PROTOCOL_V1.md's own rule, applied uniformly to both SPI profiles). */
#include "mtek_bedge_frame.h"
#include <string.h>

uint16_t mtk_bedge_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void serialize_header(const mtk_bedge_header_t *h, uint8_t out[MTK_BEDGE_HEADER_SIZE]) {
    put_u16(out + 0, h->magic);
    out[2] = h->version;
    out[3] = h->msg_type;
    put_u16(out + 4, h->msg_id);
    put_u16(out + 6, h->payload_len);
}

void mtk_bedge_build_cell(const mtk_bedge_header_t *hdr, const uint8_t *payload, uint8_t out[MTK_BEDGE_CELL_SIZE]) {
    memset(out, 0, MTK_BEDGE_CELL_SIZE);
    uint16_t plen = hdr->payload_len;
    if (plen > MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX) plen = MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX;
    serialize_header(hdr, out);
    if (payload && plen) memcpy(out + MTK_BEDGE_HEADER_SIZE, payload, plen);
    uint16_t crc = mtk_bedge_crc16(out, (size_t)MTK_BEDGE_HEADER_SIZE + plen);
    put_u16(out + MTK_BEDGE_HEADER_SIZE + plen, crc);
}

static mtk_bedge_parse_result_t parse_common(const uint8_t *buf, size_t len, mtk_bedge_header_t *hdr,
                                              const uint8_t **payload_out, int require_exact_cell) {
    if (len < MTK_BEDGE_HEADER_SIZE + MTK_BEDGE_CRC_SIZE) return MTK_BEDGE_PARSE_TRUNCATED;
    mtk_bedge_header_t h;
    h.magic = get_u16(buf + 0);
    if (h.magic != MTK_BEDGE_MAGIC) return MTK_BEDGE_PARSE_BAD_MAGIC;
    h.version = buf[2];
    if (h.version != MTK_BEDGE_VERSION) return MTK_BEDGE_PARSE_BAD_VERSION;
    h.msg_type = buf[3];
    if (h.msg_type > MTK_BEDGE_MSG_NAK) return MTK_BEDGE_PARSE_BAD_MSG_TYPE;
    h.msg_id = get_u16(buf + 4);
    h.payload_len = get_u16(buf + 6);
    if (h.payload_len > MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX) return MTK_BEDGE_PARSE_BAD_LENGTH;
    size_t frame_len = (size_t)MTK_BEDGE_HEADER_SIZE + h.payload_len + MTK_BEDGE_CRC_SIZE;
    if (frame_len > len) return MTK_BEDGE_PARSE_TRUNCATED;
    if (require_exact_cell && len != MTK_BEDGE_CELL_SIZE) return MTK_BEDGE_PARSE_BAD_LENGTH;

    uint16_t computed = mtk_bedge_crc16(buf, (size_t)MTK_BEDGE_HEADER_SIZE + h.payload_len);
    uint16_t on_wire = get_u16(buf + MTK_BEDGE_HEADER_SIZE + h.payload_len);
    if (computed != on_wire) return MTK_BEDGE_PARSE_BAD_CRC;

    *hdr = h;
    if (payload_out) *payload_out = buf + MTK_BEDGE_HEADER_SIZE;
    return MTK_BEDGE_PARSE_OK;
}

mtk_bedge_parse_result_t mtk_bedge_parse_cell(const uint8_t cell[MTK_BEDGE_CELL_SIZE], mtk_bedge_header_t *hdr,
                                               const uint8_t **payload_out) {
    return parse_common(cell, MTK_BEDGE_CELL_SIZE, hdr, payload_out, 1);
}

mtk_bedge_parse_result_t mtk_bedge_parse_bounded(const uint8_t *buf, size_t len, mtk_bedge_header_t *hdr,
                                                  const uint8_t **payload_out) {
    return parse_common(buf, len, hdr, payload_out, 0);
}

void mtk_bedge_reassembly_reset(mtk_bedge_reassembly_t *ctx) { memset(ctx, 0, sizeof(*ctx)); }

mtk_bedge_reasm_result_t mtk_bedge_reassembly_feed(mtk_bedge_reassembly_t *ctx, const mtk_bedge_header_t *hdr,
                                                    const uint8_t *payload) {
    if (hdr->msg_type == MTK_BEDGE_MSG_IDLE) return MTK_BEDGE_REASM_IN_PROGRESS; /* IDLE never enters reassembly */

    if (!ctx->active) {
        mtk_bedge_reassembly_reset(ctx);
        ctx->active = 1;
        ctx->msg_id = hdr->msg_id;
    } else if (hdr->msg_id != ctx->msg_id) {
        /* Single-outstanding-request model: a new msg_id abandons the
         * stale partial context rather than mixing frames from two
         * logical messages (matching the pinned reference's own
         * strictly-sequential dispatch loop). */
        mtk_bedge_reassembly_reset(ctx);
        ctx->active = 1;
        ctx->msg_id = hdr->msg_id;
    }

    if (ctx->len + hdr->payload_len > MTK_BEDGE_MAX_REASSEMBLY_PAYLOAD) {
        mtk_bedge_reassembly_reset(ctx);
        return MTK_BEDGE_REASM_OVERFLOW;
    }
    if (hdr->payload_len) {
        memcpy(ctx->data + ctx->len, payload, hdr->payload_len);
        ctx->len += hdr->payload_len;
    }

    if (hdr->msg_type == MTK_BEDGE_MSG_FRAG) return MTK_BEDGE_REASM_IN_PROGRESS;

    /* REQ/RESP/EVENT/NAK terminates reassembly (a single-cell message is
     * simply a zero-fragment "reassembly" of exactly one terminal frame). */
    ctx->final_msg_type = hdr->msg_type;
    ctx->active = 0;
    return MTK_BEDGE_REASM_COMPLETE;
}
