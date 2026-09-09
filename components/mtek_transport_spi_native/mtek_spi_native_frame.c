/* Clean-room implementation from MonstaTek contract (SPI_PROTOCOL_V1.md).
 * Explicit little-endian load/store throughout -- no native-structure
 * casting onto the wire buffer, per the protocol's own "Header" rule. */
#include "mtek_spi_native_frame.h"
#include <string.h>

uint32_t mtk_crc32c(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0x82F63B78u & mask); /* CRC-32C (Castagnoli), reflected */
        }
    }
    return ~crc;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void serialize_header(const mtk_spi_native_header_t *h, uint8_t *out /* 40 bytes */) {
    put_u32(out + 0, h->magic);
    out[4] = h->major;
    out[5] = h->minor;
    out[6] = h->msg_class;
    out[7] = h->flags;
    put_u16(out + 8, h->service);
    put_u16(out + 10, h->opcode);
    put_u16(out + 12, h->status);
    put_u16(out + 14, h->payload_len);
    put_u32(out + 16, h->request_id);
    put_u32(out + 20, h->packet_seq);
    put_u32(out + 24, h->boot_epoch);
    put_u32(out + 28, h->message_len);
    put_u32(out + 32, h->fragment_offset);
    /* bytes 36..39 (crc32c) filled by the caller after this call */
}

void mtk_spi_native_build_cell(const mtk_spi_native_header_t *hdr, const uint8_t *payload, uint8_t *out) {
    memset(out, 0, MTK_SPI_NATIVE_CELL_SIZE);
    serialize_header(hdr, out);
    uint16_t plen = hdr->payload_len;
    if (plen > MTK_SPI_NATIVE_MAX_PAYLOAD) plen = MTK_SPI_NATIVE_MAX_PAYLOAD;
    if (payload && plen) memcpy(out + MTK_SPI_NATIVE_HEADER_SIZE, payload, plen);
    /* CRC32C over header bytes 0..35 followed by payload_len payload bytes;
     * never covers the unused tail. */
    uint8_t crc_input[MTK_SPI_NATIVE_HEADER_SIZE - 4 + MTK_SPI_NATIVE_MAX_PAYLOAD];
    memcpy(crc_input, out, MTK_SPI_NATIVE_HEADER_SIZE - 4);
    memcpy(crc_input + (MTK_SPI_NATIVE_HEADER_SIZE - 4), out + MTK_SPI_NATIVE_HEADER_SIZE, plen);
    uint32_t crc = mtk_crc32c(crc_input, (MTK_SPI_NATIVE_HEADER_SIZE - 4) + plen);
    put_u32(out + 36, crc);
}

/* Discovery-phase variant: validates a native header+payload+CRC that
 * fits within a buffer possibly shorter than the full 1024-byte
 * steady-state cell (SPI_PROTOCOL_V1.md "Runtime transport selection":
 * discovery transactions are 512 bytes until a profile locks and, for
 * native, negotiates the 1024-byte cell). Requires
 * 40 + payload_len <= len; does not require len==1024 or trailing zeros. */
mtk_spi_parse_result_t mtk_spi_native_parse_bounded(const uint8_t *in, size_t len, mtk_spi_native_header_t *hdr,
                                                     const uint8_t **payload_out) {
    if (len < MTK_SPI_NATIVE_HEADER_SIZE) return MTK_SPI_PARSE_TRUNCATED;
    mtk_spi_native_header_t h;
    h.magic = get_u32(in + 0);
    if (h.magic != MTK_SPI_NATIVE_MAGIC) return MTK_SPI_PARSE_BAD_MAGIC;
    h.major = in[4];
    h.minor = in[5];
    if (h.major != MTK_SPI_NATIVE_MAJOR) return MTK_SPI_PARSE_BAD_VERSION;
    h.msg_class = in[6];
    if (h.msg_class > MTK_SPI_CLASS_LINK_ERROR) return MTK_SPI_PARSE_BAD_CLASS;
    h.flags = in[7];
    if (h.flags & MTK_SPI_FLAG_RESERVED_MASK) return MTK_SPI_PARSE_BAD_FLAGS;
    h.service = get_u16(in + 8);
    h.opcode = get_u16(in + 10);
    h.status = get_u16(in + 12);
    h.payload_len = get_u16(in + 14);
    if (h.payload_len > MTK_SPI_NATIVE_MAX_PAYLOAD) return MTK_SPI_PARSE_BAD_LENGTH;
    if ((size_t)MTK_SPI_NATIVE_HEADER_SIZE + h.payload_len > len) return MTK_SPI_PARSE_TRUNCATED;
    h.request_id = get_u32(in + 16);
    h.packet_seq = get_u32(in + 20);
    h.boot_epoch = get_u32(in + 24);
    h.message_len = get_u32(in + 28);
    h.fragment_offset = get_u32(in + 32);
    h.crc32c = get_u32(in + 36);

    uint8_t crc_input[MTK_SPI_NATIVE_HEADER_SIZE - 4 + MTK_SPI_NATIVE_MAX_PAYLOAD];
    memcpy(crc_input, in, MTK_SPI_NATIVE_HEADER_SIZE - 4);
    memcpy(crc_input + (MTK_SPI_NATIVE_HEADER_SIZE - 4), in + MTK_SPI_NATIVE_HEADER_SIZE, h.payload_len);
    uint32_t computed = mtk_crc32c(crc_input, (MTK_SPI_NATIVE_HEADER_SIZE - 4) + h.payload_len);
    if (computed != h.crc32c) return MTK_SPI_PARSE_BAD_CRC;

    if (h.msg_class == MTK_SPI_CLASS_REQUEST || h.msg_class == MTK_SPI_CLASS_RESPONSE ||
        h.msg_class == MTK_SPI_CLASS_EVENT || h.msg_class == MTK_SPI_CLASS_STREAM) {
        if ((uint64_t)h.fragment_offset + h.payload_len > h.message_len && h.message_len != 0) {
            return MTK_SPI_PARSE_BAD_LENGTH;
        }
    }

    *hdr = h;
    if (payload_out) *payload_out = in + MTK_SPI_NATIVE_HEADER_SIZE;
    return MTK_SPI_PARSE_OK;
}

/* ---- Multi-cell fragmentation/reassembly -------------------------------- */

void mtk_spi_native_reassembly_reset(mtk_spi_native_reassembly_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
}

mtk_spi_reasm_result_t mtk_spi_native_reassembly_feed(mtk_spi_native_reassembly_t *ctx, const mtk_spi_native_header_t *hdr,
                                                       const uint8_t *payload, uint32_t now_ms) {
    int is_first = (hdr->flags & MTK_SPI_FLAG_FIRST) != 0;
    int is_last = (hdr->flags & MTK_SPI_FLAG_LAST) != 0;

    if (is_first) {
        /* RC6 independent audit P0 "Native reassembly and duplicate safety
         * are materially incomplete": a FIRST fragment for a genuinely
         * DIFFERENT request_id while another message is still actively
         * being reassembled must NOT silently reset/overwrite it -- that
         * was the original bug (two concurrent fragmented requests
         * corrupting each other). This dispatcher's single reassembly
         * context is a documented, measured SRAM-budget exception (see the
         * enum's own doc comment); reject explicitly instead. A FIRST
         * fragment for the SAME request_id already in progress (a
         * legitimate resend/retry of the message's own first cell, e.g.
         * FLAG_RETRY) still (re)starts that context, matching the
         * single-outstanding-message-per-id model this always had. */
        if (ctx->active && ctx->request_id != hdr->request_id) {
            return MTK_SPI_REASM_BUSY;
        }
        if (hdr->message_len > MTK_SPI_NATIVE_MAX_MESSAGE) {
            mtk_spi_native_reassembly_reset(ctx);
            return MTK_SPI_REASM_OVERFLOW;
        }
        ctx->active = 1;
        ctx->request_id = hdr->request_id;
        ctx->service = hdr->service;
        ctx->opcode = hdr->opcode;
        ctx->boot_epoch = hdr->boot_epoch;
        ctx->message_len = hdr->message_len;
        ctx->received_len = 0;
        ctx->last_seen_ms = now_ms;
    } else {
        if (!ctx->active || hdr->request_id != ctx->request_id) {
            /* A continuation fragment with no matching in-progress
             * context (never started, or for a different message
             * entirely) -- a real protocol violation, never guessed or
             * silently absorbed. */
            return MTK_SPI_REASM_ORPHAN_FRAGMENT;
        }
    }

    if (hdr->fragment_offset > ctx->received_len) { mtk_spi_native_reassembly_reset(ctx); return MTK_SPI_REASM_GAP; }
    if (hdr->fragment_offset < ctx->received_len) { mtk_spi_native_reassembly_reset(ctx); return MTK_SPI_REASM_DUPLICATE; }
    if ((uint64_t)hdr->fragment_offset + hdr->payload_len > MTK_SPI_NATIVE_MAX_MESSAGE) {
        mtk_spi_native_reassembly_reset(ctx);
        return MTK_SPI_REASM_OVERFLOW;
    }
    if (hdr->payload_len) memcpy(ctx->data + hdr->fragment_offset, payload, hdr->payload_len);
    ctx->received_len += hdr->payload_len;
    ctx->last_seen_ms = now_ms;

    if (is_last) {
        mtk_spi_reasm_result_t result = (ctx->received_len == ctx->message_len) ? MTK_SPI_REASM_COMPLETE : MTK_SPI_REASM_GAP;
        if (result != MTK_SPI_REASM_COMPLETE) mtk_spi_native_reassembly_reset(ctx);
        return result;
    }
    return MTK_SPI_REASM_IN_PROGRESS;
}

int mtk_spi_native_reassembly_timed_out(const mtk_spi_native_reassembly_t *ctx, uint32_t now_ms, uint32_t timeout_ms) {
    if (!ctx->active) return 0;
    return (now_ms - ctx->last_seen_ms) >= timeout_ms; /* unsigned wraparound-safe for a monotonic now_ms */
}

/* ---- Outbound multi-cell fragmentation ---------------------------------- */

void mtk_spi_native_outbound_start(mtk_spi_native_outbound_t *ob, const mtk_spi_native_header_t *hdr_template,
                                    const uint8_t *body, uint32_t body_len) {
    if (body_len > MTK_SPI_NATIVE_MAX_MESSAGE) body_len = MTK_SPI_NATIVE_MAX_MESSAGE; /* never reached by a correctly-bounded caller; defensive floor, not a guess */
    ob->active = 1;
    ob->hdr = *hdr_template;
    ob->total_len = body_len;
    ob->sent_offset = 0;
    if (body && body_len) memcpy(ob->data, body, body_len);
}

int mtk_spi_native_outbound_next(mtk_spi_native_outbound_t *ob, uint8_t *out) {
    uint32_t remaining = ob->total_len - ob->sent_offset;
    uint16_t chunk = remaining > MTK_SPI_NATIVE_MAX_PAYLOAD ? MTK_SPI_NATIVE_MAX_PAYLOAD : (uint16_t)remaining;
    int is_first = (ob->sent_offset == 0);
    int is_last = (chunk == remaining);

    mtk_spi_native_header_t h = ob->hdr;
    h.flags = (uint8_t)((is_first ? MTK_SPI_FLAG_FIRST : 0) | (is_last ? MTK_SPI_FLAG_LAST : 0));
    h.payload_len = chunk;
    h.message_len = ob->total_len;
    h.fragment_offset = ob->sent_offset;
    mtk_spi_native_build_cell(&h, ob->data + ob->sent_offset, out);

    ob->sent_offset += chunk;
    if (is_last) { ob->active = 0; return 0; }
    return 1;
}

mtk_spi_parse_result_t mtk_spi_native_parse_cell(const uint8_t *in, mtk_spi_native_header_t *hdr,
                                                  const uint8_t **payload_out) {
    return mtk_spi_native_parse_bounded(in, MTK_SPI_NATIVE_CELL_SIZE, hdr, payload_out);
}

void mtk_spi_native_packet_seq_tracker_init(mtk_spi_native_packet_seq_tracker_t *t) {
    t->known = 0;
    t->last_seq = 0;
}

int mtk_spi_native_packet_seq_tracker_note(mtk_spi_native_packet_seq_tracker_t *t, uint32_t seq) {
    int gap = 0;
    if (t->known && seq != t->last_seq + 1) gap = 1;
    t->last_seq = seq;
    t->known = 1;
    return gap;
}
