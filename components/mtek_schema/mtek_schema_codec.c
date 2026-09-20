/* Clean-room implementation from MonstaTek contract (schemas.json field
 * vocabulary, type table).
 *
 * Generic, data-driven wire codec: every request/response/event body in this
 * firmware is encoded/decoded by walking a struct's mtk_struct_desc_t field
 * table rather than by a hand-written per-message function or by casting a wire
 * buffer onto a native struct. This matches the native SPI v1 protocol's own
 * explicit-load/store requirement and, as a side effect, gives every one of the
 * 91 canonical opcodes an encoder/decoder for free from the same generated table
 * used for host-test introspection.
 *
 * Wire rule: multi-byte integers are little-endian; bool is strictly 0x00/0x01;
 * mac6 and ipv4 are opaque byte sequences copied verbatim (ipv4 is already
 * network-byte-order in memory). */
#include "mtek_codec_api.h"
#include <string.h>

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t pos;
} mtk_writer_t;

typedef struct {
    const uint8_t *buf;
    size_t len;
    size_t pos;
} mtk_reader_t;

static int w_put(mtk_writer_t *w, const void *data, size_t n) {
    if (w->pos + n > w->cap) return MTK_CODEC_OVERFLOW;
    memcpy(w->buf + w->pos, data, n);
    w->pos += n;
    return MTK_CODEC_OK;
}

static int w_u8(mtk_writer_t *w, uint8_t v) { return w_put(w, &v, 1); }

static int w_uint(mtk_writer_t *w, uint64_t v, unsigned width) {
    uint8_t tmp[8];
    for (unsigned i = 0; i < width; i++) tmp[i] = (uint8_t)(v >> (8 * i));
    return w_put(w, tmp, width);
}

static int r_get(mtk_reader_t *r, void *out, size_t n) {
    if (r->pos + n > r->len) return MTK_CODEC_TRUNCATED;
    if (out) memcpy(out, r->buf + r->pos, n);
    r->pos += n;
    return MTK_CODEC_OK;
}

static int r_uint(mtk_reader_t *r, uint64_t *out, unsigned width) {
    uint8_t tmp[8];
    int rc = r_get(r, tmp, width);
    if (rc != MTK_CODEC_OK) return rc;
    uint64_t v = 0;
    for (unsigned i = 0; i < width; i++) v |= ((uint64_t)tmp[i]) << (8 * i);
    *out = v;
    return MTK_CODEC_OK;
}

static unsigned prim_width(mtk_field_type_t t) {
    switch (t) {
        case MTK_F_U8: case MTK_F_I8: case MTK_F_BOOL: return 1;
        case MTK_F_U16: case MTK_F_I16: return 2;
        case MTK_F_U32: case MTK_F_I32: return 4;
        case MTK_F_U64: case MTK_F_I64: return 8;
        default: return 0;
    }
}

static int encode_prim(mtk_writer_t *w, mtk_field_type_t t, const uint8_t *p) {
    unsigned width = prim_width(t);
    if (width == 0) return MTK_CODEC_PROTOCOL_ERROR;
    uint64_t v = 0;
    memcpy(&v, p, width);
    if (t == MTK_F_BOOL && v > 1) return MTK_CODEC_PROTOCOL_ERROR;
    return w_uint(w, v, width);
}

static int decode_prim(mtk_reader_t *r, mtk_field_type_t t, uint8_t *p) {
    unsigned width = prim_width(t);
    if (width == 0) return MTK_CODEC_PROTOCOL_ERROR;
    uint64_t v;
    int rc = r_uint(r, &v, width);
    if (rc != MTK_CODEC_OK) return rc;
    if (t == MTK_F_BOOL && v > 1) return MTK_CODEC_PROTOCOL_ERROR;
    memcpy(p, &v, width);
    return MTK_CODEC_OK;
}

static int encode_struct(mtk_writer_t *w, const mtk_struct_desc_t *desc, const uint8_t *base);
static int decode_struct(mtk_reader_t *r, const mtk_struct_desc_t *desc, uint8_t *base);

static int encode_field(mtk_writer_t *w, const mtk_field_desc_t *f, const uint8_t *base) {
    const uint8_t *p = base + f->offset;
    switch (f->type) {
        case MTK_F_U8: case MTK_F_U16: case MTK_F_U32: case MTK_F_U64:
        case MTK_F_I8: case MTK_F_I16: case MTK_F_I32: case MTK_F_I64:
        case MTK_F_BOOL:
            return encode_prim(w, f->type, p);
        case MTK_F_MAC6:
            return w_put(w, p, 6);
        case MTK_F_IPV4:
            return w_put(w, p, 4);
        case MTK_F_BYTES_FIXED:
            return w_put(w, p, f->max);
        case MTK_F_BYTES:
        case MTK_F_UTF8: {
            uint16_t len;
            memcpy(&len, p, sizeof(len));
            if (len > f->max) return MTK_CODEC_OVERFLOW;
            int rc = (f->len_prefix == 2) ? w_uint(w, len, 2) : w_u8(w, (uint8_t)len);
            if (rc != MTK_CODEC_OK) return rc;
            return w_put(w, p + 2, len);
        }
        case MTK_F_STRUCT:
            return encode_struct(w, f->nested, p);
        case MTK_F_ARRAY: {
            uint32_t count;
            memcpy(&count, p, sizeof(count));
            if (count > f->max) return MTK_CODEC_OVERFLOW;
            int rc = (f->len_prefix == 2) ? w_uint(w, count, 2) : w_u8(w, (uint8_t)count);
            if (rc != MTK_CODEC_OK) return rc;
            const uint8_t *items = p + 4;
            for (uint32_t i = 0; i < count; i++) {
                const uint8_t *item = items + (size_t)i * f->elem_size;
                if (f->elem_type == MTK_F_STRUCT) {
                    rc = encode_struct(w, f->nested, item);
                } else if (f->elem_type == MTK_F_MAC6) {
                    rc = w_put(w, item, 6);
                } else if (f->elem_type == MTK_F_IPV4) {
                    rc = w_put(w, item, 4);
                } else if (f->elem_type == MTK_F_BYTES) {
                    /* An array element of byte-string type is laid out as
                     * { uint16_t len; uint8_t data[capacity]; }, so its writable
                     * capacity is the element stride minus that length field.
                     * `max` on the array field bounds the element COUNT, never
                     * an individual element's byte length, so the capacity must
                     * be derived here. Refused rather than truncated: the wire
                     * length prefix for an array element is a single byte, and
                     * silently narrowing a wider in-memory length would emit a
                     * frame whose declared length disagrees with its payload. */
                    if (f->elem_size < 2) return MTK_CODEC_PROTOCOL_ERROR;
                    uint16_t elem_cap = (uint16_t)(f->elem_size - 2);
                    uint16_t blen;
                    memcpy(&blen, item, sizeof(blen));
                    if (blen > elem_cap || blen > 0xFF) return MTK_CODEC_OVERFLOW;
                    rc = w_u8(w, (uint8_t)blen);
                    if (rc == MTK_CODEC_OK) rc = w_put(w, item + 2, blen);
                } else {
                    rc = encode_prim(w, f->elem_type, item);
                }
                if (rc != MTK_CODEC_OK) return rc;
            }
            return MTK_CODEC_OK;
        }
        default:
            return MTK_CODEC_PROTOCOL_ERROR;
    }
}

static int decode_field(mtk_reader_t *r, const mtk_field_desc_t *f, uint8_t *base) {
    uint8_t *p = base + f->offset;
    switch (f->type) {
        case MTK_F_U8: case MTK_F_U16: case MTK_F_U32: case MTK_F_U64:
        case MTK_F_I8: case MTK_F_I16: case MTK_F_I32: case MTK_F_I64:
        case MTK_F_BOOL:
            return decode_prim(r, f->type, p);
        case MTK_F_MAC6:
            return r_get(r, p, 6);
        case MTK_F_IPV4:
            return r_get(r, p, 4);
        case MTK_F_BYTES_FIXED:
            return r_get(r, p, f->max);
        case MTK_F_BYTES:
        case MTK_F_UTF8: {
            uint64_t lenv;
            int rc = r_uint(r, &lenv, f->len_prefix == 2 ? 2 : 1);
            if (rc != MTK_CODEC_OK) return rc;
            if (lenv > f->max) return MTK_CODEC_OVERFLOW;
            uint16_t len = (uint16_t)lenv;
            memcpy(p, &len, sizeof(len));
            return r_get(r, p + 2, len);
        }
        case MTK_F_STRUCT:
            return decode_struct(r, f->nested, p);
        case MTK_F_ARRAY: {
            uint64_t countv;
            int rc = r_uint(r, &countv, f->len_prefix == 2 ? 2 : 1);
            if (rc != MTK_CODEC_OK) return rc;
            if (countv > f->max) return MTK_CODEC_OVERFLOW;
            uint32_t count = (uint32_t)countv;
            memcpy(p, &count, sizeof(count));
            uint8_t *items = p + 4;
            for (uint32_t i = 0; i < count; i++) {
                uint8_t *item = items + (size_t)i * f->elem_size;
                if (f->elem_type == MTK_F_STRUCT) {
                    rc = decode_struct(r, f->nested, item);
                } else if (f->elem_type == MTK_F_MAC6) {
                    rc = r_get(r, item, 6);
                } else if (f->elem_type == MTK_F_IPV4) {
                    rc = r_get(r, item, 4);
                } else if (f->elem_type == MTK_F_BYTES) {
                    /* Element layout is { uint16_t len; uint8_t data[capacity]; },
                     * so the writable capacity is the element stride minus that
                     * length field. This bound is required and cannot come from
                     * `max`, which bounds the element COUNT for an array field,
                     * not any single element's byte length: the wire length
                     * prefix here is a full byte (0..255) while a typical
                     * element holds far less, so an unchecked length writes past
                     * the element and, for the final element, past the decoded
                     * object itself. Validated before any store to the
                     * destination, matching this codec's contract that a
                     * malformed message is never partially applied. */
                    if (f->elem_size < 2) return MTK_CODEC_PROTOCOL_ERROR;
                    uint16_t elem_cap = (uint16_t)(f->elem_size - 2);
                    uint64_t blenv;
                    rc = r_uint(r, &blenv, 1);
                    if (rc != MTK_CODEC_OK) return rc;
                    if (blenv > elem_cap) return MTK_CODEC_OVERFLOW;
                    uint16_t blen = (uint16_t)blenv;
                    rc = r_get(r, item + 2, blen);
                    if (rc != MTK_CODEC_OK) return rc;
                    memcpy(item, &blen, sizeof(blen));
                } else {
                    rc = decode_prim(r, f->elem_type, item);
                }
                if (rc != MTK_CODEC_OK) return rc;
            }
            return MTK_CODEC_OK;
        }
        default:
            return MTK_CODEC_PROTOCOL_ERROR;
    }
}

static int encode_struct(mtk_writer_t *w, const mtk_struct_desc_t *desc, const uint8_t *base) {
    for (uint16_t i = 0; i < desc->field_count; i++) {
        int rc = encode_field(w, &desc->fields[i], base);
        if (rc != MTK_CODEC_OK) return rc;
    }
    return MTK_CODEC_OK;
}

static int decode_struct(mtk_reader_t *r, const mtk_struct_desc_t *desc, uint8_t *base) {
    for (uint16_t i = 0; i < desc->field_count; i++) {
        int rc = decode_field(r, &desc->fields[i], base);
        if (rc != MTK_CODEC_OK) return rc;
    }
    return MTK_CODEC_OK;
}

int mtk_encode(const mtk_struct_desc_t *desc, const void *obj, uint8_t *out, size_t cap, size_t *out_len) {
    if (!desc) { if (out_len) *out_len = 0; return MTK_CODEC_OK; }
    mtk_writer_t w = { out, cap, 0 };
    int rc = encode_struct(&w, desc, (const uint8_t *)obj);
    if (out_len) *out_len = w.pos;
    return rc;
}

int mtk_decode(const mtk_struct_desc_t *desc, void *obj, const uint8_t *in, size_t len, size_t *consumed) {
    if (!desc) { if (consumed) *consumed = 0; return MTK_CODEC_OK; }
    mtk_reader_t r = { in, len, 0 };
    int rc = decode_struct(&r, desc, (uint8_t *)obj);
    if (consumed) *consumed = r.pos;
    return rc;
}
