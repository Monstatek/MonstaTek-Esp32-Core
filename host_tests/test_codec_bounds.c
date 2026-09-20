/* Codec bounds validation.
 *
 * The generic codec decodes every request/response body in this firmware by
 * walking a generated field table, so a missing bound in one field-type
 * branch is a missing bound for every message that uses that shape, on every
 * transport. Two layers of coverage here:
 *
 *   1. A fixed regression case for the array-of-byte-strings element length,
 *      whose capacity must be derived from the element stride because the
 *      field's own `max` bounds the element COUNT instead.
 *   2. A table-driven property check that decodes adversarial bodies against
 *      EVERY registered request and response descriptor, into a heap object
 *      sized exactly to the descriptor, so a sanitizer build fails on any
 *      write past the decoded object -- including for opcodes whose handlers
 *      are scaffolding today and decode nothing yet.
 */
#include "mtk_test.h"
#include "mtek_codec_api.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_opcode_registry.h"
#include <stdlib.h>
#include <string.h>

/* ---- 1. Regression: array-of-bytes element length ---------------------- */

/* mtk_beacon_start_req_t is { struct { uint32_t count; mtk_bytes32_t items[32]; } },
 * and mtk_bytes32_t is { uint16_t len; uint8_t data[32] }. The wire length
 * prefix for an element is one byte, so a body may declare 255 for an element
 * that can hold 32. Declaring it on the LAST element puts the write past the
 * end of the object entirely. */
static void test_array_of_bytes_element_length_is_bounded(void) {
    uint8_t body[1 + 31 + 1 + 255];
    size_t n = 0;
    body[n++] = 32;                              /* count == the field's max */
    for (int i = 0; i < 31; i++) body[n++] = 0;  /* elements 0..30 empty */
    body[n++] = 255;                             /* element 31 claims 255 of 32 */
    for (int i = 0; i < 255; i++) body[n++] = 0x41;

    mtk_beacon_start_req_t req;
    memset(&req, 0, sizeof(req));
    int rc = mtk_decode(&mtk_beacon_start_req_t_desc, &req, body, n, NULL);
    MTK_CHECK_EQ(rc, MTK_CODEC_OVERFLOW);
    /* Rejected before storing anything: the length field of the offending
     * element is untouched, not left describing bytes that were never
     * copied. */
    MTK_CHECK_EQ(req.ssids.items[31].len, 0);

    /* An over-long length on a NON-final element is the same defect even
     * though the write would land inside the object; it must be refused
     * identically rather than silently corrupting later elements. */
    uint8_t body2[1 + 1 + 255];
    size_t n2 = 0;
    body2[n2++] = 1;
    body2[n2++] = 255;
    for (int i = 0; i < 255; i++) body2[n2++] = 0x42;
    memset(&req, 0, sizeof(req));
    rc = mtk_decode(&mtk_beacon_start_req_t_desc, &req, body2, n2, NULL);
    MTK_CHECK_EQ(rc, MTK_CODEC_OVERFLOW);
    MTK_CHECK_EQ(req.ssids.items[0].len, 0);
    MTK_CHECK_EQ(req.ssids.items[1].len, 0);
}

/* A length exactly at capacity, and one just over it, on both array-of-bytes
 * messages -- proves the new bound is the element capacity and that valid
 * bodies still decode unchanged. */
static void test_array_of_bytes_boundary_values(void) {
    const uint16_t cap = 32; /* mtk_bytes32_t data[32] */

    uint8_t ok_body[1 + 1 + 1 + 32];
    size_t n = 0;
    ok_body[n++] = 2;        /* two elements */
    ok_body[n++] = 0;        /* element 0: empty */
    ok_body[n++] = (uint8_t)cap;
    for (int i = 0; i < cap; i++) ok_body[n++] = 0x5A;
    mtk_beacon_start_req_t req;
    memset(&req, 0, sizeof(req));
    MTK_CHECK_EQ(mtk_decode(&mtk_beacon_start_req_t_desc, &req, ok_body, n, NULL), MTK_CODEC_OK);
    MTK_CHECK_EQ(req.ssids.count, 2);
    MTK_CHECK_EQ(req.ssids.items[0].len, 0);
    MTK_CHECK_EQ(req.ssids.items[1].len, cap);
    MTK_CHECK_EQ(req.ssids.items[1].data[0], 0x5A);
    MTK_CHECK_EQ(req.ssids.items[1].data[cap - 1], 0x5A);

    uint8_t over_body[1 + 1 + 33];
    n = 0;
    over_body[n++] = 1;
    over_body[n++] = (uint8_t)(cap + 1);
    for (int i = 0; i < cap + 1; i++) over_body[n++] = 0x5A;
    memset(&req, 0, sizeof(req));
    MTK_CHECK_EQ(mtk_decode(&mtk_beacon_start_req_t_desc, &req, over_body, n, NULL), MTK_CODEC_OVERFLOW);

    /* PROBE_FLOOD_START uses the same element type behind a leading u8. */
    uint8_t pf[1 + 1 + 1 + 255];
    n = 0;
    pf[n++] = 6;    /* channel */
    pf[n++] = 1;    /* one ssid */
    pf[n++] = 255;  /* over capacity */
    for (int i = 0; i < 255; i++) pf[n++] = 0x43;
    mtk_probe_flood_start_req_t pfreq;
    memset(&pfreq, 0, sizeof(pfreq));
    MTK_CHECK_EQ(mtk_decode(&mtk_probe_flood_start_req_t_desc, &pfreq, pf, n, NULL), MTK_CODEC_OVERFLOW);
}

/* The encoder side of the same shape: an in-memory length wider than the
 * element capacity (or than the one-byte wire prefix) must be refused, not
 * narrowed into a frame whose declared length disagrees with its payload. */
static void test_encoder_rejects_oversized_element_length(void) {
    mtk_beacon_start_req_t req;
    memset(&req, 0, sizeof(req));
    req.ssids.count = 1;
    req.ssids.items[0].len = 200; /* > capacity 32 */
    uint8_t out[512];
    size_t out_len = 0;
    MTK_CHECK_EQ(mtk_encode(&mtk_beacon_start_req_t_desc, &req, out, sizeof(out), &out_len),
                 MTK_CODEC_OVERFLOW);

    req.ssids.items[0].len = 32; /* exactly at capacity round-trips */
    memset(req.ssids.items[0].data, 0x7E, 32);
    out_len = 0;
    MTK_CHECK_EQ(mtk_encode(&mtk_beacon_start_req_t_desc, &req, out, sizeof(out), &out_len), MTK_CODEC_OK);
    mtk_beacon_start_req_t back;
    memset(&back, 0, sizeof(back));
    MTK_CHECK_EQ(mtk_decode(&mtk_beacon_start_req_t_desc, &back, out, out_len, NULL), MTK_CODEC_OK);
    MTK_CHECK_EQ(back.ssids.count, 1);
    MTK_CHECK_EQ(back.ssids.items[0].len, 32);
    MTK_CHECK(memcmp(back.ssids.items[0].data, req.ssids.items[0].data, 32) == 0);
}

/* ---- 2. Property check over every registered descriptor ---------------- */

#define ADV_BODY_MAX 8192

/* Builds a body that is structurally plausible enough to reach deep fields
 * (bools stay legal, length prefixes stay within their own declared bound)
 * but pushes every length that the codec must bound itself to its extreme.
 * `hostile_elem_len` selects the adversarial array-of-bytes element length
 * that the wire format permits but the element cannot hold. */
static size_t build_body(const mtk_struct_desc_t *desc, uint8_t *out, size_t cap,
                         size_t pos, int hostile_elem_len);

static size_t put_n(uint8_t *out, size_t cap, size_t pos, uint8_t v, size_t n) {
    for (size_t i = 0; i < n && pos < cap; i++) out[pos++] = v;
    return pos;
}

static size_t build_field(const mtk_field_desc_t *f, uint8_t *out, size_t cap,
                          size_t pos, int hostile_elem_len) {
    switch (f->type) {
        case MTK_F_BOOL: return put_n(out, cap, pos, 0x01, 1);
        case MTK_F_U8: case MTK_F_I8: return put_n(out, cap, pos, 0xFF, 1);
        case MTK_F_U16: case MTK_F_I16: return put_n(out, cap, pos, 0xFF, 2);
        case MTK_F_U32: case MTK_F_I32: return put_n(out, cap, pos, 0xFF, 4);
        case MTK_F_U64: case MTK_F_I64: return put_n(out, cap, pos, 0xFF, 8);
        case MTK_F_MAC6: return put_n(out, cap, pos, 0xFF, 6);
        case MTK_F_IPV4: return put_n(out, cap, pos, 0xFF, 4);
        case MTK_F_BYTES_FIXED: return put_n(out, cap, pos, 0xFF, f->max);
        case MTK_F_BYTES:
        case MTK_F_UTF8: {
            /* Largest length this field's own check accepts, so the payload
             * copy runs at full width. */
            if (f->len_prefix == 2) {
                if (pos + 2 <= cap) { out[pos++] = (uint8_t)(f->max & 0xFF); out[pos++] = (uint8_t)(f->max >> 8); }
            } else {
                if (pos < cap) out[pos++] = (uint8_t)(f->max & 0xFF);
            }
            return put_n(out, cap, pos, 0xAB, f->max);
        }
        case MTK_F_STRUCT: return build_body(f->nested, out, cap, pos, hostile_elem_len);
        case MTK_F_ARRAY: {
            uint16_t count = f->max; /* the largest count the field accepts */
            if (f->len_prefix == 2) {
                if (pos + 2 <= cap) { out[pos++] = (uint8_t)(count & 0xFF); out[pos++] = (uint8_t)(count >> 8); }
            } else {
                if (pos < cap) out[pos++] = (uint8_t)(count & 0xFF);
            }
            for (uint16_t i = 0; i < count && pos < cap; i++) {
                if (f->elem_type == MTK_F_STRUCT) {
                    pos = build_body(f->nested, out, cap, pos, hostile_elem_len);
                } else if (f->elem_type == MTK_F_MAC6) {
                    pos = put_n(out, cap, pos, 0xFF, 6);
                } else if (f->elem_type == MTK_F_IPV4) {
                    pos = put_n(out, cap, pos, 0xFF, 4);
                } else if (f->elem_type == MTK_F_BYTES) {
                    /* The case the element capacity must bound: a one-byte
                     * wire prefix can name far more than the element holds. */
                    uint8_t blen = hostile_elem_len ? 0xFF
                                                    : (uint8_t)(f->elem_size > 2 ? f->elem_size - 2 : 0);
                    if (pos < cap) out[pos++] = blen;
                    pos = put_n(out, cap, pos, 0xCD, blen);
                } else {
                    unsigned w = 1;
                    switch (f->elem_type) {
                        case MTK_F_U16: case MTK_F_I16: w = 2; break;
                        case MTK_F_U32: case MTK_F_I32: w = 4; break;
                        case MTK_F_U64: case MTK_F_I64: w = 8; break;
                        case MTK_F_BOOL: w = 1; break;
                        default: w = 1; break;
                    }
                    pos = put_n(out, cap, pos, f->elem_type == MTK_F_BOOL ? 0x01 : 0xFF, w);
                }
            }
            return pos;
        }
        default: return pos;
    }
}

static size_t build_body(const mtk_struct_desc_t *desc, uint8_t *out, size_t cap,
                         size_t pos, int hostile_elem_len) {
    if (!desc) return pos;
    for (uint16_t i = 0; i < desc->field_count; i++) {
        pos = build_field(&desc->fields[i], out, cap, pos, hostile_elem_len);
    }
    return pos;
}

/* Decodes into a heap object sized exactly to the descriptor: under ASan any
 * store past the object's own end is a hard failure, which is the property
 * being asserted. The returned status is deliberately not constrained beyond
 * "one of the defined codes" -- the point is that no input corrupts memory. */
static void decode_into_exact_object(const mtk_struct_desc_t *desc, const uint8_t *body, size_t len) {
    size_t sz = desc->struct_size ? desc->struct_size : 1;
    uint8_t *obj = (uint8_t *)malloc(sz);
    MTK_CHECK(obj != NULL);
    if (!obj) return;
    memset(obj, 0, sz);
    size_t consumed = 0;
    int rc = mtk_decode(desc, obj, body, len, &consumed);
    MTK_CHECK(rc == MTK_CODEC_OK || rc == MTK_CODEC_OVERFLOW ||
              rc == MTK_CODEC_TRUNCATED || rc == MTK_CODEC_PROTOCOL_ERROR);
    /* A successful decode may never claim to have consumed more than it was
     * given. */
    if (rc == MTK_CODEC_OK) MTK_CHECK(consumed <= len);
    free(obj);
}

static void exercise_descriptor(const mtk_struct_desc_t *desc) {
    if (!desc) return;
    static uint8_t body[ADV_BODY_MAX];

    /* Structure-aware bodies: one that keeps every array-of-bytes element
     * within capacity, one that pushes each to the wire maximum. */
    for (int hostile = 0; hostile <= 1; hostile++) {
        size_t n = build_body(desc, body, sizeof(body), 0, hostile);
        decode_into_exact_object(desc, body, n);
        /* Truncation sweep: every prefix of that body must also be handled
         * without a write past the object. */
        for (size_t cut = 0; cut < n; cut += (n > 512 ? 37 : 1)) {
            decode_into_exact_object(desc, body, cut);
        }
    }

    /* Flat patterns, independent of the descriptor's own shape. */
    static const uint8_t fills[] = { 0x00, 0xFF, 0xAA, 0x01 };
    static const size_t lens[] = { 0, 1, 2, 3, 7, 16, 64, 255, 256, 1024, 4096 };
    for (unsigned fi = 0; fi < sizeof(fills) / sizeof(fills[0]); fi++) {
        for (unsigned li = 0; li < sizeof(lens) / sizeof(lens[0]); li++) {
            size_t n = lens[li] < sizeof(body) ? lens[li] : sizeof(body);
            memset(body, fills[fi], n);
            decode_into_exact_object(desc, body, n);
        }
    }
}

static void test_every_registered_descriptor_is_bounded(void) {
    unsigned req_seen = 0, resp_seen = 0;
    for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {
        const mtk_opcode_entry_t *e = &mtk_opcode_table[i];
        if (e->req_desc) { exercise_descriptor(e->req_desc); req_seen++; }
        if (e->resp_desc) { exercise_descriptor(e->resp_desc); resp_seen++; }
    }
    /* Not vacuous: the table really was walked, including the scaffolded
     * opcodes whose handlers decode nothing today. */
    MTK_CHECK(req_seen > 0);
    MTK_CHECK(resp_seen > 0);
    MTK_CHECK_EQ(MTK_OPCODE_COUNT, 109);
}

MTK_TEST_MAIN_BEGIN
    test_array_of_bytes_element_length_is_bounded();
    test_array_of_bytes_boundary_values();
    test_encoder_rejects_oversized_element_length();
    test_every_registered_descriptor_is_bounded();
MTK_TEST_MAIN_END
