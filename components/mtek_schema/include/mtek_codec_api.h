/* Clean-room implementation from MonstaTek contract. Hand-written codec API
 * over the generated field-descriptor tables (mtek_schema_codec.h). */
#pragma once
#include "mtek_schema_codec.h"
#include <stddef.h>
#include <stdint.h>

enum {
    MTK_CODEC_OK = 0,
    MTK_CODEC_OVERFLOW = 1,
    MTK_CODEC_TRUNCATED = 2,
    MTK_CODEC_PROTOCOL_ERROR = 3,
};

/* Encodes `obj` (a pointer to the generated C struct matching `desc`) into
 * `out` (capacity `cap` bytes). Returns an MTK_CODEC_* result; `*out_len`
 * receives the number of bytes written on MTK_CODEC_OK. `desc` may be NULL
 * for an empty message (writes zero bytes). */
int mtk_encode(const mtk_struct_desc_t *desc, const void *obj, uint8_t *out, size_t cap, size_t *out_len);

/* Decodes `len` bytes at `in` into `obj` per `desc`. Rejects any bool byte
 * that is not exactly 0x00/0x01, any length/count exceeding its field's
 * bound, and any truncated buffer -- never partially executes a malformed
 * message (002-canonical-core-contract.md Sec 1, Sec "Parser requirements"). */
int mtk_decode(const mtk_struct_desc_t *desc, void *obj, const uint8_t *in, size_t len, size_t *consumed);
