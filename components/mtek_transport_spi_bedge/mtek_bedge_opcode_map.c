/* Clean-room implementation from MonstaTek contract. */
#include "mtek_bedge_opcode_map.h"
#include "mtek_bedge_frame.h"

unsigned mtk_bedge_opcode_lookup(uint16_t bedge_wire_opcode, const mtk_bedge_opcode_row_t **out, unsigned max_out) {
    unsigned n = 0;
    for (unsigned i = 0; i < mtk_bedge_opcode_table_count && n < max_out; i++) {
        if (mtk_bedge_opcode_table[i].bedge_wire_opcode == bedge_wire_opcode) out[n++] = &mtk_bedge_opcode_table[i];
    }
    return n;
}

int mtk_bedge_try_recognize_discovery_frame(const uint8_t *buf, unsigned len) {
    mtk_bedge_header_t hdr;
    const uint8_t *payload;
    return mtk_bedge_parse_bounded(buf, len, &hdr, &payload) == MTK_BEDGE_PARSE_OK;
}
