/* Clean-room implementation from MonstaTek contract. */
#include "mtek_compat_opcode_map.h"
#include "mtek_compat_frame.h"

int mtk_compat_try_recognize_discovery_frame(const uint8_t *buf, unsigned len) {
    mtk_compat_header_t hdr;
    const uint8_t *payload;
    return mtk_compat_parse_bounded(buf, len, &hdr, &payload) == MTK_COMPAT_PARSE_OK;
}
