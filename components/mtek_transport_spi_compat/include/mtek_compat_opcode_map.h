/* Clean-room implementation from MonstaTek contract (schemas.json
 * adapter_map.compat_c3,). */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t compat_wire_opcode;
    uint16_t canonical_service_id;
    uint16_t canonical_opcode;
    const char *canonical_name;
} mtk_compat_opcode_row_t;

extern const mtk_compat_opcode_row_t mtk_compat_opcode_table[];
extern const unsigned mtk_compat_opcode_table_count;

/* Attempts to recognize `buf` (the bytes of an AUTO-phase SPI discovery
 * transaction, per SPI_PROTOCOL_V1.md's "non-dispatching 512-byte discovery
 * phase") as a valid Mtek Compatibility/C3 `m1_link` frame, per the wire layout
 * in `mtek_compat_frame.h`. Returns 1 and recognized if `buf`/`len` contain one
 * complete, CRC-valid Mtek Compatibility frame; 0 otherwise (including a
 * valid-looking native SPI v1 frame, whose disjoint magic bytes can never pass
 * this check -- see for the cross-profile rejection argument this mirrors). */
int mtk_compat_try_recognize_discovery_frame(const uint8_t *buf, unsigned len);

#ifdef __cplusplus
}
#endif
