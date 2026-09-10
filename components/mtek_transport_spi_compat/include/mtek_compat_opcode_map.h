/* Clean-room implementation from MonstaTek contract
 * (schemas.json adapter_map.compat_spi, 002-adapter-translation-matrix.md). */
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
 * transaction, per SPI_PROTOCOL_V1.md's "non-dispatching 512-byte
 * discovery phase") as a valid Legacy SPI Compatibility `m1_link` frame, per the wire
 * layout in `mtek_compat_frame.h` (001-profile-bootstrap-feasibility.md
 * Sec 1: 8-byte header, magic 0x4D31 LE, CRC16-CCITT trailer). Returns 1
 * and recognized if `buf`/`len` contain one complete, CRC-valid Legacy SPI Compatibility
 * frame; 0 otherwise (including a valid-looking native SPI v1 frame,
 * whose disjoint magic bytes can never pass this check -- see
 * 001-profile-bootstrap-feasibility.md Sec 2 for the cross-profile
 * rejection argument this mirrors). */
int mtk_compat_try_recognize_discovery_frame(const uint8_t *buf, unsigned len);

#ifdef __cplusplus
}
#endif
