/* Clean-room implementation from MonstaTek contract
 * (schemas.json adapter_map.bedge_c3, 002-adapter-translation-matrix.md). */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t bedge_wire_opcode;
    uint16_t canonical_service_id;
    uint16_t canonical_opcode;
    const char *canonical_name;
} mtk_bedge_opcode_row_t;

extern const mtk_bedge_opcode_row_t mtk_bedge_opcode_table[];
extern const unsigned mtk_bedge_opcode_table_count;

/* Looks up every canonical opcode a given Bedge wire opcode can translate
 * to or from (a Bedge opcode is not always uniquely invertible -- e.g.
 * 0x0103 M1ESP_WIFI_SCAN answers both what this task models as
 * AP_SCAN_START and AP_SCAN_RESULTS_PAGE, since Bedge returns the full
 * result set synchronously in one RPC where the canonical core splits
 * start/poll into two opcodes). Returns the number of rows written into
 * `out` (capacity `max_out`). */
unsigned mtk_bedge_opcode_lookup(uint16_t bedge_wire_opcode, const mtk_bedge_opcode_row_t **out, unsigned max_out);

/* Attempts to recognize `buf` (the bytes of an AUTO-phase SPI discovery
 * transaction, per SPI_PROTOCOL_V1.md's "non-dispatching 512-byte
 * discovery phase") as a valid Bedge/C3 `m1_link` frame, per the wire
 * layout in `mtek_bedge_frame.h` (001-profile-bootstrap-feasibility.md
 * Sec 1: 8-byte header, magic 0x4D31 LE, CRC16-CCITT trailer). Returns 1
 * and recognized if `buf`/`len` contain one complete, CRC-valid Bedge
 * frame; 0 otherwise (including a valid-looking native SPI v1 frame,
 * whose disjoint magic bytes can never pass this check -- see
 * 001-profile-bootstrap-feasibility.md Sec 2 for the cross-profile
 * rejection argument this mirrors). */
int mtk_bedge_try_recognize_discovery_frame(const uint8_t *buf, unsigned len);

#ifdef __cplusplus
}
#endif
