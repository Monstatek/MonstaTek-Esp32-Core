/* Clean-room implementation from MonstaTek contract. See mtek_opcode_overlay.h
 * for the design rationale (item 5 (P1): a test-only opcode overlay seam, inert
 * in production). */
#include "mtek_opcode_overlay.h"
#include <stddef.h>

static const mtk_opcode_entry_t *s_overlay[MTK_OPCODE_OVERLAY_MAX];
static unsigned s_overlay_count;

void mtk_opcode_overlay_register(const mtk_opcode_entry_t *entry) {
    if (!entry) return;
    for (unsigned i = 0; i < s_overlay_count; i++) {
        if (s_overlay[i] == entry) return; /* idempotent */
    }
    if (s_overlay_count < MTK_OPCODE_OVERLAY_MAX) {
        s_overlay[s_overlay_count++] = entry;
    }
}

void mtk_opcode_overlay_clear(void) {
    s_overlay_count = 0;
}

const mtk_opcode_entry_t *mtk_opcode_overlay_find(uint16_t service_id, uint16_t opcode) {
    for (unsigned i = 0; i < s_overlay_count; i++) {
        if (s_overlay[i]->service_id == service_id && s_overlay[i]->opcode == opcode) {
            return s_overlay[i];
        }
    }
    return NULL;
}
