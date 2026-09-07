/* Clean-room implementation from MonstaTek contract. See
 * mtk_ble_op_generation.h for the design rationale (RC8 independent audit
 * P0-4 "Fix NimBLE timeout/late-callback ABA hazards"). */
#include "mtk_ble_op_generation.h"

void mtk_ble_op_generation_init(mtk_ble_op_generation_t *g) {
    g->current = 0;
    g->next = 1;
}

uint32_t mtk_ble_op_generation_arm(mtk_ble_op_generation_t *g) {
    uint32_t v = g->next++;
    if (g->next == 0) g->next = 1; /* wrap past 0 -- 0 stays reserved for "none armed" */
    g->current = v;
    return v;
}

int mtk_ble_op_generation_is_current(const mtk_ble_op_generation_t *g, uint32_t candidate) {
    return candidate != 0 && g->current == candidate;
}

void mtk_ble_op_generation_clear(mtk_ble_op_generation_t *g) {
    g->current = 0;
}
