/* Canonical core operation-token lifecycle: ACCEPTED->RUNNING->terminal
 * transitions, terminal stickiness (idempotent, no re-transition), stale-epoch
 * NOT_FOUND, and the 60s retention window / eviction-under-pressure rule. */
#include "mtk_test.h"
#include "mtek_core.h"

MTK_TEST_MAIN_BEGIN

    mtk_core_init(1000);
    MTK_CHECK_EQ(mtk_core_boot_epoch(), 1000);

    int no_mem = 0;
    mtk_operation_record_t *rec = mtk_op_alloc(0x0001, 0x0001, 0, &no_mem);
    MTK_CHECK(rec != NULL);
    MTK_CHECK_EQ(no_mem, 0);
    MTK_CHECK(rec->token != 0);
    MTK_CHECK_EQ(rec->state, MTK_OPS_ACCEPTED);

    uint32_t tok = rec->token;
    MTK_CHECK(mtk_op_find(tok, 1000) == rec);
    MTK_CHECK(mtk_op_find(tok, 999) == NULL); /* stale epoch -> NOT_FOUND-equivalent */
    MTK_CHECK(mtk_op_find(0, 1000) == NULL);  /* token 0 never valid */

    mtk_op_transition(rec, MTK_OPS_RUNNING, MTK_STATUS_OK, 10);
    MTK_CHECK_EQ(rec->state, MTK_OPS_RUNNING);
    mtk_op_transition(rec, MTK_OPS_COMPLETED, MTK_STATUS_OK, 20);
    MTK_CHECK_EQ(rec->state, MTK_OPS_COMPLETED);
    MTK_CHECK_EQ(rec->terminal_at_ms, 20);

    /* Idempotent: a repeat transition after terminal is a no-op. */
    mtk_op_transition(rec, MTK_OPS_STOPPED, MTK_STATUS_CANCELLED, 30);
    MTK_CHECK_EQ(rec->state, MTK_OPS_COMPLETED);
    MTK_CHECK_EQ(rec->final_status, MTK_STATUS_OK);

    /* Retention: still queryable just under 60s after terminal, evicted at/after it. */
    MTK_CHECK(mtk_op_find(tok, 1000) != NULL);
    mtk_op_gc(20 + MTK_OP_RETENTION_MS - 1);
    MTK_CHECK(mtk_op_find(tok, 1000) != NULL);
    mtk_op_gc(20 + MTK_OP_RETENTION_MS);
    MTK_CHECK(mtk_op_find(tok, 1000) == NULL);

    /* Budget exhaustion: fill all 8 slots with RUNNING (non-terminal, never
     * evicted), then confirm NO_MEMORY on the 9th. */
    mtk_core_init(2000);
    mtk_operation_record_t *recs[MTK_BUDGET_MAX_OPERATION_TOKENS];
    for (int i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        recs[i] = mtk_op_alloc(0x0001, 0x0001, 0, &no_mem);
        MTK_CHECK(recs[i] != NULL);
        mtk_op_transition(recs[i], MTK_OPS_RUNNING, MTK_STATUS_OK, 0);
    }
    mtk_operation_record_t *overflow = mtk_op_alloc(0x0001, 0x0001, 0, &no_mem);
    MTK_CHECK(overflow == NULL);
    MTK_CHECK_EQ(no_mem, 1);

    /* Eviction under pressure: mark one terminal, then a new alloc should
     * reuse that slot rather than reporting NO_MEMORY. */
    mtk_op_transition(recs[0], MTK_OPS_FAILED, MTK_STATUS_INTERNAL_ERROR, 5);
    mtk_operation_record_t *reused = mtk_op_alloc(0x0002, 0x0002, 6, &no_mem);
    MTK_CHECK(reused != NULL);
    MTK_CHECK_EQ(no_mem, 0);

    /* Reset invalidates every token from the prior epoch . */
    mtk_core_init(3000);
    mtk_operation_record_t *r2 = mtk_op_alloc(0x0001, 0x0001, 0, &no_mem);
    uint32_t tok2 = r2->token;
    mtk_core_reset(3001);
    MTK_CHECK(mtk_op_find(tok2, 3000) == NULL);

MTK_TEST_MAIN_END
