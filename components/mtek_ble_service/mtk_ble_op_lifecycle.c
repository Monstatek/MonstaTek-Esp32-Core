/* Clean-room implementation from MonstaTek contract. See mtk_ble_op_lifecycle.h
 * for the full design rationale (item 1 (P0) "BLE callback lifetime": close the
 * check-to-use race a generation-only guard cannot). */
#include "mtk_ble_op_lifecycle.h"
#include <stddef.h>

static void lc_lock(mtk_ble_op_lifecycle_t *lc) { if (lc->lock) lc->lock(lc->lock_ctx); }
static void lc_unlock(mtk_ble_op_lifecycle_t *lc) { if (lc->unlock) lc->unlock(lc->lock_ctx); }

void mtk_ble_op_lifecycle_init(mtk_ble_op_lifecycle_t *lc) {
    mtk_ble_op_generation_init(&lc->gen);
    lc->published = NULL;
    lc->lock = NULL;
    lc->unlock = NULL;
    lc->lock_ctx = NULL;
}

void mtk_ble_op_lifecycle_set_lock(mtk_ble_op_lifecycle_t *lc,
                                   mtk_ble_lc_lock_fn lock, mtk_ble_lc_lock_fn unlock, void *lock_ctx) {
    lc->lock = lock;
    lc->unlock = unlock;
    lc->lock_ctx = lock_ctx;
}

uint32_t mtk_ble_op_lifecycle_arm(mtk_ble_op_lifecycle_t *lc, void *ctx) {
    lc_lock(lc);
    uint32_t gen = mtk_ble_op_generation_arm(&lc->gen);
    lc->published = ctx;
    lc_unlock(lc);
    return gen;
}

int mtk_ble_op_lifecycle_callback_begin(mtk_ble_op_lifecycle_t *lc, uint32_t candidate, void **ctx_out) {
    lc_lock(lc);
    if (!mtk_ble_op_generation_is_current(&lc->gen, candidate)) {
        lc_unlock(lc);
        return 0;
    }
    if (ctx_out) *ctx_out = lc->published;
    /* Deliberately return WITH THE LOCK STILL HELD -- the caller does all of its
     * context/semaphore work before callback_end releases it, so retire (which
     * must acquire this same lock) can never interleave into the callback's own
     * critical section. */
    return 1;
}

void mtk_ble_op_lifecycle_callback_end(mtk_ble_op_lifecycle_t *lc) {
    lc_unlock(lc);
}

void mtk_ble_op_lifecycle_retire(mtk_ble_op_lifecycle_t *lc) {
    lc_lock(lc);
    mtk_ble_op_generation_clear(&lc->gen);
    lc->published = NULL;
    lc_unlock(lc);
}
