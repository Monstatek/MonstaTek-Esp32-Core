/* Clean-room implementation from MonstaTek contract (002-resource-arbiter.md). */
#include "mtek_arbiter.h"
#include <stddef.h>

extern const mtk_radio_owner_t mtk_arbiter_owner[];
extern const int mtk_arbiter_class_count;
typedef struct { mtk_arbiter_class_t a, b; mtk_arbiter_policy_t policy; } mtk_pair_entry_t;
extern const mtk_pair_entry_t mtk_arbiter_pairwise[];
extern const int mtk_arbiter_pairwise_count;
extern const mtk_pair_entry_t mtk_arbiter_self_pairs[];
extern const int mtk_arbiter_self_pairs_count;

static mtk_arbiter_class_t s_active = MTK_ARB_NONE;
static uint32_t s_active_token;

static mtk_arbiter_lock_fn s_lock, s_unlock;
void mtk_arbiter_set_lock(mtk_arbiter_lock_fn lock, mtk_arbiter_lock_fn unlock) { s_lock = lock; s_unlock = unlock; }
static void arb_lock(void) { if (s_lock) s_lock(); }
static void arb_unlock(void) { if (s_unlock) s_unlock(); }

void mtk_arbiter_init(void) {
    s_active = MTK_ARB_NONE;
    s_active_token = 0;
}

void mtk_arbiter_reset(void) {
    arb_lock();
    s_active = MTK_ARB_NONE;
    s_active_token = 0;
    arb_unlock();
}

mtk_arbiter_policy_t mtk_arbiter_policy_for(mtk_arbiter_class_t a, mtk_arbiter_class_t b) {
    if (a == b) {
        for (int i = 0; i < mtk_arbiter_self_pairs_count; i++) {
            if (mtk_arbiter_self_pairs[i].a == a) return mtk_arbiter_self_pairs[i].policy;
        }
        return MTK_POLICY_BUSY;
    }
    for (int i = 0; i < mtk_arbiter_pairwise_count; i++) {
        const mtk_pair_entry_t *e = &mtk_arbiter_pairwise[i];
        if ((e->a == a && e->b == b) || (e->a == b && e->b == a)) return e->policy;
    }
    return MTK_POLICY_CROSS_SUBSYSTEM_BUSY; /* conservative default: never silently grant an unlisted pair */
}

mtk_arbiter_grant_t mtk_arbiter_acquire(mtk_arbiter_class_t cls, uint32_t token) {
    arb_lock();
    /* Check-then-set must be one atomic critical section: two concurrent
     * callers both observing MTK_ARB_NONE and both granting themselves
     * ownership is exactly the race a single-active-owner arbiter exists
     * to prevent. */
    if (s_active == MTK_ARB_NONE) {
        s_active = cls;
        s_active_token = token;
        arb_unlock();
        return MTK_ARB_GRANT_OK;
    }
    mtk_arbiter_policy_t p = mtk_arbiter_policy_for(s_active, cls);
    arb_unlock();
    if (p == MTK_POLICY_GUARDED) return MTK_ARB_GRANT_GUARDED;
    return MTK_ARB_GRANT_BUSY;
}

void mtk_arbiter_force_transfer(mtk_arbiter_class_t cls, uint32_t token) {
    arb_lock();
    s_active = cls;
    s_active_token = token;
    arb_unlock();
}

void mtk_arbiter_force_release(void) {
    arb_lock();
    s_active = MTK_ARB_NONE;
    s_active_token = 0;
    arb_unlock();
}

void mtk_arbiter_release(mtk_arbiter_class_t cls) {
    arb_lock();
    if (s_active == cls) {
        s_active = MTK_ARB_NONE;
        s_active_token = 0;
    }
    arb_unlock();
}

int mtk_arbiter_release_if_owner(mtk_arbiter_class_t cls, uint32_t token) {
    arb_lock();
    int released = 0;
    if (s_active == cls && s_active_token == token) {
        s_active = MTK_ARB_NONE;
        s_active_token = 0;
        released = 1;
    }
    arb_unlock();
    return released;
}

mtk_arbiter_class_t mtk_arbiter_active_class(void) {
    arb_lock();
    mtk_arbiter_class_t c = s_active;
    arb_unlock();
    return c;
}
uint32_t mtk_arbiter_active_token(void) {
    arb_lock();
    uint32_t t = s_active_token;
    arb_unlock();
    return t;
}

mtk_radio_owner_t mtk_arbiter_active_owner(void) {
    arb_lock();
    mtk_arbiter_class_t c = s_active;
    arb_unlock();
    if (c == MTK_ARB_NONE) return MTK_RADIO_OWNER_NONE;
    return mtk_arbiter_owner[c];
}
