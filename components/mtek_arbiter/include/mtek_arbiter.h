/* Clean-room implementation from MonstaTek contract (002-resource-arbiter.md).
 * Single-active-class radio arbiter: the arbiter tracks one platform-wide
 * active class at a time (Sec 2), not one slot per RADIO_OWNER_*. */
#pragma once
#include "mtek_arbiter_classes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MTK_ARB_GRANT_OK = 0,       /* acquired: caller may proceed */
    MTK_ARB_GRANT_BUSY,         /* rejected: wire status BUSY */
    MTK_ARB_GRANT_GUARDED,      /* Sec 3.1: caller must drive its own bounded
                                  * stop-then-acquire handoff (D->H only in
                                  * Phase 1); arbiter has NOT granted yet */
} mtk_arbiter_grant_t;

void mtk_arbiter_init(void);
void mtk_arbiter_reset(void); /* Sec 8: radio ownership released unconditionally on reset */

/* Optional lock hooks (ESP32 target glue only; default no-op -- safe for
 * every host test and for a target build with no async runner registered).
 * When registered, brackets the whole read-modify-write of the single
 * active-class/token pair below, since mtk_arbiter_acquire's own
 * check-then-set is otherwise racy the moment more than one FreeRTOS
 * worker can call it concurrently (RC5 independent audit P0: "Shared
 * runtime state is not concurrency-safe"). Mirrors mtk_core_set_lock's
 * identical pattern -- register both with the same underlying mutex. */
typedef void (*mtk_arbiter_lock_fn)(void);
void mtk_arbiter_set_lock(mtk_arbiter_lock_fn lock, mtk_arbiter_lock_fn unlock);

mtk_arbiter_policy_t mtk_arbiter_policy_for(mtk_arbiter_class_t a, mtk_arbiter_class_t b);

/* Attempts to acquire `cls` for `token`. See mtk_arbiter_grant_t. */
mtk_arbiter_grant_t mtk_arbiter_acquire(mtk_arbiter_class_t cls, uint32_t token);

/* Used only after a caller has itself confirmed the previously-active
 * guarded-transition class reached STOPPED within its own bound (Sec 3.1
 * success path): atomically hands the active-class slot to `cls`. */
void mtk_arbiter_force_transfer(mtk_arbiter_class_t cls, uint32_t token);

/* Sec 3.1 failure path: releases the active-class slot unconditionally
 * (sets it to NONE) without transferring to any new class. */
void mtk_arbiter_force_release(void);

void mtk_arbiter_release(mtk_arbiter_class_t cls); /* no-op unless cls is the active class */

/* P0 correction (follow-up read-only audit, "final P0 concurrency-closure
 * round", issue 1): every production call site used to check ownership
 * with `mtk_arbiter_active_token() == token` and, if true, separately
 * call `mtk_arbiter_release(cls)` -- two independent lock acquisitions
 * with a real (if narrow) window between them in which a concurrent
 * caller could reassign the class to a brand-new token, which the
 * unconditional `mtk_arbiter_release(cls)` would then release out from
 * under, even though the ownership check that gated it had already
 * become stale. This performs the check and the release in ONE lock
 * acquisition, closing that window entirely: returns 1 if `token` was
 * genuinely the current owner of `cls` (and it was just released), 0
 * otherwise (a no-op -- `cls` is either not active, or currently owned
 * by a DIFFERENT token, e.g. a newer operation's own lease, which this
 * call must never touch). Every production check-then-release pattern
 * (mtek_wifi_logic.c/mtek_ble_logic.c/mtek_capture_logic.c, including
 * mtek_wifi_service.h's own mtek_wifi_restore_and_release wrapper) now
 * uses this instead. */
int mtk_arbiter_release_if_owner(mtk_arbiter_class_t cls, uint32_t token);
mtk_arbiter_class_t mtk_arbiter_active_class(void);
uint32_t mtk_arbiter_active_token(void);
mtk_radio_owner_t mtk_arbiter_active_owner(void);

/* Pure class->owner mapping: takes no lock and reads no shared arbiter
 * state at all. A caller that already holds one coherent
 * mtk_arbiter_snapshot_t (below) should derive radio_owner from THAT
 * snapshot's own class through this helper, rather than a second,
 * independently-locked mtk_arbiter_active_owner() call that could
 * observe a different ownership moment -- see GET_WIFI_RECOVERY_STATE's
 * own use in mtek_wifi_logic.c. */
mtk_radio_owner_t mtk_arbiter_owner_for_class(mtk_arbiter_class_t cls);

/* mtk_arbiter_active_class()/mtk_arbiter_active_token() are each their
 * own independent lock acquisition -- a caller that needs BOTH fields to
 * describe the SAME ownership moment (any decision made from the pair
 * together, not just displaying each independently) calling them as two
 * separate reads can observe a torn combination: class from one moment,
 * token from a later one, after a concurrent worker changed ownership in
 * between. mtk_arbiter_snapshot() returns both fields from ONE lock
 * acquisition -- the only way to get a genuinely coherent pair. Prefer
 * this over the two separate getters whenever the caller's own logic
 * branches on, publishes, or compares both fields together. */
typedef struct {
    mtk_arbiter_class_t cls;
    uint32_t token;
} mtk_arbiter_snapshot_t;
mtk_arbiter_snapshot_t mtk_arbiter_snapshot(void);

#ifdef __cplusplus
}
#endif
