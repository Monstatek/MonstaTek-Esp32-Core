/* Clean-room implementation from MonstaTek contract.
 *
 * RC12 hardening round, item 1 (P0) "BLE callback lifetime": the prior
 * generation-only guard (mtk_ble_op_generation.h) closed the ABA hazard
 * of a late callback touching a REUSED static context, but it did NOT
 * close the check-to-use race: a NimBLE callback could evaluate
 * mtk_ble_op_generation_is_current() == true and snapshot the shared
 * context pointer, and THEN -- before it dereferenced that pointer or
 * signalled the semaphore -- be preempted while the waiting task's own
 * timeout path cleared the generation, nulled the context, and deleted
 * the semaphore, so the resumed callback wrote freed stack storage and/or
 * gave a deleted semaphore handle. Making the generation fields atomic
 * cannot fix this: atomicity of the CHECK does not extend the context's
 * lifetime across the USE.
 *
 * This is a single coherent lifetime protocol built around one lock that
 * brackets ALL of: generation transitions, context publication, callback
 * entry + use + semaphore signalling, and teardown. The rules it enforces:
 *
 *  - arm(ctx): under the lock, mint a new generation and publish `ctx`;
 *    the lock is released before returning (so the caller starts the real
 *    NimBLE operation WITHOUT the lock held -- a NimBLE start call can
 *    synchronously re-enter the callback, and holding the lock across it
 *    would self-deadlock the callback's own callback_begin).
 *
 *  - callback_begin(candidate, &ctx): takes the lock; if `candidate` is
 *    still the current generation it returns 1 WITH THE LOCK STILL HELD
 *    and hands back the currently-published context; otherwise it releases
 *    the lock and returns 0. A callback that gets 1 does all of its
 *    context access AND its semaphore give while still inside this locked
 *    region, then calls callback_end().
 *
 *  - retire(): takes the lock, clears the generation, and unpublishes the
 *    context, then releases the lock. Because a callback holds the same
 *    lock for the whole span between its is-current check and its
 *    callback_end(), retire() CANNOT interleave into that span: it either
 *    runs entirely before the callback's callback_begin (so the callback
 *    then sees a stale generation and never touches the context) or
 *    entirely after the callback's callback_end (so the callback has
 *    already finished every context/semaphore access). Once retire()
 *    returns, no callback can ever observe the retired generation as
 *    current again, so the caller may now safely delete the semaphore and
 *    let the (stack-local) context go out of scope.
 *
 * Portable and host-testable: no NimBLE/FreeRTOS dependency. Thread safety
 * is via caller-supplied lock/unlock hooks (a real recursion-free mutex on
 * target; a pthread mutex in host tests) -- with no hooks registered, the
 * primitive is still correct for a single thread of control (every
 * host test that does not explicitly install hooks). The lock is NEVER
 * held across anything slow or re-entrant -- only across the small,
 * bounded critical sections above. */
#pragma once
#include <stdint.h>
#include "mtk_ble_op_generation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mtk_ble_lc_lock_fn)(void *lock_ctx);

typedef struct {
    mtk_ble_op_generation_t gen;
    void *published;          /* the context the CURRENT generation's callback may touch, or NULL */
    mtk_ble_lc_lock_fn lock;
    mtk_ble_lc_lock_fn unlock;
    void *lock_ctx;
} mtk_ble_op_lifecycle_t;

void mtk_ble_op_lifecycle_init(mtk_ble_op_lifecycle_t *lc);

/* Installs the mutual-exclusion hooks that bracket every critical section
 * below. Must be called before the lifecycle is shared across the NimBLE
 * host task and any waiting task. */
void mtk_ble_op_lifecycle_set_lock(mtk_ble_op_lifecycle_t *lc,
                                   mtk_ble_lc_lock_fn lock, mtk_ble_lc_lock_fn unlock, void *lock_ctx);

/* Arms a new operation, publishing `ctx` as the context its callback may
 * later touch. Returns the new generation to pass as the NimBLE
 * registration call's own `arg` (cast through (void*)(uintptr_t)). The
 * lifetime lock is NOT held on return. */
uint32_t mtk_ble_op_lifecycle_arm(mtk_ble_op_lifecycle_t *lc, void *ctx);

/* Callback entry. Takes the lifetime lock. If `candidate` is still the
 * current armed generation, sets *ctx_out to the published context and
 * returns 1 WITH THE LOCK STILL HELD -- the caller must do all of its
 * context access and any semaphore give, then call callback_end(). If
 * `candidate` is stale (retired or superseded), releases the lock and
 * returns 0 -- the caller must not touch the context. */
int mtk_ble_op_lifecycle_callback_begin(mtk_ble_op_lifecycle_t *lc, uint32_t candidate, void **ctx_out);

/* Releases the lifetime lock taken by a callback_begin() that returned 1.
 * Must be paired 1:1 with such a call. */
void mtk_ble_op_lifecycle_callback_end(mtk_ble_op_lifecycle_t *lc);

/* Retires the current operation (natural completion, timeout, or a start/
 * allocation failure that never armed a real callback). After this
 * returns, no in-flight or future callback can access the published
 * context, so the caller may delete the semaphore and release/reuse the
 * context storage. Idempotent. */
void mtk_ble_op_lifecycle_retire(mtk_ble_op_lifecycle_t *lc);

#ifdef __cplusplus
}
#endif
