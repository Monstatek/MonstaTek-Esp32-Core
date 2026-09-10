/* Clean-room implementation from MonstaTek contract.
 *
 * RC8 independent audit P0-4 "Fix NimBLE timeout/late-callback ABA
 * hazards": the ESP32 NimBLE HAL (mtek_ble_hal_esp32.c) arms a static
 * per-operation context (scan/connect/discover/read/write/CCCD-discovery/
 * characteristic-bound), waits on a semaphore with a timeout, then --
 * whether or not the semaphore was actually given -- clears its own
 * global context pointer and deletes the semaphore. A NimBLE controller
 * callback that is still in flight when the timeout elapses (the
 * underlying GATT/GAP procedure is not actually cancelled just because
 * the application gave up waiting) can therefore still fire AFTER a
 * brand-new operation has re-armed the SAME static context storage --
 * the late callback from operation A then reads/writes operation B's own
 * data. Real, not theoretical: this HAL's own context storage for
 * discover/read/write/CCCD-discovery/characteristic-bound is a function-
 * local `static` (the exact same memory address on every call), so a
 * second call re-arms the identical bytes a still-pending first call's
 * callback might resume writing into.
 *
 * This is the generation-validated per-operation slot the audit's own
 * required correction names: every arm() call mints a new, never-reused
 * generation value, which the caller passes as the NimBLE callback's own
 * `arg` parameter (already plumbed through by every NimBLE GAP/GATT
 * registration call, previously always NULL and ignored). NimBLE's own
 * callback dispatch guarantees a given invocation always carries the
 * exact `arg` value registered for THAT SPECIFIC procedure -- a late
 * callback from operation A therefore always carries A's own (by then
 * superseded) generation, letting is_current() reject it before it ever
 * touches the (possibly-already-reused-for-B) shared context, regardless
 * of how the underlying storage happens to be reused.
 *
 * Portable, host-testable: no NimBLE/FreeRTOS dependency. */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t current;
    uint32_t next;
} mtk_ble_op_generation_t;

void mtk_ble_op_generation_init(mtk_ble_op_generation_t *g);

/* Arms a new operation. Returns the new generation value -- pass this as
 * the NimBLE registration call's own `arg` parameter (cast through
 * (void*)(uintptr_t)), and have the callback pass the same value (cast
 * back) to is_current() before touching any shared context. Never
 * returns 0 (0 is reserved as "no generation currently armed" so a
 * callback given a raw NULL `arg` some other way can never accidentally
 * compare equal to a real armed generation). */
uint32_t mtk_ble_op_generation_arm(mtk_ble_op_generation_t *g);

/* True if `candidate` is still the CURRENT armed generation. A stale
 * generation (superseded by a later arm(), or explicitly cleared) always
 * returns false, regardless of what the underlying context storage now
 * holds. */
int mtk_ble_op_generation_is_current(const mtk_ble_op_generation_t *g, uint32_t candidate);

/* Explicitly retires the current generation (natural completion or
 * timeout) so a callback arriving after this point -- even before any
 * NEW operation calls arm() again -- is never mistaken for current. */
void mtk_ble_op_generation_clear(mtk_ble_op_generation_t *g);

#ifdef __cplusplus
}
#endif
