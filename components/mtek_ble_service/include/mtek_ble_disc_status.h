/* Clean-room implementation from MonstaTek contract.
 *
 * RC11 round 10 P0 correction, item 4 "propagate real NimBLE discovery
 * failures": every "discover all ..." NimBLE procedure mtek_ble_hal_
 * esp32.c wraps (services, characteristics, descriptors, plus the two
 * narrower internal CCCD-bound-characteristic and CCCD-descriptor
 * searches subscribe/unsubscribe use) observes the same three signals --
 * the procedure's own immediate start return code, whether its completion
 * semaphore was ever actually signaled (vs. a bounded timeout), and the
 * FINAL ble_gatt_error.status the completion callback ever recorded
 * (NimBLE's own BLE_HS_EDONE for a genuine, complete termination; any
 * other nonzero value is a real, callback-reported failure mid-procedure)
 * -- and must classify them into "trust the accumulated count" or "report
 * a failure regardless of whatever partial count was accumulated"
 * identically every time. A prior round's own fix (Round 9, item 5)
 * closed allocation failure (xSemaphoreCreateBinary returning NULL) but
 * left these three further, real cases silently reporting a fabricated
 * empty success: an immediate start failure (the completion callback is
 * NEVER armed at all), a semaphore timeout (the procedure's own real
 * outcome is simply unknown), and a genuine callback-reported error
 * partway through (whatever was collected before it must never be
 * trusted). Extracted here, once, so all five call sites in mtek_ble_
 * hal_esp32.c can never drift from each other or from this decision
 * table, and so the table itself is directly host-testable --
 * mtek_ble_hal_esp32.c is ESP-IDF/NimBLE-only and not linked into host
 * tests at all, so this is the only part of item 4's own fix that can be
 * exercised outside real hardware/NimBLE.
 *
 * Portable, host-testable: no NimBLE/FreeRTOS dependency -- NimBLE's own
 * BLE_HS_EDONE constant (14) is passed in by the caller as `done_status`
 * rather than pulled in from host/ble_hs.h, so this header needs nothing
 * from that stack, mirroring mtk_ble_op_generation.h's own identical
 * portability discipline in this same component. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 if a "discover all ..." procedure's own outcome must be
 * reported as a failure (the canonical service layer must never trust
 * whatever count was accumulated), 0 if it genuinely, completely finished
 * and the accumulated count is honest (possibly, legitimately, zero --
 * a real peer with no matching results).
 *
 *   start_rc          : the NimBLE call's own immediate return code
 *                        (nonzero = it never even started; the
 *                        completion callback will never fire for this
 *                        attempt at all).
 *   semaphore_signaled : nonzero only if the completion semaphore was
 *                        actually given before the caller's own bounded
 *                        wait timed out (0 = timeout: the procedure never
 *                        genuinely reported completion in time; its real
 *                        outcome is simply unknown, never assumed clean).
 *   final_status       : the LAST ble_gatt_error.status the completion
 *                         callback ever observed. Only meaningful when
 *                         start_rc==0 and semaphore_signaled is nonzero;
 *                         ignored otherwise.
 *   done_status        : the expected "genuinely, completely done"
 *                         sentinel value (the caller's own BLE_HS_EDONE),
 *                         passed in rather than included here. */
static inline int mtek_ble_disc_failed(int start_rc, int semaphore_signaled, int final_status, int done_status) {
    if (start_rc != 0) return 1;         /* immediate NimBLE start failure -- callback never armed */
    if (!semaphore_signaled) return 1;   /* bounded timeout -- real outcome unknown */
    if (final_status != done_status) return 1; /* a real, callback-reported failure, not a clean completion */
    return 0;                            /* genuine completion -- trust the accumulated count, even if 0 */
}

#ifdef __cplusplus
}
#endif
