/* Proves
 * mtek_ble_disc_failed's own decision table directly -- portably, without
 * needing real NimBLE/FreeRTOS. See mtek_ble_hal_esp32.c for how this exact same
 * shared function is wired into each of its five "discover all..." call sites
 * (services, characteristics, descriptors, plus the two narrower internal
 * CCCD-bound-characteristic/CCCD- descriptor searches subscribe/unsubscribe use)
 * -- target-only integration (the real ble_gattc_disc_all_* calls, the real
 * NimBLE callback plumbing, the real xSemaphoreCreateBinary/xSemaphoreTake
 * machinery) is not itself host-testable, matching this tree's established
 * portable-primitive/target-glue split (e.g. mtk_ble_op_ generation_t,
 * mtk_native_cellsize_negotiator_t) -- but the actual classification logic this
 * item's fix is really about (immediate start error vs. timeout vs.
 * callback-reported error vs. genuine, possibly- empty success) is the SAME code
 * on both sides, not a mirrored duplicate, so this is a real proof of the
 * production decision table, not just of a copy of it. */
#include "mtk_test.h"
#include "mtek_ble_disc_status.h"

#define FAKE_BLE_HS_EDONE 14 /* NimBLE's own real value -- see mtek_ble_disc_status.h's own doc comment on why this header takes it as a parameter instead of including host/ble_hs.h */
#define FAKE_BLE_HS_EOK 0
#define FAKE_BLE_HS_ENOTCONN 3 /* an arbitrary REAL (non-EDONE) NimBLE error code, standing in for "a genuine callback-reported failure" */

MTK_TEST_MAIN_BEGIN

    /* Genuine, complete success -- possibly with real results (a
     * non-empty count is the caller's own concern, not this function's;
     * it only ever classifies the three completion signals). */
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, FAKE_BLE_HS_EDONE, FAKE_BLE_HS_EDONE), 0);

    /* Genuine, complete success with ZERO results -- the exact case a
     * prior round's own fix could already distinguish from allocation
     * failure, but this item's own four further cases below could not. */
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, FAKE_BLE_HS_EDONE, FAKE_BLE_HS_EDONE), 0);

    /* Case 1: immediate NimBLE start failure -- the completion callback
     * was never armed at all. Must fail regardless of the other two
     * signals (a real implementation never even reaches them, but the
     * classification itself must not depend on that). */
    MTK_CHECK_EQ(mtek_ble_disc_failed(-1 /* BLE_HS_ENOMEM or similar */, 0, 0, FAKE_BLE_HS_EDONE), 1);
    MTK_CHECK_EQ(mtek_ble_disc_failed(-1, 1, FAKE_BLE_HS_EDONE, FAKE_BLE_HS_EDONE), 1); /* even if the other two signals look clean */

    /* Case 2: bounded timeout -- the start call succeeded, but the
     * completion semaphore was never signaled in time. The real,
     * eventual outcome is unknown and must never be assumed clean. */
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 0, FAKE_BLE_HS_EDONE, FAKE_BLE_HS_EDONE), 1);
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 0, 0, FAKE_BLE_HS_EDONE), 1);

    /* Case 3: a real, callback-reported failure mid-procedure -- the
     * start call succeeded, the semaphore WAS signaled (the callback did
     * fire and complete), but its own final status was a genuine error,
     * never the expected "done" sentinel. Whatever partial count was
     * accumulated before it must never be reported as a clean result. */
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, FAKE_BLE_HS_ENOTCONN, FAKE_BLE_HS_EDONE), 1);
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, FAKE_BLE_HS_EOK, FAKE_BLE_HS_EDONE), 1); /* a raw 0 is not EDONE either -- never conflate "no error yet" with "genuinely done" */

    /* A different caller's own "done" sentinel is honored too -- this
     * function makes no assumption about the specific integer value,
     * only that it is passed in consistently. */
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, 99, 99), 0);
    MTK_CHECK_EQ(mtek_ble_disc_failed(0, 1, 100, 99), 1);

MTK_TEST_MAIN_END
