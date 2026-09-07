/* RC12 scheduler-fix round (confirmed real M1 hardware root cause):
 * wifi_promisc_tick_task's own vTaskDelay(pdMS_TO_TICKS(5)) computed to
 * vTaskDelay(0) at this build's CONFIG_FREERTOS_HZ=100 (pdMS_TO_TICKS
 * truncates: (5*100)/1000 == 0), which never actually blocks -- FreeRTOS
 * vTaskDelay(0) only yields to equal-priority ready tasks -- so a
 * priority-6 task calling it every iteration stayed continuously ready on
 * the single-core ESP32-C6 forever, starving the idle task the FreeRTOS
 * task watchdog itself depends on running (reproduced on real hardware as
 * a repeating ~5s CPU0 task-watchdog dump). MTK_CLAMP_MIN_ONE_TICK
 * (mtek_hal_common.h) is the fix's own policy macro; this is a pure host
 * test of THAT policy in isolation (integer arithmetic only, no FreeRTOS
 * dependency, matching this suite's own "host tests need no target
 * toolchain" design) -- it does not, and cannot, fabricate coverage of
 * app_main.c's own task scheduling, which is not something a host test
 * can meaningfully exercise. What it proves instead: for every
 * (requested_ms, tick_rate_hz) pair a real target build could plausibly
 * use, the SAME truncating formula FreeRTOS's own pdMS_TO_TICKS uses,
 * clamped through this macro, is never 0, and never silently changes an
 * already-safe (>=1 tick) value. */
#include "mtk_test.h"
#include "mtek_hal_common.h"

/* Mirrors FreeRTOS's own pdMS_TO_TICKS exactly: (ms * tick_rate_hz) / 1000,
 * integer division truncating toward zero. Reimplemented here (not
 * included from a FreeRTOS header) because this suite is deliberately
 * host-portable with no FreeRTOS/target-toolchain dependency -- see this
 * file's own top-of-file doc comment. */
static long raw_ticks(long ms, long tick_rate_hz) {
    return (ms * tick_rate_hz) / 1000;
}

MTK_TEST_MAIN_BEGIN

    /* The exact real-hardware incident: 5ms requested at this project's
     * own shipped CONFIG_FREERTOS_HZ=100 truncates to 0 raw ticks: the
     * clamp must turn that into exactly 1, not 0 and not something larger
     * than necessary. */
    MTK_CHECK_EQ(raw_ticks(5, 100), 0);
    MTK_CHECK_EQ(MTK_CLAMP_MIN_ONE_TICK(raw_ticks(5, 100)), 1);

    /* At a high enough tick rate, 5ms no longer truncates to 0 -- the
     * clamp must be a true no-op here, preserving the originally-intended
     * value exactly, not silently inflating it. */
    MTK_CHECK_EQ(raw_ticks(5, 1000), 5);
    MTK_CHECK_EQ(MTK_CLAMP_MIN_ONE_TICK(raw_ticks(5, 1000)), 5);

    /* Already-safe inputs (>=1) must never be altered by the clamp, at any
     * tick rate. */
    MTK_CHECK_EQ(MTK_CLAMP_MIN_ONE_TICK(1), 1);
    MTK_CHECK_EQ(MTK_CLAMP_MIN_ONE_TICK(10), 10);
    MTK_CHECK_EQ(MTK_CLAMP_MIN_ONE_TICK(60000), 60000);

    /* Exhaustive sweep: every millisecond value from 0 to 20 (comfortably
     * spanning this project's own shortest real production delay, 5ms;
     * see main/app_main.c's own wifi_promisc_tick_task) against every tick
     * rate this project's sdkconfig has ever plausibly used or could use
     * (100Hz, the current shipped value; 1000Hz, FreeRTOS's own common
     * default elsewhere) -- the clamped result must always be >=1, and it
     * must equal max(1, raw) exactly, never anything else. */
    static const long kTickRatesHz[] = {100, 1000};
    for (unsigned r = 0; r < sizeof(kTickRatesHz) / sizeof(kTickRatesHz[0]); r++) {
        for (long ms = 0; ms <= 20; ms++) {
            long raw = raw_ticks(ms, kTickRatesHz[r]);
            long clamped = MTK_CLAMP_MIN_ONE_TICK(raw);
            MTK_CHECK(clamped >= 1);
            MTK_CHECK_EQ(clamped, (raw < 1) ? 1 : raw);
        }
    }

MTK_TEST_MAIN_END
