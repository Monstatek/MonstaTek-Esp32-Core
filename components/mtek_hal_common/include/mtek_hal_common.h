/* Clean-room implementation from MonstaTek contract. Shared HAL-facing
 * address types, included by both mtek_wifi_hal.h and mtek_ble_hal.h so
 * the two service HAL interfaces don't each define a colliding
 * mtk_hal_mac6_t. */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { uint8_t b[6]; } mtk_hal_mac6_t;
typedef struct { uint8_t b[4]; } mtk_hal_ipv4_t;

/* FreeRTOS's own pdMS_TO_TICKS(ms) is (ms * configTICK_RATE_HZ) / 1000, integer
 * division that truncates toward zero -- at a low enough tick rate (this
 * project's own CONFIG_FREERTOS_HZ=100, 10ms/tick) a short requested delay (e.g.
 * 5ms) computes to 0 ticks, and vTaskDelay(0) never actually blocks (it only
 * yields to equal-priority ready tasks), so a periodic task using it remains
 * continuously ready and starves every lower-or-equal-priority task --
 * including, on a single-core target, the idle task the FreeRTOS task watchdog
 * itself depends on running. This was confirmed on real M1 hardware:
 * wifi_promisc_tick_task's own vTaskDelay(pdMS_TO_TICKS(5)) at this build's
 * 100Hz tick rate computed to vTaskDelay(0), producing a repeating task-watchdog
 * timeout (see main/app_main.c's own doc comment on wifi_promisc_tick_task for
 * the full incident writeup).
 *
 * MTK_CLAMP_MIN_ONE_TICK clamps any already-computed tick count to a documented
 * minimum of 1, independent of configTICK_RATE_HZ or the requested ms value, so
 * a periodic task built on it can never busy-spin its own "delay" regardless of
 * a future tick-rate or literal change. A macro (not a function) specifically so
 * it stays usable inside _Static_assert with constant operands -- see its own
 * call site in main/app_main.c. Pure integer arithmetic, no FreeRTOS dependency,
 * so this exact clamping policy is directly host-testable in isolation
 * (host_tests/test_min_one_tick_policy.c) independent of any real
 * configTICK_RATE_HZ/ms pair. */
#define MTK_CLAMP_MIN_ONE_TICK(computed_ticks) ((computed_ticks) < 1 ? 1 : (computed_ticks))

#ifdef __cplusplus
}
#endif
