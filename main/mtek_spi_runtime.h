/* Clean-room implementation from MonstaTek contract. Physical spi_slave
 * transaction loop wiring both boot-exclusive SPI adapters (native v1,
 * Mtek Compatibility/C3) onto the confirmed production pins, with AUTO profile
 * discovery. See mtek_spi_runtime.c for the exact evidence citations. */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the SPI slave runtime as its own FreeRTOS task. Safe to call even if
 * the physical peripheral is never actually clocked by a master (the task
 * retains its armed DMA transaction while servicing wakeups and elapsed-time
 * housekeeping).
 *
 * `shared_mutex` : the SAME real FreeRTOS mutex app_main.c already registered
 * with mtk_core_set_lock/mtk_arbiter_set_lock/
 * mtk_router_set_lock/mtk_transport_claim_set_lock BEFORE starting any adapter
 * task -- this runtime uses it for its own per-adapter event_queue locks
 * (native_dctx.event_queue/compat_dctx.event_queue) rather than creating and
 * installing a second, independent mutex the way RC6 did (which only protected
 * state while THIS task happened to be the sole adapter running; RC7's real
 * cross-transport AUTO selection means the UART REPL task can now genuinely run
 * at the same time and must serialize against the exact same shared state
 * through the exact same lock). */
/* Returns 0 if the runtime task was created successfully, -1 if xTaskCreate
 * itself failed. A -1 return means the SPI/Mtek Compatibility-C3 transport never
 * becomes reachable this boot session; the caller (app_ main.c) logs this
 * honestly rather than silently continuing as if the task had started. */
int mtek_spi_runtime_start(SemaphoreHandle_t shared_mutex);

#ifdef __cplusplus
}
#endif
