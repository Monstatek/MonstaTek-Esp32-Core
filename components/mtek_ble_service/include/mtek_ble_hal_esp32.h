/* Clean-room implementation from MonstaTek contract. ESP32-C6 target-only
 * BLE HAL glue (built on official ESP-IDF NimBLE host APIs). Not part of
 * the portable host-testable surface. */
#pragma once
#include "mtek_ble_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once from app_main (starts the NimBLE host FreeRTOS task). Returns
 * 0 on success, -1 if a mandatory runtime resource (s_notify_mutex, the
 * NimBLE host port itself) failed to initialize -- see mtek_ble_hal_
 * esp32_init's own doc comment (mtek_ble_hal_esp32.c). On -1, the caller
 * must not wire this HAL in via mtek_ble_set_hal: every BLE opcode then
 * honestly refuses (mtek_ble_logic.c's existing `s_hal &&` guards) rather
 * than running with a partially-initialized, effectively-unlocked HAL. */
int mtek_ble_hal_esp32_init(void);
const mtk_ble_hal_t *mtek_ble_hal_esp32_get(void);

#ifdef __cplusplus
}
#endif
