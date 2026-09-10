/* Clean-room implementation from MonstaTek contract. ESP32-C6 target-only
 * Wi-Fi HAL glue (built on official ESP-IDF esp_wifi.h/esp_netif.h APIs).
 * Not part of the portable host-testable surface. */
#pragma once
#include "mtek_wifi_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once from app_main after esp_wifi_init()/esp_netif_init(). Returns
 * 0 if every required runtime resource (mutexes/event groups/queue/netif)
 * allocated successfully, nonzero otherwise -- app_main should log a
 * nonzero return prominently, but every radio-disturbing opcode already
 * refuses safely on its own (capture_prior_state_once's own gate) even if
 * the return value is ignored. */
int mtek_wifi_hal_esp32_init(void);
const mtk_wifi_hal_t *mtek_wifi_hal_esp32_get(void);
/* RC11 promiscuous-mode audit follow-up #5: app_main.c's own
 * wifi_promisc_tick_task is the ONLY thing that ever drains the deferred
 * promiscuous-frame queue (esp32_promisc_service) -- it is as much a
 * required runtime resource for a capture/handshake session as the
 * mutexes/queue mtek_wifi_hal_esp32_init already checks, just created
 * later (app_main's own xTaskCreate, after this HAL's own init). Call
 * this if that xTaskCreate fails, so capture_prior_state_once's existing
 * gate refuses every radio-disturbing opcode honestly instead of
 * accepting a capture that can never actually receive anything. */
void mtek_wifi_hal_esp32_mark_promisc_task_failed(void);
/* RC11 promiscuous-mode audit follow-up #7 "add observable accounting for
 * deferred-queue overflow": total frames the Wi-Fi driver task's own
 * promiscuous callback could not enqueue (a full deferred-delivery queue)
 * since boot -- monotonic, never reset. */
uint32_t mtek_wifi_hal_esp32_promisc_queue_overflow_count(void);

#ifdef __cplusplus
}
#endif
