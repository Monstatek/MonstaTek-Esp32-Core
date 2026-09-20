/* Clean-room implementation from MonstaTek contract. */
#pragma once
#include "mtek_router.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t product_major, product_minor, product_patch;
    const char *build_id;           /* e.g. "dev" when no reproducible build id is available */
    uint8_t build_dirty;
    uint32_t build_epoch_s;
    uint8_t build_epoch_reproducible;
    uint8_t target_chip;            /* 0 = ESP32-C6 in this build */
    const char *hw_compat_id;
    const char *esp_idf_version;
} mtk_system_build_info_t;

/* now_ms: monotonic milliseconds, supplied by the caller (portable). */
void mtek_system_service_init(const mtk_system_build_info_t *info, uint64_t (*now_ms_fn)(void));
mtk_register_result_t mtek_system_service_register(void); /* registers with mtek_router for service_id 0x0000 */

/* Injected so TIME_SYNC_START can honor its "requires an already-connected
 * station" precondition without a hard dependency on the wifi service. */
void mtek_system_set_sta_query(int (*is_sta_connected_fn)(void));

/* Injected so RESET_INTENT's terminal action (an actual device reset) is
 * the platform glue's responsibility, not this portable logic's. */
void mtek_system_set_reset_hook(void (*reset_fn)(uint32_t delay_ms));

/* Reads the reset reason to report from GET_RESET_REASON. */
void mtek_system_set_reset_reason(uint8_t reason);

#ifdef __cplusplus
}
#endif
