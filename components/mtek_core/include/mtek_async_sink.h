/* Shared canonical queue callbacks. user must point to a persistent
 * mtk_async_queue_t for the operation lifetime. Adapters retain wire delivery. */
#pragma once
#include "mtek_core.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

mtk_emit_result_t mtk_async_sink_resp(void *user, uint32_t correlation, uint8_t status, const void *body, const mtk_struct_desc_t *desc);
mtk_emit_result_t mtk_async_sink_resp_raw(void *user, uint32_t correlation, uint8_t status, const uint8_t *body, size_t len);
mtk_emit_result_t mtk_async_sink_event(void *user, uint32_t correlation_or_zero, const char *name, const void *body, const mtk_struct_desc_t *desc);
mtk_emit_result_t mtk_async_sink_stream(void *user, uint32_t session_token, uint32_t seq, const uint8_t *chunk, size_t len);

#ifdef __cplusplus
}
#endif
