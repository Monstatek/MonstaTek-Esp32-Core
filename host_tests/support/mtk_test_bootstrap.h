/* Shared host-test bring-up: wires the same four services + router + core
 * + arbiter a target boot would, against the deterministic fake HALs. */
#pragma once
#include <string.h>
#include <stdlib.h>
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_router.h"
#include "mtek_system_service.h"
#include "mtek_wifi_service.h"
#include "mtek_ble_service.h"
#include "mtek_capture_service.h"
#include "mtk_fake_sink.h"
#include "mtk_fake_wifi_hal.h"
#include "mtk_fake_ble_hal.h"

#define MTK_TEST_BOOT_EPOCH 0xABCD1234u

static uint64_t s_mtk_test_now_ms = 1000;
static uint64_t mtk_test_now_ms(void) { return s_mtk_test_now_ms; }

static inline void mtk_test_bootstrap(void) {
    mtk_core_init(MTK_TEST_BOOT_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();

    static const mtk_system_build_info_t info = {
        1, 0, 0, "test", 0, 0, 0, 0, "esp32c6-hosttest", "n/a"
    };
    mtek_system_service_init(&info, mtk_test_now_ms);
    mtek_wifi_service_init(mtk_test_now_ms);
    mtek_ble_service_init(mtk_test_now_ms);
    mtek_capture_service_init(mtk_test_now_ms);

    mtk_fake_wifi_reset();
    mtk_fake_ble_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_ble_set_hal(&g_fake_ble_hal);

    if (mtek_system_service_register() != MTK_REGISTER_OK ||
        mtek_wifi_service_register() != MTK_REGISTER_OK ||
        mtek_ble_service_register() != MTK_REGISTER_OK ||
        mtek_capture_service_register() != MTK_REGISTER_OK) abort();
}

static inline mtk_request_ctx_t mtk_test_ctx(mtk_fake_sink_state_t *sink_state, uint32_t correlation) {
    mtk_request_ctx_t ctx;
    ctx.profile = MTK_PROFILE_HOST_ADAPTER;
    ctx.dispatch_mode = MTK_DISPATCH_DEFER_ALLOWED;
    ctx.correlation = correlation;
    ctx.boot_epoch = MTK_TEST_BOOT_EPOCH;
    ctx.session_generation = 0; /* not session-scoped -- never fenced by mtek_router.c's own async_trampoline */
    ctx.authorization_level = 0;
    ctx.sink = mtk_fake_sink_make(sink_state);
    return ctx;
}

static inline const mtk_opcode_entry_t *mtk_test_find_op(const char *name) {
    for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {
        if (strcmp(mtk_opcode_table[i].name, name) == 0) return &mtk_opcode_table[i];
    }
    return NULL;
}

/* Encodes `req_struct` per `op->req_desc` into a caller-owned buffer and
 * dispatches it through the router, using `op`'s own (service_id, opcode).
 */
static inline void mtk_test_call(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op, const void *req_struct) {
    uint8_t buf[4096];
    size_t len = 0;
    if (op->req_desc) mtk_encode(op->req_desc, req_struct, buf, sizeof(buf), &len);
    mtk_router_dispatch(ctx, op->service_id, op->opcode, buf, len);
}
