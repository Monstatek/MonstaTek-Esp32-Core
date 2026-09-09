/* Malformed inputs: unknown opcodes, capability-disabled opcodes, and
 * decode-time protocol errors are all rejected with the correct wire
 * status and no side effect (002-service-registry.md Sec 3.0a). */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();

    /* Unknown opcode within a real service range: never routed. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_router_dispatch(&ctx, 0x0000, 0x00FE, NULL, 0);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* Unknown service_id entirely. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 2);
        mtk_router_dispatch(&ctx, 0x00AA, 0x0001, NULL, 0);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* Capability-disabled opcode (BLE compatibility family, DISABLED for
     * every profile in Phase 1): wire status UNSUPPORTED, no dispatch. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 3);
        const mtk_opcode_entry_t *op = mtk_test_find_op("BLE_HID_INIT");
        MTK_CHECK(op != NULL);
        mtk_router_dispatch(&ctx, op->service_id, op->opcode, NULL, 0);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_UNSUPPORTED);
    }

    /* factory-UART profile against an UNAVAILABLE opcode (STA_CONNECT):
     * also UNSUPPORTED, not UNAVAILABLE -- capability states are never
     * legal wire statuses. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 4);
        ctx.profile = MTK_PROFILE_FACTORY_UART;
        ctx.dispatch_mode = MTK_DISPATCH_INLINE;
        const mtk_opcode_entry_t *op = mtk_test_find_op("STA_CONNECT");
        mtk_router_dispatch(&ctx, op->service_id, op->opcode, NULL, 0);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_UNSUPPORTED);
    }

    /* Truncated/garbage request body against a real, supported opcode:
     * decode fails, service never executes, PROTOCOL_ERROR returned. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 5);
        const mtk_opcode_entry_t *op = mtk_test_find_op("STA_CONNECT");
        uint8_t garbage[3] = { 0xFF, 0xFF, 0xFF }; /* claims a 255-byte SSID that isn't there */
        mtk_router_dispatch(&ctx, op->service_id, op->opcode, garbage, sizeof(garbage));
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_PROTOCOL_ERROR);
    }

    /* DEAUTH_START with an invalid target_mode enum value: rejected before
     * any radio acquisition (no BUSY, no ACCEPTED). */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 6);
        const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");
        mtk_deauth_start_req_t req = {0};
        req.target_mode = 99;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_INVALID_ARGUMENT);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    }

MTK_TEST_MAIN_END
