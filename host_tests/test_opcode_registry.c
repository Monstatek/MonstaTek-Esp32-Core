/* Opcode registry integrity: exact 93-opcode count, every opcode findable by
 * (service_id, opcode), and the capability-state/wire-status closure rule holds
 * for every declared capability_state in the generated table. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_opcode_registry.h"
#include "mtek_schema_message_descs.h"

MTK_TEST_MAIN_BEGIN

    MTK_CHECK_EQ(MTK_OPCODE_COUNT, 106); /* 93 canonical + 6 ESP-NOW + 7 IEEE 802.15.4 */

    unsigned wifi_count = 0, ble_count = 0, gatt_count = 0, capture_count = 0, diag_count = 0, sys_count = 0, espnow_count = 0, i154_count = 0;
    for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {
        const mtk_opcode_entry_t *e = &mtk_opcode_table[i];
        MTK_CHECK(mtk_opcode_find(e->service_id, e->opcode) == e);
        switch (e->service_id) {
            case 0x0000: sys_count++; break;
            case 0x0001: wifi_count++; break;
            case 0x0002: ble_count++; break;
            case 0x0003: gatt_count++; break;
            case 0x0004: capture_count++; break;
            case 0x0005: diag_count++; break;
            case 0x0006: espnow_count++; break;
            case 0x0007: i154_count++; break;
            default: MTK_CHECK(0);
        }
    }
    MTK_CHECK_EQ(espnow_count, 6);
    MTK_CHECK_EQ(i154_count, 7);
    MTK_CHECK_EQ(sys_count, 9);
    MTK_CHECK_EQ(wifi_count, 41);
    MTK_CHECK_EQ(ble_count, 23); /* 0x0001..0x0017, includes the 12 compatibility-family opcodes */
    MTK_CHECK_EQ(gatt_count, 10); /* RC8 P0-6: +GATT_DISCOVER_CHARS, +GATT_DISCOVER_DESCS */
    MTK_CHECK_EQ(capture_count, 6);
    MTK_CHECK_EQ(diag_count, 4);

    MTK_CHECK(mtk_opcode_find(0x0000, 0x0001) != NULL); /* PING */
    MTK_CHECK(mtk_opcode_find(0x0001, 0x0010) != NULL); /* DEAUTH_START */
    MTK_CHECK(mtk_opcode_find(0x0009, 0x0001) == NULL); /* out-of-range service never routed */
    MTK_CHECK(mtk_opcode_find(0x0000, 0x00FF) == NULL); /* out-of-range opcode */

    /* DEAUTH_START/STOP/STATUS: List B, SUPPORTED by default for every profile
     * except factory-UART's DEAUTH_STATUS (no status query command exists on the
     * factory console),. */
    const mtk_opcode_entry_t *deauth_start = mtk_opcode_find(0x0001, 0x0010);
    MTK_CHECK_EQ(deauth_start->cap_native, MTK_CAP_SUPPORTED);
    MTK_CHECK_EQ(deauth_start->cap_factory_uart, MTK_CAP_SUPPORTED);
    MTK_CHECK_EQ(deauth_start->cap_compat_c3, MTK_CAP_SUPPORTED);

    /* STA_CONNECT: factory-UART UNAVAILABLE (no connect verb exists in the
     * audited console grammar), native/Mtek Compatibility SUPPORTED. */
    const mtk_opcode_entry_t *sta_connect = mtk_opcode_find(0x0001, 0x000A);
    MTK_CHECK_EQ(sta_connect->cap_native, MTK_CAP_SUPPORTED);
    MTK_CHECK_EQ(sta_connect->cap_factory_uart, MTK_CAP_UNAVAILABLE);
    MTK_CHECK_EQ(sta_connect->cap_compat_c3, MTK_CAP_SUPPORTED);

    /* STA_STATUS never acquires a radio lease (Sec 4.1 core contract). */
    const mtk_opcode_entry_t *sta_status = mtk_opcode_find(0x0001, 0x000C);
    MTK_CHECK_EQ(sta_status->no_radio_lease, 1);
    MTK_CHECK_EQ(sta_status->resource_class, MTK_ARB_NONE);

    /* "a property test that walks every advertised opcode and proves
     * registry/capability/dispatch agreement. The validator must test these
     * behaviors, not merely state that inspection covers them." Real end-to-end
     * proof, not a re-inspection of the same static table: calls the REAL
     * GET_CAPABILITIES opcode (exactly what a real client would) to learn each
     * opcode's own REPORTED state, then dispatches that SAME opcode for real and
     * confirms agreement in both directions -- reported SUPPORTED must never
     * dispatch to UNSUPPORTED (the exact class of bug this item fixed: the whole
     * BEACON/SOFTAP/PROBE_FLOOD/PMKID_CAPTURE/KARMA/CAPTIVE_PORTAL opcode
     * families were reported SUPPORTED by the registry's own static table while
     * mtek_wifi_logic.c's own dispatch switch unconditionally answered
     * UNSUPPORTED), and reported NOT-supported must never dispatch to anything
     * other than UNSUPPORTED. */
    {
        mtk_test_bootstrap();
        uint8_t state[MTK_OPCODE_COUNT];
        memset(state, 0xFF, sizeof(state));

        const mtk_opcode_entry_t *caps_op = mtk_test_find_op("GET_CAPABILITIES");
        MTK_CHECK(caps_op != NULL);
        uint16_t start = 0;
        do {
            mtk_get_capabilities_req_t req = {0}; req.start_index = start; req.max_items = 32;
            mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
            mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
            mtk_test_call(&ctx, caps_op, &req);
            MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
            mtk_get_capabilities_resp_t resp = {0};
            mtk_decode(caps_op->resp_desc, &resp, sink.response.body, sink.response.body_len, NULL);
            for (uint32_t k = 0; k < resp.entries.count; k++) {
                const mtk_opcode_entry_t *e = mtk_opcode_find(resp.entries.items[k].service_id, resp.entries.items[k].opcode);
                MTK_CHECK(e != NULL);
                state[e - mtk_opcode_table] = resp.entries.items[k].state;
            }
            start = resp.next_index;
        } while (start != 0);

        static const uint8_t zero_req[2048];
        unsigned checked = 0;
        for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {
            const mtk_opcode_entry_t *op = &mtk_opcode_table[i];
            MTK_CHECK(state[i] != 0xFF); /* every opcode was seen across the paginated walk above */
            mtk_fake_sink_state_t dsink; mtk_fake_sink_reset(&dsink);
            mtk_request_ctx_t dctx = mtk_test_ctx(&dsink, 1000 + i);
            uint8_t buf[2048]; size_t blen = 0;
            if (op->req_desc) mtk_encode(op->req_desc, zero_req, buf, sizeof(buf), &blen);
            mtk_router_dispatch(&dctx, op->service_id, op->opcode, buf, blen);
            MTK_CHECK(dsink.response.set); /* every dispatch answers synchronously with no async runner registered */
            uint8_t dispatch_status = dsink.response.status;
            if (state[i] == MTK_CAP_SUPPORTED) {
                MTK_CHECK(dispatch_status != MTK_STATUS_UNSUPPORTED);
            } else {
                MTK_CHECK_EQ(dispatch_status, MTK_STATUS_UNSUPPORTED);
            }
            checked++;
        }
        MTK_CHECK_EQ(checked, (unsigned)MTK_OPCODE_COUNT);
    }

MTK_TEST_MAIN_END
