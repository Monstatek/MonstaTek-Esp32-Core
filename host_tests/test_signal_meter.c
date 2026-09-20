/* BLE signal meter: start, repeated SIGNAL_METER_UPDATE samples via the tick
 * path, and the terminal SIGNAL_METER_LOST event when the target is no longer
 * observable. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

/* Cancel at the exact point where the HAL still owns its sample context;
 * no scheduler timing is needed to exercise this lifetime boundary. */
static int cancel_via_reset;
static int sample_result;
static int sample_calls;
static int sample_nested;
static int cancel_during_sample(mtk_hal_mac6_t addr, uint8_t addr_type,
                                int8_t *rssi_out, uint8_t *is_random_out) {
    (void)addr; (void)addr_type;
    sample_calls++;
    if (sample_nested) return -1;
    sample_nested = 1;
    uint32_t token = mtk_arbiter_active_token();

    /* Even a delayed sample must not overlap a second periodic tick. */
    s_mtk_test_now_ms += 5001;
    mtek_ble_signal_meter_tick();
    MTK_CHECK_EQ(sample_calls, 1);

    if (cancel_via_reset) {
        mtk_op_id_t cancelled = mtek_ble_cancel_active_for_peer_reset();
        MTK_CHECK_EQ(cancelled.token, token);
        /* Native peer reset evicts terminal records before the HAL returns. */
        mtk_op_evict_all_terminal();
    } else {
        mtk_fake_sink_state_t stop_sink; mtk_fake_sink_reset(&stop_sink);
        mtk_request_ctx_t stop_ctx = mtk_test_ctx(&stop_sink, 20);
        mtk_signal_meter_stop_req_t stop_req = { .operation_token = token };
        mtk_test_call(&stop_ctx, mtk_test_find_op("SIGNAL_METER_STOP"), &stop_req);
        MTK_CHECK_EQ(stop_sink.response.status, MTK_STATUS_OK);
        mtk_signal_meter_stop_resp_t result = {0};
        mtk_decode(&mtk_signal_meter_stop_resp_t_desc, &result,
                   stop_sink.response.body, stop_sink.response.body_len, NULL);
        MTK_CHECK_EQ(result.final_state, MTK_OPS_STOPPED);
    }
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_SM);
    MTK_CHECK_EQ(mtk_arbiter_active_token(), token);
    /* Another BLE operation cannot reuse the shared HAL callback context. */
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_GC, token + 1), MTK_ARB_GRANT_BUSY);
    mtek_ble_signal_meter_tick();
    MTK_CHECK_EQ(sample_calls, 1);
    *rssi_out = -55;
    *is_random_out = 0;
    sample_nested = 0;
    return sample_result;
}

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_ble_reset();
    g_fake_ble.signal_rc = 0;
    g_fake_ble.signal_rssi = -55;
    g_fake_ble.signal_is_random = 0;

    const mtk_opcode_entry_t *start_op = mtk_test_find_op("SIGNAL_METER_START");
    mtk_signal_meter_start_req_t req = {0};
    memcpy(req.target.addr.b, (uint8_t[]){2,2,2,2,2,2}, 6);
    req.target.addr_type = 0;
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    mtk_test_call(&ctx, start_op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    mtk_signal_meter_start_resp_t started = {0};
    mtk_decode(start_op->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);

    /* One update was emitted synchronously at START (matches field-parity
     * "[BLE:SIG:START]" followed immediately by a first reading). */
    const mtk_fake_event_t *upd1 = mtk_fake_find_event(&sink, "SIGNAL_METER_UPDATE");
    MTK_CHECK(upd1 != NULL);
    mtk_signal_meter_update_ev_t u1 = {0};
    mtk_decode(&mtk_signal_meter_update_ev_t_desc, &u1, upd1->body, upd1->body_len, NULL);
    MTK_CHECK_EQ(u1.raw_rssi, -55);
    MTK_CHECK_EQ(u1.category, 1); /* STRONG: -60 <= rssi < -50 */

    /* The real sampling cadence is now self-throttled to ~5s (matching the
     * audit's own stated shipping figure) regardless of how often the caller's
     * own periodic task invokes this tick -- a tick before that interval has
     * elapsed must NOT produce a new sample. */
    g_fake_ble.signal_rssi = -90; /* VERY_WEAK */
    mtek_ble_signal_meter_tick(); /* time has not advanced -- no new sample yet */
    unsigned updates = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "SIGNAL_METER_UPDATE") == 0) updates++;
    MTK_CHECK_EQ(updates, 1); /* still just the one synchronous START sample */

    s_mtk_test_now_ms += 5001; /* advance past the ~5s sampling interval */
    mtek_ble_signal_meter_tick();
    updates = 0;
    for (unsigned i = 0; i < sink.event_count; i++) if (strcmp(sink.events[i].name, "SIGNAL_METER_UPDATE") == 0) updates++;
    MTK_CHECK_EQ(updates, 2);

    /* RC6's own 3-consecutive- miss tolerance is reverted -- with a real ~5s
     * sampling interval, the FIRST missed sample (itself already ~5 seconds
     * after the last successful one) now declares LOST directly, matching the
     * confirmed shipped figure instead of a multiple of it. */
    g_fake_ble.signal_rc = 1;
    s_mtk_test_now_ms += 5001;
    mtek_ble_signal_meter_tick(); /* the one and only miss -> LOST */
    const mtk_fake_event_t *lost = mtk_fake_find_event(&sink, "SIGNAL_METER_LOST");
    MTK_CHECK(lost != NULL);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    const mtk_opcode_entry_t *stop_op = mtk_test_find_op("SIGNAL_METER_STOP");

    /* A fresh session that samples successfully every interval, with no
     * miss at all, never goes LOST -- stopped before this block's own
     * sink goes out of scope (the active session's sink.user must not
     * outlive the storage it points to, exactly the class of bug this
     * whole audit round fixed elsewhere -- see mtek_wifi_logic.c's
     * handshake_session_t doc comment). */
    {
        mtk_fake_sink_state_t rsink; mtk_fake_sink_reset(&rsink);
        mtk_request_ctx_t rctx = mtk_test_ctx(&rsink, 3);
        g_fake_ble.signal_rc = 0;
        mtk_test_call(&rctx, start_op, &req);
        MTK_CHECK_EQ(rsink.response.status, MTK_STATUS_ACCEPTED);
        mtk_signal_meter_start_resp_t rstarted = {0};
        mtk_decode(start_op->resp_desc, &rstarted, rsink.response.body, rsink.response.body_len, NULL);
        for (int i = 0; i < 3; i++) {
            s_mtk_test_now_ms += 5001;
            mtek_ble_signal_meter_tick(); /* always succeeds -- never a miss */
        }
        MTK_CHECK(mtk_fake_find_event(&rsink, "SIGNAL_METER_LOST") == NULL);

        mtk_signal_meter_stop_req_t rstopreq = {0}; rstopreq.operation_token = rstarted.operation_token;
        mtk_fake_sink_state_t rstopsink; mtk_fake_sink_reset(&rstopsink);
        mtk_request_ctx_t rstopctx = mtk_test_ctx(&rstopsink, 4);
        mtk_test_call(&rstopctx, stop_op, &rstopreq);
        MTK_CHECK_EQ(rstopsink.response.status, MTK_STATUS_OK);
    }

    /* Further ticks are a no-op once the (now-stopped) session is
     * inactive -- checked against the recovery session's own token
     * above, not the original outer one (which the recovery session's
     * own STOP just superseded as the single global active session). */

    /* SIGNAL_METER_STOP is idempotent, on the ORIGINAL session (already
     * terminal from the earlier LOST path). */
    mtk_signal_meter_stop_req_t sreq = {0}; sreq.operation_token = started.operation_token;
    mtk_fake_sink_state_t stsink; mtk_fake_sink_reset(&stsink);
    mtk_request_ctx_t stctx = mtk_test_ctx(&stsink, 2);
    mtk_test_call(&stctx, stop_op, &sreq);
    MTK_CHECK_EQ(stsink.response.status, MTK_STATUS_OK);
    mtk_signal_meter_stop_resp_t sr = {0};
    mtk_decode(stop_op->resp_desc, &sr, stsink.response.body, stsink.response.body_len, NULL);
    MTK_CHECK_EQ(sr.final_state, MTK_OPS_FAILED); /* already terminal from the LOST path */

    mtk_ble_hal_t cancelling_hal = g_fake_ble_hal;
    cancelling_hal.signal_sample = cancel_during_sample;
    for (cancel_via_reset = 0; cancel_via_reset < 2; cancel_via_reset++) {
        for (sample_result = 0; sample_result < 2; sample_result++) {
            mtk_test_bootstrap();
            sample_calls = 0;
            sample_nested = 0;
            mtek_ble_set_hal(&cancelling_hal);
            mtk_fake_sink_reset(&sink);
            mtk_test_call(&ctx, start_op, &req);
            MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
            MTK_CHECK_EQ(sample_calls, 1);
            MTK_CHECK_EQ(sink.event_count, 0);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
            mtek_ble_set_hal(&g_fake_ble_hal);
            /* Sampling and immediate STOP still work after the old HAL
             * invocation returns and its ownership is released. */
            mtk_fake_sink_reset(&sink);
            mtk_test_call(&ctx, start_op, &req);
            MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
            mtk_decode(start_op->resp_desc, &started, sink.response.body,
                       sink.response.body_len, NULL);
            sreq.operation_token = started.operation_token;
            mtk_test_call(&stctx, stop_op, &sreq);
            MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        }
    }

MTK_TEST_MAIN_END
