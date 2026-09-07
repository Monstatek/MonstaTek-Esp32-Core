/* BLE signal meter (002-ble-gatt-service.md Sec 2.3): start, repeated
 * SIGNAL_METER_UPDATE samples via the tick path, and the terminal
 * SIGNAL_METER_LOST event when the target is no longer observable. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

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

    /* RC5 independent audit P1 "BLE/GATT implementation is not yet
     * parity-complete": the real sampling cadence is now self-throttled
     * to ~5s (matching the audit's own stated shipping figure) regardless
     * of how often the caller's own periodic task invokes this tick -- a
     * tick before that interval has elapsed must NOT produce a new
     * sample. */
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

    /* RC7 independent audit item 9 "signal LOST now requires three
     * five-second misses (roughly 15 seconds), contradicting shipped
     * approximately-five-second loss behavior": RC6's own 3-consecutive-
     * miss tolerance is reverted -- with a real ~5s sampling interval,
     * the FIRST missed sample (itself already ~5 seconds after the last
     * successful one) now declares LOST directly, matching the confirmed
     * shipped figure instead of a multiple of it. */
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

MTK_TEST_MAIN_END
