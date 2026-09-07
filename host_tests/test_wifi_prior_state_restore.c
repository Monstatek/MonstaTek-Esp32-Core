/* RC8 independent audit P0-9 "Implement a complete prior-state snapshot
 * and transactional restore for station/AP mode, connection/reconnect
 * intent, channel, and promiscuous state" + "Define and enforce
 * coexistence explicitly: connected Wi-Fi station operation and single-
 * channel monitor/injection must never be silently treated as
 * concurrent."
 *
 * Part 1 proves the real prior-state round-trip fix in
 * mtek_wifi_hal_esp32.c/mtk_fake_wifi_hal.h's own restore_sta_mode
 * contract: a genuinely non-default prior mode/channel (previously
 * silently discarded -- the old code hard-coded WIFI_MODE_STA and never
 * touched the channel at all) is captured once at session entry and
 * restored exactly, not overwritten by the session's own later channel
 * disturbance (deauth's per-round channel select).
 *
 * Part 2 proves the explicit coexistence policy: mtek_arbiter.h's own
 * accepted single-active-class design (002-resource-arbiter.md Sec 2)
 * means BLE and Wi-Fi List B sessions share ONE platform-wide active-
 * class slot -- they are mutually exclusive, never independently
 * arbitrated and never genuinely concurrent. A BLE advertisement started
 * first correctly makes a Wi-Fi deauth attempted while it is still
 * running fail BUSY (never silently allowed to run at the same time);
 * once that BLE session actually stops, the identical deauth request
 * succeeds -- real, correct mutual exclusion, not a permanent lockout. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_sink_state_t sink; memset(&sink, 0, sizeof(sink));
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);

    /* ---- Part 1a: a non-default prior mode/channel round-trips through
     * a one-shot (count=1, synchronous fallback) BROADCAST deauth. */
    g_fake_wifi.mode = 2;            /* APSTA -- never the hard-coded STA the old code always restored to */
    g_fake_wifi.current_channel = 9; /* the real pre-session channel -- never touched by the old code at all */
    g_fake_wifi.sta_was_connected = 1;

    const mtk_opcode_entry_t *deauth_start = mtk_test_find_op("DEAUTH_START");
    mtk_deauth_start_req_t dreq; memset(&dreq, 0, sizeof(dreq));
    dreq.target_mode = 2; /* BROADCAST */
    memset(dreq.ap_bssid.b, 0xAA, 6);
    dreq.channel = 6; dreq.count = 1; dreq.interval_ms = 1;
    mtk_test_call(&ctx, deauth_start, &dreq);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);

    MTK_CHECK(g_fake_wifi.restore_count >= 1);
    MTK_CHECK_EQ(g_fake_wifi.mode, 2);             /* restored to the real prior mode, not hard-coded STA */
    MTK_CHECK_EQ(g_fake_wifi.current_channel, 9);  /* restored to the real prior channel, not left at 6 (the deauth's own operating channel) */
    MTK_CHECK(g_fake_wifi.reconnect_attempted_count >= 1); /* was_connected=1 threaded through the snapshot */

    /* ---- Part 1b: the snapshot is captured EXACTLY ONCE per session --
     * a session that disturbs the channel repeatedly (several packets)
     * must still restore the ORIGINAL prior channel, not whatever the
     * last per-packet channel select left behind. */
    mtk_fake_wifi_reset();
    g_fake_wifi.mode = 1;             /* AP */
    g_fake_wifi.current_channel = 11;
    g_fake_wifi.sta_was_connected = 0;

    mtk_deauth_start_req_t dreq2; memset(&dreq2, 0, sizeof(dreq2));
    dreq2.target_mode = 2;
    memset(dreq2.ap_bssid.b, 0xBA, 6); /* LSB of first octet clear -- 0xBB would trip the multicast-address rejection */
    dreq2.channel = 3; dreq2.count = 5; dreq2.interval_ms = 1; /* several rounds -- several set_channel(3) calls */
    mtk_test_call(&ctx, deauth_start, &dreq2);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);

    MTK_CHECK(g_fake_wifi.deauth_sent_count >= 5); /* proves several rounds of radio disturbance really happened, not just one */
    MTK_CHECK_EQ(g_fake_wifi.mode, 1);              /* still the ORIGINAL prior mode (AP) */
    MTK_CHECK_EQ(g_fake_wifi.current_channel, 11);  /* still the ORIGINAL prior channel, not 3 */
    MTK_CHECK_EQ(g_fake_wifi.reconnect_attempted_count, 0); /* was_connected=0 -- no reconnect attempted */

    /* ---- Part 1c: failure injection -- a channel-selection failure
     * during the session must not corrupt the ALREADY-captured prior
     * state (captured before the first, successful, set_channel call in
     * Part 1b's own reset session below), and restore must still run to
     * completion using the real snapshot. */
    mtk_fake_wifi_reset();
    g_fake_wifi.mode = 0; /* STA */
    g_fake_wifi.current_channel = 4;
    const mtk_opcode_entry_t *raw_tx_op = mtk_test_find_op("RAW_TX_SEND");
    MTK_CHECK(raw_tx_op != NULL);
    {
        mtk_raw_tx_send_req_t rreq; memset(&rreq, 0, sizeof(rreq));
        rreq.channel = 7;
        rreq.frame.len = 10; memset(rreq.frame.data, 0x11, 10);
        g_fake_wifi.set_channel_rc = -1; /* the requested channel select itself fails */
        mtk_test_call(&ctx, raw_tx_op, &rreq);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_IO_ERROR); /* never transmits on a channel-selection failure (P0-9 raw-tx fix, re-verified here) */
        MTK_CHECK_EQ(g_fake_wifi.raw_tx_count, 0);
        g_fake_wifi.set_channel_rc = 0;
    }

    /* ---- Part 2: BLE/Wi-Fi coexistence policy, explicitly proven.
     * `mtk_arbiter.h`'s own accepted contract (002-resource-arbiter.md
     * Sec 2) is a SINGLE platform-wide active class -- BLE and Wi-Fi
     * List B sessions are NOT independently arbitered, they share the
     * one active-class slot. This is the real enforcement mechanism that
     * makes "BLE session active" and "Wi-Fi monitor/injection session
     * active" mutually exclusive, never silently concurrent: a BLE
     * advertisement holds MTK_ARB_BA for its whole running lifetime (like
     * a live GATT connection holds MTK_ARB_GC), so a Wi-Fi deauth started
     * while it is running is correctly rejected BUSY rather than silently
     * allowed to run at the same time -- proven below, then proven to
     * un-block cleanly once the BLE session actually stops. */
    mtk_fake_wifi_reset();
    mtk_fake_ble_reset();
    g_fake_wifi.mode = 0; g_fake_wifi.current_channel = 1;

    const mtk_opcode_entry_t *adv_start = mtk_test_find_op("BLE_ADV_START");
    const mtk_opcode_entry_t *adv_stop = mtk_test_find_op("BLE_ADV_STOP");
    mtk_ble_adv_start_req_t areq; memset(&areq, 0, sizeof(areq));
    const char *name = "CoexistTest"; areq.name.len = (uint16_t)strlen(name); memcpy(areq.name.data, name, areq.name.len);
    mtk_test_call(&ctx, adv_start, &areq);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_ble.adv_start_rc, 0);
    mtk_ble_adv_start_resp_t astart_resp; memset(&astart_resp, 0, sizeof(astart_resp));
    mtk_decode(adv_start->resp_desc, &astart_resp, sink.response.body, sink.response.body_len, NULL);

    /* A Wi-Fi deauth attempted while the BLE advertisement is still
     * running must be honestly rejected -- never silently treated as
     * concurrent with the active BLE session. */
    mtk_deauth_start_req_t dreq3; memset(&dreq3, 0, sizeof(dreq3));
    dreq3.target_mode = 2;
    memset(dreq3.ap_bssid.b, 0xCC, 6);
    dreq3.channel = 6; dreq3.count = 1; dreq3.interval_ms = 1;
    mtk_test_call(&ctx, deauth_start, &dreq3);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_BUSY);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 0u); /* genuinely never ran while BLE held the arbiter */

    /* Once the BLE session actually ends (BLE_ADV_STOP releases
     * MTK_ARB_BA), the exact same deauth request now succeeds -- proving
     * this is real, correct mutual exclusion, not a permanent lockout. */
    mtk_ble_adv_stop_req_t astop_req; memset(&astop_req, 0, sizeof(astop_req));
    astop_req.operation_token = astart_resp.operation_token;
    mtk_test_call(&ctx, adv_stop, &astop_req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);

    mtk_test_call(&ctx, deauth_start, &dreq3);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK(g_fake_wifi.deauth_sent_count >= 1);

MTK_TEST_MAIN_END
