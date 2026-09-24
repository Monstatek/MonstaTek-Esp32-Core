/* Beacon multi-SSID round-robin, BEACON_STATUS, and STOP_ALL teardown.
 *
 * test_beacon_lifecycle.c only ever exercises ssids.count == 1, yet multi-SSID
 * is an advertised beacon capability (docs/CAPABILITY_MANIFEST.md) and
 * beacon_tick (mtek_wifi_logic.c) round-robins s_beacon.index across
 * ssids.items[] modulo ssids.count. This test proves: (a) with two SSIDs both
 * are actually beaconed, (b) the index WRAPS 1->0 (a genuine round-robin, not a
 * one-shot pass), (c) BEACON_STATUS reports the running operation and the
 * channels it has covered, and (d) WIFI_STOP_ALL tears the beacon down and no
 * further frames are emitted. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

static uint8_t last_frame[128];
static unsigned last_len;
static int record_tx(const uint8_t *p, uint16_t n) {
    memcpy(last_frame, p, n); last_len = n; g_fake_wifi.raw_tx_count++; return g_fake_wifi.raw_tx_rc;
}
/* Beacon frame SSID element: length at [37], data at [38..] (see
 * test_beacon_lifecycle.c's own layout assertions). */
static int ssid_is(const char *s, int len) {
    return last_frame[0] == 0x80 && last_frame[37] == len && !memcmp(last_frame + 38, s, len);
}

MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();
    mtk_wifi_hal_t hal = *mtek_wifi_get_hal(); hal.raw_tx = record_tx; mtek_wifi_set_hal(&hal);
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);

    const mtk_opcode_entry_t *start = mtk_test_find_op("BEACON_START");
    mtk_beacon_start_req_t req = {0};
    req.ssids.count = 2;
    req.ssids.items[0].len = 4; memcpy(req.ssids.items[0].data, "aaaa", 4);
    req.ssids.items[1].len = 6; memcpy(req.ssids.items[1].data, "bbbbbb", 6);
    mtk_test_call(&ctx, start, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    mtk_beacon_start_resp_t resp = {0};
    mtk_decode(start->resp_desc, &resp, sink.response.body, sink.response.body_len, NULL);

    /* beacon_tick advances one channel per tick (gated ~10ms apart) and only
     * advances the SSID index after channel 11 wraps. 26 ticks cover
     * index0 ch1-11, index1 ch1-11, then index0 again -- enough to observe both
     * SSIDs and the wrap back to the first. */
    int seen_a = 0, seen_b = 0, b_seen = 0, a_after_b = 0;
    for (int i = 0; i < 26; i++) {
        if (i) mtk_test_advance_ms(10);
        mtek_wifi_service_tick();
        if (ssid_is("aaaa", 4)) { seen_a = 1; if (b_seen) a_after_b = 1; }
        if (ssid_is("bbbbbb", 6)) { seen_b = 1; b_seen = 1; }
    }
    MTK_CHECK(seen_a);      /* first SSID beaconed */
    MTK_CHECK(seen_b);      /* second SSID beaconed -- the count>1 path */
    MTK_CHECK(a_after_b);   /* index wrapped 1->0: round-robin, not a one-shot pass */

    /* BEACON_STATUS: the operation is still running and has covered channels. */
    const mtk_opcode_entry_t *status = mtk_test_find_op("BEACON_STATUS");
    mtk_beacon_status_req_t sreq = {0}; sreq.operation_token = resp.operation_token;
    mtk_fake_sink_state_t ssink; mtk_fake_sink_reset(&ssink);
    mtk_request_ctx_t sctx = mtk_test_ctx(&ssink, 2);
    mtk_test_call(&sctx, status, &sreq);
    MTK_CHECK_EQ(ssink.response.status, MTK_STATUS_OK);
    mtk_beacon_status_resp_t st = {0};
    mtk_decode(status->resp_desc, &st, ssink.response.body, ssink.response.body_len, NULL);
    MTK_CHECK_EQ(st.state, MTK_OPS_RUNNING);
    MTK_CHECK(st.channels_active_count >= 2);

    /* WIFI_STOP_ALL tears the beacon down; no further frames are transmitted. */
    mtk_test_call(&ctx, mtk_test_find_op("WIFI_STOP_ALL"), NULL);
    MTK_CHECK_EQ(mtk_arbiter_snapshot().cls, MTK_ARB_NONE);
    unsigned after_stop = g_fake_wifi.raw_tx_count;
    mtk_test_advance_ms(1000); mtek_wifi_service_tick();
    MTK_CHECK_EQ(g_fake_wifi.raw_tx_count, after_stop);
MTK_TEST_MAIN_END
