/* Phase 11 invariant: deauth stimulation/retry must NOT destroy an in-progress
 * handshake capture.
 *
 * A partial capture (M1+M2 observed, still RUNNING because M4 has not arrived)
 * must survive the deauth bursts the same operation transmits to stimulate a
 * reassociation, and a later M4 must then complete the capture using those
 * preserved frames. In mtek_wifi_logic.c the capture buffer (s_hs.buf/len,
 * seen_mask) is reset ONLY by a fresh handle_handshake_start (its memset);
 * hs_frame_cb appends monotonically and the stimulation path touches only
 * tx_armed/tx_remaining/tx_repeat. This test locks that in end-to-end. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

/* Same minimal EAPOL-Key builder as test_handshake_capture.c, targeting the
 * BSSID {1,2,3,4,5,6} that frame_matches_target_bssid requires. */
static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install) {
    memset(buf, 0, 200);
    static const uint8_t bssid[6] = {1,2,3,4,5,6};
    memcpy(buf + 4, bssid, 6); memcpy(buf + 10, bssid, 6); memcpy(buf + 16, bssid, 6);
    buf[0] = 0x88; buf[1] = 0x02; /* QoS Data */
    unsigned off = 24 + 2;
    static const uint8_t llc[8] = {0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E};
    memcpy(buf + off, llc, 8); off += 8;
    buf[off + 0] = 2; buf[off + 1] = 3; buf[off + 2] = 0; buf[off + 3] = 95;
    unsigned eapol = off;
    buf[eapol + 4] = 2;
    uint16_t key_info = (uint16_t)((ack << 7) | (mic << 8) | (secure << 9) | (install << 6));
    buf[eapol + 5] = (uint8_t)(key_info >> 8);
    buf[eapol + 6] = (uint8_t)(key_info & 0xFF);
    off = eapol + 4 + 1 + 2 + 96;
    return (uint16_t)off;
}

/* HANDSHAKE_STATUS -> (state, total_len). */
static uint32_t hs_status(uint32_t token, int corr, uint8_t *state_out) {
    const mtk_opcode_entry_t *st = mtk_test_find_op("HANDSHAKE_STATUS");
    mtk_handshake_status_req_t q = {0}; q.operation_token = token;
    mtk_fake_sink_state_t s; mtk_fake_sink_reset(&s);
    mtk_request_ctx_t c = mtk_test_ctx(&s, corr);
    mtk_test_call(&c, st, &q);
    mtk_handshake_status_resp_t r = {0};
    mtk_decode(st->resp_desc, &r, s.response.body, s.response.body_len, NULL);
    if (state_out) *state_out = r.state;
    return r.total_len;
}

MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();
    mtk_fake_wifi_reset();
    g_fake_wifi.defer_frames = 1; /* frames replay on demand, after START returns */

    /* Stage M1 + M2 only: a useful partial capture (both crack prerequisites)
     * that leaves the operation RUNNING because M4 has not been seen. */
    uint16_t l1 = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
    g_fake_wifi.frames[0].len = l1; g_fake_wifi.frames[0].channel = 6;
    uint16_t l2 = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0); /* M2 */
    g_fake_wifi.frames[1].len = l2; g_fake_wifi.frames[1].channel = 6;
    g_fake_wifi.frame_count = 2;

    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *start = mtk_test_find_op("HANDSHAKE_START");
    mtk_handshake_start_req_t req = {0};
    memcpy(req.target_bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    req.channel = 6; req.deauth_count = 2; /* stimulation armed */
    mtk_test_call(&ctx, start, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    mtk_handshake_start_resp_t started = {0};
    mtk_decode(start->resp_desc, &started, sink.response.body, sink.response.body_len, NULL);

    /* Deliver the partial capture: two progress events, no completion. */
    mtk_fake_wifi_deliver_frames();
    MTK_CHECK(mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED") == NULL);
    uint8_t state = 0;
    uint32_t partial_len = hs_status(started.operation_token, 2, &state);
    MTK_CHECK_EQ(state, MTK_OPS_RUNNING);
    MTK_CHECK(partial_len > 0);

    /* Drive stimulation: deauth bursts are transmitted (the retry that a real
     * capture relies on), and the partial capture MUST stay byte-for-byte
     * intact across every burst. */
    unsigned deauth_before = g_fake_wifi.deauth_sent_count;
    mtek_wifi_service_tick(); /* first burst fires immediately */
    for (int i = 0; i < 5; i++) { mtk_test_advance_ms(20); mtek_wifi_service_tick(); }
    MTK_CHECK(g_fake_wifi.deauth_sent_count > deauth_before); /* stimulation actually happened */
    MTK_CHECK_EQ(hs_status(started.operation_token, 3, &state), partial_len); /* capture preserved */
    MTK_CHECK_EQ(state, MTK_OPS_RUNNING);

    /* A later M4 completes the capture using the PRESERVED M1/M2. */
    uint16_t l4 = build_eapol_frame(g_fake_wifi.frames[0].data, 0, 1, 1, 0); /* M4 */
    g_fake_wifi.frames[0].len = l4; g_fake_wifi.frames[0].channel = 6;
    g_fake_wifi.frame_count = 1;
    mtk_fake_wifi_deliver_frames();

    const mtk_fake_event_t *stopped = mtk_fake_find_event(&sink, "HANDSHAKE_STOPPED");
    MTK_CHECK(stopped != NULL);
    if (stopped) {
        mtk_handshake_stopped_ev_t done = {0};
        mtk_decode(&mtk_handshake_stopped_ev_t_desc, &done, stopped->body, stopped->body_len, NULL);
        MTK_CHECK_EQ(done.status, MTK_STATUS_OK);
        MTK_CHECK(done.captured_total_len > partial_len); /* M4 appended onto the preserved partial */
    }
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
    MTK_CHECK_EQ(g_fake_wifi.restore_count, 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
MTK_TEST_MAIN_END
