/* "Current raw TX validates the requested channel but does not select it."
 * req.channel was previously decoded and range- checked, then never passed to
 * the HAL at all -- the frame transmitted on whatever channel the radio already
 * happened to be on, not the caller's requested one. Proves the real fix: the
 * channel is genuinely selected via the HAL (hal->set_channel), and -- the fix's
 * other half, "never transmit if channel selection fails" -- a channel-selection
 * failure aborts before ever reaching hal->raw_tx at all. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    const mtk_opcode_entry_t *op = mtk_test_find_op("RAW_TX_SEND");

    /* Channel genuinely selected before transmit. */
    {
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
        mtk_raw_tx_send_req_t req = {0};
        req.channel = 7;
        req.frame.len = 24; /* within [MTK_BUDGET_RAW_TX_FRAME_MIN_BYTES, _MAX_BYTES] */
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_OK);
        MTK_CHECK_EQ(g_fake_wifi.set_channel_call_count, 1);
        MTK_CHECK_EQ(g_fake_wifi.last_requested_channel, 7); /* the REQUESTED channel was genuinely passed through */
        MTK_CHECK_EQ(g_fake_wifi.raw_tx_count, 1);
        /* Owner-approved correction (docs/DECISION_LOG.md's RC11
         * "discovered, not fixed" note): RAW_TX_SEND now genuinely
         * restores prior radio state afterward instead of leaving the
         * captured snapshot forever un-cleared -- current_channel is
         * back to its pre-call value (the fake's own default, 0) by the
         * time this call returns, and restore_count proves the real
         * restore path actually ran exactly once. */
        MTK_CHECK_EQ(g_fake_wifi.restore_count, 1u);
        MTK_CHECK_EQ(g_fake_wifi.current_channel, 0);
    }

    /* Channel selection failure: never reaches raw_tx at all. */
    {
        g_fake_wifi.set_channel_rc = -1;
        mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
        mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 2);
        mtk_raw_tx_send_req_t req = {0};
        req.channel = 9;
        req.frame.len = 24;
        unsigned raw_tx_before = g_fake_wifi.raw_tx_count;
        mtk_test_call(&ctx, op, &req);
        MTK_CHECK_EQ(sink.response.status, MTK_STATUS_IO_ERROR);
        MTK_CHECK_EQ(g_fake_wifi.raw_tx_count, raw_tx_before); /* never attempted the transmit */
        g_fake_wifi.set_channel_rc = 0;
    }

MTK_TEST_MAIN_END
