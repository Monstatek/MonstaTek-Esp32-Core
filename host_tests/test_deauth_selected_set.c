/* Deauth: SELECTED target mode with a multi-station set (the `deauth set
 * <id,id,...>` grammar's canonical shape) -- round-robin across all
 * targets, total_sent matches the accepted budget. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_schema_message_descs.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_fake_sink_state_t sink; mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx = mtk_test_ctx(&sink, 1);
    const mtk_opcode_entry_t *op = mtk_test_find_op("DEAUTH_START");

    mtk_deauth_start_req_t req = {0};
    req.target_mode = 0; /* SELECTED, multi-target */
    memcpy(req.ap_bssid.b, (uint8_t[]){2,2,3,4,5,6}, 6); /* first octet even: unicast */
    req.channel = 11;
    req.targets.count = 3;
    for (int i = 0; i < 3; i++) memset(req.targets.items[i].b, 0x20 + 2 * i, 6); /* even first octets: unicast */
    req.count = 6; /* two full round-robin passes */
    req.interval_ms = 0;

    mtk_test_call(&ctx, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 6);

    const mtk_fake_event_t *ev = mtk_fake_find_event(&sink, "DEAUTH_STOPPED");
    MTK_CHECK(ev != NULL);
    if (ev) {
        mtk_deauth_stopped_ev_t done = {0};
        mtk_decode(&mtk_deauth_stopped_ev_t_desc, &done, ev->body, ev->body_len, NULL);
        MTK_CHECK_EQ(done.total_sent, 6);
    }

    /* count=0 with a multi-target set: one full pass in's synchronous-completion
     * model (see docs/PROVENANCE.md on background scheduling scope). */
    mtk_fake_wifi_reset();
    mtk_fake_sink_reset(&sink);
    mtk_request_ctx_t ctx2 = mtk_test_ctx(&sink, 2);
    req.count = 0;
    mtk_test_call(&ctx2, op, &req);
    MTK_CHECK_EQ(sink.response.status, MTK_STATUS_ACCEPTED);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 3);

MTK_TEST_MAIN_END
