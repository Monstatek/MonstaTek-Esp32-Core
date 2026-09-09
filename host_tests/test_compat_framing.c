/* Mtek Compatibility/C3 `m1_link` framing (001-profile-bootstrap-feasibility.md Sec 1):
 * header round trip, CRC16-CCITT known-answer, malformed rejection,
 * fragmentation/reassembly, and end-to-end dispatch of a real opcode
 * (DEAUTH_START) proving actual HAL reachability, not just a precondition
 * error. */
#include "mtk_test.h"
#include "mtek_compat_frame.h"
#include "mtek_compat_dispatch.h"
#include "mtek_compat_opcode_map.h"
#include "mtek_router.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_wifi_service.h"
#include "mtek_system_service.h"
#include "mtk_fake_wifi_hal.h"
#include <string.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

MTK_TEST_MAIN_BEGIN

    /* CRC-16/CCITT-FALSE known-answer test. */
    MTK_CHECK_EQ(mtk_compat_crc16((const uint8_t *)"123456789", 9), 0x29B1);

    /* Header/CRC round trip via a real IDLE cell. */
    mtk_compat_header_t hdr = {0};
    hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
    hdr.msg_type = MTK_COMPAT_MSG_IDLE; hdr.msg_id = 0; hdr.payload_len = 0;
    uint8_t cell[MTK_COMPAT_CELL_SIZE];
    mtk_compat_build_cell(&hdr, NULL, cell);
    MTK_CHECK_EQ(cell[0], 0x31); MTK_CHECK_EQ(cell[1], 0x4D); /* 0x4D31 LE on the wire */

    mtk_compat_header_t parsed; const uint8_t *payload;
    MTK_CHECK_EQ(mtk_compat_parse_cell(cell, &parsed, &payload), MTK_COMPAT_PARSE_OK);
    MTK_CHECK_EQ(parsed.msg_type, MTK_COMPAT_MSG_IDLE);

    /* Malformed input rejected at every field. */
    uint8_t bad[MTK_COMPAT_CELL_SIZE];
    memcpy(bad, cell, sizeof(bad)); bad[0] ^= 0xFF;
    MTK_CHECK_EQ(mtk_compat_parse_cell(bad, &parsed, &payload), MTK_COMPAT_PARSE_BAD_MAGIC);
    memcpy(bad, cell, sizeof(bad)); bad[2] = 0x02;
    MTK_CHECK_EQ(mtk_compat_parse_cell(bad, &parsed, &payload), MTK_COMPAT_PARSE_BAD_VERSION);
    memcpy(bad, cell, sizeof(bad)); bad[3] = 0x0F; /* unknown msg_type */
    MTK_CHECK_EQ(mtk_compat_parse_cell(bad, &parsed, &payload), MTK_COMPAT_PARSE_BAD_MSG_TYPE);
    memcpy(bad, cell, sizeof(bad)); bad[MTK_COMPAT_HEADER_SIZE] ^= 0xFF; /* corrupt CRC-covered byte */
    hdr.payload_len = 1;
    /* build a frame with 1 payload byte to have something to corrupt */
    uint8_t onebyte = 0xAA;
    uint8_t cell2[MTK_COMPAT_CELL_SIZE];
    mtk_compat_build_cell(&hdr, &onebyte, cell2);
    uint8_t badcrc[MTK_COMPAT_CELL_SIZE]; memcpy(badcrc, cell2, sizeof(badcrc));
    badcrc[MTK_COMPAT_HEADER_SIZE] ^= 0xFF;
    MTK_CHECK_EQ(mtk_compat_parse_cell(badcrc, &parsed, &payload), MTK_COMPAT_PARSE_BAD_CRC);

    MTK_CHECK_EQ(mtk_compat_parse_bounded(cell, 3, &parsed, &payload), MTK_COMPAT_PARSE_TRUNCATED);

    /* Cross-profile rejection: native's fixed magic never parses as Mtek Compatibility. */
    uint8_t native_ish[MTK_COMPAT_CELL_SIZE] = {0};
    native_ish[0] = 0x4D; native_ish[1] = 0x31; native_ish[2] = 0x53; native_ish[3] = 0x31;
    MTK_CHECK_EQ(mtk_compat_parse_bounded(native_ish, sizeof(native_ish), &parsed, &payload), MTK_COMPAT_PARSE_BAD_MAGIC);
    MTK_CHECK_EQ(mtk_compat_try_recognize_discovery_frame(native_ish, sizeof(native_ish)), 0);
    MTK_CHECK_EQ(mtk_compat_try_recognize_discovery_frame(cell, sizeof(cell)), 1);

    /* Fragmentation/reassembly: FRAG then terminal REQ completes the message. */
    mtk_compat_reassembly_t ctx;
    mtk_compat_reassembly_reset(&ctx);
    mtk_compat_header_t frag = {0};
    frag.magic = MTK_COMPAT_MAGIC; frag.version = MTK_COMPAT_VERSION; frag.msg_type = MTK_COMPAT_MSG_FRAG;
    frag.msg_id = 7; frag.payload_len = 5;
    uint8_t part1[5] = {1,2,3,4,5};
    MTK_CHECK_EQ(mtk_compat_reassembly_feed(&ctx, &frag, part1), MTK_COMPAT_REASM_IN_PROGRESS);
    MTK_CHECK_EQ(ctx.len, 5);
    mtk_compat_header_t term = frag; term.msg_type = MTK_COMPAT_MSG_REQ; term.payload_len = 3;
    uint8_t part2[3] = {6,7,8};
    MTK_CHECK_EQ(mtk_compat_reassembly_feed(&ctx, &term, part2), MTK_COMPAT_REASM_COMPLETE);
    MTK_CHECK_EQ(ctx.len, 8);
    for (int i = 0; i < 8; i++) MTK_CHECK_EQ(ctx.data[i], i + 1);

    /* Overflow rejected, not silently truncated. */
    mtk_compat_reassembly_reset(&ctx);
    ctx.active = 1; ctx.msg_id = 1; ctx.len = MTK_COMPAT_MAX_REASSEMBLY_PAYLOAD;
    mtk_compat_header_t small = {0}; small.msg_type = MTK_COMPAT_MSG_FRAG; small.msg_id = 1; small.payload_len = 1;
    uint8_t onemore = 0;
    MTK_CHECK_EQ(mtk_compat_reassembly_feed(&ctx, &small, &onemore), MTK_COMPAT_REASM_OVERFLOW);

    /* ---- End-to-end dispatch: DEAUTH_START via real Mtek Compatibility wire bytes ---- */
    mtk_core_init(0x1234);
    mtk_arbiter_init();
    mtk_router_init();
    static const mtk_system_build_info_t info = {1,0,0,"t",0,0,0,0,"h","n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();

    mtk_compat_dispatch_ctx_t dctx;
    mtek_compat_dispatch_init(&dctx, 0x1234);

    /* m1esp_deauth_req_t {bssid[6], channel, station[6], count:u16, interval_ms:u16} */
    uint8_t req_payload[6 + 1 + 6 + 2 + 2];
    memcpy(req_payload, (uint8_t[]){0x02,0x02,0x03,0x04,0x05,0x06}, 6);
    req_payload[6] = 6;
    memcpy(req_payload + 7, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    req_payload[13] = 1; req_payload[14] = 0; /* count = 1 LE */
    req_payload[15] = 0; req_payload[16] = 0; /* interval_ms = 0 */

    mtk_compat_header_t req_hdr = {0};
    req_hdr.magic = MTK_COMPAT_MAGIC; req_hdr.version = MTK_COMPAT_VERSION;
    req_hdr.msg_type = MTK_COMPAT_MSG_REQ; req_hdr.msg_id = 0x0302; req_hdr.payload_len = sizeof(req_payload);

    mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
    mtek_compat_dispatch_request(&dctx, &req_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);

    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* the real HAL was reached, not just a precondition error */
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_ap.b, req_payload, 6) == 0);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, req_payload + 7, 6) == 0);

    /* An unmapped opcode returns a real NAK(ERR_UNSUPPORTED), not a crash
     * or a guessed payload. */
    mtk_compat_header_t unknown_hdr = req_hdr; unknown_hdr.msg_id = 0x9999; unknown_hdr.payload_len = 0;
    mtek_compat_dispatch_request(&dctx, &unknown_hdr, req_payload, &resp_hdr, resp_payload, &resp_len);
    MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_NAK);
    MTK_CHECK_EQ(resp_payload[0], MTK_COMPAT_STATUS_ERR_UNSUPPORTED);

MTK_TEST_MAIN_END
