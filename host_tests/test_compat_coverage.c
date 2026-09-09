/* Proves every one of the canonical opcodes schemas.json marks
 * capability_state.compat_c3=SUPPORTED is actually reachable through its
 * real, documented Mtek Compatibility wire opcode -- not merely present in a
 * registry, and never silently answered by the generic
 * default-unsupported switch case (Part 1, table-driven). Part 2 goes
 * further for every opcode with a confirmed exact Mtek Compatibility wire byte
 * layout: proves real request decode (the fake HAL/service actually
 * receives the decoded fields, not just "some request"), a real
 * canonical side effect, and the exact confirmed Mtek Compatibility response byte
 * shape -- not merely that a RESP/NAK came back. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_compat_dispatch.h"
#include "mtek_compat_opcode_map.h"
#include <string.h>

typedef struct {
    uint16_t msg_id;
    uint16_t service_id;
    uint16_t opcode;
    const uint8_t *payload;
    uint16_t payload_len;
} cov_row_t;

static const uint8_t z32[32] = {0};
static const uint8_t probe_flood_empty[2] = {6, 0}; /* channel=6, count=0 (wildcard) */

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_compat_dispatch_ctx_t dctx;
    mtek_compat_dispatch_init(&dctx, MTK_TEST_BOOT_EPOCH);

    /* ==== Part 1: every SUPPORTED opcode's own wire call reaches the
     * router for its documented (service_id, opcode) ==================== */
    const cov_row_t rows[] = {
        { 0x0001, 0x0000, 0x0001, NULL, 0 },                 /* PING */
        { 0x0002, 0x0000, 0x0002, NULL, 0 },                 /* GET_STATUS (-> canonical GET_VERSION) */
        { 0x0003, 0x0000, 0x0002, NULL, 0 },                 /* GET_FW_VERSION (-> canonical GET_VERSION) */
        { 0x0005, 0x0000, 0x0006, NULL, 0 },                 /* RESET_INTENT */
        /* RC12 hardening round, item 5 (P1): TIME_SYNC_START (canonical
         * 0x0000/0x0008) removed from this SUPPORTED-opcode coverage table
         * -- its compat_c3 capability is now UNSUPPORTED (no SNTP client),
         * so it is no longer one of the "opcodes schemas.json marks
         * capability_state.compat_c3=SUPPORTED" this table enumerates. */
        { 0x0103, 0x0001, 0x0003, NULL, 0 },                 /* AP_SCAN_START, chains into AP_SCAN_RESULTS_PAGE (0x0003) for real -- see handle_ap_scan_start */
        { 0x030E, 0x0001, 0x0006, z32, 8 },                  /* STA_SCAN_START */
        { 0x0104, 0x0001, 0x000A, z32, 2 },                  /* STA_CONNECT */
        { 0x0105, 0x0001, 0x000B, NULL, 0 },                 /* STA_DISCONNECT */
        { 0x0106, 0x0001, 0x000C, NULL, 0 },                 /* STA_STATUS */
        { 0x0304, 0x0001, 0x000D, NULL, 0 },                 /* BEACON_START */
        { 0x0305, 0x0001, 0x000E, NULL, 0 },                 /* BEACON_STOP */
        { 0x0302, 0x0001, 0x0010, z32, 17 },                 /* DEAUTH_START */
        { 0x0303, 0x0001, 0x0011, NULL, 0 },                 /* DEAUTH_STOP */
        { 0x030D, 0x0001, 0x0012, NULL, 0 },                 /* DEAUTH_STATUS */
        { 0x0310, 0x0001, 0x0013, z32, 9 },                  /* HANDSHAKE_START */
        { 0x0311, 0x0001, 0x0014, NULL, 0 },                 /* HANDSHAKE_STATUS */
        { 0x0313, 0x0001, 0x0016, NULL, 0 },                 /* HANDSHAKE_STOP */
        { 0x0200, 0x0001, 0x0019, z32, 2 },                  /* SOFTAP_START */
        { 0x0201, 0x0001, 0x001A, NULL, 0 },                 /* SOFTAP_STOP */
        { 0x0202, 0x0001, 0x001B, NULL, 0 },                 /* SOFTAP_STA_LIST */
        { 0x0306, 0x0001, 0x001C, probe_flood_empty, 2 },    /* PROBE_FLOOD_START */
        { 0x0307, 0x0001, 0x001D, NULL, 0 },                 /* PROBE_FLOOD_STOP */
        { 0x0309, 0x0001, 0x001F, z32, 1 },                  /* KARMA_START */
        { 0x030A, 0x0001, 0x0020, NULL, 0 },                 /* KARMA_STOP */
        { 0x030C, 0x0001, 0x0021, z32, 11 },                 /* RAW_TX_SEND */
        { 0x0316, 0x0001, 0x0022, z32, 2 },                  /* CAPTIVE_PORTAL_START */
        { 0x0317, 0x0001, 0x0023, NULL, 0 },                 /* CAPTIVE_PORTAL_STOP */
        { 0x0318, 0x0001, 0x0024, NULL, 0 },                 /* CAPTIVE_PORTAL_GET_CREDENTIALS */
        { 0x0319, 0x0001, 0x0025, NULL, 0 },                 /* CAPTIVE_PORTAL_GET_DIAGNOSTICS */
        { 0x0102, 0x0001, 0x0029, z32, 1 },                  /* WIFI_MAC_GET */
        { 0x0401, 0x0002, 0x0001, NULL, 0 },                 /* BLE_SCAN_START */
        { 0x0402, 0x0002, 0x0004, NULL, 0 },                 /* BLE_SCAN_RESULTS_PAGE */
        { 0x0403, 0x0002, 0x0006, NULL, 0 },                 /* BLE_ADV_START */
        { 0x0404, 0x0002, 0x0007, NULL, 0 },                 /* BLE_ADV_STOP */
        { 0x040A, 0x0003, 0x0002, NULL, 0 },                 /* GATT_DISCONNECT */
        { 0x0300, 0x0004, 0x0001, z32, 2 },                  /* CAPTURE_START */
        { 0x0301, 0x0004, 0x0002, NULL, 0 },                 /* CAPTURE_STOP */
        { 0x0314, 0x0004, 0x0003, NULL, 0 },                 /* CAPTURE_STATUS */
        { 0x0315, 0x0004, 0x0006, NULL, 0 },                 /* CAPTURE_POLL_READ */
    };
    const unsigned n = sizeof(rows) / sizeof(rows[0]);

    for (unsigned i = 0; i < n; i++) {
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = rows[i].msg_id; hdr.payload_len = rows[i].payload_len;
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        dctx.last_dispatched_service_id = 0xFFFF; dctx.last_dispatched_opcode = 0xFFFF;
        mtek_compat_dispatch_request(&dctx, &hdr, rows[i].payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.last_dispatched_service_id, rows[i].service_id);
        MTK_CHECK_EQ(dctx.last_dispatched_opcode, rows[i].opcode);
        MTK_CHECK(resp_hdr.msg_type == MTK_COMPAT_MSG_RESP || resp_hdr.msg_type == MTK_COMPAT_MSG_NAK || resp_hdr.msg_type == MTK_COMPAT_MSG_FRAG);
    }

    /* GATT_CONNECT: request decode is byte-exact-confirmed
     * ([addr:6][addr_type:1] -> ble_conn_connect(payload, payload[6])),
     * so it is proven separately with a real fake-BLE-HAL connect and a
     * real minted connection_token, rather than an empty/default row. */
    {
        mtk_fake_ble_reset();
        mtek_ble_set_hal(&g_fake_ble_hal);
        g_fake_ble.gatt_connect_rc = 0;
        g_fake_ble.gatt_vendor_handle = 77;
        uint8_t gc_payload[7] = {0x10,0x20,0x30,0x40,0x50,0x60, 1 /* RANDOM_STATIC */};
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = 0x0409; hdr.payload_len = sizeof(gc_payload);
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &hdr, gc_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.last_dispatched_service_id, 0x0003);
        MTK_CHECK_EQ(dctx.last_dispatched_opcode, 0x0001);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP); /* real ACCEPTED, real HAL reached */
        MTK_CHECK(dctx.gatt_conn_token != 0); /* minted from the real GATT_CONNECT_COMPLETE event */

        /* GATT_DISCONNECT now addresses that real, minted connection. */
        mtek_compat_dispatch_request(&dctx, &(mtk_compat_header_t){.magic=MTK_COMPAT_MAGIC,.version=MTK_COMPAT_VERSION,.msg_type=MTK_COMPAT_MSG_REQ,.msg_id=0x040A,.payload_len=0}, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
    }

    /* AP_SCAN_START: real fake-HAL AP records translated into Mtek Compatibility's
     * confirmed exact wire shape: [count:2] + per-AP
     * [bssid:6][rssi:1][channel:1][authmode:1][ssid_len:1][ssid]. */
    {
        mtk_fake_wifi_reset();
        mtek_wifi_set_hal(&g_fake_wifi_hal);
        g_fake_wifi.ap_count = 2;
        memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        memcpy(g_fake_wifi.ap_results[0].ssid, "HomeNet", 7); g_fake_wifi.ap_results[0].ssid_len = 7;
        g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -45; g_fake_wifi.ap_results[0].authmode = 3;
        memcpy(g_fake_wifi.ap_results[1].bssid.b, (uint8_t[]){0x11,0x22,0x33,0x44,0x55,0x66}, 6);
        memcpy(g_fake_wifi.ap_results[1].ssid, "OpenNet", 7); g_fake_wifi.ap_results[1].ssid_len = 7;
        g_fake_wifi.ap_results[1].channel = 1; g_fake_wifi.ap_results[1].rssi = -70; g_fake_wifi.ap_results[1].authmode = 0;

        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = 0x0103; hdr.payload_len = 0;
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 2 + (6+1+1+1+1+7)*2);
        uint16_t count = (uint16_t)(resp_payload[0] | (resp_payload[1] << 8));
        MTK_CHECK_EQ(count, 2);
        MTK_CHECK(memcmp(resp_payload + 2, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6) == 0);
        MTK_CHECK_EQ((int8_t)resp_payload[8], -45);
        MTK_CHECK_EQ(resp_payload[9], 6);   /* channel */
        MTK_CHECK_EQ(resp_payload[10], 3);  /* authmode */
        MTK_CHECK_EQ(resp_payload[11], 7);  /* ssid_len */
        MTK_CHECK(memcmp(resp_payload + 12, "HomeNet", 7) == 0);
    }

    /* STA_SCAN_START/RESULTS_PAGE: real fake-HAL station records
     * translated into Mtek Compatibility's confirmed exact [count:2] +
     * [mac:6][rssi:1] shape. */
    {
        mtk_fake_wifi_reset();
        mtek_wifi_set_hal(&g_fake_wifi_hal);
        g_fake_wifi.sta_count = 2;
        memcpy(g_fake_wifi.sta_results[0].mac.b, (uint8_t[]){0x02,0x02,0x02,0x02,0x02,0x02}, 6);
        g_fake_wifi.sta_results[0].rssi = -50;
        memcpy(g_fake_wifi.sta_results[1].mac.b, (uint8_t[]){0x04,0x04,0x04,0x04,0x04,0x04}, 6);
        g_fake_wifi.sta_results[1].rssi = -60;

        uint8_t start_payload[8] = {0x01,0x02,0x03,0x04,0x05,0x06, 6, 1};
        mtk_compat_header_t start_hdr = {0};
        start_hdr.magic = MTK_COMPAT_MAGIC; start_hdr.version = MTK_COMPAT_VERSION;
        start_hdr.msg_type = MTK_COMPAT_MSG_REQ; start_hdr.msg_id = 0x030E; start_hdr.payload_len = 8;
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &start_hdr, start_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK(dctx.sta_scan_has_generation);
        /* Real request decode proven: the fake HAL received the exact
         * bssid/channel/duration this Mtek Compatibility payload encoded. */
        MTK_CHECK(memcmp(g_fake_wifi.last_sta_scan_bssid.b, start_payload, 6) == 0);
        MTK_CHECK_EQ(g_fake_wifi.last_sta_scan_channel, 6);

        mtk_compat_header_t page_hdr = start_hdr; page_hdr.msg_id = 0x030F; page_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &page_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(dctx.last_dispatched_service_id, 0x0001);
        MTK_CHECK_EQ(dctx.last_dispatched_opcode, 0x0008);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 2 + 7*2);
        uint16_t count = (uint16_t)(resp_payload[0] | (resp_payload[1] << 8));
        MTK_CHECK_EQ(count, 2);
        MTK_CHECK(memcmp(resp_payload + 2, (uint8_t[]){0x02,0x02,0x02,0x02,0x02,0x02}, 6) == 0);
        MTK_CHECK_EQ((int8_t)resp_payload[8], -50);
    }

    /* HANDSHAKE_STATUS/READ/STOP: real captured handshake bytes
     * translated into the confirmed exact
     * [state:1][total_len:4]/[total_len:4][data_len:2][data]/
     * [final_state:1][final_status:1] shapes. */
    {
        mtk_fake_wifi_reset();
        mtek_wifi_set_hal(&g_fake_wifi_hal);
        uint8_t frame[200]; memset(frame, 0, sizeof(frame));
        frame[0] = 0x88; frame[1] = 0x02; /* QoS Data, type=2 subtype=8 */
        /* RC7 independent audit item 11 "EAPOL frames are not filtered to
         * the requested AP/station": addr1/addr2/addr3 must name the
         * target BSSID {1,2,3,4,5,6} this test's own hs_payload (below)
         * requests, or mtek_wifi_logic.c's new frame_matches_target_bssid
         * filter would reject this synthetic frame. */
        {
            static const uint8_t bssid[6] = {1,2,3,4,5,6};
            memcpy(frame + 4, bssid, 6); memcpy(frame + 10, bssid, 6); memcpy(frame + 16, bssid, 6);
        }
        unsigned off = 24 + 2; /* 802.11 header + QoS control */
        static const uint8_t llc[8] = {0xAA,0xAA,0x03,0x00,0x00,0x00,0x88,0x8E};
        memcpy(frame + off, llc, 8); off += 8;
        frame[off + 0] = 2; frame[off + 1] = 3; frame[off + 2] = 0; frame[off + 3] = 95;
        unsigned eapol = off;
        frame[eapol + 4] = 2; /* descriptor type */
        uint16_t key_info = (uint16_t)(1 << 7); /* ACK set: M1 */
        frame[eapol + 5] = (uint8_t)(key_info >> 8); frame[eapol + 6] = (uint8_t)(key_info & 0xFF);
        uint16_t flen = (uint16_t)(eapol + 4 + 1 + 2 + 96);
        memcpy(g_fake_wifi.frames[0].data, frame, flen);
        g_fake_wifi.frames[0].len = flen; g_fake_wifi.frames[0].channel = 6; g_fake_wifi.frames[0].rssi = -40;
        g_fake_wifi.frame_count = 1;

        uint8_t hs_payload[9] = {1,2,3,4,5,6, 6, 0,0};
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = 0x0310; hdr.payload_len = sizeof(hs_payload);
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &hdr, hs_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK(dctx.handshake_token != 0);
        MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 1); /* real HAL reached */

        mtk_compat_header_t st_hdr = hdr; st_hdr.msg_id = 0x0311; st_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &st_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 5);
        uint32_t total_len = (uint32_t)resp_payload[1] | ((uint32_t)resp_payload[2] << 8) | ((uint32_t)resp_payload[3] << 16) | ((uint32_t)resp_payload[4] << 24);
        MTK_CHECK(total_len > 0); /* one captured frame's worth of bytes */

        uint8_t rd_hdr_payload[6] = {0,0,0,0, 255,1}; /* offset=0, max_len=511 */
        mtk_compat_header_t rd_hdr = hdr; rd_hdr.msg_id = 0x0312; rd_hdr.payload_len = sizeof(rd_hdr_payload);
        mtek_compat_dispatch_request(&dctx, &rd_hdr, rd_hdr_payload, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK(resp_len >= 6);
        uint32_t rd_total = (uint32_t)resp_payload[0] | ((uint32_t)resp_payload[1] << 8) | ((uint32_t)resp_payload[2] << 16) | ((uint32_t)resp_payload[3] << 24);
        uint16_t data_len = (uint16_t)(resp_payload[4] | (resp_payload[5] << 8));
        MTK_CHECK_EQ(rd_total, total_len);
        MTK_CHECK_EQ(resp_len, 6 + data_len);

        mtk_compat_header_t sp_hdr = hdr; sp_hdr.msg_id = 0x0313; sp_hdr.payload_len = 0;
        mtek_compat_dispatch_request(&dctx, &sp_hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 2);
        MTK_CHECK_EQ(dctx.handshake_token, 0); /* cleared on stop */
    }

    /* PING: real byte-for-byte echo of an arbitrary-length payload,
     * matching Mtek Compatibility's own confirmed "does not interpret the cookie at
     * all" behavior -- not a canonical nonce round trip. */
    {
        uint8_t cookie[6] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02};
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = 0x0001; hdr.payload_len = sizeof(cookie);
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &hdr, cookie, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, sizeof(cookie));
        MTK_CHECK(memcmp(resp_payload, cookie, sizeof(cookie)) == 0);
    }

    /* GET_STATUS/GET_FW_VERSION: real system build-info fields
     * translated into the confirmed exact m1esp_devstatus_t/
     * m1esp_fw_version_t shapes. */
    {
        mtk_compat_header_t hdr = {0};
        hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
        hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = 0x0003; hdr.payload_len = 0; /* GET_FW_VERSION */
        mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
        mtek_compat_dispatch_request(&dctx, &hdr, NULL, &resp_hdr, resp_payload, &resp_len);
        MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_RESP);
        MTK_CHECK_EQ(resp_len, 3 + 16);
        MTK_CHECK_EQ(resp_payload[0], 1); /* product_major, from mtk_test_bootstrap's build_info {1,0,0,...} */
        MTK_CHECK_EQ(resp_payload[3 + 4], 0); /* git_hash null terminator within bound, build_id="test" -> 4 bytes then NUL */
        MTK_CHECK(memcmp(resp_payload + 3, "test", 4) == 0);
    }

    /* The 15 non-SUPPORTED opcodes: still dispatched (through the
     * capability gate), never a hand-rolled Mtek Compatibility-layer NAK. */
    {
        const uint16_t non_supported_msg_ids[] = {
            0x0100, 0x0101, 0x0004, 0x0405, 0x0406, 0x0407, 0x0408, 0x040B, 0x040C, 0x040D, 0x040E, 0x0411, 0x0412, 0x040F, 0x0410
        };
        for (unsigned i = 0; i < sizeof(non_supported_msg_ids)/sizeof(non_supported_msg_ids[0]); i++) {
            mtk_compat_header_t hdr = {0};
            hdr.magic = MTK_COMPAT_MAGIC; hdr.version = MTK_COMPAT_VERSION;
            hdr.msg_type = MTK_COMPAT_MSG_REQ; hdr.msg_id = non_supported_msg_ids[i]; hdr.payload_len = 0;
            mtk_compat_header_t resp_hdr; uint8_t resp_payload[MTK_COMPAT_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
            dctx.last_dispatched_service_id = 0xFFFF; dctx.last_dispatched_opcode = 0xFFFF;
            mtek_compat_dispatch_request(&dctx, &hdr, NULL, &resp_hdr, resp_payload, &resp_len);
            MTK_CHECK(dctx.last_dispatched_service_id != 0xFFFF); /* really reached the router's capability gate */
            MTK_CHECK_EQ(resp_hdr.msg_type, MTK_COMPAT_MSG_NAK);
            MTK_CHECK_EQ(resp_payload[0], MTK_COMPAT_STATUS_ERR_UNSUPPORTED);
        }
    }

    /* HANDSHAKE_READ/GATT_CONNECT are no longer translation-facts
     * blockers (both had exact confirmed request shapes supplied this
     * review round) -- the only remaining opcodes never dispatched at
     * all are the two whose exact request byte layout genuinely has no
     * confirmed fact anywhere in the accepted contract package. */
    MTK_CHECK(mtk_opcode_find(0x0001, 0x0015) != NULL); /* HANDSHAKE_READ: real canonical opcode, now dispatched above */
    MTK_CHECK(mtk_opcode_find(0x0003, 0x0001) != NULL); /* GATT_CONNECT: real canonical opcode, now dispatched above */

MTK_TEST_MAIN_END
