/* Schema encoding: round-trips every field kind the generic codec
 * supports (primitives, mac6/ipv4, bytes/utf8 with length prefix,
 * bytes_fixed, nested ref struct, array of primitives, array of structs),
 * plus the OVERFLOW/PROTOCOL_ERROR rejection paths. */
#include "mtk_test.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    /* PING: single u32 field. */
    {
        mtk_ping_req_t req = { .nonce = 0x11223344 };
        uint8_t buf[16]; size_t len = 0;
        MTK_CHECK_EQ(mtk_encode(&mtk_ping_req_t_desc, &req, buf, sizeof(buf), &len), MTK_CODEC_OK);
        MTK_CHECK_EQ(len, 4);
        MTK_CHECK_EQ(buf[0], 0x44); MTK_CHECK_EQ(buf[1], 0x33); MTK_CHECK_EQ(buf[2], 0x22); MTK_CHECK_EQ(buf[3], 0x11);
        mtk_ping_req_t out = {0};
        MTK_CHECK_EQ(mtk_decode(&mtk_ping_req_t_desc, &out, buf, len, NULL), MTK_CODEC_OK);
        MTK_CHECK_EQ(out.nonce, req.nonce);
    }

    /* STA_STATUS response: bool, bytes(max=32) w/ u8 length prefix, mac6, ipv4, i8. */
    {
        mtk_sta_status_resp_t r = {0};
        r.connected = 1;
        const char *ssid = "TestNet";
        r.ssid.len = (uint16_t)strlen(ssid);
        memcpy(r.ssid.data, ssid, r.ssid.len);
        memcpy(r.bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
        memcpy(r.ip_addr.b, (uint8_t[]){192,168,1,42}, 4);
        r.channel = 6;
        r.rssi = -55;
        uint8_t buf[128]; size_t len = 0;
        MTK_CHECK_EQ(mtk_encode(&mtk_sta_status_resp_t_desc, &r, buf, sizeof(buf), &len), MTK_CODEC_OK);
        /* connected(1) + len-prefix(1)+7 + bssid(6) + channel(1) + ip_present(1) + ip(4) + rssi(1) */
        MTK_CHECK_EQ(len, 1 + 1 + 7 + 6 + 1 + 1 + 4 + 1);
        mtk_sta_status_resp_t out = {0};
        MTK_CHECK_EQ(mtk_decode(&mtk_sta_status_resp_t_desc, &out, buf, len, NULL), MTK_CODEC_OK);
        MTK_CHECK_EQ(out.connected, 1);
        MTK_CHECK_EQ(out.ssid.len, 7);
        MTK_CHECK(memcmp(out.ssid.data, ssid, 7) == 0);
        MTK_CHECK(memcmp(out.bssid.b, r.bssid.b, 6) == 0);
        MTK_CHECK(memcmp(out.ip_addr.b, r.ip_addr.b, 4) == 0);
        MTK_CHECK_EQ(out.rssi, -55);
    }

    /* AP_SCAN_RESULTS_PAGE: array of ref-struct ApRecord. */
    {
        mtk_ap_scan_results_page_resp_t r = {0};
        r.items.count = 2;
        memcpy(r.items.items[0].bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
        r.items.items[0].ssid.len = 3; memcpy(r.items.items[0].ssid.data, "abc", 3);
        r.items.items[0].channel = 1; r.items.items[0].rssi = -40; r.items.items[0].authmode = 3;
        r.items.items[1].channel = 11; r.items.items[1].rssi = -80;
        r.next_index = 0;
        uint8_t buf[256]; size_t len = 0;
        MTK_CHECK_EQ(mtk_encode(&mtk_ap_scan_results_page_resp_t_desc, &r, buf, sizeof(buf), &len), MTK_CODEC_OK);
        mtk_ap_scan_results_page_resp_t out = {0};
        MTK_CHECK_EQ(mtk_decode(&mtk_ap_scan_results_page_resp_t_desc, &out, buf, len, NULL), MTK_CODEC_OK);
        MTK_CHECK_EQ(out.items.count, 2);
        MTK_CHECK_EQ(out.items.items[0].channel, 1);
        MTK_CHECK_EQ(out.items.items[0].ssid.len, 3);
        MTK_CHECK_EQ(out.items.items[1].channel, 11);
        MTK_CHECK_EQ(out.items.items[1].rssi, -80);
    }

    /* Malformed bool byte is rejected, never coerced (canonical core Sec 1:
     * "the only valid encoded values are 0x00 and 0x01"). STA_STATUS's
     * first response field is `connected: bool`, at wire offset 0. */
    {
        uint8_t buf[1] = { 0x05 };
        mtk_sta_status_resp_t out = {0};
        MTK_CHECK_EQ(mtk_decode(&mtk_sta_status_resp_t_desc, &out, buf, sizeof(buf), NULL), MTK_CODEC_PROTOCOL_ERROR);
    }

    /* Truncated buffer is rejected, not partially decoded. */
    {
        mtk_deauth_start_req_t req = {0};
        req.target_mode = 2; req.channel = 6;
        memcpy(req.ap_bssid.b, (uint8_t[]){1,1,1,1,1,1}, 6);
        uint8_t buf[64]; size_t len = 0;
        mtk_encode(&mtk_deauth_start_req_t_desc, &req, buf, sizeof(buf), &len);
        mtk_deauth_start_req_t out = {0};
        MTK_CHECK_EQ(mtk_decode(&mtk_deauth_start_req_t_desc, &out, buf, len - 1, NULL), MTK_CODEC_TRUNCATED);
    }

    /* OVERFLOW: bytes field longer than its bound is rejected at encode. */
    {
        mtk_gatt_write_req_t req = {0};
        req.data.len = 200; /* max=64 */
        uint8_t buf[512]; size_t len = 0;
        MTK_CHECK_EQ(mtk_encode(&mtk_gatt_write_req_t_desc, &req, buf, sizeof(buf), &len), MTK_CODEC_OVERFLOW);
    }

MTK_TEST_MAIN_END
