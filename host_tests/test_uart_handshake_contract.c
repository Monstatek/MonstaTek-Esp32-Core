/* Regression lock for the historically validated STM32-facing handshake
 * contract over the factory-UART REPL (docs/COMPATIBILITY_LEDGER.md).
 *
 * The STM32 handshake flow (m1_wifi.c) keys completion on a line matching
 * "[DONE] Capture finished. Mask: 0x%X, Count: %d", drives its live M1-M4 view
 * from any line containing "Mask: 0x", and retrieves the capture with
 * "list -h" (storing the returned text to SD). Current Core had regressed all
 * three: no list -h handler, and a "[HANDSHAKE:STOPPED] status=.. captured_len"
 * completion line the STM32 does not parse. This test asserts the restored
 * behavior so it cannot silently disappear again. */
#include "mtk_test.h"
#include "mtek_uart_adapter.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_router.h"
#include "mtek_system_service.h"
#include "mtek_wifi_service.h"
#include "mtek_ble_service.h"
#include "mtek_capture_service.h"
#include "mtk_fake_wifi_hal.h"
#include "mtk_fake_ble_hal.h"
#include <string.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static uint16_t build_eapol_frame(uint8_t *buf, int ack, int mic, int secure, int install) {
    memset(buf, 0, 200);
    static const uint8_t bssid[6] = {1,2,3,4,5,6};
    memcpy(buf + 4, bssid, 6); memcpy(buf + 10, bssid, 6); memcpy(buf + 16, bssid, 6);
    buf[0] = 0x88; buf[1] = 0x02;
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

MTK_TEST_MAIN_BEGIN
    mtk_core_init(0x9911);
    mtk_arbiter_init();
    mtk_router_init();
    static const mtk_system_build_info_t info = {1, 0, 0, "t", 0, 0, 0, 0, "h", "n"};
    mtek_system_service_init(&info, now_ms);
    mtek_wifi_service_init(now_ms);
    mtek_ble_service_init(now_ms);
    mtek_capture_service_init(now_ms);
    mtk_fake_wifi_reset();
    mtk_fake_ble_reset();
    mtek_wifi_set_hal(&g_fake_wifi_hal);
    mtek_ble_set_hal(&g_fake_ble_hal);
    mtek_system_service_register();
    mtek_wifi_service_register();
    mtek_ble_service_register();
    mtek_capture_service_register();

    mtk_uart_adapter_state_t st;
    mtek_uart_adapter_init(&st, 0x9911);

    char out[4096];

    /* Before any capture, `list -h` reports nothing (a distinct, non-payload
     * line the STM32 treats as "No handshake data"). */
    mtek_uart_process_line(&st, "list -h", out, sizeof(out));
    MTK_CHECK(strstr(out, "No packets captured") != NULL);

    /* Select an AP so `handshake` has a target. */
    g_fake_wifi.ap_count = 1;
    memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    memcpy(g_fake_wifi.ap_results[0].ssid, "Target", 6); g_fake_wifi.ap_results[0].ssid_len = 6;
    g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -40; g_fake_wifi.ap_results[0].authmode = 3;
    mtek_uart_process_line(&st, "scan -a", out, sizeof(out));
    mtek_uart_process_line(&st, "select -a 0", out, sizeof(out));

    /* Deferred full 4-way: M1,M2,M3,M4 -> running mask 0x01,0x03,0x07,0x0F. */
    g_fake_wifi.defer_frames = 1;
    g_fake_wifi.frames[0].len = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
    g_fake_wifi.frames[1].len = build_eapol_frame(g_fake_wifi.frames[1].data, 0, 1, 0, 0); /* M2 */
    g_fake_wifi.frames[2].len = build_eapol_frame(g_fake_wifi.frames[2].data, 1, 1, 1, 1); /* M3 */
    g_fake_wifi.frames[3].len = build_eapol_frame(g_fake_wifi.frames[3].data, 0, 1, 1, 0); /* M4 */
    for (int i = 0; i < 4; i++) g_fake_wifi.frames[i].channel = 6;
    g_fake_wifi.frame_count = 4;

    size_t n = mtek_uart_process_line(&st, "handshake", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK_EQ(st.handshake_running, 1);

    mtk_fake_wifi_deliver_frames();

    /* Drain all background lines into one buffer. */
    char acc[8192]; size_t acc_len = 0; acc[0] = 0;
    for (int i = 0; i < 16; i++) {
        size_t bn = mtek_uart_adapter_poll_background(&st, out, sizeof(out));
        if (bn == 0) break;
        if (acc_len + bn < sizeof(acc)) { memcpy(acc + acc_len, out, bn); acc_len += bn; acc[acc_len] = 0; }
    }

    /* Live per-message lines carry the running M1-M4 mask (STM32 sscanf's
     * "Mask: 0x%X" off any of these). */
    MTK_CHECK(strstr(acc, " >>> [SUCCESS] Message 1 stored! (Mask: 0x01)") != NULL);
    MTK_CHECK(strstr(acc, "(Mask: 0x03)") != NULL);
    MTK_CHECK(strstr(acc, "(Mask: 0x0F)") != NULL);
    /* Completion line the STM32 keys success on; full 4-way => Mask 0x0F. */
    MTK_CHECK(strstr(acc, "[DONE] Capture finished. Mask: 0x0F, Count: 4") != NULL);
    /* Core's earlier, STM32-unparseable wording must be gone. */
    MTK_CHECK(strstr(acc, "[HANDSHAKE:STOPPED]") == NULL);

    /* `list -h` now returns the captured EAPOL bytes (non-empty payload). */
    size_t ln = mtek_uart_process_line(&st, "list -h", out, sizeof(out));
    MTK_CHECK(ln > 0);
    MTK_CHECK(strstr(out, "EAPOL Packets Captured:") != NULL);
    MTK_CHECK(strstr(out, "No packets captured") == NULL);
    /* Header + at least one hex line => strictly more than the header alone. */
    MTK_CHECK(ln > (size_t)(strchr(out, '\n') - out + 1));
MTK_TEST_MAIN_END
