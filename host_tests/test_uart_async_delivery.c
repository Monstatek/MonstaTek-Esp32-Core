/* + the matching UART-side fix (mtek_uart_adapter.c's session_queue): proves
 * real background delivery survives past the initiating command's own call,
 * end-to-end through the UART adapter. Uses the fake Wi-Fi HAL's `defer_frames`
 * mode (frames delivered only when the test explicitly calls
 * mtk_fake_wifi_deliver_frames, standing in for a real target's promiscuous-mode
 * callback firing later from the Wi-Fi driver's own task, well after
 * `handshake`'s synchronous command call has already returned and its own stack
 * frame has been reused by the REPL loop's subsequent commands). Before this
 * fix, the handshake session stored a raw `mtk_request_ctx_t*` into the router's
 * transient async-pool slot / the UART handler's own stack; delivering a frame
 * after that call returned would write through a dead/reused pointer. With the
 * fix, the session's sink points at mtk_uart_adapter_state_t::session_queue --
 * alive for the REPL task's entire boot session -- so delivery after return is
 * not just crash-free but functionally correct:
 * mtek_uart_adapter_poll_background must actually surface the real event text. */
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
    /* Addr1/addr2/addr3 must name the target BSSID {1,2,3,4,5,6} used by this
     * file's own HANDSHAKE_START requests, or mtek_wifi_logic.c's new
     * frame_matches_target_bssid filter would reject every synthetic frame this
     * test builds. */
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

    /* Set up AP scan + selection so `handshake` has a valid target. */
    g_fake_wifi.ap_count = 1;
    memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    memcpy(g_fake_wifi.ap_results[0].ssid, "Target", 6); g_fake_wifi.ap_results[0].ssid_len = 6;
    g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -40; g_fake_wifi.ap_results[0].authmode = 3;

    char out[4096];
    mtek_uart_process_line(&st, "scan -a", out, sizeof(out));
    mtek_uart_process_line(&st, "select -a 0", out, sizeof(out));

    /* Arm deferred delivery: promisc_start returns immediately, capturing
     * the callback/user pointer instead of replaying frames synchronously
     * -- exactly modeling the real target's asynchronous HAL. */
    g_fake_wifi.defer_frames = 1;
    uint16_t len = build_eapol_frame(g_fake_wifi.frames[0].data, 1, 0, 0, 0); /* M1 */
    g_fake_wifi.frames[0].len = len; g_fake_wifi.frames[0].channel = 6;
    g_fake_wifi.frame_count = 1;

    size_t n = mtek_uart_process_line(&st, "handshake", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK(strcmp(out, "[*] Handshake capture started.\n") == 0);
    MTK_CHECK_EQ(st.handshake_running, 1);

    /* Nothing pending yet -- the frame has not been delivered. */
    MTK_CHECK_EQ(mtek_uart_adapter_poll_background(&st, out, sizeof(out)), 0);

    /* Now deliver the M1 frame -- as far as this test's call stack is
     * concerned, this happens strictly after handle_handshake (and every
     * function it called) has already returned; any further UART command
     * could have run in between and reused that stack. The old
     * raw-pointer design would write through a stale/reused
     * mtk_request_ctx_t* here. */
    mtek_uart_process_line(&st, "version", out, sizeof(out)); /* an unrelated command in between, further disturbing the stack */
    mtk_fake_wifi_deliver_frames();

    size_t bn = mtek_uart_adapter_poll_background(&st, out, sizeof(out));
    MTK_CHECK(bn > 0);
    MTK_CHECK(strcmp(out, "[FOUND EAPOL] key_frame=M1\n") == 0);

    /* Queue is drained after one poll; a second poll finds nothing new. */
    MTK_CHECK_EQ(mtek_uart_adapter_poll_background(&st, out, sizeof(out)), 0);

    /* a genuine peer-initiated disconnect, observed asynchronously
     * (mtek_ble_gatt_tick, driven by the same periodic-tick shape as every other
     * background delivery this test file proves), must surface through
     * mtek_uart_adapter_poll_background with the exact same frozen transcript
     * line handle_ble_disconnect already prints for a caller-issued disconnect.
     * ----------------- */
    {
        /* The handshake session above is still RUNNING (only M1 was
         * observed, not a full M1+M2+M4 capture) and still holds the
         * MTK_ARB_H arbiter lease -- stop it first so MTK_ARB_BS (BLE
         * scan, below) can acquire the arbiter. */
        mtek_uart_process_line(&st, "", out, sizeof(out)); /* bare Enter stops the running handshake */
        st.mode = MTK_UART_MODE_BLE;
        memset(&g_fake_ble.scan_results[0], 0, sizeof(g_fake_ble.scan_results[0]));
        memcpy(g_fake_ble.scan_results[0].addr.b, (uint8_t[]){9,9,9,9,9,9}, 6);
        g_fake_ble.scan_results[0].rssi = -50;
        g_fake_ble.scan_count = 1;
        g_fake_ble.gatt_connect_rc = 0;
        g_fake_ble.gatt_vendor_handle = 42;

        mtek_uart_process_line(&st, "scan", out, sizeof(out));
        size_t cn = mtek_uart_process_line(&st, "connect 0", out, sizeof(out));
        MTK_CHECK(cn > 0);
        MTK_CHECK(st.gatt_connected == 1);

        /* Nothing pending yet. */
        MTK_CHECK_EQ(mtek_uart_adapter_poll_background(&st, out, sizeof(out)), 0);

        /* The peer disconnects on its own, strictly after `connect`
         * already returned -- mtek_ble_gatt_tick (this target's own
         * periodic driver, main/app_main.c's ble_tick_task) is what
         * would observe this on real hardware; called directly here. */
        /* A genuinely nonzero reason code here (0x213, NimBLE's own
         * BLE_HS_HCI_ERR connection-timeout example) proves the real value
         * threads all the way through the HAL/service/adapter chain, not just
         * that SOME reason (indistinguishable from the old hard-coded 0) was
         * printed. */
        g_fake_ble.remote_disconnect_pending = 1;
        g_fake_ble.remote_disconnect_reason = 0x13;
        mtek_ble_gatt_tick();

        size_t dn = mtek_uart_adapter_poll_background(&st, out, sizeof(out));
        MTK_CHECK(dn > 0);
        MTK_CHECK(strcmp(out, "[BLE:CONN] disconnected reason=19\n") == 0);

        /* Consumed exactly once. */
        MTK_CHECK_EQ(mtek_uart_adapter_poll_background(&st, out, sizeof(out)), 0);
    }

MTK_TEST_MAIN_END
