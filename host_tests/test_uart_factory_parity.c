/* Factory UART adapter: verbatim response strings from for the wired command
 * subset (see docs/PROVENANCE.md for scope), driven end-to-end through the
 * router against the fake Wi-Fi HAL. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_uart_adapter_state_t st;
    mtek_uart_adapter_init(&st, MTK_TEST_BOOT_EPOCH);

    char out[4096];

    mtek_uart_process_line(&st, "mode", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: WIFI\n") == 0);

    mtek_uart_process_line(&st, "mode -b", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: BLE\n") == 0);
    mtek_uart_process_line(&st, "mode -w", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: WIFI\n") == 0);

    /* scan -a: canned two-AP fake result, exact row format
     * "[%02d] %-34s %-4d %-5d %-18s %-8s" + completion line. */
    mtk_fake_wifi_reset();
    g_fake_wifi.ap_count = 2;
    memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6);
    memcpy(g_fake_wifi.ap_results[0].ssid, "HomeNet", 7); g_fake_wifi.ap_results[0].ssid_len = 7;
    g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -45; g_fake_wifi.ap_results[0].authmode = 3;
    memcpy(g_fake_wifi.ap_results[1].bssid.b, (uint8_t[]){0x11,0x22,0x33,0x44,0x55,0x66}, 6);
    memcpy(g_fake_wifi.ap_results[1].ssid, "OpenNet", 7); g_fake_wifi.ap_results[1].ssid_len = 7;
    g_fake_wifi.ap_results[1].channel = 1; g_fake_wifi.ap_results[1].rssi = -70; g_fake_wifi.ap_results[1].authmode = 0;

    size_t n = mtek_uart_process_line(&st, "scan -a", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK(strstr(out, "AA:BB:CC:DD:EE:FF") != NULL);
    MTK_CHECK(strstr(out, "HomeNet") != NULL);
    MTK_CHECK(strstr(out, "[+] Scan complete. 2 AP(s) found.\n") != NULL);

    /* deauth grammar: precondition-error strings, exact from the matrix. */
    mtek_uart_process_line(&st, "deauth", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Invalid deauth target selection. Scan/select targets first.\n") == 0);

    mtek_uart_process_line(&st, "deauth set 1,2,x", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Usage: deauth [set <id,id,...>|all|broadcast]\n") == 0);

    mtek_uart_process_line(&st, "deauth bogus", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Usage: deauth [set <id,id,...>|all|broadcast]\n") == 0);

    /* Real feature test: select -> scan -s -> select -> deauth actually reaches
     * the Wi-Fi HAL (not merely a precondition check). */
    mtek_uart_process_line(&st, "select -a 0", out, sizeof(out));
    MTK_CHECK(strstr(out, "HomeNet") != NULL);
    MTK_CHECK_EQ(st.ap_selected, 0);

    g_fake_wifi.sta_count = 2;
    memcpy(g_fake_wifi.sta_results[0].mac.b, (uint8_t[]){0x02,0x02,0x02,0x02,0x02,0x02}, 6);
    g_fake_wifi.sta_results[0].rssi = -50;
    memcpy(g_fake_wifi.sta_results[1].mac.b, (uint8_t[]){0x04,0x04,0x04,0x04,0x04,0x04}, 6);
    g_fake_wifi.sta_results[1].rssi = -60;
    mtek_uart_process_line(&st, "scan -s", out, sizeof(out));
    MTK_CHECK(strstr(out, "[+] Station scan complete. 2 station(s) found.\n") != NULL);
    MTK_CHECK(strstr(out, "02:02:02:02:02:02") != NULL);

    mtek_uart_process_line(&st, "list -a", out, sizeof(out));
    MTK_CHECK(strstr(out, "HomeNet") != NULL);
    mtek_uart_process_line(&st, "list -s", out, sizeof(out));
    MTK_CHECK(strstr(out, "02:02:02:02:02:02") != NULL);

    mtek_uart_process_line(&st, "select -s 0", out, sizeof(out));
    MTK_CHECK_EQ(st.sta_selected, 0);
    mtek_uart_process_line(&st, "select -l", out, sizeof(out));
    MTK_CHECK(strstr(out, "HomeNet") != NULL);
    MTK_CHECK(strstr(out, "02:02:02:02:02:02") != NULL);

    unsigned restore_before = g_fake_wifi.restore_count;
    g_fake_wifi.deauth_sent_count = 0;
    mtek_uart_process_line(&st, "deauth", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Deauth started.\n") == 0);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* the real HAL was reached */
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_ap.b, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, 6) == 0);
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, (uint8_t[]){0x02,0x02,0x02,0x02,0x02,0x02}, 6) == 0);
    /* In's synchronous-dispatch model (see docs/PROVENANCE.md), DEAUTH_START's
     * whole round-robin already ran and reached its own terminal COMPLETED state
     * -- including restoring STA mode -- by the time this call returns;
     * `st.deauth_running` therefore already reflects "no longer active" and STOP
     * is correctly a no-op/idempotent call against an already-terminal
     * operation. */
    MTK_CHECK(g_fake_wifi.restore_count > restore_before); /* STA mode restored (by deauth's own completion) */

    mtek_uart_process_line(&st, "stop", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Stopping...\nAttack stopped\n") == 0);
    MTK_CHECK_EQ(st.deauth_running, 0);

    /* `deauth set <id,id,...>` with a real, resolvable id list. */
    g_fake_wifi.deauth_sent_count = 0;
    mtek_uart_process_line(&st, "deauth set 0,1", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Deauth started.\n") == 0);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 2);
    mtek_uart_process_line(&st, "stop", out, sizeof(out));

    /* `deauth all` against the retained station-scan snapshot. */
    g_fake_wifi.deauth_sent_count = 0;
    mtek_uart_process_line(&st, "deauth all", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Deauth started.\n") == 0);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 2); /* both retained stations */
    mtek_uart_process_line(&st, "stop", out, sizeof(out));

    /* `deauth broadcast` against the selected AP. */
    g_fake_wifi.deauth_sent_count = 0;
    mtek_uart_process_line(&st, "deauth broadcast", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Deauth started.\n") == 0);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1);
    static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    MTK_CHECK(memcmp(g_fake_wifi.last_deauth_station.b, bcast, 6) == 0);
    mtek_uart_process_line(&st, "stop", out, sizeof(out));

    /* unknown-line fallback, per mode. */
    mtek_uart_process_line(&st, "frobnicate", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] WIFI mode supports only WIFI commands. Type 'help' for available commands.\n") == 0);
    mtek_uart_process_line(&st, "mode -b", out, sizeof(out));
    mtek_uart_process_line(&st, "frobnicate", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] BLE mode supports 'scan', 'list <id>|all', 'advertise', and 'stop' commands. Type 'help' for available commands.\n") == 0);

    /* `reboot` sets a flag the target REPL task (app_main.c) uses to perform a
     * real delayed esp_restart -- this portable component only sets the flag
     * (host-testable), never calls esp_restart itself. */
    mtek_uart_process_line(&st, "mode -w", out, sizeof(out));
    MTK_CHECK_EQ(st.reboot_requested, 0);
    mtek_uart_process_line(&st, "reboot", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Rebooting system...\n") == 0);
    MTK_CHECK_EQ(st.reboot_requested, 1);

    /* Bare Enter (an empty line) stops a running List B operation,
     * matching the shipped command-behavior matrix's documented
     * bare-Enter attack stop -- and is a no-op when nothing is running. */
    mtek_uart_adapter_init(&st, MTK_TEST_BOOT_EPOCH);
    size_t noop_len = mtek_uart_process_line(&st, "", out, sizeof(out));
    MTK_CHECK_EQ(noop_len, 0); /* nothing running: no-op, not an error */

    mtek_uart_process_line(&st, "scan -a", out, sizeof(out)); /* g_fake_wifi.ap_count is still 2 from earlier in this test */
    mtek_uart_process_line(&st, "select -a 0", out, sizeof(out));
    mtek_uart_process_line(&st, "deauth broadcast", out, sizeof(out));
    MTK_CHECK_EQ(st.deauth_running, 1);
    mtek_uart_process_line(&st, "", out, sizeof(out)); /* bare Enter */
    MTK_CHECK_EQ(st.deauth_running, 0); /* stopped, same as typing 'stop' */

    /* mtek_uart_adapter_line_is_recognized is the side-effect-free pre-check
     * app_main.c's own REPL loop must gate mtk_transport_claim_try on --
     * noise/partial/unknown input must never recognize as a real command (which
     * would otherwise permanently lock out a genuine SPI peer for this boot
     * session), while every real command from either mode's own table does. */
    {
        mtk_uart_adapter_state_t rst; mtek_uart_adapter_init(&rst, MTK_TEST_BOOT_EPOCH);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, ""), 0); /* empty line: never itself a claim signal */
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "\xFF\x01garbage"), 0);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "frobnicate"), 0);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "sca"), 0); /* partial input: not a real command */
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "help"), 1);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "reboot"), 1);
        rst.mode = MTK_UART_MODE_WIFI;
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "scan -a"), 1);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "deauth broadcast"), 1);
        /* A real BLE-mode command is NOT recognized while in WIFI mode --
         * matches the real dispatcher's own mode-scoped grammar exactly. */
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "advertise"), 0);
        rst.mode = MTK_UART_MODE_BLE;
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "select -a 0"), 0); /* WIFI-only grammar, not recognized in BLE mode */
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "scan"), 1);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "scan -t 10 -n Foo"), 1);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "subscribe 7"), 1);
        MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&rst, "advertise"), 1);
    }

MTK_TEST_MAIN_END
