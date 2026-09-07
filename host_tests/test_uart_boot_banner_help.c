/* RC9 independent correction order P0 "strict List A UART parity is
 * still knowingly incomplete": byte-exact golden test for the boot
 * banner + `help` command block, both required to print the identical
 * text this test's own EXPECTED array reconstructs independently (not
 * shared code with mtek_uart_adapter.c's own MTK_HELP_BLOCK_LINES) from
 * the accepted factory image's own raw rodata bytes -- see
 * mtek_uart_adapter.c's own doc comment on MTK_HELP_BLOCK_LINES for the
 * clean-room observable-interface-reimplementation rationale. Never
 * copies the factory .bin into this tree; this test only embeds the
 * text strings themselves, exactly as any other golden-transcript test
 * in this suite already embeds expected UART output. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include <string.h>

static const char *const EXPECTED_LINES[] = {
    "Warning! This program is designed solely for educational and ethical security research purposes.",
    "Please familiarize yourself with local laws and always obtain appropriate permissions before conducting network tests.",
    "Available commands:",
    "  mode -w                   - Switch to WIFI mode",
    "  mode -b                   - Switch to BLE mode",
    "  mode                      - Show current mode",
    "  WIFI mode commands:",
    "    beacon \"s1\" \"s2\" ...      - Broadcast fake beacon frames for given SSIDs(Max.11)",
    "    scan -a                   - Scan for access points",
    "    list -a                   - List scanned access points",
    "    select -a <id>            - Select access point by ID",
    "    scan -s                   - Scan for stations on selected AP",
    "    list -s                   - List scanned stations",
    "    select -s <id>            - Select station by ID",
    "    select -l                 - Show selected AP and station",
    "    deauth [set <id,id,...>|all|broadcast] - Start deauth on selected target(s)",
    "    handshake                 - Start handshake capture on selected AP",
    "    list -h                   - List captured handshake packets",
    "  BLE mode commands:",
    "    scan [-a]                 - Active scan (aggressive, finds more devices)",
    "    scan -p                   - Passive scan (conservative, less power)",
    "    scan -t <sec>             - Set scan duration in seconds",
    "    scan -n <name>            - Filter devices by name substring",
    "    list                      - Show all scanned devices",
    "    list <id>                 - Show detailed ADV fields for scan result ID",
    "    list -d                   - Show detailed ADV fields for all scanned devices",
    "    signal <id>               - Signal Meter: live RSSI for a device (no connect)",
    "    signal stop | status      - Stop / query the Signal Meter",
    "    advertise                 - Start BLE advertising 'ESP32C6-M1-BLE'",
    "    advertise -n <name>       - Set advertising name and start",
    "    connect <id>              - Connect to scanned device; auto-discover GATT",
    "    services                  - List discovered services/chars (props)/descriptors",
    "    read <handle>             - Read a characteristic value",
    "    write <handle> <hex>      - Stage write WITH response (needs 'confirm')",
    "    writenr <handle> <hex>    - Stage write WITHOUT response (needs 'confirm')",
    "    confirm | cancel          - Send or discard the staged write",
    "    subscribe <handle>        - Enable notifications; indicate for indications",
    "    unsubscribe <handle>      - Disable notifications/indications",
    "    status                    - Connection + dropped-notification status",
    "    disconnect                - Disconnect the active BLE connection",
    "    stop                      - Stop BLE advertising/scan/connection",
    "  stop (or press Enter key) - Stop the running command",
    "  reboot                    - Rebooting system",
    "  help                      - Show this help message",
};
#define EXPECTED_LINE_COUNT (sizeof(EXPECTED_LINES) / sizeof(EXPECTED_LINES[0]))

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_uart_adapter_state_t st;
    mtek_uart_adapter_init(&st, MTK_TEST_BOOT_EPOCH);
    char out[4096];

    char expected[4096];
    size_t used = 0;
    for (unsigned i = 0; i < EXPECTED_LINE_COUNT; i++) {
        int n = snprintf(expected + used, sizeof(expected) - used, "%s\n", EXPECTED_LINES[i]);
        MTK_CHECK(n > 0);
        used += (size_t)n;
    }
    MTK_CHECK(used < sizeof(expected));

    /* `help` command: exact byte-for-byte match, including line order,
     * leading-space indentation, and the trailing newline after the
     * final line. */
    size_t help_len = mtek_uart_process_line(&st, "help", out, sizeof(out));
    MTK_CHECK_EQ(help_len, used);
    MTK_CHECK(memcmp(out, expected, used) == 0);

    /* Boot banner: the SAME exact block, via the dedicated boot-time
     * entry point app_main.c calls before the first `>> ` prompt --
     * proven identical to `help`'s own output, not just independently
     * "close". */
    size_t banner_len = mtek_uart_adapter_boot_banner(out, sizeof(out));
    MTK_CHECK_EQ(banner_len, used);
    MTK_CHECK(memcmp(out, expected, used) == 0);

    /* Spot-check a handful of individually significant lines -- exact
     * indentation width (2 vs 4 spaces), embedded quote characters, and
     * the two-line warning's own exact punctuation -- so a future
     * accidental re-wrap/re-indent of any one line fails loudly here,
     * not just via the bulk memcmp above. */
    MTK_CHECK(strstr(out, "Warning! This program is designed solely for educational and ethical security research purposes.\n") != NULL);
    MTK_CHECK(strstr(out, "Please familiarize yourself with local laws and always obtain appropriate permissions before conducting network tests.\n") != NULL);
    MTK_CHECK(strstr(out, "Available commands:\n  mode -w                   - Switch to WIFI mode\n") != NULL);
    MTK_CHECK(strstr(out, "\n  WIFI mode commands:\n    beacon \"s1\" \"s2\" ...      - Broadcast fake beacon frames for given SSIDs(Max.11)\n") != NULL);
    MTK_CHECK(strstr(out, "\n  BLE mode commands:\n    scan [-a]                 - Active scan (aggressive, finds more devices)\n") != NULL);
    MTK_CHECK(strstr(out, "    advertise                 - Start BLE advertising 'ESP32C6-M1-BLE'\n") != NULL);
    MTK_CHECK(strstr(out, "    write <handle> <hex>      - Stage write WITH response (needs 'confirm')\n") != NULL);
    MTK_CHECK(strstr(out, "  stop (or press Enter key) - Stop the running command\n  reboot                    - Rebooting system\n  help                      - Show this help message\n") != NULL);

MTK_TEST_MAIN_END
