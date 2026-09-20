/* Clean-room implementation from MonstaTek contract. Factory M1 UART
 * compatibility adapter: parses the legacy ASCII REPL grammar into canonical
 * requests and formats canonical responses/events back into the legacy printed
 * lines -- never calling a legacy handler and reparsing its output. */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "mtek_schema_structs.h"
#include "mtek_async_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { MTK_UART_MODE_WIFI = 0, MTK_UART_MODE_BLE = 1 } mtk_uart_mode_t;

#define MTK_UART_AP_MAX 50
#define MTK_UART_STA_MAX 32
#define MTK_UART_BLE_MAX 40

typedef struct mtk_uart_adapter_state {
    mtk_uart_mode_t mode;
    uint32_t boot_epoch;
    uint32_t next_correlation;

    /* Wi-Fi: cached scan tables and target selection. */
    uint8_t ap_scan_valid;
    uint16_t ap_count;
    mtk_aprecord_t ap_table[MTK_UART_AP_MAX];
    int ap_selected; /* -1 = none */

    uint8_t sta_scan_valid;
    uint32_t sta_scan_generation;
    uint16_t sta_count;
    mtk_stationrecord_t sta_table[MTK_UART_STA_MAX];
    int sta_selected;

    uint8_t deauth_running; uint32_t deauth_token;
    uint8_t handshake_running; uint32_t handshake_token;
    uint8_t beacon_running; uint32_t beacon_token;

    /* BLE: cached scan table, advertising/signal-meter/GATT session state. */
    uint8_t ble_scan_valid;
    uint16_t ble_count;
    /* Needed to dispatch BLE_DEVICE_DETAILS for `list <id>`/`list -d` (the same
     * result_generation the scan itself produced -- never a guessed/stale one). */
    uint32_t ble_generation;
    mtk_mac6_t ble_addr[MTK_UART_BLE_MAX];
    uint8_t ble_addr_type[MTK_UART_BLE_MAX];
    int8_t ble_rssi[MTK_UART_BLE_MAX];
    uint8_t ble_name[MTK_UART_BLE_MAX][32];
    uint8_t ble_name_len[MTK_UART_BLE_MAX];
    int ble_selected;

    uint8_t ble_adv_running; uint32_t ble_adv_token;
    uint8_t ble_signal_running; uint32_t ble_signal_token;

    uint8_t gatt_connected;
    uint32_t gatt_conn_token;
    uint8_t write_staged;
    uint16_t write_handle;
    uint8_t write_data[64];
    uint8_t write_len;
    uint8_t write_with_response;

    /* Persistent, adapter-owned (this struct lives for the whole REPL task's
     * boot-session lifetime -- see app_main.c's uart_repl_task, which allocates
     * one on its own stack once and loops on it forever) delivery queue for the
     * three UART commands whose canonical operation has a HAL callback that can
     * fire after the synchronous command call has already returned: `handshake`
     * (HANDSHAKE_EVENT), `signal` (SIGNAL_METER_UPDATE/LOST), and `connect`
     * (GATT_VALUE_EVENT notifications/indications). Every other UART command
     * still uses a stack-local capture, which remains safe because the router
     * never defers a FACTORY_UART-profile dispatch (mtek_router.c) and those
     * commands' own canonical services never touch their sink again once the
     * synchronous call returns. */
    mtk_async_queue_t session_queue;

    /* `reboot` used to only print a message with no actual restart. This
     * portable component has no ESP-IDF dependency (host-testable) so it cannot
     * call esp_restart itself -- it sets this flag instead; the target-specific
     * REPL task (app_main.c) checks it after each mtek_uart_process_line call
     * and performs the real delayed restart, matching the established
     * portable-core/thin-target-glue split used throughout this tree. */
    uint8_t reboot_requested;
} mtk_uart_adapter_state_t;

/* #2 -- see mtek_spi_native_dispatch.h's own matching _Static_assert for the
 * full rationale. This struct (~12.3KB, independently measured by the audit) is
 * static in app_main.c's uart_repl_task, not stack-local -- this bound only
 * guards against a future silent size regression, not a hardware-measured budget
 * claim. */
#include <assert.h>
_Static_assert(sizeof(mtk_uart_adapter_state_t) < 20000,
                "mtk_uart_adapter_state_t grew past its documented RESOURCE_BUDGET.md ceiling -- "
                "re-measure before proceeding");

void mtek_uart_adapter_init(mtk_uart_adapter_state_t *st, uint32_t boot_epoch);

/* Formats the exact boot warning + full command reference (identical text to the
 * `help` command) into `out`. The target REPL loop (app_main.c) must call this
 * exactly once, before writing the first `>> ` prompt -- side-effect-free, does
 * not touch `st`. Returns the number of bytes written (excluding the NUL). */
size_t mtek_uart_adapter_boot_banner(char *out, size_t out_cap);

/* Processes exactly one input line (no trailing CR/LF) and writes the
 * formatted legacy response text into `out` (NUL-terminated, truncated to
 * out_cap-1 if longer). Returns the number of bytes written (excluding
 * the NUL). */
size_t mtek_uart_process_line(mtk_uart_adapter_state_t *st, const char *line, char *out, size_t out_cap);

/* Side-effect-free -- returns 1 if `line` matches a real, currently-dispatchable
 * command grammar (mode-aware, mirroring mtek_uart_process_line's own dispatch
 * table exactly), 0 for noise, partial input, an unknown command, or an empty
 * line. The REPL loop (app_main.c) checks this BEFORE ever attempting
 * mtk_transport_claim_try, so arbitrary/garbage bytes can never permanently lock
 * out a genuine SPI peer for the rest of the boot session. */
int mtek_uart_adapter_line_is_recognized(const mtk_uart_adapter_state_t *st, const char *line);

/* Drains any frame the persistent session_queue has accumulated since the
 * last call that this REPL loop hasn't already consumed synchronously --
 * i.e. a real background delivery (HANDSHAKE_EVENT mid-capture,
 * SIGNAL_METER_UPDATE/LOST between samples, GATT_VALUE_EVENT
 * notifications/indications) that arrived while the REPL was idle at the
 * prompt, not as part of any command's own synchronous response. Formats
 * at most one such event into `out` per call (NUL-terminated). Returns
 * the number of bytes written (0 if nothing was pending). Intended to be
 * called from the REPL's own idle/read-timeout path (never blocks). */
size_t mtek_uart_adapter_poll_background(mtk_uart_adapter_state_t *st, char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif
