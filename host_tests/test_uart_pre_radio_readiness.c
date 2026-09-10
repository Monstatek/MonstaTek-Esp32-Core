/* RC12 hardware-compatibility round 2 (read-only gate): "the actual
 * UART command-processing loop is still created only after the same
 * unconditional Wi-Fi/NimBLE initialization span ... RC12 can falsely
 * announce readiness and then fail the mode command exactly as RC11 did."
 * app_main.c now creates uart_repl_task (and wires mtk_core_init/
 * mtk_arbiter_init/mtk_router_init/mtk_transport_claim_set_lock/
 * mtk_transport_claim_reset) BEFORE esp_netif_init/esp_wifi_init/
 * mtek_ble_hal_esp32_init, deliberately NOT calling mtek_system_service_
 * init/register, mtek_wifi_service_init/register, mtek_ble_service_init/
 * register, mtek_capture_service_init/register, mtek_wifi_set_hal, or
 * mtek_ble_set_hal that early.
 *
 * This is a HOST-LEVEL proof of the dependency-graph claim behind that
 * reordering -- it does NOT exercise a real UART peripheral (no target
 * access from this host), only the same production mtek_uart_adapter.c/
 * mtek_router.c/mtek_transport_select.c logic app_main.c actually calls,
 * driven through the identical minimal bring-up sequence (deliberately
 * OMITTING every service register()/set_hal() call) to prove two things
 * mechanically rather than by inspection alone:
 *
 *   1. Bare CRLF (line_is_recognized("") == 0, matching app_main's own
 *      "never even attempt process_line" bare-Enter path) and every
 *      global command the STM32's own frozen readiness probe actually
 *      sends ("mode -w"/"mode -b"/"mode") answer with their exact,
 *      correct wire text using ONLY mtk_core_init/mtk_arbiter_init/
 *      mtk_router_init/mtk_transport_claim_set_lock/mtk_transport_claim_
 *      reset -- zero wifi/ble/system/capture service or HAL dependency.
 *
 *   2. A command that DOES need the router (a WIFI-mode opcode, "scan -a")
 *      issued in this same pre-registration state degrades to its own
 *      pre-existing, already-shipped honest failure text ("[!] Scan
 *      failed.\n" -- mtek_uart_adapter.c's own existing cap.status !=
 *      MTK_STATUS_ACCEPTED branch, reached via mtk_router.c's find_service
 *      == NULL -> MTK_STATUS_UNSUPPORTED path) -- never a crash, a hang,
 *      or malformed output, exactly the same honest-refusal contract this
 *      tree already relies on for wifi_hal_ready==0/ble_hal_ready==0. */
#include "mtk_test.h"
#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_router.h"
#include "mtek_transport_select.h"
#include "mtek_uart_adapter.h"
#include <string.h>
#include <pthread.h>

#define TEST_BOOT_EPOCH 0x11223344u

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void thread_lock(void) { pthread_mutex_lock(&s_mutex); }
static void thread_unlock(void) { pthread_mutex_unlock(&s_mutex); }

/* Mirrors app_main.c's own new early-boot sequence EXACTLY -- and only
 * that sequence. No mtek_*_service_init/register, no mtek_wifi_set_hal/
 * mtek_ble_set_hal: this is the state a real target is in the instant
 * uart_repl_task starts reading bytes, before esp_wifi_init/esp_wifi_start/
 * mtek_ble_hal_esp32_init have even been called. */
static void pre_radio_bringup(void) {
    mtk_core_init(TEST_BOOT_EPOCH);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_transport_claim_set_lock(thread_lock, thread_unlock);
    mtk_transport_claim_reset();
}

MTK_TEST_MAIN_BEGIN

    pre_radio_bringup();
    mtk_uart_adapter_state_t st;
    mtek_uart_adapter_init(&st, TEST_BOOT_EPOCH);
    char out[4096];

    /* Bare CRLF: app_main.c's own uart_repl_task never even calls
     * mtek_uart_process_line for an unrecognized/empty line while unclaimed
     * -- it is the recognizer itself, checked first, that must be
     * dependency-free. */
    MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&st, ""), 0);

    /* The STM32's own frozen readiness probe: "mode -w" / "mode -b", plus
     * "mode" -- recognized, claimable, and fully answerable with zero
     * radio/service/HAL dependency. */
    MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&st, "mode -w"), 1);
    MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&st, "mode -b"), 1);
    MTK_CHECK_EQ(mtek_uart_adapter_line_is_recognized(&st, "mode"), 1);

    /* The claim itself (mtk_transport_claim_try) must succeed here too --
     * app_main.c's uart_repl_task calls this BEFORE ever reaching
     * mtek_uart_process_line for a freshly recognized line. */
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_FACTORY_UART), 1);

    mtek_uart_process_line(&st, "mode -b", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: BLE\n") == 0);
    mtek_uart_process_line(&st, "mode -w", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: WIFI\n") == 0);
    mtek_uart_process_line(&st, "mode", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Current mode: WIFI\n") == 0);

    /* Contrast case: a WIFI-mode opcode that DOES need the router, issued
     * in this same pre-registration state (no mtek_wifi_service_register,
     * no mtek_wifi_set_hal). Must degrade to its own existing, honest
     * failure text -- never crash, hang, or return empty/garbage output --
     * proving the race window this fix opens is already safely handled by
     * this tree's own pre-existing router/adapter contract, not a new one. */
    size_t n = mtek_uart_process_line(&st, "scan -a", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK(strcmp(out, "[*] Starting AP scan...\n[!] Scan failed.\n") == 0);

MTK_TEST_MAIN_END
