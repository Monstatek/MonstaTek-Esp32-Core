/* RC5 independent audit P0 "UART asynchronous response lifetime is
 * unsafe": the universal target installs one global mtk_router async
 * runner regardless of which transport originated a request
 * (main/mtek_spi_runtime.c); the router used to defer every
 * ACCEPTED_ASYNC opcode's handler whenever that runner was registered,
 * with no regard for which adapter dispatched it. The factory UART
 * adapter (mtek_uart_adapter.c) builds its response capture/sink on the
 * calling handler function's own stack for every command -- exactly the
 * shape mtek_router.h's own SAFETY CONTRACT documents as unsafe to defer,
 * since that stack frame is gone by the time a background worker would
 * get around to writing through it.
 *
 * This test installs the SAME kind of global async runner + lock the
 * universal firmware registers (a real pthread worker, not a
 * single-threaded simulation, with a deliberate delay standing in for
 * genuine radio-HAL latency), then issues a UART command ("scan -a") that
 * maps to AP_SCAN_START -- a real ACCEPTED_ASYNC opcode -- and proves the
 * fix (mtek_router.c's transport-aware gate: never defer
 * MTK_PROFILE_FACTORY_UART) holds: the command's response is real and
 * available synchronously, in the same call, matching the exact shipped
 * UART command/response behavior, not deferred to the background worker
 * at all. Built and run under ASan/UBSan; a regression here (the gate
 * removed or bypassed) would show up either as a wrong/empty synchronous
 * result (the old, functionally-broken behavior -- dispatch returns
 * before the stack-local capture is ever populated) or, if the delayed
 * worker eventually did touch the by-then-reused stack, as a sanitizer
 * violation. */
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
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

static uint64_t s_now = 1000;
static uint64_t now_ms(void) { return s_now; }

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void router_lock(void) { pthread_mutex_lock(&s_mutex); }
static void router_unlock(void) { pthread_mutex_unlock(&s_mutex); }

typedef struct { void (*fn)(void *); void *arg; } trampoline_arg_t;
static void *pthread_trampoline(void *arg) {
    trampoline_arg_t *ta = (trampoline_arg_t *)arg;
    /* Long enough that if this worker ever legitimately ran a UART-
     * originated handler, the calling function (mtek_uart_process_line's
     * handle_scan_a) would have long since returned and its stack reused
     * for subsequent test code -- exactly the window a real target's
     * FreeRTOS scheduler would also give a deferred worker relative to a
     * synchronous REPL loop moving on to read the next byte. */
    usleep(20000);
    ta->fn(ta->arg);
    free(ta);
    return NULL;
}
static int pthread_runner(void (*fn)(void *arg), void *arg) {
    trampoline_arg_t *ta = malloc(sizeof(*ta));
    ta->fn = fn; ta->arg = arg;
    pthread_t t;
    if (pthread_create(&t, NULL, pthread_trampoline, ta) != 0) { free(ta); return -1; }
    pthread_detach(t);
    return 0;
}

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0xA5A5);
    mtk_arbiter_init();
    mtk_router_init();
    /* The exact global registration main/mtek_spi_runtime.c performs on a
     * universal target build -- one runner/lock pair shared by the whole
     * router, independent of which adapter is calling. */
    mtk_router_set_async_runner(pthread_runner);
    mtk_router_set_lock(router_lock, router_unlock);
    /* RC11 independent correction order P0 verification fallout (same
     * real TSan-caught gap as test_spi_native_dup_cache.c/test_spi_
     * native_async.c's own identical fix): a real async runner is
     * registered above, and this file exercises AP_SCAN_START and
     * DEAUTH_START's own completion paths concurrently, each genuinely
     * touching shared operation-table/wifi-service/fake-HAL state from a
     * different thread than a concurrent STOP/status request -- every one
     * of these must share the SAME real mutex as mtk_router_set_lock
     * above. */
    mtk_core_set_lock(router_lock, router_unlock);
    mtek_wifi_service_set_lock(router_lock, router_unlock);
    mtk_fake_wifi_set_lock(router_lock, router_unlock);

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
    mtek_uart_adapter_init(&st, 0xA5A5);

    g_fake_wifi.ap_count = 1;
    memcpy(g_fake_wifi.ap_results[0].bssid.b, (uint8_t[]){0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 6);
    memcpy(g_fake_wifi.ap_results[0].ssid, "HomeNet", 7); g_fake_wifi.ap_results[0].ssid_len = 7;
    g_fake_wifi.ap_results[0].channel = 6; g_fake_wifi.ap_results[0].rssi = -45; g_fake_wifi.ap_results[0].authmode = 3;

    char out[4096];
    /* AP_SCAN_START (opcode 0x0001/0x0001) is lifecycle ACCEPTED_ASYNC --
     * with the async runner registered above and no transport-aware gate,
     * this dispatch would be deferred and mtek_uart_adapter.c's
     * stack-local uart_capture_t would go out of scope before the
     * pthread worker (sleeping 20ms) ever touches it. With the gate in
     * place, FACTORY_UART always runs synchronously here regardless of
     * the registered runner, so the real AP scan result must already be
     * in `out` by the time this call returns. */
    size_t n = mtek_uart_process_line(&st, "scan -a", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK(strstr(out, "HomeNet") != NULL);
    MTK_CHECK(strstr(out, "AA:BB:CC:DD:EE:FF") != NULL);
    MTK_CHECK(strstr(out, "[+] Scan complete. 1 AP(s) found.\n") != NULL);
    /* The old (broken) behavior under an unconditionally-deferred
     * dispatch would print this instead, since the stack-local cap.status
     * never gets set to ACCEPTED synchronously. */
    MTK_CHECK(strstr(out, "[!] Scan failed.") == NULL);

    /* A second UART-originated ACCEPTED_ASYNC command (DEAUTH_START via
     * `deauth`) after a real select, same proof: synchronous, real HAL
     * side effect visible immediately, no deferral. */
    mtek_uart_process_line(&st, "select -a 0", out, sizeof(out));
    g_fake_wifi.sta_count = 1;
    memcpy(g_fake_wifi.sta_results[0].mac.b, (uint8_t[]){0x02, 0x02, 0x02, 0x02, 0x02, 0x02}, 6);
    g_fake_wifi.sta_results[0].rssi = -50;
    mtek_uart_process_line(&st, "scan -s", out, sizeof(out));
    mtek_uart_process_line(&st, "select -s 0", out, sizeof(out));
    mtek_uart_process_line(&st, "deauth", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[*] Deauth started.\n") == 0);
    MTK_CHECK_EQ(g_fake_wifi.deauth_sent_count, 1); /* real HAL already reached synchronously */
    mtek_uart_process_line(&st, "stop", out, sizeof(out));

    /* Let every detached worker thread this test spawned actually run to
     * completion (they touch only their own async-pool slot copies and
     * the shared, still-live router/queue locks -- never UART's stack --
     * so this is just cleanup, not a correctness dependency) before the
     * test process exits, so ASan's leak/thread checking sees a clean
     * final state rather than racing process teardown. */
    usleep(100000);

MTK_TEST_MAIN_END
