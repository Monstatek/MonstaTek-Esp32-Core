#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include "mtek_ble_adv_merge.h"
#include <string.h>
static mtk_hal_ble_adv_t items[8];
static unsigned count, starts, stops, samples;
static mtk_hal_mac6_t sampled;
static int live_start(const mtk_hal_ble_adv_t *previous, unsigned n) {
    if (starts) { MTK_CHECK_EQ(n, count); MTK_CHECK(previous != NULL); }
    starts++; return 0;
}
static int connections;
static int unexpected_connection(mtk_hal_mac6_t addr, uint8_t type, uint32_t timeout, uint16_t *handle) {
    (void)addr; (void)type; (void)timeout; (void)handle; connections++; return -1;
}
static void live_stop(void) { stops++; }
static int snapshot(mtk_hal_ble_adv_t *out, unsigned max) {
    unsigned n = count < max ? count : max; memcpy(out, items, n * sizeof(*out)); return n;
}
static int sample(mtk_hal_mac6_t addr, uint8_t type, int8_t *rssi, uint8_t *random) {
    sampled = addr; samples++; *rssi = -52; *random = type != 0; return 0;
}
MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap(); mtk_fake_ble_reset();
    uint8_t nimble[6] = {0x34,0x12,0xDD,0xCC,0xBB,0xAA}, canonical[6], roundtrip[6];
    mtk_ble_addr_order(canonical, nimble);
    MTK_CHECK(!memcmp(canonical, (uint8_t[]){0xAA,0xBB,0xCC,0xDD,0x12,0x34},6));
    mtk_ble_addr_order(roundtrip, canonical);
    MTK_CHECK(!memcmp(roundtrip,nimble,6)); /* selected GATT peer stays identical */
    mtk_ble_hal_t hal = g_fake_ble_hal;
    hal.gatt_connect = unexpected_connection;
    hal.live_start = live_start; hal.live_stop = live_stop; hal.live_snapshot = snapshot; hal.signal_sample = sample;
    mtek_ble_set_hal(&hal);
    uint8_t adv[] = {2,1,6,4,8,'T','a','g',3,255,0x4c,0};
    uint8_t rsp[] = {7,9,'S','e','n','s','o','r'};
    mtk_ble_adv_merge(&items[0], adv, sizeof(adv), 0, 0, -61, 1);
    MTK_CHECK_EQ(items[0].name_len, 3);
    mtk_ble_adv_merge(&items[0], rsp, sizeof(rsp), 1, 4, -52, 2);
    MTK_CHECK_EQ(items[0].name_len, 6);
    MTK_CHECK(!memcmp(items[0].name, "Sensor", 6));
    MTK_CHECK_EQ(items[0].mfg_len, 2);
    MTK_CHECK_EQ(items[0].raw_adv_len, sizeof(adv));
    MTK_CHECK_EQ(items[0].raw_scan_rsp_len, sizeof(rsp));
    mtk_ble_adv_merge(&items[0], adv, sizeof(adv), 0, 0, -50, 3);
    MTK_CHECK_EQ(items[0].name_len, 6); /* short ADV cannot erase complete response name */
    MTK_CHECK_EQ(items[0].rssi, -50);
    uint8_t truncated[] = {9,9,'X'};
    mtk_ble_adv_merge(&items[1], truncated, sizeof(truncated), 0, 0, -70, 4);
    MTK_CHECK_EQ(items[1].name_len, 0);
    mtk_hal_ble_adv_t tracked[1] = {0}; unsigned tracked_count = 0;
    uint8_t target[6] = {99, 0, 0, 0, 0, 0};
    for (unsigned i = 0; i < 20; ++i) {
        uint8_t other[6] = {i, 0, 0, 0, 0, 0};
        mtk_ble_adv_observe(tracked, 1, &tracked_count, other, 1, target, 1, adv, sizeof(adv), 0, 0, -30, 10);
    }
    MTK_CHECK_EQ(tracked_count, 0);
    mtk_ble_adv_observe(tracked, 1, &tracked_count, target, 0, target, 1, adv, sizeof(adv), 0, 0, -30, 10);
    MTK_CHECK_EQ(tracked_count, 0); /* address type is part of target identity */
    mtk_ble_adv_observe(tracked, 1, &tracked_count, target, 1, target, 1, adv, sizeof(adv), 0, 0, -71, 11);
    MTK_CHECK_EQ(tracked_count, 1); MTK_CHECK_EQ(tracked[0].rssi, -71);
    mtk_ble_adv_observe(tracked, 1, &tracked_count, target, 1, target, 1, rsp, sizeof(rsp), 1, 4, -51, 12);
    MTK_CHECK_EQ(tracked_count, 1); MTK_CHECK_EQ(tracked[0].rssi, -51);
    MTK_CHECK(!memcmp(tracked[0].name, "Sensor", 6));
    for (unsigned i = 0; i < 8; i++) { items[i].addr.b[0] = i + 1; items[i].rssi = -50 - i; }
    count = 8;
    mtk_uart_adapter_state_t st; mtek_uart_adapter_init(&st, mtk_core_boot_epoch());
    char out[4096];
    mtek_uart_process_line(&st, "mode -b", out, sizeof(out));
    mtek_uart_process_line(&st, "scan live", out, sizeof(out));
    MTK_CHECK(strstr(out, "[BLE:SCAN:LIVE]") != NULL);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);
    mtek_uart_process_line(&st, "list", out, sizeof(out));
    MTK_CHECK(strstr(out, "NAME=Sensor") != NULL);
    MTK_CHECK(strstr(out, "MFG=76") != NULL);
    /* Reorder the live HAL table before selection: last published address wins. */
    mtk_hal_ble_adv_t tmp = items[0]; items[0] = items[7]; items[7] = tmp;
    mtk_mac6_t saved_address = st.ble_addr[0];
    st.ble_addr[0] = st.ble_addr[7];
    mtek_uart_process_line(&st, "signal 08:00:00:00:00:00", out, sizeof(out));
    MTK_CHECK(strstr(out, "Ambiguous") != NULL);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);
    st.ble_addr[0] = saved_address;
    mtek_uart_process_line(&st, "signal 08:00:00:00:00:00", out, sizeof(out));
    MTK_CHECK(strstr(out, "[BLE:SIG:START]") != NULL);
    MTK_CHECK_EQ(sampled.b[0], 8); /* target beyond the old first-four limit */
    MTK_CHECK_EQ(stops, 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_SM);
    mtek_uart_process_line(&st, "signal stop", out, sizeof(out));
    MTK_CHECK(strstr(out, "[BLE:SIG:STOP]") != NULL);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    unsigned saved_samples = samples;
    s_mtk_test_now_ms += 501; mtek_ble_signal_meter_tick();
    MTK_CHECK_EQ(samples, saved_samples);
    mtek_uart_process_line(&st, "resume", out, sizeof(out));
    MTK_CHECK_EQ(starts, 2);
    mtek_uart_process_line(&st, "list", out, sizeof(out));
    mtek_uart_process_line(&st, "signal 01:00:00:00:00:00", out, sizeof(out));
    MTK_CHECK_EQ(sampled.b[0], 1);
    mtek_uart_process_line(&st, "signal stop", out, sizeof(out));
    mtek_uart_process_line(&st, "resume", out, sizeof(out));
    mtek_uart_process_line(&st, "stop", out, sizeof(out));
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    mtek_uart_process_line(&st, "resume", out, sizeof(out));
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS);
    mtek_ble_cancel_active_for_peer_reset();
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(connections, 0);
    mtek_uart_process_line(&st, "resume", out, sizeof(out));
    mtek_uart_process_line(&st, "list", out, sizeof(out));
    mtek_uart_process_line(&st, "connect 0", out, sizeof(out));
    MTK_CHECK_EQ(connections, 1); /* intentional GATT request fails */
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_BS); /* picker still discovers */
    mtek_uart_process_line(&st, "stop", out, sizeof(out));
    mtek_ble_set_hal(&g_fake_ble_hal);
MTK_TEST_MAIN_END
