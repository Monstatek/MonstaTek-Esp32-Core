/* Dedicated golden-transcript coverage for the two remaining disclosed BLE gaps
 * closed -- `list <id>`/`list -d`'s full per-AD-type detail block (Service
 * UUID16/128, Service Data UUID16, unknown-AD-type fallback, Shortened/Complete
 * Local Name), and `connect`/`services`'s nested
 * service/characteristic/descriptor discovery (GATT_DISCOVER_CHARS/
 * GATT_DISCOVER_DESCS, new purely-additive opcodes). Evidence: lines 68, 69, 74,
 * 75. */
#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include <string.h>

MTK_TEST_MAIN_BEGIN

    mtk_test_bootstrap();
    mtk_uart_adapter_state_t st;
    mtek_uart_adapter_init(&st, MTK_TEST_BOOT_EPOCH);
    st.mode = MTK_UART_MODE_BLE;
    char out[4096];

    /* `list <id>` / `list -d`: one scanned device carrying every AD type this
     * feature knows how to surface, spread across raw_adv (the primary
     * advertisement) and raw_scan_rsp (a scan response, proving the two buffers
     * are walked and merged as documented). */
    mtk_fake_ble_reset();
    mtk_hal_ble_adv_t *d = &g_fake_ble.scan_results[0];
    memcpy(d->addr.b, (uint8_t[]){0x10,0x20,0x30,0x40,0x50,0x60}, 6);
    d->rssi = -55;
    d->flags = 0x06;
    memcpy(d->name, "Test", 4); d->name_len = 4;
    d->tx_power = 4;
    memcpy(d->mfg_data, (uint8_t[]){0x4C, 0x00}, 2); d->mfg_len = 2;
    {
        /* raw_adv: Flags(0x01)=0x06, Complete Local Name(0x09)="Test",
         * Service UUID16 Complete(0x03)={0x180F}, TxPower(0x0A)=4,
         * Manufacturer Data(0xFF)={0x4C,0x00}, Service Data UUID16(0x16)
         * ={0x180F, 0xAA,0xBB}, and one deliberately-unknown AD type
         * (0x20)={0xDE,0xAD} to prove the fallback line. */
        uint8_t raw[] = {
            0x02,0x01,0x06,
            0x05,0x09,'T','e','s','t',
            0x03,0x03,0x0F,0x18,
            0x02,0x0A,0x04,
            0x03,0xFF,0x4C,0x00,
            0x05,0x16,0x0F,0x18,0xAA,0xBB,
            0x03,0x20,0xDE,0xAD,
        };
        memcpy(d->raw_adv, raw, sizeof(raw)); d->raw_adv_len = sizeof(raw);
    }
    {
        /* raw_scan_rsp: one 128-bit Complete Service UUID (0x07), bytes
         * 0x00..0x0F (sequential, so the byte-reversed display string is
         * easy to hand-verify: 0f0e0d0c-0b0a-0908-0706-050403020100). */
        uint8_t raw[] = {
            0x11,0x07, 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
        };
        memcpy(d->raw_scan_rsp, raw, sizeof(raw)); d->raw_scan_rsp_len = sizeof(raw);
    }
    g_fake_ble.scan_count = 1;

    mtek_uart_process_line(&st, "scan", out, sizeof(out));
    size_t n = mtek_uart_process_line(&st, "list 0", out, sizeof(out));
    MTK_CHECK(n > 0);
    MTK_CHECK(strstr(out, "[00] 10:20:30:40:50:60\n") != NULL);
    MTK_CHECK(strstr(out, "  RSSI=-55\n") != NULL);
    MTK_CHECK(strstr(out, "  Flags=0x06\n") != NULL);
    MTK_CHECK(strstr(out, "  Complete Local Name : Test\n") != NULL);
    MTK_CHECK(strstr(out, "  Service UUID16 : 0x180F\n") != NULL);
    MTK_CHECK(strstr(out, "  Service UUID128 : 0f0e0d0c-0b0a-0908-0706-050403020100\n") != NULL);
    MTK_CHECK(strstr(out, "  TxPower=4\n") != NULL);
    MTK_CHECK(strstr(out, "  Service Data (UUID16 0x180F) : AABB\n") != NULL);
    MTK_CHECK(strstr(out, "  Manufacturer Data : 4C00\n") != NULL);
    MTK_CHECK(strstr(out, "  Unknown AD type 0x20 : DEAD\n") != NULL);
    MTK_CHECK(strstr(out, "  Raw ADV data : ") != NULL);
    MTK_CHECK(strstr(out, "  Raw SCAN_RSP data : ") != NULL);

    /* Field ORDER matters (the accepted matrix specifies an exact
     * sequence) -- assert every field appears strictly after the one
     * before it in the same transcript. */
    const char *p_rssi = strstr(out, "RSSI=-55");
    const char *p_flags = strstr(out, "Flags=0x06");
    const char *p_name = strstr(out, "Complete Local Name");
    const char *p_uuid16 = strstr(out, "Service UUID16");
    const char *p_uuid128 = strstr(out, "Service UUID128");
    const char *p_tx = strstr(out, "TxPower=4");
    const char *p_svcdata = strstr(out, "Service Data (UUID16");
    const char *p_mfg = strstr(out, "Manufacturer Data");
    const char *p_unk = strstr(out, "Unknown AD type");
    const char *p_rawadv = strstr(out, "Raw ADV data");
    const char *p_rawrsp = strstr(out, "Raw SCAN_RSP data");
    MTK_CHECK(p_rssi && p_flags && p_name && p_uuid16 && p_uuid128 && p_tx && p_svcdata && p_mfg && p_unk && p_rawadv && p_rawrsp);
    MTK_CHECK(p_rssi < p_flags); MTK_CHECK(p_flags < p_name); MTK_CHECK(p_name < p_uuid16);
    MTK_CHECK(p_uuid16 < p_uuid128); MTK_CHECK(p_uuid128 < p_tx); MTK_CHECK(p_tx < p_svcdata);
    MTK_CHECK(p_svcdata < p_mfg); MTK_CHECK(p_mfg < p_unk); MTK_CHECK(p_unk < p_rawadv); MTK_CHECK(p_rawadv < p_rawrsp);

    /* `list -d`: same detail block, driven for every scanned device. */
    size_t nd = mtek_uart_process_line(&st, "list -d", out, sizeof(out));
    MTK_CHECK(nd > 0);
    MTK_CHECK(strstr(out, "[00] 10:20:30:40:50:60\n") != NULL);
    MTK_CHECK(strstr(out, "  Complete Local Name : Test\n") != NULL);

    /* Invalid id: distinct error strings from the accepted matrix. */
    mtek_uart_process_line(&st, "list 99", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Invalid BLE ID: 99\n") == 0);
    mtek_uart_process_line(&st, "list abc", out, sizeof(out));
    MTK_CHECK(strcmp(out, "[!] Invalid BLE ID format: abc\n") == 0);

    /* `connect <id>` / `services`: nested service/characteristic/ descriptor
     * discovery. Two services; service 0 has two characteristics (one
     * Notify-only with a CCCD descriptor, one Read+Write with no descriptor);
     * service 1 has one Read-only characteristic with no descriptor. */
    mtk_fake_ble_reset();
    memcpy(g_fake_ble.scan_results[0].addr.b, (uint8_t[]){1,2,3,4,5,6}, 6);
    g_fake_ble.scan_count = 1;
    g_fake_ble.gatt_connect_rc = 0;
    g_fake_ble.gatt_vendor_handle = 7;

    g_fake_ble.gatt_service_count = 2;
    g_fake_ble.gatt_services[0].uuid_width = 0; g_fake_ble.gatt_services[0].uuid_value[0] = 0x00; g_fake_ble.gatt_services[0].uuid_value[1] = 0x18;
    g_fake_ble.gatt_services[0].start_handle = 1; g_fake_ble.gatt_services[0].end_handle = 10;
    g_fake_ble.gatt_services[1].uuid_width = 0; g_fake_ble.gatt_services[1].uuid_value[0] = 0x0F; g_fake_ble.gatt_services[1].uuid_value[1] = 0x18;
    g_fake_ble.gatt_services[1].start_handle = 11; g_fake_ble.gatt_services[1].end_handle = 20;

    g_fake_ble.gatt_char_count = 3;
    /* svc0 char0: Notify (0x10), def=2 val=3, has a CCCD at handle 4. */
    g_fake_ble.gatt_chars[0].uuid_width = 0; g_fake_ble.gatt_chars[0].uuid_value[0] = 0xA0; g_fake_ble.gatt_chars[0].uuid_value[1] = 0xFF;
    g_fake_ble.gatt_chars[0].def_handle = 2; g_fake_ble.gatt_chars[0].val_handle = 3; g_fake_ble.gatt_chars[0].properties = 0x10;
    /* svc0 char1: Read+Write (0x0A), def=5 val=6, no descriptor. */
    g_fake_ble.gatt_chars[1].uuid_width = 0; g_fake_ble.gatt_chars[1].uuid_value[0] = 0xA1; g_fake_ble.gatt_chars[1].uuid_value[1] = 0xFF;
    g_fake_ble.gatt_chars[1].def_handle = 5; g_fake_ble.gatt_chars[1].val_handle = 6; g_fake_ble.gatt_chars[1].properties = 0x0A;
    /* svc1 char0: Read-only (0x02), def=12 val=13, no descriptor. */
    g_fake_ble.gatt_chars[2].uuid_width = 0; g_fake_ble.gatt_chars[2].uuid_value[0] = 0xB0; g_fake_ble.gatt_chars[2].uuid_value[1] = 0xFF;
    g_fake_ble.gatt_chars[2].def_handle = 12; g_fake_ble.gatt_chars[2].val_handle = 13; g_fake_ble.gatt_chars[2].properties = 0x02;

    g_fake_ble.gatt_desc_count = 1;
    g_fake_ble.gatt_descs[0].uuid_width = 0; g_fake_ble.gatt_descs[0].uuid_value[0] = 0x02; g_fake_ble.gatt_descs[0].uuid_value[1] = 0x29; /* 0x2902 CCCD */
    g_fake_ble.gatt_descs[0].handle = 4;

    mtek_uart_process_line(&st, "scan", out, sizeof(out));
    size_t cn = mtek_uart_process_line(&st, "connect 0", out, sizeof(out));
    MTK_CHECK(cn > 0);
    MTK_CHECK(st.gatt_connected == 1);
    MTK_CHECK(strstr(out, "[BLE:CONN] connected handle=") != NULL);
    MTK_CHECK(strstr(out, "[BLE:DISC] complete: 2 service(s), 3 characteristic(s), 1 descriptor(s)\n") != NULL);

    size_t sn = mtek_uart_process_line(&st, "services", out, sizeof(out));
    MTK_CHECK(sn > 0);
    MTK_CHECK(strstr(out, "[SVC 0] UUID16 0x1800 handles 1-10\n") != NULL);
    MTK_CHECK(strstr(out, "  [CHR] UUID16 0xFFA0 val=3 props=0x10 [N]\n") != NULL);
    MTK_CHECK(strstr(out, "    [DSC] UUID16 0x2902 handle=4\n") != NULL);
    MTK_CHECK(strstr(out, "  [CHR] UUID16 0xFFA1 val=6 props=0x0A [R W]\n") != NULL);
    MTK_CHECK(strstr(out, "[SVC 1] UUID16 0x180F handles 11-20\n") != NULL);
    MTK_CHECK(strstr(out, "  [CHR] UUID16 0xFFB0 val=13 props=0x02 [R]\n") != NULL);

    /* Nesting order: [DSC] for char0 must appear strictly between char0's
     * own [CHR] row and char1's [CHR] row (proves real per-characteristic
     * scoping, not all descriptors dumped after all characteristics). */
    const char *p_chr0 = strstr(out, "0xFFA0");
    const char *p_dsc = strstr(out, "[DSC]");
    const char *p_chr1 = strstr(out, "0xFFA1");
    MTK_CHECK(p_chr0 && p_dsc && p_chr1);
    MTK_CHECK(p_chr0 < p_dsc); MTK_CHECK(p_dsc < p_chr1);

MTK_TEST_MAIN_END
