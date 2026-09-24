/* Shared production AD parser, also exercised by host tests. */
#pragma once
#include "mtek_ble_hal.h"
#include <string.h>
/* NimBLE stores LE addresses least-significant byte first; canonical MAC
 * records and the UART formatter use display/network order. Conversion is
 * symmetric and is also used when passing a selected peer back to NimBLE. */
static inline void mtk_ble_addr_order(uint8_t dst[6], const uint8_t src[6]) {
    for (unsigned i = 0; i < 6; ++i) dst[i] = src[5 - i];
}
static inline void mtk_ble_adv_merge(mtk_hal_ble_adv_t *o, const uint8_t *data,
    unsigned len, int response, uint8_t event_type, int8_t rssi, uint32_t now) {
    if (rssi != 127) o->rssi = rssi;
    o->last_seen_ms = now;
    unsigned rawlen = len > 31 ? 31 : len;
    if (response) { memcpy(o->raw_scan_rsp, data, rawlen); o->raw_scan_rsp_len = rawlen; }
    else { memcpy(o->raw_adv, data, rawlen); o->raw_adv_len = rawlen; o->adv_type = event_type; }
    for (unsigned i = 0; i < len;) {
        unsigned n = data[i++];
        if (!n || n > len - i) break;
        uint8_t type = data[i]; const uint8_t *v = data + i + 1; unsigned size = n - 1;
        if ((type == 8 || type == 9) && size && (type == 9 || !o->name_complete)) {
            if (size > sizeof(o->name)) size = sizeof(o->name);
            /* UART names are single-line text. Ignore controls, never fabricate a name. */
            unsigned w = 0;
            for (unsigned j = 0; j < size; ++j) if (v[j] >= 32 && v[j] != 127) o->name[w++] = v[j];
            if (w) { o->name_len = w; o->name_complete = type == 9; }
        } else if (type == 1 && size) o->flags = v[0];
        else if (type == 10 && size) o->tx_power = v[0];
        else if (type == 255 && size) {
            if (size > sizeof(o->mfg_data)) size = sizeof(o->mfg_data);
            memcpy(o->mfg_data, v, size); o->mfg_len = size;
        }
        i += n;
    }
}

/* Address/type filtering happens BEFORE reserving a slot. A crowded room cannot
 * fill a signal listener with unrelated advertisers. */
static inline void mtk_ble_adv_observe(mtk_hal_ble_adv_t *out, unsigned max, unsigned *count,
    const uint8_t addr[6], uint8_t addr_type, const uint8_t *target, uint8_t target_type,
    const uint8_t *data, unsigned len, int response, uint8_t event_type, int8_t rssi, uint32_t now) {
    if (target && (memcmp(target, addr, 6) || target_type != addr_type)) return;
    unsigned i;
    for (i = 0; i < *count; ++i)
        if (!memcmp(out[i].addr.b, addr, 6) && out[i].addr_type == addr_type) break;
    if (i == *count) {
        if (i == max) return;
        memset(&out[i], 0, sizeof(out[i]));
        memcpy(out[i].addr.b, addr, 6); out[i].addr_type = addr_type;
        out[i].tx_power = 127; out[i].rssi = 127;
        (*count)++;
    }
    mtk_ble_adv_merge(&out[i], data, len, response, event_type, rssi, now);
}
