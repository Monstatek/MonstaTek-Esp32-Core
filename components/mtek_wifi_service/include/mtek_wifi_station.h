#ifndef MTEK_WIFI_STATION_H
#define MTEK_WIFI_STATION_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/* Infrastructure station inference from the validated 78e6543 callback.
 * Addr3 is an endpoint behind the distribution system, not station evidence.
 * A downlink's RSSI measures the AP transmitter, not the destination client. */
static inline const uint8_t *mtk_wifi_station_from_frame(const uint8_t *p, size_t n,
                                                        const uint8_t ap[6]) {
    if (!p || !ap || n < 24 || (p[0] & 0x0f) != 0x08) return NULL;
    const uint8_t *sta;
    switch (p[1] & 3) {
    case 1: if (memcmp(p+4, ap, 6)) return NULL; sta = p+10; break;
    case 2: if (memcmp(p+10, ap, 6)) return NULL; sta = p+4; break;
    default: return NULL;
    }
    static const uint8_t zero[6] = {0};
    if ((sta[0] & 1) || !memcmp(sta, ap, 6) || !memcmp(sta, zero, 6)) return NULL;
    return sta;
}
#endif
