/* Clean-room implementation from MonstaTek contract. Blocking/pollable HAL
 * the portable BLE+GATT service logic (mtek_ble_logic.c) calls into. See
 * mtek_wifi_hal.h for the same design rationale. */
#pragma once
#include <stdint.h>
#include "mtek_hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* See gatt_subscribe's own doc comment below. */
#define MTK_HAL_GATT_SUBSCRIBE_NO_CCCD (-2)

typedef struct {
    mtk_hal_mac6_t addr;
    uint8_t addr_type;
    int8_t rssi;
    uint8_t adv_type;
    uint8_t name[32];
    uint8_t name_len : 6; /* 0..32; pack internal name quality into the spare bits */
    uint8_t name_complete : 1;
    uint8_t flags;
    uint8_t tx_power; /* 127 = unset, matching the canonical i8|127 encoding */
    uint8_t mfg_data[31];
    uint8_t mfg_len;
    uint8_t raw_adv[31];
    uint8_t raw_adv_len;
    /* (rework): a real scan response PDU, merged in by the HAL's own scan
     * callback when one arrives for an already-recorded device -- NOT populated
     * by `raw_adv` itself. Fixes a genuine, previously-undiscovered dead-field
     * bug: the canonical schema's `raw_scan_rsp` and this UART layer's own
     * "additive Raw SCAN_RSP data" line had existed since RC6/RC7 but nothing
     * ever wrote to the field this struct is decoded from -- it was always empty
     * regardless of whether a real scan response was seen. */
    uint8_t raw_scan_rsp[31];
    uint8_t raw_scan_rsp_len;
    uint32_t last_seen_ms;
} mtk_hal_ble_adv_t;

typedef struct {
    uint8_t uuid_width;   /* 0=BIT16, 1=BIT128 */
    uint8_t uuid_value[16];
    uint16_t start_handle;
    uint16_t end_handle;
} mtk_hal_gatt_service_t;

/* New, purely additive result types -- gatt_discover (above) only ever
 * enumerated services. */
typedef struct {
    uint8_t uuid_width;   /* 0=BIT16, 1=BIT128 */
    uint8_t uuid_value[16];
    uint16_t def_handle;
    uint16_t val_handle;
    uint8_t properties; /* raw ATT properties octet: bit0=Broadcast,
                          * 1=Read, 2=WriteNoResp, 3=Write, 4=Notify,
                          * 5=Indicate, 6=AuthSignedWrite, 7=ExtendedProps */
} mtk_hal_gatt_char_t;

typedef struct {
    uint8_t uuid_width;   /* 0=BIT16, 1=BIT128 */
    uint8_t uuid_value[16];
    uint16_t handle;
} mtk_hal_gatt_desc_t;

typedef struct mtk_ble_hal {
    int (*scan)(uint8_t mode, uint16_t duration_ms, const char *name_filter,
                mtk_hal_ble_adv_t *out, unsigned max_out);
    int (*adv_start)(const uint8_t *name, uint8_t name_len);
    void (*adv_stop)(void);
    /* One RSSI sample toward `addr`. Returns 0 on success, nonzero if the
     * target is no longer observable (signal lost). */
    int (*signal_sample)(mtk_hal_mac6_t addr, uint8_t addr_type, int8_t *rssi_out, uint8_t *is_random_out);

    int (*gatt_connect)(mtk_hal_mac6_t addr, uint8_t addr_type, uint32_t timeout_ms, uint16_t *vendor_handle_out);
    void (*gatt_disconnect)(uint16_t vendor_handle);
    int (*gatt_discover)(uint16_t vendor_handle, mtk_hal_gatt_service_t *out, unsigned max_out);
    /* Enumerate every characteristic whose declaration handle falls within
     * [start_handle, end_handle] (a real prior gatt_discover service's own
     * start_handle/end_handle -- never a guessed range). Returns the count
     * written to `out` (0..max_out), or negative on a real transport/GATT error.
     * NULL-safe (older/host- fake HALs that predate this feature report it
     * UNSUPPORTED via the capability overlay instead of ever being called). */
    int (*gatt_discover_chars)(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                mtk_hal_gatt_char_t *out, unsigned max_out);
    /* Enumerate every descriptor between a characteristic's own value
     * handle+1 and the containing service's end_handle -- the same
     * search-range contract gatt_subscribe's own CCCD lookup already
     * uses (see gatt_subscribe's doc comment below). Returns the count
     * written to `out` (0..max_out), or negative on a real error. */
    int (*gatt_discover_descs)(uint16_t vendor_handle, uint16_t start_handle, uint16_t end_handle,
                                mtk_hal_gatt_desc_t *out, unsigned max_out);
    int (*gatt_read)(uint16_t vendor_handle, uint16_t attr_handle, uint8_t *out, uint16_t max_len, uint16_t *len_out);
    int (*gatt_write)(uint16_t vendor_handle, uint16_t attr_handle, const uint8_t *data, uint16_t len, uint8_t with_response);
    /* end_handle: the real containing service's end_handle from an actual prior
     * gatt_discover call (mtek_ble_logic.c tracks and supplies this -- never a
     * guessed window), bounding the CCCD (UUID16 0x2902) descriptor discovery
     * this call must perform between attr_handle+1 and end_handle.
     * Implementations must fail (return nonzero) rather than guess a handle if
     * no 0x2902 descriptor is found in that exact range.
     *
     * The accepted baseline requires a DISTINCT `[!] Characteristic <h> has no
     * CCCD (not subscribable).` error, never conflated with an ordinary write
     * failure -- so this return value is now a three-way contract, not a bare
     * zero/nonzero: 0 = OK; MTK_HAL_GATT_SUBSCRIBE_NO_CCCD = no 0x2902
     * descriptor found in range (never a guessed handle); any other nonzero =
     * the CCCD write itself failed for some other reason (a real transport/GATT
     * error). */
    int (*gatt_subscribe)(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle, uint8_t mode);
    int (*gatt_unsubscribe)(uint16_t vendor_handle, uint16_t attr_handle, uint16_t end_handle);
    /* Polls at most one pending notification/indication. Returns 1 if one
     * was delivered into *attr_handle_out/data_out/len_out, else 0. */
    int (*gatt_poll_notify)(uint16_t vendor_handle, uint16_t *attr_handle_out, uint8_t *data_out, uint16_t *len_out);
    /* Real remote-disconnect visibility ("Remote disconnect is ignored...
     * leaving connection/arbiter state stale."). Returns 1 exactly once for a
     * genuine peer-initiated disconnect of `vendor_handle` the service hasn't
     * yet observed, so it can release the GC arbiter lease and clear its own
     * connected-state bookkeeping the same way a local GATT_DISCONNECT already
     * does. NULL-safe (older/host-fake HALs that never model an async remote
     * disconnect may omit it).
     *
     * `*reason_out` (valid only when this returns 1) is the real HCI-level
     * disconnect reason -- never a hard-coded placeholder. */
    int (*gatt_poll_disconnected)(uint16_t vendor_handle, uint8_t *reason_out);
    /* A running total of notifications the HAL's own bounded queue has dropped
     * since the current connection was established (reset to 0 on each new
     * gatt_connect). NULL-safe (older/host-fake HALs that never model overflow
     * may omit it -- mtek_ble_logic.c treats a NULL function pointer the same as
     * "always reports 0"). */
    uint32_t (*gatt_notify_dropped_count)(void);
    /* Optional continuous discovery / signal teardown; UART live mode only. */
    int (*live_start)(const mtk_hal_ble_adv_t *previous, unsigned count);
    int (*live_snapshot)(mtk_hal_ble_adv_t *out, unsigned max);
    void (*live_stop)(void);
    void (*signal_stop)(void);
} mtk_ble_hal_t;

void mtek_ble_set_hal(const mtk_ble_hal_t *hal);

#ifdef __cplusplus
}
#endif
