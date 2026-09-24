/* Clean-room implementation from MonstaTek contract. */
#pragma once
#include "mtek_router.h"
#include "mtek_ble_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void mtek_ble_service_init(uint64_t (*now_ms_fn)(void));
mtk_register_result_t mtek_ble_service_register(void); /* registers service_id 0x0002 (ble) and 0x0003 (gatt) */

/* This file previously had NO lock of any kind -- s_sig (signal meter) and
 * s_gatt (GATT connection/subscriptions) are both genuinely shared between
 * request handlers (a synchronous dispatch),
 * mtek_ble_signal_meter_tick/mtek_ble_gatt_tick (a separate,
 * independently-scheduled periodic task on a real target), and, for s_gatt, the
 * real Wi-Fi/BLE driver's own remote-disconnect/notify delivery -- a genuine,
 * previously-unguarded data race on a real target build. Optional lock hooks
 * (default no-op, safe for every single-threaded host test that never registers
 * one), matching mtek_wifi_service_set_lock/mtek_capture_set_lock's own
 * established pattern exactly: bracket every mutation and cross-task read of
 * s_sig/s_gatt below, and are always released before any external HAL/sink call
 * (this tree's own established "never hold a lock across an external call"
 * rule). Live discovery is the bounded exception: start/stop/snapshot hold
 * this mutex while invoking only nonblocking HAL operations whose callbacks
 * take the separate lifecycle mutex and never this service mutex. */
typedef void (*mtk_ble_lock_fn)(void);
void mtek_ble_service_set_lock(mtk_ble_lock_fn lock, mtk_ble_lock_fn unlock);

/* A SEPARATE, dedicated GATT-operation lease -- genuinely distinct from
 * mtek_ble_service_set_lock's own field-level mutex above. Round 9's own fix
 * (gatt_snapshot_identity/gatt_snapshot_identity_with_ range, captured before a
 * blocking HAL call, re-validated after) protects every LOCAL state commit
 * (svc_ranges[]/subs[] merges, GATT_DISCOVER/ GATT_SUBSCRIBE's own response
 * content) from a disconnect-then-reconnect that reuses the same small
 * vendor_handle integer -- but it cannot prevent the PHYSICAL HAL call itself
 * (the real CCCD write, the real characteristic read/write, the real discovery
 * procedure) from executing against a REPLACEMENT connection's own live peer
 * once that handle is reused, since nothing previously stopped a new
 * GATT_CONNECT from reinitializing s_gatt while an older operation's own
 * blocking HAL call was still genuinely in flight. Every handler that issues
 * such a call (GATT_DISCOVER/_CHARS/_DESCS, GATT_READ, GATT_WRITE,
 * GATT_SUBSCRIBE, GATT_UNSUBSCRIBE) now acquires this lease BEFORE its own final
 * identity snapshot and holds it across the HAL call and its own post-HAL
 * revalidation/commit; handle_gatt_connect's own successful-connect reinit
 * acquires the SAME lease before touching s_gatt at all -- so a replacement
 * connection's reinit is structurally excluded from ever running while a prior
 * operation for the connection it would replace is still mid-flight, exactly
 * mirroring mtek_capture_set_action_lock's own design for the capture
 * channel-hop race (mtek_capture_service.h). Deliberately NEVER acquired by
 * handle_gatt_disconnect (a local disconnect) or by mtek_ble_gatt_tick's own
 * remote-disconnect/notify-poll paths -- both remain free to run immediately,
 * without ever blocking on an in-flight operation, so a genuine remote
 * disconnect during an operation can never deadlock this lease against
 * ble_tick_task (which also drives the signal-meter and capture-hop ticks and
 * must never stall for as long as a blocking GATT HAL call can take). Optional
 * (default no-op), matching every other lock hook in this file. */
void mtek_ble_service_set_gatt_op_lease_lock(mtk_ble_lock_fn lock, mtk_ble_lock_fn unlock);

/* Driven periodically (ESP32 glue: esp_timer; host tests: direct loop) to
 * deliver signal-meter samples and GATT notifications, since both are
 * inherently repeated-over-time rather than one blocking call. */
void mtek_ble_signal_meter_tick(void);
void mtek_ble_gatt_tick(void);

/* mtek_ble_signal_meter_tick/mtek_ble_gatt_tick above are only ever driven by
 * main/app_main.c's signal-sampling and periodic-delivery tasks. If either
 * required task fails to start (xTaskCreate returning non-pdPASS), periodic BLE
 * operations are conservatively disabled for this boot. A serial log at that
 * point is not honest protocol-level failure handling: SIGNAL_METER_START and
 * GATT_ SUBSCRIBE would still be ACCEPTED as if their own background delivery
 * mechanism worked, when in fact no sample/notification could ever be delivered.
 * Call this once (mirrors mtek_wifi_hal_esp32_mark_promisc_ task_failed's own
 * established pattern exactly) if that xTaskCreate fails;
 * handle_signal_meter_start and handle_gatt_subscribe then refuse honestly
 * (MTK_STATUS_NOT_READY) instead of accepting an operation that could never
 * actually deliver anything. Always ready (the default) until explicitly marked
 * otherwise -- this is a one-way, boot-session-permanent degradation, never
 * re-armed. */
void mtek_ble_service_mark_tick_task_failed(void);

/* The factory UART adapter's own frozen List A text transcript prints this line
 * for a CALLER-issued GATT_DISCONNECT (mtek_uart_adapter.c's handle_ble_
 * disconnect) but had no way to also print it for a genuine PEER- initiated
 * disconnect, which mtek_ble_gatt_tick above only ever surfaced as silent
 * internal state cleanup. This is deliberately NOT routed through the canonical
 * mtk_sink_t emit_event mechanism (unlike every other async delivery in this
 * tree): there is no canonical schema-defined event for an unsolicited GATT
 * disconnect, and native SPI v1 genuinely relays every emit_event call onto the
 * wire -- inventing a non-canonical event name would leak adapter-internal
 * signaling onto a real M1 peer's own wire protocol. This poll-and-consume
 * accessor (matching mtk_ble_hal_t's own gatt_poll_disconnected shape) is
 * instead a UART-adapter-only channel: returns 1 exactly once per remote
 * disconnect the caller hasn't yet consumed, 0 otherwise.
 *
 * "Remote disconnect currently loses the real reason and emits a hard-coded
 * reason." `*reason_out` (valid only when this returns 1) is now the real
 * HCI-level disconnect reason from the HAL's own GAP callback
 * (mtek_ble_hal_esp32.c's conn_gap_cb), not a placeholder. */
uint8_t mtek_ble_gatt_take_remote_disconnect_notice(uint8_t *reason_out);

/* See mtek_wifi_service.h's mtek_wifi_cancel_ active_for_peer_reset for the full
 * rationale -- the BLE-owned counterpart. Cancels and cleans up whatever
 * BLE-owned operation (BLE_SCAN/BLE_ADV/SIGNAL_METER) or GATT connection is
 * currently arbiter-active, releasing HAL/arbiter resources exactly as a real
 * STOP/ DISCONNECT would. No-op if the currently active arbiter class (if any)
 * does not belong to this service. Returns the cancelled operation's own {token,
 * boot_epoch} (mtk_op_id_t, mtek_core.h) so the caller can immediately evict it
 * via mtk_op_evict -- {0,0} if nothing was cancelled or the resource released
 * (GATT) carries no live operation token of its own to evict. */
mtk_op_id_t mtek_ble_cancel_active_for_peer_reset(void);

/* Factory UART live discovery; caller serializes start/snapshot/stop. */
int mtek_ble_live_start(mtk_request_ctx_t *ctx);
void mtek_ble_live_stop(void);
unsigned mtek_ble_live_snapshot(uint32_t *generation);
const mtk_hal_ble_adv_t *mtek_ble_live_item(unsigned index);
uint32_t mtek_ble_observation_age(const mtk_hal_ble_adv_t *item);

#ifdef __cplusplus
}
#endif
