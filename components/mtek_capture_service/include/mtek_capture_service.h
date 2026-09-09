/* Clean-room implementation from MonstaTek contract
 * (002-capture-diagnostics-service.md). */
#pragma once
#include "mtek_router.h"
#include "mtek_wifi_hal.h" /* reuses the Wi-Fi radio's promiscuous-mode primitives */

#ifdef __cplusplus
extern "C" {
#endif

void mtek_capture_service_init(uint64_t (*now_ms_fn)(void));
mtk_register_result_t mtek_capture_service_register(void); /* service_id 0x0004 (capture) and 0x0005 (diagnostics) */

/* RC7 independent audit item 11 "the global session is raced between the
 * Wi-Fi callback and control/tick tasks": this service's own capture
 * session state (frame counters, credit budget, the POLL-mode ring
 * buffer) is written from the Wi-Fi HAL's frame callback (a real
 * target's own promiscuous-mode driver task) while also being read and
 * written from request dispatch (CAPTURE_STATUS/STATS/POLL_READ/STOP,
 * whatever task originates the request) and the periodic hop tick below
 * -- genuinely different FreeRTOS tasks, unlike a synchronous HAL call's
 * own natural single-thread-of-control safety. Optional lock hooks
 * (default no-op, safe for every single-threaded host test), mirroring
 * mtek_core.h/mtek_arbiter.h's own established pattern -- a real target
 * registers the same shared mutex used for those. */
typedef void (*mtk_capture_lock_fn)(void);
void mtek_capture_set_lock(mtk_capture_lock_fn lock, mtk_capture_lock_fn unlock);

/* P0 correction (RC11 round 10, item 1 "close the capture-hop pre-HAL race
 * completely"): a SEPARATE, dedicated lease/ownership lock -- genuinely
 * distinct from mtek_capture_set_lock's own s_cap field-level mutex above
 * -- covering the window from a hop tick's final identity re-validation,
 * through the real hal->set_channel() HAL round-trip, through its own
 * post-HAL state-commit re-validation. Round 9's own fix re-validated
 * identity both immediately before AND immediately after the HAL call, but
 * a genuine window remained BETWEEN releasing the field-level lock after
 * the pre-HAL check and actually invoking hal->set_channel(): a STOP
 * followed by a brand-new CAPTURE_START could reinitialize s_cap for a
 * wholly different session in exactly that gap, so the OLD tick's own HAL
 * call could still land after a replacement session already existed --
 * closed for the STATE/EVENT side by the post-HAL re-check, but the HAL
 * action itself was never actually excluded. handle_capture_start now
 * acquires this SAME lease before reinitializing s_cap -- so a replacement
 * session's own reinit cannot happen while a hop tick's action (validate-
 * >HAL-call->commit) is in flight for the session being replaced, closing
 * the window completely rather than only its state-commit half. Never
 * held across a blocking NimBLE/HAL callback path that could re-enter it
 * (frame_cb/handle_capture_stop/etc. never acquire this lock, only
 * mtek_capture_channel_hop_tick and handle_capture_start do) -- safe to
 * hold across the real hal->set_channel() round-trip without risking a
 * callback-reentrancy deadlock. Optional (default no-op), matching every
 * other lock hook in this file. */
typedef void (*mtk_capture_action_lock_fn)(void);
void mtek_capture_set_action_lock(mtk_capture_action_lock_fn lock, mtk_capture_action_lock_fn unlock);

/* STREAM credit grant for the active push-mode session (Sec 3.3/4 of the
 * canonical core contract; STM32 CREDIT class on the wire). Adapters call
 * this when a CREDIT cell arrives. Saturating add (never wraps); returns
 * 1 if `token` matched the active session (credit applied), 0 otherwise
 * (a safe, explicit no-op -- RC7 independent audit P0 "Native CREDIT and
 * CANCEL are not implemented" / item 11 "CREDIT addition is
 * unsynchronized and can overflow"). */
uint8_t mtek_capture_grant_credit(uint32_t token, uint32_t bytes);

/* RC5 independent audit P1 "MonstaShark capture path is incomplete":
 * "capture filters and channel plans are stored but not applied". Real
 * channel hopping needs a periodic tick, the same shape as
 * mtek_ble_signal_meter_tick/mtek_ble_gatt_tick (mtek_ble_service.h) --
 * a target build calls this from its own periodic task at whatever
 * cadence it likes (a call more often than the plan's own hop_dwell_ms
 * is harmless; it only actually switches channel once dwell has
 * elapsed). A no-op unless a hop-mode (channel_plan.mode==1) capture is
 * currently RUNNING. */
void mtek_capture_channel_hop_tick(uint64_t now_ms);

/* P0 correction (Round 9 item 3, tightened Round 10 item 1 "close the
 * capture-hop pre-HAL race completely"): test-only pause hook, mirroring
 * mtek_core.h's own mtk_op_set_won_hook pattern exactly. Fires from inside
 * a hop-tick invocation immediately after its own FINAL identity
 * re-validation (token/hop_active, re-checked under the field-level lock)
 * -- while still holding mtek_capture_set_action_lock's own lease -- and
 * immediately before the real hal->set_channel() HAL call. Lets a
 * deterministic test pause a "delayed" tick invocation, on a background
 * thread, at exactly the point a real target's own scheduler could
 * preempt it for an arbitrarily long time, while the main thread stops the
 * captured session and a concurrent CAPTURE_START attempt for a brand-new
 * hop-mode session genuinely BLOCKS on the same action lease (never
 * completing its own reinit until this tick resumes and releases it),
 * then resumes delivery -- proving the paused tick can only ever act on
 * ITS OWN session (no replacement can exist yet while it holds the lease)
 * and that the replacement's own reinit/hop proceeds normally once
 * released. Always NULL (a true no-op call) outside of a test that
 * explicitly sets it. */
typedef void (*mtk_capture_hop_tick_pause_hook_t)(void);
void mtek_capture_set_hop_tick_pause_hook(mtk_capture_hop_tick_pause_hook_t hook);

/* P0 correction (this round, item 4 "required task-creation failures"):
 * mtek_capture_channel_hop_tick above is only ever driven by main/
 * app_main.c's own periodic_delivery_task (despite this whole file's own capture-
 * specific name, the SAME task also drives the BLE signal-meter/GATT
 * ticks -- see that task's own doc comment) -- if that task fails to
 * start, a CAPTURE_START with a nonzero duration_ms (auto-stop) or a
 * hop-mode (channel_plan.mode==1) channel plan would be silently ACCEPTED
 * as if its own duration/hop enforcement worked, when in fact neither
 * could ever run. Call this once (mirrors mtek_ble_service_mark_tick_
 * task_failed's own established pattern exactly) if that xTaskCreate
 * fails; handle_capture_start then refuses honestly (MTK_STATUS_NOT_READY)
 * for exactly those two variants -- an unbounded, non-hopping capture
 * (duration_ms==0, channel_plan.mode==0) needs no tick at all (frame_cb
 * alone drives it) and remains fully available regardless. Always ready
 * (the default) until explicitly marked otherwise -- a one-way, boot-
 * session-permanent degradation, never re-armed. */
void mtek_capture_service_mark_tick_task_failed(void);

/* Release-tooling-round P0 correction (independent audit, "peer
 * session invalidation"): see mtek_wifi_service.h's mtek_wifi_cancel_
 * active_for_peer_reset for the full rationale -- the capture-owned
 * counterpart (MonstaShark, MTK_ARB_M). Reuses the exact same teardown
 * path a real STOP/duration-elapsed completion would run (promisc_stop,
 * restore-or-quarantine, truthful terminal transition, terminal event).
 * No-op if MTK_ARB_M is not the currently active arbiter class. Returns
 * the cancelled operation's own {token, boot_epoch} (mtk_op_id_t,
 * mtek_core.h) so the caller can immediately evict it via mtk_op_evict --
 * {0,0} if nothing was cancelled. */
mtk_op_id_t mtek_capture_cancel_active_for_peer_reset(void);

#ifdef __cplusplus
}
#endif
