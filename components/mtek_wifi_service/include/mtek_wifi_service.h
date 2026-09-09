/* Clean-room implementation from MonstaTek contract (002-wifi-service.md). */
#pragma once
#include "mtek_router.h"
#include "mtek_wifi_hal.h"
#include "mtek_arbiter.h"

#ifdef __cplusplus
extern "C" {
#endif

void mtek_wifi_service_init(uint64_t (*now_ms_fn)(void));
mtk_register_result_t mtek_wifi_service_register(void);
/* Drain deferred promiscuous frames from a normal application task. */
void mtek_wifi_service_tick(void);
/* Restore prior radio state and release its lease only when restoration
 * succeeds. Failure quarantines the radio behind the existing lease.
 *
 * P0 correction (follow-up read-only audit, "final P0 concurrency-closure
 * round", issue 1 "audit all production call sites, including Wi-Fi
 * restore/release paths -- not only GATT"): `owner_token` is the calling
 * operation's own token; the release below goes through mtk_arbiter_
 * release_if_owner(owner_class, owner_token) -- an atomic check-and-
 * release under ONE arbiter lock acquisition -- instead of the caller
 * separately checking `mtk_arbiter_active_token() == owner_token` before
 * calling this function (the previous shape, which left a real window
 * between that check and this call, wide open across restore_sta_mode's
 * own real HAL round-trip, for a concurrent reassignment of the class to
 * a newer operation to be silently released out from under it). Callers
 * no longer need to pre-check ownership at all -- restore_sta_mode() is
 * now always attempted (the caller's own HAL call is what disturbed the
 * radio, regardless of arbiter bookkeeping), and only the release is
 * gated on genuine, atomically-checked ownership. */
int mtek_wifi_restore_and_release(mtk_arbiter_class_t owner_class, uint32_t owner_token);
int mtek_wifi_radio_is_quarantined(void);
int mtek_wifi_is_sta_connected(void); /* for mtek_system_set_sta_query */
const mtk_wifi_hal_t *mtek_wifi_get_hal(void); /* reused by mtek_capture_service for promiscuous mode */

/* RC8 independent audit P0-3 "synchronize service session state": "The
 * Wi-Fi singletons (s_ap, s_sta, s_deauth, s_hs) ... are accessed
 * concurrently by transport handlers, workers, callbacks, and tick tasks
 * without service-level synchronization." A real ThreadSanitizer run
 * confirmed this for s_deauth/s_hs specifically (a live data race between
 * handle_deauth_start's own worker-thread progress writes and
 * handle_deauth_status's own concurrent read, and the same shape for
 * s_hs between hs_frame_cb -- a real Wi-Fi driver RX callback -- and
 * handle_handshake_status/_read). Optional lock hooks (default no-op,
 * safe for every single-threaded host test) bracket every mutation and
 * cross-task read of s_deauth/s_hs below, matching mtek_capture_service.h's
 * own established mtek_capture_set_lock pattern exactly. */
typedef void (*mtk_wifi_lock_fn)(void);
void mtek_wifi_service_set_lock(mtk_wifi_lock_fn lock, mtk_wifi_lock_fn unlock);

/* Release-tooling-round P0 correction (independent audit, "peer
 * session invalidation"): SPI_PROTOCOL_V1.md's own "Reset and
 * resynchronization" rule requires a changed peer boot_epoch (a native SPI
 * peer reboot) to invalidate "in-flight requests ... and operation tokens
 * for that peer" -- not merely the transport's own reassembly/dup-cache/
 * queue state (which mtek_spi_native_dispatch.c's own invalidate_prior_
 * epoch_state already clears). Cancels and cleans up whatever WiFi-owned
 * operation (AP_SCAN/STA_SCAN/DEAUTH/HANDSHAKE/STA_CONNECT) is currently
 * arbiter-active, running exactly the same claim/cleanup/truthful-
 * transition sequence a real STOP would -- radio state is genuinely
 * restored (or quarantined on failure, never silently freed), never left
 * stuck in whatever mode the cancelled operation left it in. No-op if the
 * currently active arbiter class (if any) does not belong to this
 * service. Called by a transport adapter that must invalidate every
 * operation belonging to a peer session that just ended -- currently only
 * native SPI has such a concept (Mtek Compatibility/C3 and factory UART have no peer
 * reboot detection of their own, mtek_core.h's boot_epoch doc comment).
 * Returns the cancelled operation's own {token, boot_epoch} (mtk_op_id_t,
 * mtek_core.h) so the caller can immediately evict it via mtk_op_evict
 * (making it genuinely unusable, not merely terminal-but-still-
 * reportable) -- {0,0} if nothing was cancelled. */
mtk_op_id_t mtek_wifi_cancel_active_for_peer_reset(void);

/* M3 correction (independent review P0 "the D->H handoff uses a coherent
 * but unstable snapshot and an unconditional release"): test-only pause
 * seam, called from handle_handshake_start immediately AFTER its own D-
 * ownership snapshot is taken (still holding the admission guard's
 * pub_lock, never the arbiter lock), letting a test pause deterministically
 * at exactly the point a real race must be proven closed -- D's own
 * natural finalization, and a completely independent admission installing
 * a brand-new owner, both racing concurrently against this held snapshot
 * before the guarded owner-checked release runs. Always NULL (a true
 * no-op call) outside of a test that explicitly sets it -- never used for
 * any runtime decision. */
typedef void (*mtk_wifi_dh_handoff_pause_hook_t)(void);
void mtek_wifi_set_dh_handoff_pause_hook(mtk_wifi_dh_handoff_pause_hook_t hook);

#ifdef __cplusplus
}
#endif
