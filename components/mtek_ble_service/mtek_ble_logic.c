/* Clean-room implementation from MonstaTek contract. Portable: no ESP-IDF/NimBLE
 * dependency, host-testable against mtek_ble_hal_t. */
#include "mtek_ble_service.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include <string.h>

/* (service_id, originating START opcode) per BLE token family (service
 * 0x0002), for the mtk_op_*_family core APIs -- a token from another family
 * handed to a BLE STOP/STATUS is rejected NOT_FOUND with no side effect. (GATT
 * handlers, service 0x0003, are separately family-isolated by their own
 * s_gatt.connection_token identity match -- see their doc comments -- because a
 * live GATT connection outlives the GATT_CONNECT operation record it was minted
 * under, so an op-table family check would wrongly reject a valid read after
 * that record is GC'd.) */
#define BLE_SERVICE_ID          0x0002
#define BLE_SCAN_START_OPCODE   0x0001
#define BLE_ADV_START_OPCODE    0x0006
#define SIGNAL_METER_START_OPCODE 0x0009

#define BLE_MAX_DEVICES 40
#define GATT_MAX_SVCS 16
#define GATT_MAX_SUBS 16

static const mtk_ble_hal_t *s_hal;
static uint64_t (*s_now_ms)(void);
void mtek_ble_set_hal(const mtk_ble_hal_t *hal) { s_hal = hal; }
void mtek_ble_service_init(uint64_t (*now_ms_fn)(void)) { s_now_ms = now_ms_fn; }
static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

/* See mtek_ble_service.h's own doc comment on mtek_ble_service_set_lock for the
 * full rationale. */
static mtk_ble_lock_fn s_ble_lock, s_ble_unlock;
void mtek_ble_service_set_lock(mtk_ble_lock_fn lock, mtk_ble_lock_fn unlock) { s_ble_lock = lock; s_ble_unlock = unlock; }
static void ble_lock(void) { if (s_ble_lock) s_ble_lock(); }
static void ble_unlock(void) { if (s_ble_unlock) s_ble_unlock(); }

/* A SEPARATE, dedicated GATT-operation lease/ownership lock -- genuinely
 * distinct from s_ble_lock/s_ble_unlock's own field-level mutex above -- see
 * mtek_ble_service.h's own doc comment on mtek_ble_service_
 * set_gatt_op_lease_lock for the full rationale. Every handler that issues a
 * blocking GATT HAL call (discover/discover_chars/discover_descs/read/
 * write/subscribe/unsubscribe) acquires this BEFORE its own final identity
 * validation and holds it across that HAL call and its own post-HAL
 * revalidation/commit; handle_gatt_connect's own successful-connect reinit
 * acquires the SAME lease before touching s_gatt, so a replacement connection's
 * reinit cannot happen while any prior operation for the connection it would
 * replace is still in flight -- closing the vendor-handle-reuse window for the
 * PHYSICAL HAL action itself, not only for local state. Never acquired by
 * mtek_ble_gatt_tick's remote- disconnect/notify-poll paths or by
 * handle_gatt_disconnect (a local disconnect) -- both remain lock-free with
 * respect to this lease so neither can ever block on (or deadlock against) an
 * in-flight operation; see capture_teardown's own identical, deliberate
 * non-participation (mtek_capture_logic.c) for the same
 * rationale. */
static mtk_ble_lock_fn s_gatt_op_lease_lock, s_gatt_op_lease_unlock;
void mtek_ble_service_set_gatt_op_lease_lock(mtk_ble_lock_fn lock, mtk_ble_lock_fn unlock) { s_gatt_op_lease_lock = lock; s_gatt_op_lease_unlock = unlock; }
static void gatt_op_lease_lock(void) { if (s_gatt_op_lease_lock) s_gatt_op_lease_lock(); }
static void gatt_op_lease_unlock(void) { if (s_gatt_op_lease_unlock) s_gatt_op_lease_unlock(); }

/* See mtek_ble_service.h's own doc comment on
 * mtek_ble_service_mark_tick_task_failed. Ready by default; app_main.c marks
 * this false, once, if ble_tick_task's own xTaskCreate fails -- never re-armed
 * for the rest of this boot session. */
static uint8_t s_tick_task_ready = 1;
void mtek_ble_service_mark_tick_task_failed(void) { s_tick_task_ready = 0; }

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}
static mtk_hal_mac6_t to_hal_mac(mtk_mac6_t m) { mtk_hal_mac6_t h; memcpy(h.b, m.b, 6); return h; }
static mtk_mac6_t from_hal_mac(mtk_hal_mac6_t h) { mtk_mac6_t m; memcpy(m.b, h.b, 6); return m; }

/* BLE scan ------------------------------ */

static struct { uint32_t generation; uint16_t count; mtk_hal_ble_adv_t items[BLE_MAX_DEVICES]; } s_scan;

static void fill_device_details_from_hal(const mtk_hal_ble_adv_t *a, mtk_ble_device_details_resp_t *r) {
    memset(r, 0, sizeof(*r));
    r->addr = from_hal_mac(a->addr);
    r->addr_type = a->addr_type;
    r->rssi = a->rssi;
    r->adv_type = a->adv_type;
    r->flags = a->flags;
    r->name.len = a->name_len; memcpy(r->name.data, a->name, a->name_len);
    r->tx_power = (int8_t)a->tx_power;
    r->mfg_data.len = a->mfg_len; memcpy(r->mfg_data.data, a->mfg_data, a->mfg_len);
    r->raw_adv.len = a->raw_adv_len; memcpy(r->raw_adv.data, a->raw_adv, a->raw_adv_len);
    r->raw_scan_rsp.len = a->raw_scan_rsp_len; memcpy(r->raw_scan_rsp.data, a->raw_scan_rsp, a->raw_scan_rsp_len);
}

static void handle_ble_scan_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_scan_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_connect's identical comment above. MTK_ARB_BS is GUARDED
     * with MTK_ARB_GC in the arbiter policy table, but neither side's current
     * admission code treats GUARDED as anything other than BUSY (this `!=
     * GRANT_OK` check is unchanged from before), so that behavior is preserved
     * exactly as-is. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the (potentially HAL-synchronous) scan call below. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_BS, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    mtk_ble_scan_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ble_scan_start_resp_t_desc);
    mtk_op_end_admission_guard();

    char namebuf[33]; memset(namebuf, 0, sizeof(namebuf));
    if (req.name_filter.len) memcpy(namebuf, req.name_filter.data, req.name_filter.len);
    int n = s_hal && s_hal->scan ? s_hal->scan(req.mode, req.duration_ms, namebuf, s_scan.items, BLE_MAX_DEVICES) : 0;
    if (n < 0) n = 0;
    if ((unsigned)n > BLE_MAX_DEVICES) n = BLE_MAX_DEVICES;
    /* State publish/event emission below remain gated on actually WINNING
     * finalization for this exact token. s_hal->scan above can block for the
     * whole requested duration with no cancel hook of its own (unlike AP/STA
     * scan's ap_scan_cancel/sta_scan_ cancel) -- if a concurrent peer-session
     * invalidation (or any other STOP-equivalent) already claimed/evicted this
     * token while the call was blocked, this path must never publish s_scan or
     * emit an event into a session that has already moved on.
     *
     * `won` alone only proves nobody else finalized this token BEFORE this exact
     * instant -- a peer-session reset can still land in the window between
     * winning and actually publishing below. A single point-in-time re-check
     * cannot close that window either (it only proves the generation had not YET
     * changed the instant it ran). mtk_op_begin_publish_guard, held across the
     * WHOLE publish (both the per-device events and the final summary event,
     * merged into one guarded region below so nothing externally visible ever
     * happens outside it), makes "check" and "the generation actually changing"
     * mutually exclusive instead -- see its own doc comment in mtek_core.h for
     * the full rationale. The already-won op-table transition and the arbiter
     * release below are unaffected by staleness and run OUTSIDE the guard, as
     * before. */
    int won = mtk_op_claim_finalization(id.token, id.boot_epoch);
    if (won) {
        mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_COMPLETED, MTK_STATUS_OK, now_ms());
    }
    /* The arbiter release is OUTSIDE the `won` gate -- a peer-reset canceller
     * may already have won the op-table claim (see
     * mtek_ble_cancel_active_for_peer_reset's own BS branch, which deliberately
     * never touches the arbiter itself for this reason) while this call's
     * blocking scan is still genuinely running; releasing only inside the `won`
     * block would then be skipped entirely and permanently orphan MTK_ARB_BS.
     * The ownership check and the release now happen atomically, in ONE arbiter
     * lock acquisition (mtk_arbiter_release_if_owner) -- this is the ONE place
     * that ever releases it for an in-progress scan, exactly once, whenever the
     * real HAL call actually returns. */
    mtk_arbiter_release_if_owner(MTK_ARB_BS, id.token);
    if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
        s_scan.generation = id.token; s_scan.count = (uint16_t)n;
        for (int i = 0; i < n; i++) {
            mtk_ble_device_found_ev_t ev; memset(&ev, 0, sizeof(ev));
            ev.operation_token = id.token; ev.addr = from_hal_mac(s_scan.items[i].addr);
            ev.addr_type = s_scan.items[i].addr_type; ev.rssi = s_scan.items[i].rssi;
            ev.name.len = s_scan.items[i].name_len; memcpy(ev.name.data, s_scan.items[i].name, s_scan.items[i].name_len);
            ctx->sink.emit_event(ctx->sink.user, id.token, "BLE_DEVICE_FOUND", &ev, &mtk_ble_device_found_ev_t_desc);
        }
        mtk_ble_scan_complete_ev_t done = {0};
        done.operation_token = id.token; done.status = MTK_STATUS_OK;
        done.result_generation = s_scan.generation; done.result_count = s_scan.count;
        ctx->sink.emit_event(ctx->sink.user, id.token, "BLE_SCAN_COMPLETE", &done, &mtk_ble_scan_complete_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_ble_scan_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_scan_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Never retain a raw mtk_op_find pointer past the lock that produced it -- a
     * concurrent transition/gc/alloc on another worker can evict and reuse the
     * slot for a completely different token before this call reads/writes
     * through it. mtk_op_snapshot/mtk_op_transition_by_token both re-validate
     * (token, boot_epoch) under their own lock acquisition every time, so they
     * are immune to that reuse regardless of what now occupies the slot. */
    mtk_operation_record_t snap;
    /* Family gate (BLE_SCAN_START) -- a non-BLE_SCAN token is rejected NOT_FOUND
     * before any transition/arbiter interaction. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Scan has no cancel hook of its own -- winning this transition (which
     * happens whenever the worker's own s_hal->scan call has not yet reached its
     * own mtk_op_claim_finalization) previously released MTK_ARB_BS immediately,
     * letting a brand-new BLE_SCAN_START acquire it and start a second,
     * overlapping scan call while the first was still genuinely blocked. Only
     * the TOKEN is fenced here now, exactly mirroring
     * mtek_ble_cancel_active_for_peer_reset's own BS handling;
     * handle_ble_scan_start's own tail (now gated on arbiter TOKEN ownership,
     * independent of winning this same transition) is the ONLY place that safely
     * releases this class, once its real HAL call returns. */
    mtk_op_transition_by_token_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_SCAN_START_OPCODE, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ble_scan_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_scan_stop_resp_t_desc);
}

static void handle_ble_scan_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_scan_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_SCAN_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ble_scan_status_resp_t r; r.state = (uint8_t)snap.state; r.found_so_far = s_scan.count;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_scan_status_resp_t_desc);
}

static void handle_ble_scan_results_page(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                          const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_scan_results_page_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.result_generation != s_scan.generation || s_scan.generation == 0) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    uint8_t max_items = req.max_items ? req.max_items : 40;
    if (max_items > 40) max_items = 40;
    mtk_ble_scan_results_page_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = (uint16_t)req.start_index, n = 0;
    for (; i < s_scan.count && n < max_items; i++, n++) {
        r.items.items[n].addr = from_hal_mac(s_scan.items[i].addr);
        r.items.items[n].addr_type = s_scan.items[i].addr_type;
        r.items.items[n].rssi = s_scan.items[i].rssi;
        r.items.items[n].name.len = s_scan.items[i].name_len;
        memcpy(r.items.items[n].name.data, s_scan.items[i].name, s_scan.items[i].name_len);
    }
    r.items.count = n; r.next_index = (i < s_scan.count) ? i : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_scan_results_page_resp_t_desc);
}

static void handle_ble_device_details(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                       const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_device_details_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.result_generation != s_scan.generation || req.index >= s_scan.count) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ble_device_details_resp_t r;
    fill_device_details_from_hal(&s_scan.items[req.index], &r);
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_device_details_resp_t_desc);
}

/* Advertising ----------------------------- */

static uint8_t s_adv_name[32]; static uint8_t s_adv_name_len;

static void handle_ble_adv_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_adv_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_connect's identical comment above.
     *
     * M2 independent-review addendum ("BLE advertising's target HAL call needs
     * explicit boundedness justification, not just 'stays inside the guard'"):
     * adv_start below (esp32_ble_adv_start, mtek_ble_hal_esp32.c) calls exactly
     * two NimBLE host API functions -- ble_gap_adv_set_fields (a local,
     * synchronous write into the host's own outgoing-advertisement-data buffers;
     * no radio I/O, no wait) and ble_gap_adv_start called with a NULL completion
     * callback (per NimBLE's own documented ble_gap_adv_start contract: it arms
     * the advertising state machine and returns immediately with a status code
     * -- passing a NULL callback here means this call does not even register
     * for, let alone block on, any later advertising event; the BLE controller
     * then advertises asynchronously in the background, entirely independent of
     * this call's own return). Both calls are therefore genuinely
     * non-blocking/bounded by construction (no semaphore wait, no timeout, no
     * queue drain), unlike e.g. gatt_connect's real up-to-30s blocking round
     * trip -- adv_start safely stays inside the admission guard along with the
     * rest of this handler's own already-fast admission work, and its immediate
     * IO_ERROR response below (on ble_gap_adv_set_fields/ble_gap_adv_ start
     * returning nonzero) remains truthful: a genuine synchronous failure, not a
     * guess made before the real outcome is known. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_BA, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    const uint8_t *name = req.name.len ? req.name.data : (const uint8_t *)"M1-BLE";
    uint8_t name_len = req.name.len ? (uint8_t)req.name.len : 6;
    if (name_len > 31) name_len = 31;
    memcpy(s_adv_name, name, name_len); s_adv_name_len = name_len;
    int rc = s_hal && s_hal->adv_start ? s_hal->adv_start(s_adv_name, s_adv_name_len) : -1;
    if (rc != 0) {
        mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_FAILED, MTK_STATUS_IO_ERROR, now_ms());
        mtk_arbiter_release(MTK_ARB_BA);
        respond_empty(ctx, MTK_STATUS_IO_ERROR);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());
    mtk_ble_adv_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ble_adv_start_resp_t_desc);
    mtk_op_end_admission_guard();
}

static void handle_ble_adv_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_adv_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_ble_scan_stop's doc comment above -- token-based snapshot/
     * transition, never a retained mtk_op_find pointer. */
    mtk_operation_record_t snap;
    /* Family gate (BLE_ADV_START) -- and the transition itself is family-gated
     * too, so a non-BLE_ADV token can never reach the adv_stop HAL call, the
     * MTK_ARB_BA release, or the BLE_ADV_STOPPED emission for a foreign
     * operation. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_ADV_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (mtk_op_transition_by_token_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_ADV_START_OPCODE, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
        if (s_hal && s_hal->adv_stop) s_hal->adv_stop();
        mtk_arbiter_release(MTK_ARB_BA);
        mtk_ble_adv_stopped_ev_t ev; ev.operation_token = req.operation_token; ev.status = MTK_STATUS_OK;
        ctx->sink.emit_event(ctx->sink.user, req.operation_token, "BLE_ADV_STOPPED", &ev, &mtk_ble_adv_stopped_ev_t_desc);
    }
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_ADV_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ble_adv_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_adv_stop_resp_t_desc);
}

static void handle_ble_adv_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_ble_adv_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, BLE_ADV_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_ble_adv_status_resp_t r; memset(&r, 0, sizeof(r));
    r.state = (uint8_t)snap.state; r.name.len = s_adv_name_len; memcpy(r.name.data, s_adv_name, s_adv_name_len);
    /* Frozen shipped List A parity : advertising is always non-connectable with
     * no scan response -- see mtek_ble_hal_esp32.c's esp32_ble_adv_start, the
     * real HAL call this status must agree with. */
    r.connectable = 0; r.has_scan_response = 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ble_adv_status_resp_t_desc);
}

/* Signal meter ----------------------------- */
/* s_sig.ctx (and s_gatt.ctx below) store a byte-for-byte VALUE COPY of *ctx, not
 * a pointer -- safe regardless of the dispatching call's own stack/pool-slot
 * lifetime, since mtk_request_ctx_t holds no pointer back into itself; the only
 * field whose target must independently outlive is sink.user, which the
 * dispatching adapter is responsible for pointing at storage stable for the
 * operation's whole lifetime (see mtek_wifi_logic.c's handshake_session_t doc
 * comment for the fuller rationale -- this is the same pattern, applied here
 * from the start rather than retrofitted, as the raw-pointer variant of this
 * bug had to be in the handshake session). */

/* "Signal meter scans for 300 ms every 500 ms and declares LOST after one miss,
 * conflicting with the approximately five-second shipping behavior."
 * MTK_SIGNAL_METER_SAMPLE_INTERVAL_MS matches the audit's own stated confirmed
 * figure exactly.
 *
 * RC6's own MTK_SIGNAL_METER_MISS_TOLERANCE=3 (a disclosed engineering choice --
 * the RC5 audit confirmed one miss was too aggressive but did not itself state
 * an exact tolerance) over-corrected: once the sample interval itself is a real
 * 5 seconds, waiting for 3 CONSECUTIVE misses before declaring LOST takes ~15
 * seconds total, actively contradicting the confirmed "approximately five-second
 * signal loss" shipped behavior this whole change exists to match. Reverted to
 * 1: the very first missed sample (itself ~5 seconds after the last successful
 * one, given the real sampling cadence above) now declares LOST, matching the
 * confirmed ~5-second figure directly instead of a multiple of it. */
#define MTK_SIGNAL_METER_SAMPLE_INTERVAL_MS 5000
#define MTK_SIGNAL_METER_MISS_TOLERANCE 1

/* TOKEN, not a retained `mtk_operation_record_t *` -- same ABA-hazard rationale
 * as mtek_wifi_logic.c's s_deauth/s_hs and mtek_capture_logic.c's s_cap (each
 * service's own struct doc comment). */
/* session_generation, the ORIGINATING session's own (mtk_request_ctx_t's own
 * field, 0 for every non-native-SPI adapter -- never fenced), stored once at
 * handle_ signal_meter_start time. mtek_ble_signal_meter_tick -- an
 * independently-scheduled periodic tick with no request ctx of its own --
 * validates against THIS stored value via mtk_op_begin_publish_guard, never a
 * caller's own ctx. Every field below is now protected by ble_lock/ble_unlock
 * (requirement 4: this struct had no dedicated lock of any kind before). */
static struct { uint32_t token; mtk_request_ctx_t ctx; mtk_hal_mac6_t addr; uint8_t addr_type;
                int32_t avg_x10; uint32_t started_ms; uint8_t active;
                uint64_t last_sample_ms; uint8_t sampled_once; uint8_t consecutive_misses;
                uint32_t session_generation; uint8_t sample_in_flight; } s_sig;

/* STOP prevents new samples immediately, but the radio lease remains held until
 * the outstanding sample has returned and retired its HAL callbacks. */
static void signal_meter_cancel(uint32_t token) {
    int release = 0;
    ble_lock();
    if (s_sig.token == token) {
        s_sig.active = 0;
        release = !s_sig.sample_in_flight;
    }
    ble_unlock();
    if (release) mtk_arbiter_release_if_owner(MTK_ARB_SM, token);
}

static void signal_meter_sample_finished(uint32_t token) {
    int release = 0;
    ble_lock();
    if (s_sig.token == token) {
        s_sig.sample_in_flight = 0;
        release = !s_sig.active;
    }
    ble_unlock();
    if (release) mtk_arbiter_release_if_owner(MTK_ARB_SM, token);
}

static uint8_t rssi_category(int8_t rssi) {
    if (rssi >= -50) return 0;
    if (rssi >= -60) return 1;
    if (rssi >= -70) return 2;
    if (rssi >= -80) return 3;
    return 4;
}

static void handle_signal_meter_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                       const uint8_t *req_bytes, size_t req_len) {
    mtk_signal_meter_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Every sample after the first is only ever delivered by
     * mtek_ble_signal_meter_tick, itself only ever called from main/app_main.c's
     * own ble_tick_task -- refuse honestly, before acquiring any resource, if
     * that task never started. */
    if (!s_tick_task_ready) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    /* See handle_gatt_connect's identical comment above. The guard ends right
     * after the ACCEPTED response, BEFORE mtek_ble_signal_meter_tick below --
     * that call makes a real, unguarded HAL round-trip (see its own doc comment)
     * and must never run while this lock is held. */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the immediate mtek_ble_signal_meter_tick call just below
     * (which itself re-resolves s_sig.token via a fresh locked snapshot, never a
     * pointer). */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    if (mtk_arbiter_acquire(MTK_ARB_SM, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());
    ble_lock();
    memset(&s_sig, 0, sizeof(s_sig));
    s_sig.token = id.token; s_sig.ctx = *ctx; s_sig.addr = to_hal_mac(req.target.addr); s_sig.addr_type = req.target.addr_type;
    s_sig.started_ms = (uint32_t)now_ms(); s_sig.active = 1;
    s_sig.session_generation = ctx->session_generation;
    ble_unlock();
    mtk_signal_meter_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_signal_meter_start_resp_t_desc);
    mtk_op_end_admission_guard();
    mtek_ble_signal_meter_tick(); /* first sample immediately, matching field-parity "[BLE:SIG:START]" then first reading */
}

void mtek_ble_signal_meter_tick(void) {
    /* Every field this tick touches is now protected by ble_lock -- this struct
     * previously had NO lock at all, a genuine data race against
     * handle_signal_meter_ start/_stop's own concurrent mutation on a real
     * target (this tick runs on its own independently-scheduled task,
     * main/app_main.c's ble_tick_task). Snapshotted here, then
     * s_hal->signal_sample (a real HAL round-trip) runs UNLOCKED and un-guarded
     * -- never hold a lock or the publish guard across an external call. */
    ble_lock();
    uint8_t active = s_sig.active;
    uint32_t token = s_sig.token;
    uint32_t boot_epoch = s_sig.ctx.boot_epoch;
    ble_unlock();
    if (!active) return;
    /* A retained mtk_op_find pointer here would still race a concurrent STOP
     * (handle_signal_meter_stop) between this check and the FAILED transition
     * below, even though s_sig.token itself (a plain uint32_t) is already immune
     * to slot reuse -- the snapshot re-validates (token, boot_epoch) fresh,
     * under lock, right now. */
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot(token, boot_epoch, &snap) || mtk_op_state_is_terminal(snap.state)) return;

    ble_lock();
    /* Re-validate s_sig is STILL the same session just observed above -- a
     * concurrent STOP followed by a brand-new handle_signal_meter_start could
     * have reinitialized it for a wholly different session in the meantime. */
    if (!s_sig.active || s_sig.token != token || s_sig.sample_in_flight) { ble_unlock(); return; }
    /* Self-throttle to the real sampling cadence regardless of how often
     * the caller's own periodic task invokes this tick (main/app_main.c's
     * ble_tick_task runs every 500ms for GATT-notification-polling
     * reasons unrelated to signal-meter timing) -- except the very first
     * call right after START, which always samples immediately (matching
     * the shipped "[BLE:SIG:START]" then first-reading behavior). */
    uint64_t now = now_ms();
    if (s_sig.sampled_once && now - s_sig.last_sample_ms < MTK_SIGNAL_METER_SAMPLE_INTERVAL_MS) { ble_unlock(); return; }
    s_sig.last_sample_ms = now;
    s_sig.sampled_once = 1;
    s_sig.sample_in_flight = 1;
    mtk_hal_mac6_t addr = s_sig.addr;
    uint8_t addr_type = s_sig.addr_type;
    ble_unlock();

    int8_t rssi = 0; uint8_t is_random = 0;
    int rc = s_hal && s_hal->signal_sample ? s_hal->signal_sample(addr, addr_type, &rssi, &is_random) : -1;

    if (rc != 0) {
        ble_lock();
        if (!s_sig.active || s_sig.token != token) { ble_unlock(); signal_meter_sample_finished(token); return; }
        s_sig.consecutive_misses++;
        if (s_sig.consecutive_misses < MTK_SIGNAL_METER_MISS_TOLERANCE) { ble_unlock(); signal_meter_sample_finished(token); return; } /* one missed sample is not yet LOST */
        uint32_t session_generation = s_sig.session_generation;
        mtk_sink_t sink = s_sig.ctx.sink;
        ble_unlock();
        /* Gated on mtk_op_transition's own return value (linearization), not a
         * separate racy pre-check -- a concurrent STOP could otherwise race this
         * exact LOST transition and double-release the arbiter/double-emit. */
        if (!mtk_op_transition_by_token(token, boot_epoch, MTK_OPS_FAILED, MTK_STATUS_NOT_FOUND, now_ms())) {
            ble_lock(); if (s_sig.token == token) s_sig.active = 0; ble_unlock();
            signal_meter_sample_finished(token);
            return;
        }
        /* Atomic ownership-checked release, matching every other class release
         * site across this tree -- this one was missed in that round since s_sig
         * had no lock/ generation infrastructure until now. */
        ble_lock(); if (s_sig.token == token) s_sig.active = 0; ble_unlock();
        signal_meter_sample_finished(token);
        /* mtk_op_begin_publish_guard, held across the whole publish,
         * closes the window between winning the transition above and
         * actually emitting -- see its own doc comment in mtek_core.h. */
        if (mtk_op_begin_publish_guard(session_generation)) {
            mtk_signal_meter_lost_ev_t ev; ev.operation_token = token; ev.status = MTK_STATUS_NOT_FOUND;
            sink.emit_event(sink.user, token, "SIGNAL_METER_LOST", &ev, &mtk_signal_meter_lost_ev_t_desc);
            mtk_op_end_publish_guard();
        }
        return;
    }

    ble_lock();
    if (!s_sig.active || s_sig.token != token) { ble_unlock(); signal_meter_sample_finished(token); return; }
    s_sig.consecutive_misses = 0;
    s_sig.avg_x10 = s_sig.avg_x10 ? (s_sig.avg_x10 * 3 + rssi * 10) / 4 : rssi * 10;
    int32_t avg_x10 = s_sig.avg_x10;
    uint32_t started_ms = s_sig.started_ms;
    uint32_t session_generation = s_sig.session_generation;
    mtk_sink_t sink = s_sig.ctx.sink;
    ble_unlock();

    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_signal_meter_update_ev_t ev = {0};
        ev.operation_token = token; ev.raw_rssi = rssi; ev.avg_rssi = (int8_t)(avg_x10 / 10);
        ev.category = rssi_category(rssi); ev.age_ms = (uint16_t)((uint32_t)now_ms() - started_ms);
        ev.is_random_address = is_random;
        sink.emit_event(sink.user, token, "SIGNAL_METER_UPDATE", &ev, &mtk_signal_meter_update_ev_t_desc);
        mtk_op_end_publish_guard();
    }
    signal_meter_sample_finished(token);
}

static void handle_signal_meter_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                        const uint8_t *req_bytes, size_t req_len) {
    mtk_signal_meter_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, SIGNAL_METER_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_signal_meter_status_resp_t r; r.state = (uint8_t)snap.state; r.is_random_address = 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_signal_meter_status_resp_t_desc);
}

static void handle_signal_meter_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                      const uint8_t *req_bytes, size_t req_len) {
    mtk_signal_meter_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    /* Family gate (SIGNAL_METER_START) -- the transition is family-gated too, so
     * a foreign token never releases MTK_ARB_SM or clears s_sig for an operation
     * it does not name. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, SIGNAL_METER_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Gated on mtk_op_transition_by_token_family's own return value, matching
     * every other STOP-vs-natural-completion race fixed. */
    if (mtk_op_transition_by_token_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, SIGNAL_METER_START_OPCODE, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
        /* Atomic ownership-checked release. */
        signal_meter_cancel(req.operation_token);
    }
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, BLE_SERVICE_ID, SIGNAL_METER_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_signal_meter_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_signal_meter_stop_resp_t_desc);
}

/* GATT client (single active connection, matched to the arbiter's
 * single-active-BLE-class model) --------------------- */

typedef struct { uint32_t connection_token; uint16_t attr_handle; uint8_t mode; uint8_t active; } gatt_sub_t;
typedef struct { uint16_t start_handle; uint16_t end_handle; } gatt_svc_range_t;
/* Every field below is now protected by ble_lock/ble_unlock -- this struct
 * previously had NO lock of any kind, a genuine data race between request
 * handlers (a synchronous dispatch) and mtek_ble_gatt_tick (a separate,
 * independently-scheduled periodic task on a real target) that both read/write
 * it. session_generation is the ORIGINATING connection's own
 * (mtk_request_ctx_t's own field, 0 for every non-native-SPI adapter -- never
 * fenced), stored once at handle_gatt_connect's own successful- connect time;
 * mtek_ble_gatt_tick's own notify-delivery path validates against THIS stored
 * value via mtk_op_begin_publish_guard, never a caller's own ctx (the tick has
 * none). */
static struct {
    uint32_t connection_token; uint16_t vendor_handle; uint8_t connected;
    /* dropped_notification_count is now read live from the HAL, not tracked here
     * -- see handle_gatt_status's own doc comment. */
    uint8_t write_pending;
    gatt_sub_t subs[GATT_MAX_SUBS];
    mtk_request_ctx_t ctx;
    uint32_t session_generation;
    /* Every service range seen across all GATT_DISCOVER calls on the
     * current connection (reset on connect/disconnect) -- lets SUBSCRIBE/
     * UNSUBSCRIBE bound their CCCD descriptor search to the real
     * containing service's end_handle instead of an arbitrary window. */
    gatt_svc_range_t svc_ranges[GATT_MAX_SVCS];
    uint8_t svc_range_count;
} s_gatt;

/* The prior design (a standalone gatt_find_containing_end_handle, called AFTER
 * gatt_snapshot_identity had already released the lock) captured
 * connection_token/vendor_handle in one lock acquisition and then looked up the
 * CCCD-bounding end_handle in a SEPARATE, later lock acquisition, against
 * whatever s_gatt.svc_ranges[] happened to hold at THAT instant -- with no
 * correlation back to the identity already captured. A disconnect (and a real,
 * in-flight reconnect -- s_gatt is memset, including svc_ranges[], by
 * handle_gatt_connect's own successful-connect path) landing in the gap between
 * the two calls could make this function return an end_handle that genuinely
 * belongs to a DIFFERENT, newer connection's own discovered ranges, while the
 * caller still believes it is bounding a CCCD search for the connection it
 * originally validated. Both fields are now captured together, under ONE lock
 * acquisition, so "this connection_token names this vendor_handle" and "this
 * attr_handle falls inside this connection's own discovered range" are proven
 * atomically, as a single fact about one connection instance -- never two facts
 * about possibly-different ones. Returns 0 (identity itself invalid: wrong or no
 * connection) or 1 with *out_end_handle set to 0 (identity valid, but
 * attr_handle was never discovered on THIS connection -- the caller must fail
 * PROTOCOL_ERROR, never guess a window) or a real, nonzero end_handle. */
static int gatt_snapshot_identity_with_range(uint32_t connection_token, uint16_t attr_handle,
                                              uint16_t *out_vendor_handle, uint16_t *out_end_handle,
                                              uint32_t *out_session_generation) {
    ble_lock();
    int ok = s_gatt.connected && s_gatt.connection_token == connection_token;
    if (ok) {
        if (out_vendor_handle) *out_vendor_handle = s_gatt.vendor_handle;
        if (out_session_generation) *out_session_generation = s_gatt.session_generation;
        uint16_t end_handle = 0;
        for (unsigned i = 0; i < s_gatt.svc_range_count; i++) {
            if (attr_handle >= s_gatt.svc_ranges[i].start_handle && attr_handle <= s_gatt.svc_ranges[i].end_handle) {
                end_handle = s_gatt.svc_ranges[i].end_handle;
                break;
            }
        }
        if (out_end_handle) *out_end_handle = end_handle;
    }
    ble_unlock();
    return ok;
}

/* Every handler below that needs vendor_handle for a HAL call previously read
 * `s_gatt.vendor_handle` in its OWN separate, unlocked statement, well after
 * gatt_check_conn's own (already-released) lock had confirmed connection_token
 * matched -- a genuine TOCTOU: a disconnect-then-reconnect landing in that
 * window (the BLE stack is free to reuse a small vendor_handle integer for a
 * brand-new, unrelated connection) could hand a HAL call the WRONG physical
 * connection's own canonical vendor_handle even though connection_token still
 * nominally matched at the instant of the earlier check. Snapshots
 * connection_token (the caller's own, already known) together with vendor_handle
 * and session_generation in ONE lock acquisition -- the only atomic proof "this
 * connection_token's own connection is what vendor_handle names, right now". */
static int gatt_snapshot_identity(uint32_t connection_token, uint16_t *out_vendor_handle, uint32_t *out_session_generation) {
    ble_lock();
    int ok = s_gatt.connected && s_gatt.connection_token == connection_token;
    if (ok) {
        if (out_vendor_handle) *out_vendor_handle = s_gatt.vendor_handle;
        if (out_session_generation) *out_session_generation = s_gatt.session_generation;
    }
    ble_unlock();
    return ok;
}

/* Re-validates the FULL connection identity (connection_token AND
 * vendor_handle -- never vendor_handle alone) immediately after a
 * blocking HAL call returns, before publishing its result or mutating any
 * further shared state. A mismatch means a disconnect (and, on a real
 * target, possibly an immediate reconnect that happened to be handed this
 * exact vendor_handle back by the stack) occurred while the HAL call was
 * in flight -- its result belongs to a connection that is no longer this
 * one's own to touch. */
static int gatt_revalidate_identity(uint32_t connection_token, uint16_t vendor_handle) {
    ble_lock();
    int ok = s_gatt.connected && s_gatt.connection_token == connection_token && s_gatt.vendor_handle == vendor_handle;
    ble_unlock();
    return ok;
}

static void handle_gatt_connect(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_connect_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Admission is now held under the same lock that serializes
     * mtk_core_bump_session_ generation, from final session validation through
     * arbiter ownership publication and the ACCEPTED response, so a peer-session
     * reset can never observe MTK_ARB_GC owned by a not-yet-real token -- see
     * mtk_op_begin_admission_guard's own doc comment (mtek_core.h). */
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* The identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the (potentially slow/blocking) gatt_connect HAL call below. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    /* Publishes the REAL token in the SAME arbiter call -- never the
     * acquire(class, 0) + later force_transfer(class, token) sequence
     * that let the arbiter observably report MTK_ARB_GC owned by token 0. */
    if (mtk_arbiter_acquire(MTK_ARB_GC, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());
    mtk_gatt_connect_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_gatt_connect_resp_t_desc);
    mtk_op_end_admission_guard();

    uint16_t vh = 0;
    int rc = s_hal && s_hal->gatt_connect ? s_hal->gatt_connect(to_hal_mac(req.target.addr), req.target.addr_type, 30000, &vh) : -1;
    /* gatt_connect above can block for up to its own 30000ms timeout with no
     * cancel hook of its own (mtek_ble_hal.h has none); everything externally
     * visible below is gated on actually WINNING this transition -- if a
     * concurrent peer-session invalidation already claimed/evicted this token
     * while the call was blocked, this path must never publish s_gatt as
     * connected, emit an event, or release an arbiter class that may by now
     * belong to a completely different, newer GATT_CONNECT. */
    uint8_t connected_now = (rc == 0);
    mtk_op_state_t final_state = connected_now ? MTK_OPS_COMPLETED : MTK_OPS_TIMED_OUT;
    uint8_t final_status = connected_now ? MTK_STATUS_OK : MTK_STATUS_TIMEOUT;
    /* + "final P0 concurrency-closure round", issue 1: both arbiter release call
     * sites below now go through mtk_arbiter_ release_if_owner -- the ownership
     * check and the release happen atomically, in ONE lock acquisition, instead
     * of a separate `mtk_ arbiter_active_token == id.token` check followed by a
     * separate `mtk_arbiter_release` call. In ordinary operation this worker
     * still legitimately holds the class whenever it reaches either site
     * (nothing else ever releases MTK_ARB_GC out from under a still-live token
     * -- the peer-reset canceller's own mid-connect branch deliberately never
     * releases it either, see mtek_ble_cancel_active_ for_peer_reset), so this
     * changes no normal-path behavior; it is defense-in-depth against ever
     * releasing a newer GATT_CONNECT's own lease should that invariant change.
     *
     * Winning this transition alone only proves nobody else finalized this token
     * BEFORE this exact instant -- a peer-session reset can still land in the
     * window between winning and actually publishing/emitting below. A single
     * point-in-time re-check cannot close that window either (it only proves the
     * generation had not YET changed the instant it ran).
     * mtk_op_begin_publish_guard, held across the whole publish, makes "check"
     * and "the generation actually changing" mutually exclusive instead -- see
     * its own doc comment in mtek_core.h. `won` still stands regardless (the
     * transition already happened and must not be undone; the
     * connected_now/final_state teardown logic below is unaffected). */
    int won = mtk_op_transition_by_token(id.token, id.boot_epoch, final_state, final_status, now_ms());
    /* M3 correction (nested-lock audit, following the real capture-service
     * AB-BA deadlock): gatt_op_lease_lock must be acquired BEFORE
     * mtk_op_begin_publish_guard (pub_lock), never after -- the same
     * global nested-lock order mtek_capture_logic.c's handle_capture_
     * start/mtek_capture_channel_hop_tick now establish for cap_action_
     * lock. Acquiring it AFTER the guard opens (the previous shape here)
     * is exactly the hazardous pattern that deadlocked handle_capture_
     * start against mtek_capture_channel_hop_tick: a path that already
     * holds pub_lock and blocks waiting for the lease, racing a path that
     * already holds the lease and blocks waiting for pub_lock. No such
     * reverse-order path exists for gatt_op_lease_lock today (mtek_ble_
     * gatt_tick never acquires it), so this was not yet a live deadlock
     * -- but it violated the established order and was a latent hazard
     * against any future path that acquires the lease before pub_lock.
     * Acquired unconditionally whenever this call could reach the one
     * branch that ever needs it (won && connected_now) -- harmless to
     * hold briefly and release unused on any other path below -- and
     * released the instant the guarded section ends, never held across
     * the emit_event call. */
    int need_lease = won && connected_now;
    if (need_lease) gatt_op_lease_lock();
    int guard_open = won && mtk_op_begin_publish_guard(ctx->session_generation);
    if (guard_open) {
        mtk_gatt_connect_complete_ev_t ev = {0}; ev.operation_token = id.token;
        if (connected_now) {
            /* ble_lock now protects this publish against mtek_ble_gatt_tick's
             * own concurrent read/write of the SAME fields (a genuine,
             * previously-unguarded data race -- see this struct's own doc
             * comment above). session_generation is stored here, once, for
             * mtek_ble_gatt_tick's own later notify-delivery guard to validate
             * against (that tick has no request ctx of its own).
             *
             * The gatt_op_lease_lock lease (acquired above, before the publish
             * guard) is what actually excludes some OTHER, still-live operation
             * for the connection this reinit is about to replace -- if it is
             * currently mid-flight (already past its own final identity
             * validation, blocked inside or just returned from a blocking HAL
             * call), this reinit genuinely cannot proceed until that operation
             * fully completes (commits, or discovers it is stale and bails) and
             * releases the SAME lease, so it can never land while a HAL
             * round-trip using the SAME (about to be reused) vendor_handle is
             * still genuinely in flight. */
            ble_lock();
            memset(&s_gatt, 0, sizeof(s_gatt));
            s_gatt.connection_token = id.token; s_gatt.vendor_handle = vh; s_gatt.connected = 1; s_gatt.ctx = *ctx;
            s_gatt.session_generation = ctx->session_generation;
            ble_unlock();
            ev.status = MTK_STATUS_OK; ev.connection_token = id.token;
        } else {
            ev.status = MTK_STATUS_TIMEOUT;
            mtk_arbiter_release_if_owner(MTK_ARB_GC, id.token);
        }
        ctx->sink.emit_event(ctx->sink.user, id.token, "GATT_CONNECT_COMPLETE", &ev, &mtk_gatt_connect_complete_ev_t_desc);
        mtk_op_end_publish_guard();
    } else {
        /* Lost the race, OR won it but the session went stale before we could
         * publish (e.g. a peer-session reset while gatt_connect above was
         * blocked, or landing in the narrow post-win window): the arbiter lease
         * this operation held is no longer ours to keep, and must never be left
         * orphaned. If the HAL itself genuinely connected in the meantime, that
         * real connection is now untracked by the canonical layer and could
         * never be disconnected again otherwise -- tear it back down immediately
         * rather than leave it silently attached. gatt_disconnect(vh) uses THIS
         * worker's own local vendor handle from its own gatt_connect call, never
         * shared state, so it can never disconnect a different, newer
         * connection. */
        if (connected_now && s_hal && s_hal->gatt_disconnect) s_hal->gatt_disconnect(vh);
        mtk_arbiter_release_if_owner(MTK_ARB_GC, id.token);
    }
    if (need_lease) gatt_op_lease_unlock();
}

static void handle_gatt_disconnect(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_disconnect_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    ble_lock();
    int do_disconnect = s_gatt.connected && s_gatt.connection_token == req.connection_token;
    uint16_t vendor_handle = s_gatt.vendor_handle;
    if (do_disconnect) s_gatt.connected = 0;
    ble_unlock();
    if (do_disconnect) {
        if (s_hal && s_hal->gatt_disconnect) s_hal->gatt_disconnect(vendor_handle);
        /* Atomic ownership-checked release (mtk_arbiter_release_if_owner)
         * instead of an unconditional mtk_arbiter_release, for the same
         * defense-in- depth reason as handle_gatt_connect's own two release
         * sites. */
        mtk_arbiter_release_if_owner(MTK_ARB_GC, req.connection_token);
    }
    respond_empty(ctx, MTK_STATUS_OK);
}

static void handle_gatt_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_gatt_status_resp_t r;
    ble_lock();
    r.connected = (s_gatt.connected && s_gatt.connection_token == req.connection_token) ? 1 : 0;
    r.write_pending = s_gatt.write_pending;
    ble_unlock();
    /* Read live from the HAL (the party that actually owns the bounded notify
     * queue and observes an overflow), not a service-layer field nothing ever
     * incremented. NULL-safe for a HAL that doesn't model this. */
    r.dropped_notification_count = (s_hal && s_hal->gatt_notify_dropped_count) ? s_hal->gatt_notify_dropped_count() : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_gatt_status_resp_t_desc);
}

static void handle_gatt_discover(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_discover_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Acquired BEFORE the identity snapshot below and held across the whole HAL
     * call + post-HAL revalidation -- see mtek_ble_service.h's own doc comment
     * on mtek_ble_service_set_gatt_op_lease_lock. Blocks a concurrent
     * GATT_CONNECT's own successful-connect reinit from reusing this exact
     * vendor_handle for a replacement connection while this call's own HAL
     * round-trip is still in flight. */
    gatt_op_lease_lock();
    /* The vendor_handle used for the HAL call below is snapshotted together with
     * the connection_token check, in one lock acquisition -- never read
     * separately/unlocked, which could straddle a disconnect-then-reconnect that
     * reuses this exact small integer handle for a different peer. */
    uint16_t vendor_handle;
    if (!gatt_snapshot_identity(req.connection_token, &vendor_handle, NULL)) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    /* Zeroed before any HAL call populates it -- defense in depth on top of the
     * HAL's own fix (mtek_ble_hal_esp32.c's disc_svc_cb), so a
     * differently-implemented HAL (a future adapter, or a host-test fake) that
     * does not itself fully populate every field can never leak uninitialized
     * stack memory onto the wire through this array. */
    mtk_hal_gatt_service_t hal_svcs[GATT_MAX_SVCS]; memset(hal_svcs, 0, sizeof(hal_svcs));
    int n = s_hal && s_hal->gatt_discover ? s_hal->gatt_discover(vendor_handle, hal_svcs, GATT_MAX_SVCS) : 0;
    /* A negative return is this HAL's own genuine, distinct failure signal (see
     * mtek_ble_hal_esp32.c's esp32_gatt_discover doc comment) -- clamping it to
     * 0 here silently turned a real resource/ IO failure into an apparently
     * successful, genuinely-empty discovery indistinguishable from a peer that
     * truly has zero GATT services. `hal_failed` is checked after the identity
     * re-validation below (matching this file's own established "identity
     * mismatch always takes priority over the HAL's own result" ordering, e.g.
     * handle_ gatt_read/handle_gatt_write), never before it. */
    int hal_failed = (n < 0);
    if (n < 0) n = 0;
    if ((unsigned)n > GATT_MAX_SVCS) n = GATT_MAX_SVCS;
    /* Re-validate the FULL connection identity (never vendor_handle alone) in
     * the SAME lock acquisition as the svc_ranges[] merge below -- gatt_discover
     * above can block, and a disconnect (or disconnect-then-reconnect reusing
     * this exact vendor_handle) landing while it was in flight must never let
     * this call's own results merge into a DIFFERENT connection's state. */
    ble_lock();
    int still_valid = s_gatt.connected && s_gatt.connection_token == req.connection_token && s_gatt.vendor_handle == vendor_handle;
    if (still_valid && !hal_failed) {
        /* Record every service range this HAL call actually returned
         * (from index 0, not just this page) so SUBSCRIBE/UNSUBSCRIBE can
         * bound their CCCD search correctly regardless of which page the
         * caller requested. */
        for (int k = 0; k < n && s_gatt.svc_range_count < GATT_MAX_SVCS; k++) {
            uint8_t dup = 0;
            for (unsigned j = 0; j < s_gatt.svc_range_count; j++) {
                if (s_gatt.svc_ranges[j].start_handle == hal_svcs[k].start_handle) { dup = 1; break; }
            }
            if (!dup) {
                s_gatt.svc_ranges[s_gatt.svc_range_count].start_handle = hal_svcs[k].start_handle;
                s_gatt.svc_ranges[s_gatt.svc_range_count].end_handle = hal_svcs[k].end_handle;
                s_gatt.svc_range_count++;
            }
        }
    }
    ble_unlock();
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (hal_failed) { respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    uint16_t start = req.start_index; uint8_t max_items = req.max_items ? req.max_items : GATT_MAX_SVCS;
    mtk_gatt_discover_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = start, cnt = 0;
    for (; i < (uint16_t)n && cnt < max_items; i++, cnt++) {
        r.services.items[cnt].uuid.width = hal_svcs[i].uuid_width;
        memcpy(r.services.items[cnt].uuid.value, hal_svcs[i].uuid_value, 16);
        /* A 16-bit UUID's own meaningful bytes are only value[0..1] -- force the
         * rest to a defined 0 here, at the canonical layer, rather than trusting
         * every current and future HAL implementation to have zeroed its own
         * tail correctly (the real bug this closes: mtek_ble_hal_esp32.c's
         * disc_svc_cb once left these bytes as whatever uninitialized/stale
         * memory preceded them). */
        if (hal_svcs[i].uuid_width == 0) memset(r.services.items[cnt].uuid.value + 2, 0, 14);
        r.services.items[cnt].start_handle = hal_svcs[i].start_handle;
        r.services.items[cnt].end_handle = hal_svcs[i].end_handle;
    }
    r.services.count = cnt; r.next_index = (i < (uint16_t)n) ? i : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_gatt_discover_resp_t_desc);
}

/* New, purely additive opcodes (GATT_DISCOVER_CHARS/GATT_DISCOVER_DESCS) --
 * GATT_DISCOVER above only ever enumerated services. `start_handle`/`end_handle`
 * must come from a real prior discovery result (a service's own range for
 * GATT_DISCOVER_CHARS, or a characteristic's val_handle+1..containing- service's
 * end_handle for GATT_DISCOVER_DESCS -- the same bounded- range contract
 * gatt_subscribe's own CCCD search already uses); this handler never widens or
 * guesses a range beyond what the caller supplies. */
static void handle_gatt_discover_chars(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                        const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_discover_chars_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_discover's own doc comment above. */
    gatt_op_lease_lock();
    /* See gatt_snapshot_identity's own doc comment -- vendor_handle for the HAL
     * call below must never be read separately/unlocked from the
     * connection_token check. */
    uint16_t vendor_handle;
    if (!gatt_snapshot_identity(req.connection_token, &vendor_handle, NULL)) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_hal_gatt_char_t hal_chrs[GATT_MAX_SVCS]; memset(hal_chrs, 0, sizeof(hal_chrs));
    int n = (s_hal && s_hal->gatt_discover_chars)
                ? s_hal->gatt_discover_chars(vendor_handle, req.start_handle, req.end_handle, hal_chrs, GATT_MAX_SVCS)
                : 0;
    /* See handle_gatt_discover's own doc comment -- a negative return is a
     * genuine, distinct HAL failure, never silently clamped into an apparently
     * successful empty result. */
    int hal_failed = (n < 0);
    if (n < 0) n = 0;
    if ((unsigned)n > GATT_MAX_SVCS) n = GATT_MAX_SVCS;
    /* This call publishes no shared state, but its own RESPONSE content is
     * meaningless (about whatever connection this vendor_handle happened to
     * reach) once the connection this request was actually issued against is
     * gone -- revalidate the full identity before trusting/responding with it. */
    int still_valid = gatt_revalidate_identity(req.connection_token, vendor_handle);
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (hal_failed) { respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    uint16_t start = req.start_index; uint8_t max_items = req.max_items ? req.max_items : GATT_MAX_SVCS;
    mtk_gatt_discover_chars_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = start, cnt = 0;
    for (; i < (uint16_t)n && cnt < max_items; i++, cnt++) {
        r.items.items[cnt].uuid.width = hal_chrs[i].uuid_width;
        memcpy(r.items.items[cnt].uuid.value, hal_chrs[i].uuid_value, 16);
        if (hal_chrs[i].uuid_width == 0) memset(r.items.items[cnt].uuid.value + 2, 0, 14);
        r.items.items[cnt].def_handle = hal_chrs[i].def_handle;
        r.items.items[cnt].val_handle = hal_chrs[i].val_handle;
        r.items.items[cnt].properties = hal_chrs[i].properties;
    }
    r.items.count = cnt; r.next_index = (i < (uint16_t)n) ? i : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_gatt_discover_chars_resp_t_desc);
}

static void handle_gatt_discover_descs(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                        const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_discover_descs_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_discover's own doc comment above. */
    gatt_op_lease_lock();
    uint16_t vendor_handle;
    if (!gatt_snapshot_identity(req.connection_token, &vendor_handle, NULL)) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_hal_gatt_desc_t hal_dscs[GATT_MAX_SVCS]; memset(hal_dscs, 0, sizeof(hal_dscs));
    int n = (s_hal && s_hal->gatt_discover_descs)
                ? s_hal->gatt_discover_descs(vendor_handle, req.start_handle, req.end_handle, hal_dscs, GATT_MAX_SVCS)
                : 0;
    /* See handle_gatt_discover's own doc comment -- a negative return is a
     * genuine, distinct HAL failure, never silently clamped into an apparently
     * successful empty result. */
    int hal_failed = (n < 0);
    if (n < 0) n = 0;
    if ((unsigned)n > GATT_MAX_SVCS) n = GATT_MAX_SVCS;
    int still_valid = gatt_revalidate_identity(req.connection_token, vendor_handle);
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (hal_failed) { respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    uint16_t start = req.start_index; uint8_t max_items = req.max_items ? req.max_items : GATT_MAX_SVCS;
    mtk_gatt_discover_descs_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = start, cnt = 0;
    for (; i < (uint16_t)n && cnt < max_items; i++, cnt++) {
        r.items.items[cnt].uuid.width = hal_dscs[i].uuid_width;
        memcpy(r.items.items[cnt].uuid.value, hal_dscs[i].uuid_value, 16);
        if (hal_dscs[i].uuid_width == 0) memset(r.items.items[cnt].uuid.value + 2, 0, 14);
        r.items.items[cnt].handle = hal_dscs[i].handle;
    }
    r.items.count = cnt; r.next_index = (i < (uint16_t)n) ? i : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_gatt_discover_descs_resp_t_desc);
}

static void handle_gatt_read(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                              const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_read_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_discover's own doc comment above. */
    gatt_op_lease_lock();
    uint16_t vendor_handle;
    if (!gatt_snapshot_identity(req.connection_token, &vendor_handle, NULL)) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_gatt_read_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t len = 0;
    int rc = s_hal && s_hal->gatt_read ? s_hal->gatt_read(vendor_handle, req.handle, r.data.data, 512, &len) : -1;
    /* A disconnect (or disconnect-then- reconnect reusing this exact
     * vendor_handle) while gatt_read above was blocked must never let its result
     * be trusted/returned as if it genuinely came from THIS connection_token's
     * own peer. */
    int still_valid = gatt_revalidate_identity(req.connection_token, vendor_handle);
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (rc != 0) { respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    r.data.len = len;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_gatt_read_resp_t_desc);
}

static void handle_gatt_write(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                               const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_write_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_discover's own doc comment above. */
    gatt_op_lease_lock();
    uint16_t vendor_handle;
    if (!gatt_snapshot_identity(req.connection_token, &vendor_handle, NULL)) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    int rc = s_hal && s_hal->gatt_write ? s_hal->gatt_write(vendor_handle, req.handle, req.data.data, req.data.len, req.with_response) : -1;
    int still_valid = gatt_revalidate_identity(req.connection_token, vendor_handle);
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    respond_empty(ctx, rc == 0 ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR);
}

static void handle_gatt_subscribe(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_subscribe_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Every notification this subscription could ever produce is only ever
     * delivered by mtek_ble_gatt_tick's own notify-poll branch, itself only ever
     * called from main/app_main.c's own ble_tick_task -- accepting a
     * subscription that can never actually notify would be dishonest; refuse
     * before ever touching the peer's own CCCD. */
    if (!s_tick_task_ready) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    /* Acquired BEFORE the identity snapshot below and held across the whole HAL
     * call + post-HAL revalidation/commit -- see mtek_ble_service.h's own doc
     * comment on mtek_ble_service_set_gatt_ op_lease_lock. */
    gatt_op_lease_lock();
    /* connection_token, vendor_handle, AND the CCCD-bounding end_handle are now
     * captured together, in ONE lock acquisition -- see
     * gatt_snapshot_identity_with_range's own doc comment for why the service
     * range must never be looked up separately from the identity that produced
     * it. */
    uint16_t vendor_handle, end_handle;
    if (!gatt_snapshot_identity_with_range(req.connection_token, req.handle, &vendor_handle, &end_handle, NULL)) {
        gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    /* The CCCD search must be bounded by the real containing service's
     * end_handle from an actual GATT_DISCOVER call on this connection --
     * never an arbitrary window or an attr_handle+1 fallback. If this
     * handle wasn't seen in any discovered service range, fail cleanly
     * (PROTOCOL_ERROR: the caller must discover before subscribing). */
    if (!end_handle) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* This is only a fast-fail PRE-check, under lock (the original scan ran
     * completely unlocked, racing concurrent GATT_ SUBSCRIBE/_UNSUBSCRIBE calls
     * and mtek_ble_gatt_tick) -- preserved so a subs[] table that is already
     * full is rejected WITHOUT ever writing to the peer's own CCCD, exactly as
     * before. The real, atomic slot claim happens after the HAL call below, in
     * the SAME lock acquisition as the post-call identity re-validation: a slot
     * that looks free here could be raced by a concurrent subscribe before the
     * HAL call returns, and picking it here, then blindly writing it later, is
     * exactly the non-atomic "select, then separately commit" pattern that let
     * two concurrent subscribes silently collide. */
    ble_lock();
    int any_free = 0;
    for (int i = 0; i < GATT_MAX_SUBS; i++) if (!s_gatt.subs[i].active) { any_free = 1; break; }
    ble_unlock();
    if (!any_free) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NO_MEMORY); return; }
    /* An absent HAL/
     * entry point (s_hal itself unavailable, e.g. mtek_ble_hal_esp32_init
     * failed at boot -- see main/app_main.c) is a real resource failure,
     * not an implicit "nothing to subscribe to"; rc stays -1 and falls
     * through to the same IO_ERROR report below as any other HAL failure. */
    int rc = s_hal && s_hal->gatt_subscribe ? s_hal->gatt_subscribe(vendor_handle, req.handle, end_handle, req.mode) : -1;
    /* The accepted baseline requires a DISTINCT "no CCCD" error, never conflated
     * with an ordinary write failure -- MTK_STATUS_NOT_FOUND (the CCCD
     * descriptor itself was not found) vs MTK_STATUS_IO_ERROR (the CCCD write
     * failed for some other real reason). */
    if (rc == MTK_HAL_GATT_SUBSCRIBE_NO_CCCD) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (rc != 0) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }
    /* Atomic: re-validate the FULL connection identity AND claim a slot in ONE
     * lock acquisition -- a disconnect (or disconnect-then-reconnect reusing
     * this exact vendor_handle) while gatt_subscribe above was blocked must
     * never insert an entry into a DIFFERENT connection's subs[] table, and a
     * slot that was free at the pre-check above must be re-verified here,
     * atomically with the write that claims it. */
    ble_lock();
    int still_valid = s_gatt.connected && s_gatt.connection_token == req.connection_token && s_gatt.vendor_handle == vendor_handle;
    int slot = -1;
    if (still_valid) {
        for (int i = 0; i < GATT_MAX_SUBS; i++) if (!s_gatt.subs[i].active) { slot = i; break; }
        if (slot >= 0) {
            s_gatt.subs[slot].connection_token = req.connection_token;
            s_gatt.subs[slot].attr_handle = req.handle;
            s_gatt.subs[slot].mode = req.mode;
            s_gatt.subs[slot].active = 1;
        }
    }
    ble_unlock();
    gatt_op_lease_unlock();
    if (!still_valid) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    if (slot < 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); return; }
    respond_empty(ctx, MTK_STATUS_OK);
}

/* The prior design had FOUR distinct defects. (1) gatt_check_conn (its own
 * separate lock acquisition) validated connection_token, then RELEASED the lock,
 * then a SECOND, later lock acquisition read s_gatt.vendor_handle
 * unconditionally -- a genuine check-then-use TOCTOU, structurally identical to
 * the one gatt_snapshot_ identity was introduced to close for
 * discover/read/write/subscribe. (2) the end_handle lookup
 * (gatt_find_containing_end_handle) was a THIRD, separate lock acquisition with
 * no identity correlation of its own -- the same "service range retrieved
 * separately from the connection identity" hazard subscribe had. (3) the local
 * subs[] entry was cleared UNCONDITIONALLY before the HAL call even ran, so a
 * HAL failure left local state claiming "unsubscribed" while the peer's own CCCD
 * may still be armed and notifying. (4) the response was always MTK_STATUS_OK
 * regardless of the HAL call's own (entirely discarded) return value -- a silent
 * false success. Fixed by mirroring handle_gatt_subscribe's own three-phase
 * shape exactly: one atomic snapshot (identity + range together), a locked
 * pre-check for a genuinely active local subscription (never issue a physical
 * CCCD write for a handle that isn't actually subscribed -- preserves the prior
 * no-op-success behavior for that case), the HAL call itself unlocked, then ONE
 * final lock acquisition that re-validates the FULL identity again (never
 * vendor_handle alone) before clearing any local state -- a stale request whose
 * identity changed mid-call (a disconnect, or disconnect-then-reconnect reusing
 * this exact vendor_handle) can therefore never clear a REPLACEMENT connection's
 * own subscription table, and a genuine HAL failure is reported honestly
 * (IO_ERROR) rather than silently discarded into an OK. */
static void handle_gatt_unsubscribe(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                     const uint8_t *req_bytes, size_t req_len) {
    mtk_gatt_unsubscribe_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* See handle_gatt_subscribe's own doc comment above -- acquired BEFORE the
     * identity snapshot and held across the whole HAL call + post-HAL
     * revalidated commit. */
    gatt_op_lease_lock();
    uint16_t vendor_handle, end_handle;
    if (!gatt_snapshot_identity_with_range(req.connection_token, req.handle, &vendor_handle, &end_handle, NULL)) {
        gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    if (!end_handle) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }

    /* Fast-fail pre-check only (matching subscribe's own any_free
     * pre-check pattern): a handle never subscribed (or already
     * unsubscribed) on this exact connection remains a no-op success,
     * never a spurious physical write to the peer. The real, atomic
     * commit is the re-validated clear below, after the HAL call. */
    ble_lock();
    uint8_t any_active = 0;
    for (int i = 0; i < GATT_MAX_SUBS; i++) {
        if (s_gatt.subs[i].active && s_gatt.subs[i].connection_token == req.connection_token && s_gatt.subs[i].attr_handle == req.handle) { any_active = 1; break; }
    }
    ble_unlock();
    if (!any_active) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_OK); return; }

    int rc = s_hal && s_hal->gatt_unsubscribe ? s_hal->gatt_unsubscribe(vendor_handle, req.handle, end_handle) : -1;
    if (rc != 0) { gatt_op_lease_unlock(); respond_empty(ctx, MTK_STATUS_IO_ERROR); return; }

    ble_lock();
    if (s_gatt.connected && s_gatt.connection_token == req.connection_token && s_gatt.vendor_handle == vendor_handle) {
        for (int i = 0; i < GATT_MAX_SUBS; i++) {
            if (s_gatt.subs[i].active && s_gatt.subs[i].connection_token == req.connection_token && s_gatt.subs[i].attr_handle == req.handle) {
                s_gatt.subs[i].active = 0;
            }
        }
    }
    ble_unlock();
    gatt_op_lease_unlock();
    respond_empty(ctx, MTK_STATUS_OK);
}

/* See mtek_ble_service.h's own doc comment for the full rationale. Set by
 * mtek_ble_gatt_tick's remote- disconnect branch below; cleared by whichever
 * caller (the UART adapter's own background poll) consumes it first. */
static uint8_t s_gatt_remote_disconnect_pending;
/* The real reason captured alongside the pending flag above -- never a
 * hard-coded placeholder. */
static uint8_t s_gatt_remote_disconnect_reason;

uint8_t mtek_ble_gatt_take_remote_disconnect_notice(uint8_t *reason_out) {
    /* This accessor and mtek_ble_gatt_tick's remote-disconnect branch below run
     * on separate tasks on a real target (the UART adapter's own background poll
     * vs. the BLE tick task) -- both statics need the same dedicated lock as
     * s_gatt. */
    ble_lock();
    uint8_t pending = s_gatt_remote_disconnect_pending;
    uint8_t reason = s_gatt_remote_disconnect_reason;
    if (pending) s_gatt_remote_disconnect_pending = 0;
    ble_unlock();
    if (!pending) return 0;
    if (reason_out) *reason_out = reason;
    return 1;
}

void mtek_ble_gatt_tick(void) {
    /* s_gatt previously had NO lock of any kind -- a genuine data race between
     * this tick (a separate, independently-scheduled task on a real target,
     * main/app_main.c's own ble_tick_task) and handle_gatt_connect/
     * _disconnect/_subscribe/etc.'s own concurrent mutation of the SAME fields.
     * Snapshotted here, then every HAL/sink call below runs UNLOCKED and (for
     * the notify path) un-guarded until the final validated publish -- never
     * hold a lock or the publish guard across an external call. */
    /* connection_token is now part of this snapshot too, and part of BOTH
     * re-validations below -- vendor_handle alone cannot distinguish this
     * connection from a DIFFERENT one the HAL/stack later assigns the same small
     * integer handle to after a disconnect-then-reconnect race. */
    ble_lock();
    uint8_t connected = s_gatt.connected;
    uint16_t vendor_handle = s_gatt.vendor_handle;
    uint32_t connection_token = s_gatt.connection_token;
    ble_unlock();
    if (!connected || !s_hal) return;

    /* "Remote disconnect is ignored, leaving connection/arbiter state stale."
     * Real cleanup -- releases the GC arbiter lease and clears connected-state
     * bookkeeping -- the moment a genuine peer-initiated disconnect is observed,
     * not only the next time some unrelated call happens to fail against the
     * now-dead connection. There is no confirmed wire event for this in the
     * accepted facts package (only the caller-driven GATT_DISCONNECT
     * request/response exists), so this fixes the STATE going stale; a
     * subsequent GATT_STATUS/any GATT_* call now correctly observes the
     * disconnect via s_gatt.connected (checked by every handler's own identity
     * snapshot), exactly as if the caller had issued GATT_ DISCONNECT itself.
     * also sets s_gatt_remote_disconnect_pending so the factory UART adapter can
     * print its own frozen transcript line for this case too (see
     * mtek_ble_service.h's doc comment on the accessor above -- deliberately not
     * routed through emit_event, so this branch needs no publish guard: nothing
     * is ever emitted). */
    uint8_t disc_reason = 0;
    if (s_hal->gatt_poll_disconnected && s_hal->gatt_poll_disconnected(vendor_handle, &disc_reason)) {
        ble_lock();
        /* Re-validate -- s_gatt could have already been
         * disconnected-and-reconnected for a wholly different connection since
         * the snapshot above (a concurrent GATT_DISCONNECT + a brand-new
         * GATT_CONNECT, both genuinely possible while this HAL poll call was in
         * flight), possibly reusing this exact vendor_handle for the new one. A
         * mismatch on EITHER field means this stale disconnect notice no longer
         * applies to whatever connection s_gatt now holds. */
        if (s_gatt.connected && s_gatt.connection_token == connection_token && s_gatt.vendor_handle == vendor_handle) {
            s_gatt.connected = 0;
            ble_unlock();
            /* Atomic ownership- checked release, matching every other GC release
             * site in this file. */
            mtk_arbiter_release_if_owner(MTK_ARB_GC, connection_token);
            ble_lock();
            s_gatt_remote_disconnect_pending = 1;
            s_gatt_remote_disconnect_reason = disc_reason;
            ble_unlock();
        } else {
            ble_unlock();
        }
        return;
    }
    if (!s_hal->gatt_poll_notify) return;
    uint16_t attr_handle = 0, len = 0; uint8_t data[32];
    if (!s_hal->gatt_poll_notify(vendor_handle, &attr_handle, data, &len)) return;

    ble_lock();
    /* Re-validate: still the SAME connection this tick started observing
     * (not a stale notify landing after this connection was torn down and
     * possibly replaced by a brand-new one that reused this exact
     * vendor_handle -- Round 8 item 2, full identity, never vendor_handle
     * alone)? */
    if (!s_gatt.connected || s_gatt.connection_token != connection_token || s_gatt.vendor_handle != vendor_handle) { ble_unlock(); return; }
    int sub_idx = -1;
    for (int i = 0; i < GATT_MAX_SUBS; i++) {
        if (s_gatt.subs[i].active && s_gatt.subs[i].attr_handle == attr_handle) { sub_idx = i; break; }
    }
    if (sub_idx < 0) { ble_unlock(); return; }
    uint8_t mode = s_gatt.subs[sub_idx].mode;
    uint32_t session_generation = s_gatt.session_generation;
    mtk_sink_t sink = s_gatt.ctx.sink;
    ble_unlock();

    /* mtk_op_begin_publish_guard, held across the whole publish, closes
     * the window between the re-validation above and actually emitting
     * -- a peer-session reset landing exactly here must still never let
     * this notification reach the wire under the OLD session. See its
     * own doc comment in mtek_core.h. GATT connections are not operation-
     * token-addressed for their ongoing lifetime (002-canonical-core-
     * contract.md's own carve-out), so there is no separate "won" gate
     * here -- the guard alone decides. */
    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_gatt_value_event_ev_t ev = {0};
        ev.connection_token = connection_token; ev.handle = attr_handle; ev.mode = mode;
        ev.data.len = len > 32 ? 32 : len; memcpy(ev.data.data, data, ev.data.len);
        sink.emit_event(sink.user, connection_token, "GATT_VALUE_EVENT", &ev, &mtk_gatt_value_event_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

/* See mtek_ble_service.h's own doc comment on this function's declaration for
 * the full rationale. BLE_SCAN's own handler always runs its scan and finalizes
 * synchronously within one call (this codebase's HAL contract for it is not a
 * genuinely backgroundable operation the way DEAUTH/HANDSHAKE are), so
 * MTK_ARB_BS should never actually be found still active here -- handled
 * defensively anyway, never assumed. GATT_CONNECT's own operation token is
 * already terminal (COMPLETED) by the time a connection exists
 * (002-canonical-core- contract.md §4.2's own STA_DISCONNECT/GATT_DISCONNECT
 * carve-out: not operation-token-addressed for teardown), so only the connection
 * resource itself (vendor handle, arbiter lease) needs releasing. */
mtk_op_id_t mtek_ble_cancel_active_for_peer_reset(void) {
    mtk_op_id_t id = {0, 0};
    /* Coherent-reader fix: one snapshot instead of a separate class read
     * and a separate token read. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    mtk_arbiter_class_t active = snap.cls;
    if (active != MTK_ARB_BS && active != MTK_ARB_BA && active != MTK_ARB_SM && active != MTK_ARB_GC) return id;
    uint32_t tok = snap.token;
    uint32_t epoch = mtk_core_boot_epoch();
    switch (active) {
        case MTK_ARB_BS:
            /* Deliberately does NOT release MTK_ARB_BS here -- the old scan call
             * has no cancel hook and may still genuinely be running on another
             * thread; releasing here would let a brand-new BLE_SCAN_START
             * acquire it and start a second, overlapping scan call.
             * handle_ble_scan_start's own tail (now gated on arbiter TOKEN
             * ownership, independent of winning this same transition) is the
             * ONLY place that safely releases this class, exactly once the real
             * HAL call returns. */
            if (mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
                id.token = tok; id.boot_epoch = epoch;
            }
            break;
        case MTK_ARB_BA:
            if (mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
                if (s_hal && s_hal->adv_stop) s_hal->adv_stop();
                mtk_arbiter_release(MTK_ARB_BA);
                id.token = tok; id.boot_epoch = epoch;
            }
            break;
        case MTK_ARB_SM:
            if (mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
                /* This write reached s_sig completely unlocked, a genuine,
                 * previously-missed data race against mtek_ble_
                 * signal_meter_tick's own concurrent access, matching every
                 * other s_sig mutation site's own established
                 * locked-and-token-checked pattern (handle_signal_ meter_stop). */
                signal_meter_cancel(tok);
                id.token = tok; id.boot_epoch = epoch;
            }
            break;
        case MTK_ARB_GC:
            /* Two distinct, mutually exclusive cases: (1) still mid- connect
             * (the operation itself is not yet terminal) -- fence it via the
             * SAME token-based transition handle_gatt_ connect's own tail uses
             * (a worker that later loses this race tears down any real HAL-level
             * connection it still manages to form, on its own); or (2) already
             * connected (the operation's own token is already terminal/COMPLETED
             * -- STA_DISCONNECT/GATT_ DISCONNECT carve-out: not
             * operation-token-addressed for teardown), where the live resource
             * is the CONNECTION itself.
             *
             * Case (1) deliberately does NOT release MTK_ARB_GC here -- the old
             * blocking gatt_connect call has no cancel hook and may still
             * genuinely be running; releasing here would let a brand-new
             * GATT_CONNECT acquire it and start a second, overlapping connect
             * against the same peripheral/link. handle_gatt_connect's own tail
             * (now token-ownership-gated, independent of winning this same
             * transition) is the ONLY place that safely releases the class for
             * an in-progress connect. Case (2) is a genuinely idle,
             * already-established connection with no HAL call in flight --
             * gatt_disconnect there completes synchronously, so releasing
             * immediately after it returns is safe and unchanged. */
            if (mtk_op_transition_by_token(tok, epoch, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms())) {
                id.token = tok; id.boot_epoch = epoch;
            } else {
                /* connected/connection_token/ vendor_handle were previously
                 * read, and connected=0 written, completely UNLOCKED here -- a
                 * genuine, previously-missed data race against every other
                 * s_gatt accessor in this file (handle_gatt_connect/_disconnect/
                 * mtek_ble_gatt_tick), all of which already lock. Snapshot under
                 * lock, commit connected=0 under the SAME lock (matching
                 * handle_gatt_disconnect's own established pattern exactly),
                 * then call the HAL and release the arbiter OUTSIDE it. */
                ble_lock();
                uint8_t was_connected = s_gatt.connected;
                uint32_t connection_token = s_gatt.connection_token;
                uint16_t vendor_handle = s_gatt.vendor_handle;
                if (was_connected) s_gatt.connected = 0;
                ble_unlock();
                if (was_connected) {
                    if (s_hal && s_hal->gatt_disconnect) s_hal->gatt_disconnect(vendor_handle);
                    /* Atomic ownership-checked release, matching every other GC
                     * release site in this file. */
                    mtk_arbiter_release_if_owner(MTK_ARB_GC, connection_token);
                }
            }
            break;
        default:
            break;
    }
    return id;
}

/* Dispatch ------------------------------- */

static void mtek_ble_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                               const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: handle_ble_scan_start(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_ble_scan_stop(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_ble_scan_status(ctx, op, req_bytes, req_len); return;
        case 0x0004: handle_ble_scan_results_page(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_ble_device_details(ctx, op, req_bytes, req_len); return;
        case 0x0006: handle_ble_adv_start(ctx, op, req_bytes, req_len); return;
        case 0x0007: handle_ble_adv_stop(ctx, op, req_bytes, req_len); return;
        case 0x0008: handle_ble_adv_status(ctx, op, req_bytes, req_len); return;
        case 0x0009: handle_signal_meter_start(ctx, op, req_bytes, req_len); return;
        case 0x000A: handle_signal_meter_status(ctx, op, req_bytes, req_len); return;
        case 0x000B: handle_signal_meter_stop(ctx, op, req_bytes, req_len); return;
        /* 0x000C-0x0017: BLE compatibility family -- capability_state
         * DISABLED per contract for every profile in Phase 1; the router
         * rejects these before dispatch ever reaches here. */
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

static void mtek_gatt_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: handle_gatt_connect(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_gatt_disconnect(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_gatt_status(ctx, op, req_bytes, req_len); return;
        case 0x0004: handle_gatt_discover(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_gatt_read(ctx, op, req_bytes, req_len); return;
        case 0x0006: handle_gatt_write(ctx, op, req_bytes, req_len); return;
        case 0x0007: handle_gatt_subscribe(ctx, op, req_bytes, req_len); return;
        case 0x0008: handle_gatt_unsubscribe(ctx, op, req_bytes, req_len); return;
        case 0x0009: handle_gatt_discover_chars(ctx, op, req_bytes, req_len); return;
        case 0x000A: handle_gatt_discover_descs(ctx, op, req_bytes, req_len); return;
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

mtk_register_result_t mtek_ble_service_register(void) {
    mtk_register_result_t rc = mtk_router_register(0x0002, mtek_ble_dispatch);
    if (rc != MTK_REGISTER_OK) return rc;
    return mtk_router_register(0x0003, mtek_gatt_dispatch);
}
