/* ESP-NOW service (service 0x0006). Portable: no ESP-IDF dependency, so the
 * whole lifecycle is host-testable against a fake HAL.
 *
 * ESP-NOW transmits on the 2.4GHz Wi-Fi radio, so a session holds the ESPNOW
 * arbiter class for its entire duration and serializes against every Wi-Fi
 * class (schemas.json's arbiter_pairwise_policy). One operation token, one
 * lease, and a single teardown path that user STOP, start failure and peer
 * reset all funnel through -- the same shape the Wi-Fi and capture services
 * already use.
 *
 * Received frames land in a bounded ring filled by the HAL's receive callback
 * and drained by ESPNOW_POLL_RECV, so a peer that never polls cannot cause
 * unbounded buffering: the oldest frame is dropped and counted instead. */
#include "mtek_espnow_hal.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include "mtek_core.h"
#include "mtek_router.h"
#include <string.h>

#define ESPNOW_SERVICE_ID    0x0006
#define ESPNOW_START_OPCODE  0x0001
#define ESPNOW_RING_SLOTS    16
#define ESPNOW_MAX_PAYLOAD   250

static const mtk_espnow_hal_t *s_hal;
void mtek_espnow_set_hal(const mtk_espnow_hal_t *hal) { s_hal = hal; }

static uint64_t (*s_now_ms)(void);
void mtek_espnow_service_init(uint64_t (*now_ms_fn)(void)) { s_now_ms = now_ms_fn; }
static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

static void (*s_lock_fn)(void), (*s_unlock_fn)(void);
void mtek_espnow_set_lock(void (*lock)(void), void (*unlock)(void)) { s_lock_fn = lock; s_unlock_fn = unlock; }
static void en_lock(void)   { if (s_lock_fn) s_lock_fn(); }
static void en_unlock(void) { if (s_unlock_fn) s_unlock_fn(); }

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}

typedef struct {
    mtk_hal_mac6_t src;
    int8_t rssi;
    uint8_t len;
    uint8_t data[ESPNOW_MAX_PAYLOAD];
} espnow_rx_rec_t;

static struct {
    uint32_t token;
    uint32_t boot_epoch;
    uint32_t session_generation;
    mtk_sink_t sink;
    uint8_t active;
    uint8_t channel;
    uint32_t received;
    uint32_t dropped;
    espnow_rx_rec_t ring[ESPNOW_RING_SLOTS];
    unsigned ring_head, ring_count;
} s_en;

/* Drop-oldest on overflow: a full ring means the peer is not draining, and
 * losing the oldest frame is preferable to refusing every new one. */
static void espnow_rx_cb(void *user, mtk_hal_mac6_t src, int8_t rssi, const uint8_t *data, uint8_t len) {
    (void)user;
    if (len > ESPNOW_MAX_PAYLOAD) len = ESPNOW_MAX_PAYLOAD;
    en_lock();
    if (!s_en.active) { en_unlock(); return; }
    s_en.received++;
    if (s_en.ring_count >= ESPNOW_RING_SLOTS) {
        s_en.ring_head = (s_en.ring_head + 1) % ESPNOW_RING_SLOTS;
        s_en.ring_count--;
        s_en.dropped++;
    }
    espnow_rx_rec_t *r = &s_en.ring[(s_en.ring_head + s_en.ring_count) % ESPNOW_RING_SLOTS];
    memset(r, 0, sizeof(*r));
    r->src = src; r->rssi = rssi; r->len = len;
    if (len) memcpy(r->data, data, len);
    s_en.ring_count++;
    en_unlock();
}

void mtek_espnow_service_tick(void) {
    if (s_hal && s_hal->service) s_hal->service();
}

static void espnow_teardown(uint32_t token, uint32_t boot_epoch, uint8_t status) {
    if (!mtk_op_claim_finalization(token, boot_epoch)) return;
    en_lock();
    mtk_sink_t sink = s_en.sink;
    uint32_t session_generation = s_en.session_generation;
    uint32_t sent_ok = 0, send_fail = 0;
    uint32_t received = s_en.received;
    s_en.active = 0;
    s_en.ring_head = 0; s_en.ring_count = 0;
    en_unlock();
    if (s_hal && s_hal->stats) s_hal->stats(&sent_ok, &send_fail);
    /* Always called, including when start never succeeded. */
    if (s_hal && s_hal->stop) s_hal->stop();
    mtk_arbiter_release_if_owner(MTK_ARB_ESPNOW, token);
    mtk_op_transition_by_token(token, boot_epoch, MTK_OPS_STOPPED, status, now_ms());
    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_espnow_stopped_ev_t ev = {0};
        ev.operation_token = token;
        ev.status = status;
        ev.sent_ok = sent_ok;
        ev.received = received;
        sink.emit_event(sink.user, token, "ESPNOW_STOPPED", &ev, &mtk_espnow_stopped_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_espnow_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_espnow_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.channel < 1 || req.channel > 13) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }

    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return; }
    /* Serializes against every Wi-Fi class: they share one radio. */
    if (mtk_arbiter_acquire(MTK_ARB_ESPNOW, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    en_lock();
    memset(&s_en, 0, sizeof(s_en));
    s_en.token = id.token; s_en.boot_epoch = id.boot_epoch;
    s_en.session_generation = ctx->session_generation;
    s_en.sink = ctx->sink;
    s_en.channel = req.channel;
    s_en.active = 1;
    en_unlock();

    mtk_espnow_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_espnow_start_resp_t_desc);
    mtk_op_end_admission_guard();

    int rc = (s_hal && s_hal->start) ? s_hal->start(req.channel, espnow_rx_cb, NULL) : -1;
    if (rc != 0) espnow_teardown(id.token, id.boot_epoch, MTK_STATUS_IO_ERROR);
}

static void handle_espnow_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_espnow_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, ESPNOW_SERVICE_ID, ESPNOW_START_OPCODE, &precheck)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    espnow_teardown(req.operation_token, ctx->boot_epoch, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, ESPNOW_SERVICE_ID, ESPNOW_START_OPCODE, &snap)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    mtk_espnow_stop_resp_t r;
    r.final_state = (uint8_t)snap.state;
    r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_espnow_stop_resp_t_desc);
}

/* Every data-plane opcode below requires a live session: without one there is
 * no radio lease, so acting would touch the radio another owner may hold. */
static int session_active(void) {
    en_lock(); uint8_t a = s_en.active; en_unlock();
    return a != 0;
}

static void handle_espnow_add_peer(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_espnow_add_peer_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { memset(&req, 0, sizeof(req)); respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    uint8_t lmk_len = (uint8_t)req.lmk.len;
    /* An encrypted peer needs a full 16-byte local master key; anything else
     * is refused rather than silently downgraded to plaintext. */
    if (req.channel > 13 || (req.encrypt && lmk_len != 16) || (!req.encrypt && lmk_len != 0)) {
        memset(&req, 0, sizeof(req));
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }
    if (!session_active()) { memset(&req, 0, sizeof(req)); respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    mtk_hal_mac6_t peer; memcpy(peer.b, req.peer.b, 6);
    int rc = (s_hal && s_hal->add_peer) ? s_hal->add_peer(peer, req.channel, req.encrypt, req.lmk.data, lmk_len) : -1;
    memset(&req, 0, sizeof(req)); /* the key must not outlive the call */
    respond_empty(ctx, rc == 0 ? MTK_STATUS_OK : (rc == 1 ? MTK_STATUS_OVERFLOW : MTK_STATUS_IO_ERROR));
}

static void handle_espnow_send(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                const uint8_t *req_bytes, size_t req_len) {
    mtk_espnow_send_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    uint8_t len = (uint8_t)req.data.len;
    if (len == 0 || len > ESPNOW_MAX_PAYLOAD) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    mtk_hal_mac6_t peer; memcpy(peer.b, req.peer.b, 6);
    int rc = (s_hal && s_hal->send) ? s_hal->send(peer, req.data.data, len) : -1;
    respond_empty(ctx, rc == 0 ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR);
}

/* Tagged response, hand-assembled for the same reason CAPTURE_POLL_READ is:
 * the EMPTY variant transmits zero body bytes, which the generic struct codec
 * cannot express. */
static void handle_espnow_poll_recv(mtk_request_ctx_t *ctx) {
    uint8_t out[1 + 6 + 1 + 1 + ESPNOW_MAX_PAYLOAD];
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    en_lock();
    if (s_en.ring_count == 0) {
        en_unlock();
        out[0] = 0; /* EMPTY */
        ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, 1);
        return;
    }
    espnow_rx_rec_t *rec = &s_en.ring[s_en.ring_head];
    s_en.ring_head = (s_en.ring_head + 1) % ESPNOW_RING_SLOTS;
    s_en.ring_count--;
    size_t off = 0;
    out[off++] = 1; /* RECORD */
    memcpy(out + off, rec->src.b, 6); off += 6;
    out[off++] = (uint8_t)rec->rssi;
    out[off++] = rec->len;
    memcpy(out + off, rec->data, rec->len); off += rec->len;
    en_unlock();
    ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, off);
}

static void handle_espnow_stats(mtk_request_ctx_t *ctx) {
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    mtk_espnow_stats_resp_t r; memset(&r, 0, sizeof(r));
    uint32_t sent_ok = 0, send_fail = 0;
    /* Unknown counters report zero rather than a guess. */
    if (s_hal && s_hal->stats && s_hal->stats(&sent_ok, &send_fail) != 0) { sent_ok = 0; send_fail = 0; }
    en_lock();
    r.received = s_en.received;
    r.dropped = s_en.dropped;
    en_unlock();
    r.sent_ok = sent_ok;
    r.send_fail = send_fail;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_espnow_stats_resp_t_desc);
}

mtk_op_id_t mtek_espnow_cancel_active_for_peer_reset(void) {
    mtk_op_id_t id = {0, 0};
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    if (snap.cls != MTK_ARB_ESPNOW) return id;
    uint32_t tok = snap.token;
    uint32_t epoch = mtk_core_boot_epoch();
    espnow_teardown(tok, epoch, MTK_STATUS_OK);
    id.token = tok; id.boot_epoch = epoch;
    return id;
}

static void mtek_espnow_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: handle_espnow_start(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_espnow_stop(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_espnow_add_peer(ctx, op, req_bytes, req_len); return;
        case 0x0004: handle_espnow_send(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_espnow_poll_recv(ctx); return;
        case 0x0006: handle_espnow_stats(ctx); return;
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

mtk_register_result_t mtek_espnow_service_register(void) {
    return mtk_router_register(ESPNOW_SERVICE_ID, mtek_espnow_dispatch);
}
