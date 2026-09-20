/* IEEE 802.15.4 service (service 0x0007). Portable: no ESP-IDF dependency, so
 * the whole lifecycle is host-testable against a fake HAL.
 *
 * Protocol-neutral by construction. The service moves PHY payloads and
 * reports radio metadata; it never parses or synthesises 802.15.4 MAC
 * headers. Thread (via the host driving an RCP), a host-side Zigbee stack, a
 * sniffer, or any other 802.15.4 consumer all sit above the same primitives,
 * so a new consumer needs host work rather than a Core redesign.
 *
 * Ownership uses the existing single-owner arbiter: one operation token, one
 * IEEE154 lease held for the session, and one teardown path that user STOP,
 * start failure and peer reset all funnel through. The ESP32-C6 has a single
 * 2.4GHz radio, so IEEE154 is serialized against every Wi-Fi class and
 * ESP-NOW (schemas.json arbiter_pairwise_policy).
 *
 * Received frames land in a bounded ring filled by the HAL's receive callback
 * and drained by IEEE154_POLL_RECV, so a host that stops polling cannot cause
 * unbounded buffering: the oldest frame is dropped and counted. */
#include "mtek_ieee802154_hal.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include <string.h>

#define I154_SERVICE_ID    0x0007
#define I154_START_OPCODE  0x0001
/* 16 slots x 127-byte payload plus metadata. Sized against the variant's
 * measured DIRAM headroom, not guessed: see docs/RESOURCE_BUDGET.md. */
#define I154_RING_SLOTS    16
#define I154_MAX_ENERGY_RESULTS 16

static const mtk_ieee802154_hal_t *s_hal;
void mtek_ieee802154_set_hal(const mtk_ieee802154_hal_t *hal) { s_hal = hal; }

static uint64_t (*s_now_ms)(void);
void mtek_ieee802154_service_init(uint64_t (*now_ms_fn)(void)) { s_now_ms = now_ms_fn; }
static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

static void (*s_lock_fn)(void), (*s_unlock_fn)(void);
void mtek_ieee802154_set_lock(void (*lock)(void), void (*unlock)(void)) { s_lock_fn = lock; s_unlock_fn = unlock; }
static void i_lock(void)   { if (s_lock_fn) s_lock_fn(); }
static void i_unlock(void) { if (s_unlock_fn) s_unlock_fn(); }

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}

typedef struct {
    uint64_t timestamp_us;
    uint8_t channel;
    int8_t rssi;
    uint8_t lqi;
    uint8_t flags;        /* bit0: frame was truncated to the ring's snap length */
    uint16_t original_len;
    uint8_t len;
    uint8_t data[MTK_154_MAX_PHY_LEN];
} i154_rx_rec_t;

static struct {
    uint32_t token;
    uint32_t boot_epoch;
    uint32_t session_generation;
    mtk_sink_t sink;
    uint8_t active;
    uint8_t channel;
    uint8_t promiscuous;
    uint8_t scanning;            /* an energy scan owns the lease instead of a receive session */
    uint8_t rcp;                 /* the RCP runtime owns the lease instead of either */
    uint32_t received, dropped;
    i154_rx_rec_t ring[I154_RING_SLOTS];
    unsigned ring_head, ring_count;
} s_i154;

static int channel_valid(uint8_t ch) { return ch >= MTK_154_CHANNEL_MIN && ch <= MTK_154_CHANNEL_MAX; }

/* Drop-oldest on overflow: a full ring means the host is not draining, and
 * losing the oldest frame is preferable to refusing every new one or growing
 * without bound. */
static void i154_rx_cb(void *user, uint64_t timestamp_us, uint8_t channel,
                        int8_t rssi, uint8_t lqi, const uint8_t *data, uint8_t len) {
    (void)user;
    uint16_t original_len = len;
    if (len > MTK_154_MAX_PHY_LEN) len = MTK_154_MAX_PHY_LEN;
    i_lock();
    if (!s_i154.active) { i_unlock(); return; }
    s_i154.received++;
    if (s_i154.ring_count >= I154_RING_SLOTS) {
        s_i154.ring_head = (s_i154.ring_head + 1) % I154_RING_SLOTS;
        s_i154.ring_count--;
        s_i154.dropped++;
    }
    i154_rx_rec_t *r = &s_i154.ring[(s_i154.ring_head + s_i154.ring_count) % I154_RING_SLOTS];
    memset(r, 0, sizeof(*r));
    r->timestamp_us = timestamp_us;
    r->channel = channel;
    r->rssi = rssi;
    r->lqi = lqi;
    r->original_len = original_len;
    r->flags = (original_len > len) ? 0x01 : 0x00;
    r->len = len;
    if (len) memcpy(r->data, data, len);
    s_i154.ring_count++;
    i_unlock();
}

void mtek_ieee802154_service_tick(void) {
    if (s_hal && s_hal->service) s_hal->service();
}

static void i154_teardown(uint32_t token, uint32_t boot_epoch, uint8_t status) {
    if (!mtk_op_claim_finalization(token, boot_epoch)) return;
    i_lock();
    mtk_sink_t sink = s_i154.sink;
    uint32_t session_generation = s_i154.session_generation;
    uint32_t received = s_i154.received, dropped = s_i154.dropped;
    uint8_t was_scan = s_i154.scanning;
    uint8_t was_rcp = s_i154.rcp;
    s_i154.active = 0;
    s_i154.scanning = 0;
    s_i154.rcp = 0;
    s_i154.ring_head = 0; s_i154.ring_count = 0;
    i_unlock();
    /* Always called, including when start never succeeded. The RCP runtime
     * owns the radio through OpenThread rather than through the raw HAL, so
     * each mode tears down through its own owner -- never both. */
    if (was_rcp) { if (s_hal && s_hal->rcp_stop) s_hal->rcp_stop(); }
    else if (s_hal && s_hal->stop) s_hal->stop();
    mtk_arbiter_release_if_owner(MTK_ARB_IEEE154, token);
    mtk_op_transition_by_token(token, boot_epoch, MTK_OPS_STOPPED, status, now_ms());
    /* An energy scan reports its own terminal event at completion; a receive
     * session reports IEEE154_STOPPED. A scan torn down before it completed
     * still reports the stop so the host is never left waiting. */
    if (was_rcp) {
        if (mtk_op_begin_publish_guard(session_generation)) {
            mtk_ieee154_rcp_stopped_ev_t ev = {0};
            ev.operation_token = token;
            ev.status = status;
            sink.emit_event(sink.user, token, "IEEE154_RCP_STOPPED", &ev, &mtk_ieee154_rcp_stopped_ev_t_desc);
            mtk_op_end_publish_guard();
        }
    } else if (!was_scan && mtk_op_begin_publish_guard(session_generation)) {
        mtk_ieee154_stopped_ev_t ev = {0};
        ev.operation_token = token;
        ev.status = status;
        ev.frames_received = received;
        ev.frames_dropped = dropped;
        sink.emit_event(sink.user, token, "IEEE154_STOPPED", &ev, &mtk_ieee154_stopped_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

/* Shared admission for both session-owning opcodes (receive session and
 * energy scan): both take the one IEEE154 lease, so neither may run while
 * the other or any Wi-Fi/ESP-NOW operation owns the radio. */
static int i154_admit(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op, mtk_op_id_t *id_out) {
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) { respond_empty(ctx, MTK_STATUS_NOT_READY); return 0; }
    int no_mem = 0;
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); mtk_op_end_admission_guard(); return 0; }
    if (mtk_arbiter_acquire(MTK_ARB_IEEE154, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        return 0;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());
    *id_out = id;
    return 1;
}

static void handle_i154_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                               const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Validated before any resource is taken, so a rejected request leaves the
     * radio untouched. */
    if (!channel_valid(req.channel)) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }

    mtk_op_id_t id;
    if (!i154_admit(ctx, op, &id)) return;

    i_lock();
    memset(&s_i154, 0, sizeof(s_i154));
    s_i154.token = id.token; s_i154.boot_epoch = id.boot_epoch;
    s_i154.session_generation = ctx->session_generation;
    s_i154.sink = ctx->sink;
    s_i154.channel = req.channel;
    s_i154.promiscuous = req.promiscuous ? 1 : 0;
    s_i154.active = 1;
    i_unlock();

    mtk_ieee154_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ieee154_start_resp_t_desc);
    mtk_op_end_admission_guard();

    int rc = (s_hal && s_hal->start) ? s_hal->start(req.channel, req.promiscuous ? 1 : 0, i154_rx_cb, NULL) : -1;
    if (rc != 0) i154_teardown(id.token, id.boot_epoch, MTK_STATUS_IO_ERROR);
}

static void handle_i154_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                              const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck;
    /* Family gate: a token this service did not mint can never finalize a
     * foreign operation or release its arbiter class. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, I154_SERVICE_ID, I154_START_OPCODE, &precheck)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    i154_teardown(req.operation_token, ctx->boot_epoch, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, I154_SERVICE_ID, I154_START_OPCODE, &snap)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    mtk_ieee154_stop_resp_t r;
    r.final_state = (uint8_t)snap.state;
    r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ieee154_stop_resp_t_desc);
}

static void handle_i154_status(mtk_request_ctx_t *ctx) {
    mtk_ieee154_status_resp_t r; memset(&r, 0, sizeof(r));
    uint32_t tx = 0, txf = 0;
    /* Unknown counters report zero rather than a stale or invented value. */
    if (s_hal && s_hal->stats && s_hal->stats(&tx, &txf) != 0) { tx = 0; txf = 0; }
    i_lock();
    r.state = s_i154.active ? (uint8_t)MTK_OPS_RUNNING : (uint8_t)MTK_OPS_STOPPED;
    r.channel = s_i154.channel;
    r.promiscuous = s_i154.promiscuous;
    r.frames_received = s_i154.received;
    r.frames_dropped = s_i154.dropped;
    i_unlock();
    r.frames_transmitted = tx;
    r.tx_failures = txf;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ieee154_status_resp_t_desc);
}

/* The raw data plane is only meaningful for a raw receive session: while the
 * RCP or an energy scan owns the radio, OpenThread or the scan owns it, and a
 * raw TX/poll/retune would fight them. */
static int session_active(void) {
    i_lock(); uint8_t ok = s_i154.active && !s_i154.rcp && !s_i154.scanning; i_unlock();
    return ok != 0;
}

static void handle_i154_set_channel(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                     const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_set_channel_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (!channel_valid(req.channel)) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int rc = (s_hal && s_hal->set_channel) ? s_hal->set_channel(req.channel) : -1;
    if (rc == 0) { i_lock(); s_i154.channel = req.channel; i_unlock(); }
    respond_empty(ctx, rc == 0 ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR);
}

static void handle_i154_tx(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                            const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_tx_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    uint8_t len = (uint8_t)req.data.len;
    /* A zero-length PHY payload is not a legal frame, and the codec already
     * bounds the upper end at aMaxPHYPacketSize. */
    if (len == 0 || len > MTK_154_MAX_PHY_LEN) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int rc = (s_hal && s_hal->transmit) ? s_hal->transmit(req.data.data, len, req.cca ? 1 : 0) : -1;
    mtk_ieee154_tx_resp_t r; r.tx_status = (rc == 0) ? 0 : 1;
    respond(ctx, rc == 0 ? MTK_STATUS_OK : MTK_STATUS_IO_ERROR, &r, &mtk_ieee154_tx_resp_t_desc);
}

/* Tagged response, hand-assembled for the same reason CAPTURE_POLL_READ is:
 * the EMPTY variant transmits zero body bytes, which the generic struct codec
 * cannot express. */
static void handle_i154_poll_recv(mtk_request_ctx_t *ctx) {
    uint8_t out[1 + 8 + 1 + 1 + 1 + 1 + 2 + 1 + MTK_154_MAX_PHY_LEN];
    if (!session_active()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    i_lock();
    if (s_i154.ring_count == 0) {
        i_unlock();
        out[0] = 0; /* EMPTY */
        ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, 1);
        return;
    }
    i154_rx_rec_t *rec = &s_i154.ring[s_i154.ring_head];
    s_i154.ring_head = (s_i154.ring_head + 1) % I154_RING_SLOTS;
    s_i154.ring_count--;
    size_t off = 0;
    out[off++] = 1; /* RECORD */
    for (int i = 0; i < 8; i++) out[off++] = (uint8_t)(rec->timestamp_us >> (8 * i));
    out[off++] = rec->channel;
    out[off++] = (uint8_t)rec->rssi;
    out[off++] = rec->lqi;
    out[off++] = rec->flags;
    out[off++] = (uint8_t)rec->original_len;
    out[off++] = (uint8_t)(rec->original_len >> 8);
    out[off++] = rec->len;
    memcpy(out + off, rec->data, rec->len); off += rec->len;
    i_unlock();
    ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, off);
}

static void handle_i154_energy_scan(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                     const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_energy_scan_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    /* Only bits for real channels may be set, at least one of them, and a
     * dwell that is actually measurable. */
    uint32_t legal = 0;
    for (unsigned c = MTK_154_CHANNEL_MIN; c <= MTK_154_CHANNEL_MAX; c++) legal |= (1u << c);
    if (req.channel_mask == 0 || (req.channel_mask & ~legal) != 0 || req.dwell_ms == 0) {
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return;
    }

    mtk_op_id_t id;
    if (!i154_admit(ctx, op, &id)) return;
    i_lock();
    memset(&s_i154, 0, sizeof(s_i154));
    s_i154.token = id.token; s_i154.boot_epoch = id.boot_epoch;
    s_i154.session_generation = ctx->session_generation;
    s_i154.sink = ctx->sink;
    s_i154.active = 1;
    s_i154.scanning = 1;
    i_unlock();

    mtk_ieee154_energy_scan_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ieee154_energy_scan_resp_t_desc);
    mtk_op_end_admission_guard();

    mtk_ieee154_energy_result_ev_t ev; memset(&ev, 0, sizeof(ev));
    ev.operation_token = id.token;
    uint8_t status = MTK_STATUS_OK;
    unsigned n = 0;
    for (unsigned c = MTK_154_CHANNEL_MIN; c <= MTK_154_CHANNEL_MAX && n < I154_MAX_ENERGY_RESULTS; c++) {
        if (!(req.channel_mask & (1u << c))) continue;
        int8_t peak = 0;
        int rc = (s_hal && s_hal->energy_scan) ? s_hal->energy_scan((uint8_t)c, req.dwell_ms, &peak) : -1;
        if (rc != 0) { status = MTK_STATUS_IO_ERROR; break; }
        ev.results.items[n].channel = (uint8_t)c;
        ev.results.items[n].peak_rssi = peak;
        n++;
    }
    ev.results.count = n;
    ev.status = status;

    /* The scan owns the lease only while it runs; releasing happens through
     * the shared teardown so a peer reset mid-scan behaves identically. */
    if (mtk_op_claim_finalization(id.token, id.boot_epoch)) {
        i_lock();
        mtk_sink_t sink = s_i154.sink;
        uint32_t gen = s_i154.session_generation;
        s_i154.active = 0; s_i154.scanning = 0;
        i_unlock();
        if (s_hal && s_hal->stop) s_hal->stop();
        mtk_arbiter_release_if_owner(MTK_ARB_IEEE154, id.token);
        mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_STOPPED, status, now_ms());
        if (mtk_op_begin_publish_guard(gen)) {
            sink.emit_event(sink.user, id.token, "IEEE154_ENERGY_RESULT", &ev, &mtk_ieee154_energy_result_ev_t_desc);
            mtk_op_end_publish_guard();
        }
    }
}


static void handle_i154_rcp_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    (void)req_bytes; (void)req_len;
    mtk_op_id_t id;
    if (!i154_admit(ctx, op, &id)) return;

    i_lock();
    memset(&s_i154, 0, sizeof(s_i154));
    s_i154.token = id.token; s_i154.boot_epoch = id.boot_epoch;
    s_i154.session_generation = ctx->session_generation;
    s_i154.sink = ctx->sink;
    s_i154.active = 1;
    s_i154.rcp = 1;
    i_unlock();

    mtk_ieee154_rcp_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_ieee154_rcp_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* A failed start tears the session down rather than leaving the lease
     * held by a co-processor that is not serving the host. */
    int rc = (s_hal && s_hal->rcp_start) ? s_hal->rcp_start() : -1;
    if (rc != 0) i154_teardown(id.token, id.boot_epoch, MTK_STATUS_IO_ERROR);
}

static void handle_i154_rcp_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_ieee154_rcp_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, I154_SERVICE_ID, 0x0008, &precheck)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    i154_teardown(req.operation_token, ctx->boot_epoch, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, I154_SERVICE_ID, 0x0008, &snap)) {
        respond_empty(ctx, MTK_STATUS_NOT_FOUND); return;
    }
    mtk_ieee154_rcp_stop_resp_t r;
    r.final_state = (uint8_t)snap.state;
    r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ieee154_rcp_stop_resp_t_desc);
}

static void handle_i154_rcp_status(mtk_request_ctx_t *ctx) {
    mtk_ieee154_rcp_status_resp_t r; memset(&r, 0, sizeof(r));
    i_lock();
    uint8_t is_rcp = s_i154.rcp;
    i_unlock();
    r.running = (is_rcp && s_hal && s_hal->rcp_is_running && s_hal->rcp_is_running()) ? 1 : 0;
    /* 0 = none, 1 = UART Spinel. Reported only while the RCP owns the radio. */
    r.host_link = r.running ? 1 : 0;
    r.baud_rate = r.running ? 460800u : 0u;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_ieee154_rcp_status_resp_t_desc);
}

mtk_op_id_t mtek_ieee802154_cancel_active_for_peer_reset(void) {
    mtk_op_id_t id = {0, 0};
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    if (snap.cls != MTK_ARB_IEEE154) return id;
    uint32_t tok = snap.token;
    uint32_t epoch = mtk_core_boot_epoch();
    i154_teardown(tok, epoch, MTK_STATUS_OK);
    id.token = tok; id.boot_epoch = epoch;
    return id;
}

/* Dispatch obeys exactly the same build-time mode the capability table
 * reports, so GET_CAPABILITIES and real dispatch can never disagree: the
 * 802.15.4 driver has one callback owner, so an image serves either the raw
 * radio or the OpenThread RCP, never both. The property test in
 * test_opcode_registry.c enforces this agreement. */
static int opcode_served_in_this_mode(uint16_t opcode) {
#if CONFIG_OPENTHREAD_ENABLED
    return opcode >= 0x0008;   /* RCP mode: only the RCP opcodes */
#else
    return opcode <= 0x0007;   /* raw mode: only the raw radio opcodes */
#endif
}

static void mtek_ieee802154_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                      const uint8_t *req_bytes, size_t req_len) {
    if (!opcode_served_in_this_mode(op->opcode)) { respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return; }
    switch (op->opcode) {
        case 0x0001: handle_i154_start(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_i154_stop(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_i154_status(ctx); return;
        case 0x0004: handle_i154_set_channel(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_i154_energy_scan(ctx, op, req_bytes, req_len); return;
        case 0x0006: handle_i154_tx(ctx, op, req_bytes, req_len); return;
        case 0x0007: handle_i154_poll_recv(ctx); return;
        case 0x0008: handle_i154_rcp_start(ctx, op, req_bytes, req_len); return;
        case 0x0009: handle_i154_rcp_stop(ctx, op, req_bytes, req_len); return;
        case 0x000A: handle_i154_rcp_status(ctx); return;
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

mtk_register_result_t mtek_ieee802154_service_register(void) {
    return mtk_router_register(I154_SERVICE_ID, mtek_ieee802154_dispatch);
}
