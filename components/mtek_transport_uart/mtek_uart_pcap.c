#include "mtek_uart_pcap.h"

#include "mtek_capture_service.h"
#include "mtek_codec_api.h"
#include "mtek_router.h"
#include "mtek_schema_constants.h"
#include "mtek_schema_message_descs.h"
#include "mtek_schema_structs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PCAP_CONTROL_RESERVE 128u
#define PCAP_READY_INTERVAL_MS 5u
#define PCAP_TX_DRAIN_MS 200u
#define PCAP_MAX_DURATION_MS 600000u

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t crc32c(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0x82F63B78u & mask);
        }
    }
    return ~crc;
}

static size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst,
                          size_t dst_cap) {
    size_t read = 0, out = 1, code_pos = 0;
    uint8_t code = 1;
    if (!dst || dst_cap == 0) return 0;
    while (read < len) {
        if (src[read] == 0) {
            dst[code_pos] = code;
            code_pos = out++;
            if (out > dst_cap) return 0;
            code = 1;
            read++;
            continue;
        }
        if (out >= dst_cap) return 0;
        dst[out++] = src[read++];
        code++;
        if (code == 0xFFu) {
            dst[code_pos] = code;
            code_pos = out++;
            if (out > dst_cap) return 0;
            code = 1;
        }
    }
    if (code_pos >= dst_cap) return 0;
    dst[code_pos] = code;
    return out;
}

static size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst,
                          size_t dst_cap) {
    size_t read = 0, write = 0;
    if (!src || !dst) return 0;
    while (read < len) {
        uint8_t code = src[read++];
        if (code == 0 || (size_t)(code - 1u) > len - read) return 0;
        for (uint8_t i = 1; i < code; i++) {
            if (write >= dst_cap) return 0;
            dst[write++] = src[read++];
        }
        if (code < 0xFFu && read < len) {
            if (write >= dst_cap) return 0;
            dst[write++] = 0;
        }
    }
    return write;
}

size_t mtek_uart_pcap_frame_encode(uint8_t msg_type, uint16_t session_id,
                                   uint32_t sequence, const uint8_t *payload,
                                   uint16_t payload_len, uint8_t *dst,
                                   size_t dst_cap) {
    uint8_t env[MTEK_UART_PCAP_MAX_ENVELOPE];
    if (payload_len > MTEK_UART_PCAP_MAX_PAYLOAD ||
        (payload_len != 0 && !payload) || !dst) return 0;
    env[0] = MTEK_UART_PCAP_PROTO_VERSION;
    env[1] = msg_type;
    put_u16(env + 2, session_id);
    put_u32(env + 4, sequence);
    put_u16(env + 8, payload_len);
    if (payload_len) memcpy(env + MTEK_UART_PCAP_HDR_LEN, payload, payload_len);
    size_t env_len = MTEK_UART_PCAP_HDR_LEN + payload_len;
    put_u32(env + env_len, crc32c(env, env_len));
    env_len += MTEK_UART_PCAP_CRC_LEN;
    size_t encoded = cobs_encode(env, env_len, dst, dst_cap);
    if (encoded == 0 || encoded + 1u > dst_cap) return 0;
    dst[encoded] = 0;
    return encoded + 1u;
}

mtek_uart_pcap_decode_result_t mtek_uart_pcap_frame_decode(
    const uint8_t *cobs_block, size_t block_len, uint8_t *decode_buf,
    size_t decode_cap, mtek_uart_pcap_msg_t *out) {
    if (!cobs_block || !decode_buf || !out)
        return MTEK_UART_PCAP_DECODE_FORMAT;
    size_t env_len = cobs_decode(cobs_block, block_len, decode_buf, decode_cap);
    if (env_len < MTEK_UART_PCAP_HDR_LEN + MTEK_UART_PCAP_CRC_LEN)
        return MTEK_UART_PCAP_DECODE_FORMAT;
    uint16_t payload_len = get_u16(decode_buf + 8);
    if ((size_t)MTEK_UART_PCAP_HDR_LEN + payload_len + MTEK_UART_PCAP_CRC_LEN != env_len)
        return MTEK_UART_PCAP_DECODE_FORMAT;
    uint32_t wire_crc = get_u32(decode_buf + env_len - MTEK_UART_PCAP_CRC_LEN);
    if (crc32c(decode_buf, env_len - MTEK_UART_PCAP_CRC_LEN) != wire_crc)
        return MTEK_UART_PCAP_DECODE_CRC;
    if (decode_buf[0] != MTEK_UART_PCAP_PROTO_VERSION)
        return MTEK_UART_PCAP_DECODE_VERSION;
    out->version = decode_buf[0];
    out->msg_type = decode_buf[1];
    out->session_id = get_u16(decode_buf + 2);
    out->sequence = get_u32(decode_buf + 4);
    out->payload_len = payload_len;
    out->payload = payload_len ? decode_buf + MTEK_UART_PCAP_HDR_LEN : NULL;
    return MTEK_UART_PCAP_DECODE_OK;
}

void mtek_uart_pcap_init(mtek_uart_pcap_state_t *st, uint32_t boot_epoch) {
    memset(st, 0, sizeof(*st));
    st->boot_epoch = boot_epoch;
}

void mtek_uart_pcap_set_lock(mtek_uart_pcap_state_t *st,
                             mtek_uart_pcap_lock_fn lock,
                             mtek_uart_pcap_lock_fn unlock,
                             void *lock_ctx) {
    st->lock = lock;
    st->unlock = unlock;
    st->lock_ctx = lock_ctx;
}

static void state_lock(mtek_uart_pcap_state_t *st) {
    if (st->lock) st->lock(st->lock_ctx);
}

static void state_unlock(mtek_uart_pcap_state_t *st) {
    if (st->unlock) st->unlock(st->lock_ctx);
}

int mtek_uart_pcap_parse_start(const char *line, uint8_t *channel,
                               uint32_t *duration_ms) {
    static const char prefix[] = "PCAP_START ";
    if (!line || strncmp(line, prefix, sizeof(prefix) - 1u) != 0) return 0;
    const char *p = line + sizeof(prefix) - 1u;
    char *end = NULL;
    unsigned long ch = strtoul(p, &end, 10);
    if (end == p || *end != ' ' || ch < 1u || ch > 14u) return 0;
    p = end + 1;
    if (*p == 0 || *p == ' ') return 0;
    unsigned long duration = strtoul(p, &end, 10);
    if (end == p || *end != 0 || duration == 0 || duration > PCAP_MAX_DURATION_MS)
        return 0;
    if (channel) *channel = (uint8_t)ch;
    if (duration_ms) *duration_ms = (uint32_t)duration;
    return 1;
}

static mtk_emit_result_t session_response(void *user, uint32_t correlation,
                                          uint8_t status, const void *body,
                                          const mtk_struct_desc_t *desc) {
    (void)correlation;
    (void)desc;
    mtek_uart_pcap_state_t *st = (mtek_uart_pcap_state_t *)user;
    state_lock(st);
    st->start_response_seen = 1;
    st->start_status = status;
    if (status == MTK_STATUS_ACCEPTED && body)
        st->operation_token = ((const mtk_capture_start_resp_t *)body)->operation_token;
    state_unlock(st);
    return MTK_EMIT_OK;
}

static mtk_emit_result_t session_response_raw(void *user, uint32_t correlation,
                                              uint8_t status,
                                              const uint8_t *body, size_t len) {
    (void)body;
    (void)len;
    return session_response(user, correlation, status, NULL, NULL);
}

static mtk_emit_result_t session_event(void *user, uint32_t correlation,
                                       const char *name, const void *body,
                                       const mtk_struct_desc_t *desc) {
    (void)correlation;
    (void)desc;
    if (!name || strcmp(name, "CAPTURE_STOPPED") != 0 || !body)
        return MTK_EMIT_OK;
    const mtk_capture_stopped_ev_t *ev = (const mtk_capture_stopped_ev_t *)body;
    mtek_uart_pcap_state_t *st = (mtek_uart_pcap_state_t *)user;
    state_lock(st);
    if (ev->operation_token == st->operation_token) {
        st->terminal_seen = 1;
        st->terminal_status = ev->status;
        st->terminal_frames = ev->total_frames;
        st->terminal_drops = ev->dropped_frames;
        st->terminal_truncated = ev->truncated_frames;
        st->active = 0;
    }
    state_unlock(st);
    return MTK_EMIT_OK;
}

static mtk_emit_result_t session_stream(void *user, uint32_t token,
                                        uint32_t sequence,
                                        const uint8_t *chunk, size_t len) {
    (void)user;
    (void)token;
    (void)sequence;
    (void)chunk;
    (void)len;
    return MTK_EMIT_DROPPED;
}

typedef struct {
    mtek_uart_pcap_state_t *st;
    uint8_t status;
    uint8_t seen;
} call_capture_t;

static mtk_emit_result_t call_response(void *user, uint32_t correlation,
                                       uint8_t status, const void *body,
                                       const mtk_struct_desc_t *desc) {
    (void)correlation;
    call_capture_t *cap = (call_capture_t *)user;
    cap->status = status;
    cap->seen = 1;
    cap->st->poll_body_len = 0;
    if (body && desc && mtk_encode(desc, body,
                                  cap->st->decode_or_poll.poll_body,
                                  sizeof(cap->st->decode_or_poll.poll_body),
                                  &cap->st->poll_body_len) != MTK_CODEC_OK)
        cap->status = MTK_STATUS_INTERNAL_ERROR;
    return MTK_EMIT_OK;
}

static mtk_emit_result_t call_response_raw(void *user, uint32_t correlation,
                                           uint8_t status,
                                           const uint8_t *body, size_t len) {
    (void)correlation;
    call_capture_t *cap = (call_capture_t *)user;
    cap->status = status;
    cap->seen = 1;
    if (len > sizeof(cap->st->decode_or_poll.poll_body)) {
        cap->status = MTK_STATUS_OVERFLOW;
        cap->st->poll_body_len = 0;
        return MTK_EMIT_CAPACITY_FAILED;
    }
    cap->st->poll_body_len = len;
    if (body && len) memcpy(cap->st->decode_or_poll.poll_body, body, len);
    return MTK_EMIT_OK;
}

static mtk_emit_result_t call_event(void *user, uint32_t correlation,
                                    const char *name, const void *body,
                                    const mtk_struct_desc_t *desc) {
    (void)user; (void)correlation; (void)name; (void)body; (void)desc;
    return MTK_EMIT_OK;
}

static mtk_emit_result_t call_stream(void *user, uint32_t token,
                                     uint32_t sequence,
                                     const uint8_t *chunk, size_t len) {
    (void)user; (void)token; (void)sequence; (void)chunk; (void)len;
    return MTK_EMIT_DROPPED;
}

static mtk_request_ctx_t make_call_ctx(mtek_uart_pcap_state_t *st,
                                       call_capture_t *cap) {
    mtk_request_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    /* Internal adapter-to-core call. CAPTURE_POLL_READ is intentionally
     * not exposed as a factory-UART command, but this compatibility
     * endpoint uses it as its private translation seam. */
    ctx.profile = MTK_PROFILE_HOST_ADAPTER;
    ctx.dispatch_mode = MTK_DISPATCH_INLINE;
    ctx.correlation = ++st->next_correlation;
    ctx.boot_epoch = st->boot_epoch;
    ctx.sink.user = cap;
    ctx.sink.emit_response = call_response;
    ctx.sink.emit_response_raw = call_response_raw;
    ctx.sink.emit_event = call_event;
    ctx.sink.emit_stream = call_stream;
    return ctx;
}

static mtek_uart_pcap_run_result_t start_capture(mtek_uart_pcap_state_t *st,
                                                 uint8_t channel,
                                                 uint32_t duration_ms) {
    st->channel = channel;
    st->requested_duration_ms = duration_ms;
    st->operation_token = 0;
    st->start_response_seen = 0;
    st->terminal_seen = 0;
    st->granted_bytes = st->consumed_bytes = 0;
    st->grants = st->starved = st->wire_sequence = 0;
    st->batches = st->wire_bytes = 0;
    st->pending_wire_len = 0;
    st->rx_len = 0;
    st->rx_overrun = 0;

    mtk_capture_start_req_t req;
    memset(&req, 0, sizeof(req));
    req.mode = 1; /* canonical POLL mode; UART credits remain authoritative */
    req.snap_len = 1000;
    req.duration_ms = 0; /* this endpoint owns the exact external deadline */
    req.channel_plan.mode = 0;
    req.channel_plan.channel = channel;

    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0004, 0x0001);
    if (!op) return MTEK_UART_PCAP_RUN_START_FAILED;
    uint8_t encoded[64];
    size_t encoded_len = 0;
    if (mtk_encode(op->req_desc, &req, encoded, sizeof(encoded),
                   &encoded_len) != MTK_CODEC_OK)
        return MTEK_UART_PCAP_RUN_START_FAILED;

    mtk_request_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.profile = MTK_PROFILE_FACTORY_UART;
    ctx.dispatch_mode = MTK_DISPATCH_INLINE;
    ctx.correlation = ++st->next_correlation;
    ctx.boot_epoch = st->boot_epoch;
    ctx.sink.user = st;
    ctx.sink.emit_response = session_response;
    ctx.sink.emit_response_raw = session_response_raw;
    ctx.sink.emit_event = session_event;
    ctx.sink.emit_stream = session_stream;
    mtk_router_dispatch(&ctx, op->service_id, op->opcode, encoded, encoded_len);

    state_lock(st);
    uint8_t seen = st->start_response_seen;
    uint8_t status = st->start_status;
    uint32_t token = st->operation_token;
    uint8_t terminal = st->terminal_seen;
    if (seen && status == MTK_STATUS_ACCEPTED && token != 0 && !terminal)
        st->active = 1;
    state_unlock(st);

    if (!seen) return MTEK_UART_PCAP_RUN_START_FAILED;
    if (status == MTK_STATUS_BUSY || status == MTK_STATUS_RADIO_CONFLICT)
        return MTEK_UART_PCAP_RUN_RADIO_BUSY;
    if (status != MTK_STATUS_ACCEPTED || token == 0 || terminal)
        return MTEK_UART_PCAP_RUN_START_FAILED;
    return MTEK_UART_PCAP_RUN_OK;
}

static void stop_capture(mtek_uart_pcap_state_t *st) {
    state_lock(st);
    uint8_t active = st->active;
    uint32_t token = st->operation_token;
    state_unlock(st);
    if (!active || token == 0) return;

    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0004, 0x0002);
    if (!op) return;
    mtk_capture_stop_req_t req;
    memset(&req, 0, sizeof(req));
    req.operation_token = token;
    req.reason = 0;
    uint8_t encoded[16];
    size_t encoded_len = 0;
    if (mtk_encode(op->req_desc, &req, encoded, sizeof(encoded),
                   &encoded_len) != MTK_CODEC_OK) return;
    call_capture_t cap = { .st = st };
    mtk_request_ctx_t ctx = make_call_ctx(st, &cap);
    mtk_router_dispatch(&ctx, op->service_id, op->opcode, encoded, encoded_len);
    state_lock(st);
    st->active = 0;
    state_unlock(st);
}

static int poll_record(mtek_uart_pcap_state_t *st) {
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0004, 0x0006);
    if (!op || st->operation_token == 0) return -1;
    mtk_capture_poll_read_req_t req = { .operation_token = st->operation_token };
    uint8_t encoded[8];
    size_t encoded_len = 0;
    if (mtk_encode(op->req_desc, &req, encoded, sizeof(encoded),
                   &encoded_len) != MTK_CODEC_OK) return -1;
    call_capture_t cap = { .st = st };
    mtk_request_ctx_t ctx = make_call_ctx(st, &cap);
    mtk_router_dispatch(&ctx, op->service_id, op->opcode, encoded, encoded_len);
    if (!cap.seen || cap.status != MTK_STATUS_OK || st->poll_body_len < 1)
        return -1;
    if (st->decode_or_poll.poll_body[0] == 0) return 0;
    if (st->decode_or_poll.poll_body[0] != 1 || st->poll_body_len < 23)
        return -1;

    const uint8_t *in = st->decode_or_poll.poll_body;
    uint16_t captured_len = get_u16(in + 19);
    uint16_t data_len = get_u16(in + 21);
    if (captured_len != data_len || captured_len > 1000u ||
        st->poll_body_len != (size_t)23u + captured_len)
        return -1;
    const uint8_t *raw = in + 23;

    uint8_t *payload = st->record_or_rx.payload;
    memset(payload, 0, MTEK_UART_PCAP_RECORD_HDR_LEN);
    payload[0] = 1; /* M1_CAPFRAME_VERSION */
    payload[1] = captured_len ? (uint8_t)((raw[0] >> 2) & 0x03u) : 3u;
    payload[2] = in[14];
    payload[3] = 0; /* secondary channel not exposed by canonical HAL */
    payload[4] = in[15];
    payload[5] = 0; /* PHY unknown */
    payload[6] = 0; /* legacy rate unavailable */
    payload[7] = (in[16] & 0x01u) ? 0x04u : 0u;
    memcpy(payload + 8, in + 17, 2);  /* original length */
    memcpy(payload + 10, in + 19, 2); /* captured length */
    memcpy(payload + 12, in + 1, 4);  /* frame sequence */
    memcpy(payload + 16, in + 5, 8);  /* timestamp usec */
    if (captured_len) memcpy(payload + 24, raw, captured_len);

    st->pending_wire_len = mtek_uart_pcap_frame_encode(
        MTEK_UART_PCAP_FRAME_BATCH, MTEK_UART_PCAP_SESSION_ID,
        st->wire_sequence, payload,
        (uint16_t)(MTEK_UART_PCAP_RECORD_HDR_LEN + captured_len),
        st->wire, sizeof(st->wire));
    return st->pending_wire_len ? 1 : -1;
}

static uint32_t available_credit(const mtek_uart_pcap_state_t *st) {
    return st->granted_bytes > st->consumed_bytes
        ? st->granted_bytes - st->consumed_bytes : 0;
}

/* 0=no complete command, 1=credit, 2=stop. */
static int feed_rx_byte(mtek_uart_pcap_state_t *st, uint8_t byte) {
    if (byte != 0) {
        if (st->rx_len < sizeof(st->record_or_rx.rx_block))
            st->record_or_rx.rx_block[st->rx_len++] = byte;
        else
            st->rx_overrun = 1;
        return 0;
    }
    size_t len = st->rx_len;
    uint8_t overrun = st->rx_overrun;
    st->rx_len = 0;
    st->rx_overrun = 0;
    if (overrun || len == 0) return 0;
    mtek_uart_pcap_msg_t msg;
    if (mtek_uart_pcap_frame_decode(st->record_or_rx.rx_block, len,
                                    st->decode_or_poll.decode_buf,
                                    sizeof(st->decode_or_poll.decode_buf),
                                    &msg) !=
        MTEK_UART_PCAP_DECODE_OK ||
        msg.session_id != MTEK_UART_PCAP_SESSION_ID)
        return 0;
    if (msg.msg_type == MTEK_UART_PCAP_CREDIT && msg.payload_len >= 4) {
        uint32_t amount = get_u32(msg.payload);
        uint32_t sum = st->granted_bytes + amount;
        st->granted_bytes = sum < st->granted_bytes ? UINT32_MAX : sum;
        st->grants++;
        return 1;
    }
    if (msg.msg_type == MTEK_UART_PCAP_STOP) return 2;
    return 0;
}

static int write_all(const mtek_uart_pcap_io_t *io, const uint8_t *data,
                     size_t len) {
    return io->write && io->write(io->ctx, data, len) == (int)len;
}

static int send_control(mtek_uart_pcap_state_t *st,
                        const mtek_uart_pcap_io_t *io, uint8_t type,
                        uint32_t sequence, const uint8_t *payload,
                        uint16_t payload_len) {
    size_t len = mtek_uart_pcap_frame_encode(type, MTEK_UART_PCAP_SESSION_ID,
                                             sequence, payload, payload_len,
                                             st->wire, sizeof(st->wire));
    return len && write_all(io, st->wire, len);
}

static int send_stopped(mtek_uart_pcap_state_t *st,
                        const mtek_uart_pcap_io_t *io) {
    uint8_t payload[28];
    memset(payload, 0, sizeof(payload));
    state_lock(st);
    uint32_t captured = st->terminal_frames;
    uint32_t dropped = st->terminal_drops;
    state_unlock(st);
    put_u32(payload + 0, st->wire_sequence);
    put_u32(payload + 4, st->batches);
    put_u32(payload + 8, st->wire_bytes);
    put_u32(payload + 12, captured);
    put_u32(payload + 16, dropped);
    put_u32(payload + 20, st->starved);
    put_u32(payload + 24, st->grants);
    return send_control(st, io, MTEK_UART_PCAP_STOPPED,
                        st->wire_sequence, payload, sizeof(payload));
}

static int valid_io(const mtek_uart_pcap_io_t *io) {
    return io && io->write && io->wait_tx_done && io->set_baud &&
           io->flush_input && io->read && io->now_ms && io->delay_ms;
}

mtek_uart_pcap_run_result_t mtek_uart_pcap_run(
    mtek_uart_pcap_state_t *st, uint8_t channel, uint32_t duration_ms,
    const mtek_uart_pcap_io_t *io) {
    if (!st || !valid_io(io) || channel < 1 || channel > 14 ||
        duration_ms == 0 || duration_ms > PCAP_MAX_DURATION_MS)
        return MTEK_UART_PCAP_RUN_INVALID;

    mtek_uart_pcap_run_result_t result = start_capture(st, channel, duration_ms);
    if (result == MTEK_UART_PCAP_RUN_RADIO_BUSY) {
        static const uint8_t busy[] = "Radio busy\r\n";
        (void)write_all(io, busy, sizeof(busy) - 1u);
        return result;
    }
    if (result != MTEK_UART_PCAP_RUN_OK) {
        static const uint8_t failed[] = "PCAP start failed\r\n";
        (void)write_all(io, failed, sizeof(failed) - 1u);
        return result;
    }

    char text[160];
    int text_len = snprintf(text, sizeof(text),
                            "PCAP ACK ch=%u baud=%u ms=%lu\r\n",
                            (unsigned)channel,
                            (unsigned)MTEK_UART_PCAP_CAPTURE_BAUD,
                            (unsigned long)duration_ms);
    int io_failed = text_len <= 0 || (size_t)text_len >= sizeof(text) ||
                    !write_all(io, (const uint8_t *)text, (size_t)text_len) ||
                    io->wait_tx_done(io->ctx, PCAP_TX_DRAIN_MS) != 0;

    if (io->set_binary_logging) io->set_binary_logging(io->ctx, 1);
    if (!io_failed && io->set_baud(io->ctx, MTEK_UART_PCAP_CAPTURE_BAUD) != 0)
        io_failed = 1;
    if (!io_failed && io->flush_input(io->ctx) != 0) io_failed = 1;
    if (!io_failed) io->delay_ms(io->ctx, 20);

    uint64_t started = io->now_ms(io->ctx);
    uint64_t last_ready = 0;
    uint8_t ready_sent = 0;
    uint8_t rx[256];
    while (!io_failed) {
        int n = io->read(io->ctx, rx, sizeof(rx), 1);
        if (n < 0) { io_failed = 1; break; }
        int stop_requested = 0;
        for (int i = 0; i < n; i++) {
            if (feed_rx_byte(st, rx[i]) == 2) stop_requested = 1;
        }

        uint64_t now = io->now_ms(io->ctx);
        if (stop_requested || now - started >= duration_ms) stop_capture(st);

        state_lock(st);
        uint8_t terminal = st->terminal_seen;
        uint8_t active = st->active;
        state_unlock(st);
        if (terminal || !active) break;

        if (st->grants == 0) {
            if (!ready_sent || now - last_ready >= PCAP_READY_INTERVAL_MS) {
                if (!send_control(st, io, MTEK_UART_PCAP_READY, 0, NULL, 0)) {
                    io_failed = 1;
                    break;
                }
                ready_sent = 1;
                last_ready = now;
            }
            io->delay_ms(io->ctx, 1);
            continue;
        }

        if (st->pending_wire_len == 0 &&
            available_credit(st) > PCAP_CONTROL_RESERVE) {
            int polled = poll_record(st);
            if (polled < 0) { io_failed = 1; break; }
        }
        if (st->pending_wire_len) {
            uint32_t available = available_credit(st);
            if (available > PCAP_CONTROL_RESERVE &&
                st->pending_wire_len <= available - PCAP_CONTROL_RESERVE) {
                if (!write_all(io, st->wire, st->pending_wire_len)) {
                    io_failed = 1;
                    break;
                }
                st->consumed_bytes += (uint32_t)st->pending_wire_len;
                st->wire_bytes += (uint32_t)st->pending_wire_len;
                st->batches++;
                st->wire_sequence++;
                st->pending_wire_len = 0;
            } else {
                st->starved++;
            }
        }
        if (st->pending_wire_len == 0) io->delay_ms(io->ctx, 1);
    }

    stop_capture(st);
    if (!io_failed && !send_stopped(st, io)) io_failed = 1;
    if (io->wait_tx_done(io->ctx, PCAP_TX_DRAIN_MS) != 0) io_failed = 1;

    if (io->set_baud(io->ctx, MTEK_UART_PCAP_CONSOLE_BAUD) != 0) io_failed = 1;
    if (io->flush_input(io->ctx) != 0) io_failed = 1;
    if (io->set_binary_logging) io->set_binary_logging(io->ctx, 0);

    state_lock(st);
    uint32_t captured = st->terminal_frames;
    uint32_t dropped = st->terminal_drops;
    state_unlock(st);
    text_len = snprintf(text, sizeof(text),
                        "PCAP DONE ch=%u batches=%lu bytes=%lu cap_frames=%lu "
                        "drops=%lu starved=%lu grants=%lu stop=1\r\n",
                        (unsigned)channel, (unsigned long)st->batches,
                        (unsigned long)st->wire_bytes,
                        (unsigned long)captured, (unsigned long)dropped,
                        (unsigned long)st->starved,
                        (unsigned long)st->grants);
    if (text_len <= 0 || (size_t)text_len >= sizeof(text) ||
        !write_all(io, (const uint8_t *)text, (size_t)text_len)) io_failed = 1;
    return io_failed ? MTEK_UART_PCAP_RUN_IO_FAILED : MTEK_UART_PCAP_RUN_OK;
}
