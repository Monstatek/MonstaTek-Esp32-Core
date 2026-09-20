/* Header-only fake mtk_sink_t for host tests: records every
 * response/event/stream chunk a service emits so a test can assert on it
 * directly, instead of needing a real transport adapter in the loop. */
#pragma once
#include "mtek_core.h"
#include "mtek_codec_api.h"
#include <string.h>

#define MTK_FAKE_MAX_EVENTS 32
#define MTK_FAKE_MAX_STREAMS 64
#define MTK_FAKE_BODY_CAP 2048

typedef struct {
    uint8_t status;
    uint8_t body[MTK_FAKE_BODY_CAP];
    size_t body_len;
    int set;
} mtk_fake_response_t;

typedef struct {
    uint32_t correlation_or_zero;
    char name[40];
    uint8_t body[MTK_FAKE_BODY_CAP];
    size_t body_len;
} mtk_fake_event_t;

typedef struct {
    uint32_t session_token, sequence;
    uint8_t data[1024];
    size_t len;
} mtk_fake_stream_t;

typedef struct {
    mtk_fake_response_t response;
    mtk_fake_event_t events[MTK_FAKE_MAX_EVENTS];
    unsigned event_count;
    mtk_fake_stream_t streams[MTK_FAKE_MAX_STREAMS];
    unsigned stream_count;
} mtk_fake_sink_state_t;

static inline void mtk_fake_sink_reset(mtk_fake_sink_state_t *s) { memset(s, 0, sizeof(*s)); }

/* (verification sweep): a real TSan run against the new async-worker tests
 * (test_sta_scan_ lifecycle.c, test_handshake_monitor_entry_failure.c -- both
 * poll a stack-local mtk_fake_sink_state_t's own response.set/status/body from
 * the main thread while a real pthread worker thread's own dispatch call writes
 * into that exact same object via this header's own emit_* functions) caught a
 * genuine data race: this header had no locking of any kind, unlike
 * mtk_fake_wifi_hal.h's own already-established
 * mtk_fake_wifi_set_lock/mtk_fake_wifi_lock pattern for the identical class of
 * hazard. Optional lock hooks, same pattern, same safe no-op default for every
 * other (single-threaded) host test that never registers one. A test with a real
 * concurrent dispatch path must register the SAME mutex here as it does for the
 * router/service/HAL locks it already needs, and must hold it across its own
 * polling reads of the sink fields too -- not just rely on this header locking
 * its own writes. */
typedef void (*mtk_fake_sink_lock_fn)(void);
static mtk_fake_sink_lock_fn s_fake_sink_lock_fn, s_fake_sink_unlock_fn;
static inline void mtk_fake_sink_set_lock(mtk_fake_sink_lock_fn lock, mtk_fake_sink_lock_fn unlock) {
    s_fake_sink_lock_fn = lock; s_fake_sink_unlock_fn = unlock;
}
static inline void mtk_fake_sink_lock(void) { if (s_fake_sink_lock_fn) s_fake_sink_lock_fn(); }
static inline void mtk_fake_sink_unlock(void) { if (s_fake_sink_unlock_fn) s_fake_sink_unlock_fn(); }

static inline mtk_emit_result_t mtk_fake_emit_response(void *user, uint32_t correlation, uint8_t status,
                                           const void *body, const mtk_struct_desc_t *desc) {
    (void)correlation;
    mtk_fake_sink_state_t *s = (mtk_fake_sink_state_t *)user;
    mtk_fake_sink_lock();
    s->response.status = status;
    size_t n = 0;
    mtk_emit_result_t result = MTK_EMIT_OK;
    if (body && desc && mtk_encode(desc, body, s->response.body, sizeof(s->response.body), &n) != MTK_CODEC_OK) {
        s->response.status = MTK_STATUS_INTERNAL_ERROR;
        n = 0;
        result = MTK_EMIT_ENCODING_FAILED;
    }
    s->response.body_len = n;
    s->response.set = 1; /* set last: a lock-holding reader that only checks `set` still sees a fully-written body/status */
    mtk_fake_sink_unlock();
    return result;
}

static inline mtk_emit_result_t mtk_fake_emit_response_raw(void *user, uint32_t correlation, uint8_t status,
                                               const uint8_t *body, size_t body_len) {
    (void)correlation;
    mtk_fake_sink_state_t *s = (mtk_fake_sink_state_t *)user;
    mtk_fake_sink_lock();
    s->response.status = status;
    if (body_len > sizeof(s->response.body)) {
        s->response.status = MTK_STATUS_OVERFLOW;
        s->response.body_len = 0;
        s->response.set = 1;
        mtk_fake_sink_unlock();
        return MTK_EMIT_CAPACITY_FAILED;
    }
    size_t n = body_len;
    if (body && n) memcpy(s->response.body, body, n);
    s->response.body_len = n;
    s->response.set = 1;
    mtk_fake_sink_unlock();
    return MTK_EMIT_OK;
}

static inline mtk_emit_result_t mtk_fake_emit_event(void *user, uint32_t correlation_or_zero, const char *event_name,
                                        const void *body, const mtk_struct_desc_t *desc) {
    mtk_fake_sink_state_t *s = (mtk_fake_sink_state_t *)user;
    mtk_fake_sink_lock();
    mtk_emit_result_t result = MTK_EMIT_CAPACITY_FAILED;
    if (s->event_count < MTK_FAKE_MAX_EVENTS) {
        mtk_fake_event_t *e = &s->events[s->event_count];
        e->correlation_or_zero = correlation_or_zero;
        strncpy(e->name, event_name, sizeof(e->name) - 1);
        size_t n = 0;
        if (body && desc && mtk_encode(desc, body, e->body, sizeof(e->body), &n) != MTK_CODEC_OK) {
            mtk_fake_sink_unlock();
            return MTK_EMIT_ENCODING_FAILED;
        }
        e->body_len = n;
        s->event_count++;
        result = strlen(event_name) >= sizeof(e->name) ? MTK_EMIT_TRUNCATED : MTK_EMIT_OK;
    }
    mtk_fake_sink_unlock();
    return result;
}

static inline mtk_emit_result_t mtk_fake_emit_stream(void *user, uint32_t session_token, uint32_t sequence,
                                         const uint8_t *chunk, size_t len) {
    mtk_fake_sink_state_t *s = (mtk_fake_sink_state_t *)user;
    mtk_fake_sink_lock();
    mtk_emit_result_t result = MTK_EMIT_CAPACITY_FAILED;
    if (s->stream_count < MTK_FAKE_MAX_STREAMS) {
        mtk_fake_stream_t *c = &s->streams[s->stream_count];
        c->session_token = session_token; c->sequence = sequence;
        c->len = len > sizeof(c->data) ? sizeof(c->data) : len;
        memcpy(c->data, chunk, c->len);
        s->stream_count++;
        result = len > sizeof(c->data) ? MTK_EMIT_TRUNCATED : MTK_EMIT_OK;
    }
    mtk_fake_sink_unlock();
    return result;
}

static inline mtk_sink_t mtk_fake_sink_make(mtk_fake_sink_state_t *state) {
    mtk_sink_t sink;
    sink.user = state;
    sink.emit_response = mtk_fake_emit_response;
    sink.emit_response_raw = mtk_fake_emit_response_raw;
    sink.emit_event = mtk_fake_emit_event;
    sink.emit_stream = mtk_fake_emit_stream;
    return sink;
}

static inline const mtk_fake_event_t *mtk_fake_find_event(mtk_fake_sink_state_t *s, const char *name) {
    for (unsigned i = 0; i < s->event_count; i++) if (strcmp(s->events[i].name, name) == 0) return &s->events[i];
    return NULL;
}
