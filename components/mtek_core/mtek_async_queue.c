/* Clean-room implementation from MonstaTek contract. See mtek_async_queue.h for
 * the design rationale (fixed priority slots -- RESPONSE > EVENT > STREAM --
 * replacing the original plain FIFO). */
#include "mtek_async_queue.h"
#include "mtek_async_sink.h"
#include "mtek_codec_api.h"
#include "mtek_core.h"
#include <string.h>

void mtk_async_queue_init(mtk_async_queue_t *q) {
    memset(q, 0, sizeof(*q));
}

mtk_emit_result_t mtk_async_sink_resp(void *user, uint32_t correlation, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_RESPONSE;
    f.correlation = correlation;
    f.seq_or_status = status;
    if (body && desc && mtk_encode(desc, body, f.body, sizeof(f.body), &f.body_len) != MTK_CODEC_OK)
        return MTK_EMIT_ENCODING_FAILED;
    if (!mtk_async_queue_push((mtk_async_queue_t *)user, &f)) return MTK_EMIT_CAPACITY_FAILED;
    return MTK_EMIT_OK;
}

mtk_emit_result_t mtk_async_sink_resp_raw(void *user, uint32_t correlation, uint8_t status, const uint8_t *body, size_t len) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_RESPONSE;
    f.correlation = correlation;
    f.seq_or_status = status;
    if (len > sizeof(f.body)) return MTK_EMIT_CAPACITY_FAILED;
    f.body_len = len;
    if (body) memcpy(f.body, body, f.body_len);
    if (!mtk_async_queue_push((mtk_async_queue_t *)user, &f)) return MTK_EMIT_CAPACITY_FAILED;
    return MTK_EMIT_OK;
}

mtk_emit_result_t mtk_async_sink_event(void *user, uint32_t correlation_or_zero, const char *name, const void *body, const mtk_struct_desc_t *desc) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_EVENT;
    f.correlation = correlation_or_zero;
    if (name) { size_t n = strlen(name); if (n >= sizeof(f.event_name)) n = sizeof(f.event_name) - 1; memcpy(f.event_name, name, n); }
    if (body && desc && mtk_encode(desc, body, f.body, sizeof(f.body), &f.body_len) != MTK_CODEC_OK)
        return MTK_EMIT_ENCODING_FAILED;
    if (!mtk_async_queue_push((mtk_async_queue_t *)user, &f)) return MTK_EMIT_CAPACITY_FAILED;
    return name && strlen(name) >= sizeof(f.event_name) ? MTK_EMIT_TRUNCATED : MTK_EMIT_OK;
}

mtk_emit_result_t mtk_async_sink_stream(void *user, uint32_t session_token, uint32_t seq, const uint8_t *chunk, size_t len) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_STREAM;
    f.correlation = session_token;
    f.seq_or_status = seq;
    f.body_len = len > sizeof(f.body) ? sizeof(f.body) : len;
    if (chunk) memcpy(f.body, chunk, f.body_len);
    if (!mtk_async_queue_push((mtk_async_queue_t *)user, &f)) return MTK_EMIT_CAPACITY_FAILED;
    return len > sizeof(f.body) ? MTK_EMIT_TRUNCATED : MTK_EMIT_OK;
}

void mtk_async_queue_set_lock(mtk_async_queue_t *q, mtk_async_lock_fn lock, mtk_async_lock_fn unlock, void *lock_ctx) {
    q->lock = lock;
    q->unlock = unlock;
    q->lock_ctx = lock_ctx;
}

static void do_lock(mtk_async_queue_t *q) { if (q->lock) q->lock(q->lock_ctx); }
static void do_unlock(mtk_async_queue_t *q) { if (q->unlock) q->unlock(q->lock_ctx); }

void mtk_async_queue_set_notify(mtk_async_queue_t *q, mtk_async_notify_fn notify, void *notify_ctx) {
    q->notify = notify;
    q->notify_ctx = notify_ctx;
}

/* Returns the index of the empty slot, or -1 if none. */
static int find_empty_slot(mtk_async_queue_t *q) {
    for (int i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) if (!q->slots[i].occupied) return i;
    return -1;
}

/* Returns the index of the occupied slot with the LOWEST priority
 * (highest mtk_async_frame_kind_t numeric value; ties broken toward the
 * OLDEST -- the natural eviction candidate), or -1 if the queue is
 * empty. */
static int find_lowest_priority_slot(mtk_async_queue_t *q) {
    int best = -1;
    for (int i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        if (!q->slots[i].occupied) continue;
        if (best < 0) { best = i; continue; }
        if (q->slots[i].frame.kind > q->slots[best].frame.kind) { best = i; continue; }
        if (q->slots[i].frame.kind == q->slots[best].frame.kind && q->slots[i].seq < q->slots[best].seq) { best = i; }
    }
    return best;
}

/* Returns the index of the occupied slot with the HIGHEST priority
 * (lowest mtk_async_frame_kind_t numeric value; ties broken toward the
 * OLDEST, i.e. FIFO within a tier), or -1 if the queue is empty. */
static int find_highest_priority_slot(mtk_async_queue_t *q) {
    int best = -1;
    for (int i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        if (!q->slots[i].occupied) continue;
        if (best < 0) { best = i; continue; }
        if (q->slots[i].frame.kind < q->slots[best].frame.kind) { best = i; continue; }
        if (q->slots[i].frame.kind == q->slots[best].frame.kind && q->slots[i].seq < q->slots[best].seq) { best = i; }
    }
    return best;
}

int mtk_async_queue_push(mtk_async_queue_t *q, const mtk_async_frame_t *frame) {
    do_lock(q);
    int ok;
    int slot = find_empty_slot(q);
    if (slot < 0) {
        /* Full: only displace a slot that is STRICTLY lower priority than
         * the incoming frame (a response may evict a queued stream chunk;
         * a stream may never evict anything). If every occupied slot is
         * the same or higher priority than the incoming frame, the
         * incoming frame itself is dropped instead -- this queue never
         * drops a response/event to make room for a lower- or equal-
         * priority arrival. */
        int victim = find_lowest_priority_slot(q);
        if (victim >= 0 && q->slots[victim].frame.kind > frame->kind) {
            slot = victim;
            q->dropped_count++; /* the displaced frame is lost */
            mtk_transport_counters_add_dropped_frame(); /* GET_TRANSPORT_COUNTERS.dropped_frames -- real, live count, any adapter's queue */
        }
    }
    if (slot < 0) {
        q->dropped_count++; /* the incoming frame itself is dropped */
        mtk_transport_counters_add_dropped_frame();
        ok = 0;
    } else {
        if (!q->slots[slot].occupied) q->count++;
        q->slots[slot].occupied = 1;
        q->slots[slot].seq = q->next_seq++;
        q->slots[slot].frame = *frame;
        ok = 1;
    }
    do_unlock(q);
    if (ok && q->notify) q->notify(q->notify_ctx);
    return ok;
}

int mtk_async_queue_pop(mtk_async_queue_t *q, mtk_async_frame_t *out) {
    do_lock(q);
    int ok;
    int slot = find_highest_priority_slot(q);
    if (slot < 0) {
        ok = 0;
    } else {
        *out = q->slots[slot].frame;
        q->slots[slot].occupied = 0;
        q->count--;
        ok = 1;
    }
    do_unlock(q);
    return ok;
}

unsigned mtk_async_queue_count(mtk_async_queue_t *q) {
    do_lock(q);
    unsigned c = q->count;
    do_unlock(q);
    return c;
}

unsigned mtk_async_queue_dropped_count(mtk_async_queue_t *q) {
    do_lock(q);
    unsigned c = q->dropped_count;
    do_unlock(q);
    return c;
}

void mtk_async_queue_reset(mtk_async_queue_t *q) {
    do_lock(q);
    for (int i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) q->slots[i].occupied = 0;
    q->count = 0;
    do_unlock(q);
}
