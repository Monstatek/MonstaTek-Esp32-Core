/* Clean-room implementation from MonstaTek contract. See
 * mtek_async_queue.h for the design rationale (RC7 independent audit P0
 * "Native scheduling can starve or drop control and terminal traffic":
 * fixed priority slots -- RESPONSE > EVENT > STREAM -- replacing the
 * original plain FIFO). */
#include "mtek_async_queue.h"
#include "mtek_core.h"
#include <string.h>

void mtk_async_queue_init(mtk_async_queue_t *q) {
    memset(q, 0, sizeof(*q));
}

void mtk_async_queue_set_lock(mtk_async_queue_t *q, mtk_async_lock_fn lock, mtk_async_lock_fn unlock, void *lock_ctx) {
    q->lock = lock;
    q->unlock = unlock;
    q->lock_ctx = lock_ctx;
}

static void do_lock(mtk_async_queue_t *q) { if (q->lock) q->lock(q->lock_ctx); }
static void do_unlock(mtk_async_queue_t *q) { if (q->unlock) q->unlock(q->lock_ctx); }

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
