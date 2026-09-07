/* Clean-room implementation from MonstaTek contract. Canonical core
 * primitive (not compatibility-adapter-specific): a bounded, thread-safe
 * FIFO of pending outbound delivery frames (responses/events/stream
 * chunks) that a persistent, adapter-owned object can hold so an
 * ACCEPTED_ASYNC operation's background completion survives past the
 * request call that accepted it. This is the "persistent adapter-owned
 * queue" the router's own async SAFETY CONTRACT (mtek_router.h) requires
 * before mtk_router_set_async_runner is safe to register.
 *
 * Portable: no ESP-IDF/FreeRTOS dependency. Thread safety is opt-in via
 * caller-supplied lock/unlock hooks (mtk_async_queue_set_lock) -- with no
 * hooks registered, push/pop are safe only from a single thread of
 * control (matching every host test, which is single-threaded unless a
 * test explicitly supplies real pthread-mutex hooks, as
 * test_async_queue.c does to prove real cross-thread correctness).
 * Target glue supplies real FreeRTOS mutex hooks. */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MTK_ASYNC_QUEUE_DEPTH 8
/* Large enough to carry MonstaShark's own largest PUSH-mode stream chunk
 * (mtek_capture_logic.c: up to 8-byte continuation header + 944-byte
 * payload = 952 bytes) without truncation when a capture session's
 * emit_stream calls are relayed through this queue on the native SPI v1
 * adapter (RC5 independent audit P0 "Native events and streams are not
 * implemented") -- a smaller cap here would silently corrupt captured
 * frame data by truncating every oversized chunk before it ever reaches
 * the wire. */
#define MTK_ASYNC_FRAME_MAX_BODY 960
#define MTK_ASYNC_EVENT_NAME_MAX 24

typedef enum {
    MTK_ASYNC_FRAME_RESPONSE = 0, /* a deferred emit_response/emit_response_raw call */
    MTK_ASYNC_FRAME_EVENT,        /* a deferred emit_event call */
    MTK_ASYNC_FRAME_STREAM,       /* a deferred emit_stream call */
} mtk_async_frame_kind_t;

typedef struct {
    mtk_async_frame_kind_t kind;
    uint32_t correlation;    /* RESPONSE: correlation token. EVENT: correlation_or_zero. STREAM: session_token. */
    uint32_t seq_or_status;  /* RESPONSE: wire status (low byte). EVENT: unused (0). STREAM: sequence. */
    char event_name[MTK_ASYNC_EVENT_NAME_MAX]; /* EVENT only; NUL-terminated, truncated if longer. */
    uint8_t body[MTK_ASYNC_FRAME_MAX_BODY];
    size_t body_len;
} mtk_async_frame_t;

typedef void (*mtk_async_lock_fn)(void *lock_ctx);

/* RC7 independent audit P0 "Native scheduling can starve or drop control
 * and terminal traffic": responses, progress events, terminal events, and
 * streams used to share one plain FIFO -- a stream/progress flood could
 * fill it and cause a queued response or terminal event to be DROPPED
 * (silently, since a full queue previously always dropped the NEWEST
 * incoming frame regardless of its own or the buffer's contents'
 * relative importance), and strict FIFO order could deliver a stream
 * ahead of an already-queued response. mtk_async_frame_kind_t's own
 * numeric values (RESPONSE=0, EVENT=1, STREAM=2) are already in the
 * accepted protocol's own priority order (SPI_PROTOCOL_V1.md/
 * 002-canonical-core-contract.md Sec "response/link > event > stream"),
 * so this queue now stores frames as fixed slots (not a ring) and
 * enforces that order structurally at both ends: `mtk_async_queue_push`
 * evicts the LOWEST-priority occupied slot (if any is strictly lower
 * priority than the incoming frame) to make room when full, rather than
 * always dropping the newest arrival -- a response/event can now displace
 * a queued stream chunk, but nothing can ever displace a response;
 * `mtk_async_queue_pop` always returns the highest-priority occupied slot
 * (ties broken by insertion order via `seq`), not simply the oldest.
 * Total capacity is unchanged (MTK_ASYNC_QUEUE_DEPTH slots) -- this is a
 * selection-order change, not a bigger queue. Disclosed scope note: this
 * distinguishes RESPONSE/EVENT/STREAM (three tiers), not a further
 * "terminal vs. progress" sub-priority within EVENT specifically -- no
 * field on mtk_async_frame_t currently tags an event as terminal, and the
 * accepted contract's own "terminal event reserve" budget
 * (MTK_BUDGET_TERMINAL_EVENT_RESERVE, docs/RESOURCE_BUDGET.md) is a
 * canonical-core concept this transport-level queue does not itself
 * re-implement a second time -- see docs/PROVENANCE.md's own gap list. */
typedef struct {
    uint8_t occupied;
    uint32_t seq; /* monotonic insertion order -- FIFO tie-break within the same priority tier */
    mtk_async_frame_t frame;
} mtk_async_slot_t;

typedef struct {
    mtk_async_slot_t slots[MTK_ASYNC_QUEUE_DEPTH];
    uint32_t next_seq;
    unsigned count;
    unsigned dropped_count; /* sticky backpressure counter -- incremented whenever ANY frame (the incoming one, or a lower-priority victim it displaced) is dropped; never blocks, never silently loses count of how many */
    mtk_async_lock_fn lock, unlock;
    void *lock_ctx;
} mtk_async_queue_t;

void mtk_async_queue_init(mtk_async_queue_t *q);

/* Registers real mutual-exclusion hooks bracketing every push/pop/reset/
 * count call below. Must be called before the queue is shared across
 * more than one thread of control. */
void mtk_async_queue_set_lock(mtk_async_queue_t *q, mtk_async_lock_fn lock, mtk_async_lock_fn unlock, void *lock_ctx);

/* Enqueues `frame` (copied by value). Returns 1 on success, 0 if the
 * queue was already full -- the frame is dropped (not blocked on) and
 * `dropped_count` increments; a full queue never blocks the producer
 * (bounded memory, no unbounded backpressure stall). */
int mtk_async_queue_push(mtk_async_queue_t *q, const mtk_async_frame_t *frame);

/* Dequeues the oldest pending frame into `*out`. Returns 1 if one was
 * available, 0 if the queue was empty. */
int mtk_async_queue_pop(mtk_async_queue_t *q, mtk_async_frame_t *out);

unsigned mtk_async_queue_count(mtk_async_queue_t *q);
unsigned mtk_async_queue_dropped_count(mtk_async_queue_t *q);

/* Drops every pending frame immediately (operation cancellation/STOP, or
 * a boot_epoch reset invalidating everything still queued). Does not
 * reset `dropped_count` (a historical backpressure counter, not part of
 * the live queue state). */
void mtk_async_queue_reset(mtk_async_queue_t *q);

#ifdef __cplusplus
}
#endif
