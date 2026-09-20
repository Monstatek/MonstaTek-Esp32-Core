/* mtk_async_queue: bounded thread-safe delivery FIFO -- the canonical
 * core primitive underpinning every adapter's persistent, adapter-owned
 * async response/event queue (mtek_router.h's SAFETY CONTRACT). Proves
 * ordering, overflow/backpressure, reset/cancellation, and -- via a real
 * pthread producer racing a consumer under a real mutex, not a
 * single-threaded simulation -- genuine cross-thread lifetime/ownership
 * safety under AddressSanitizer. */
#include "mtk_test.h"
#include "mtek_async_queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

static void frame_with_seq(mtk_async_frame_t *f, uint32_t seq) {
    memset(f, 0, sizeof(*f));
    f->kind = MTK_ASYNC_FRAME_EVENT;
    f->correlation = seq;
    f->seq_or_status = seq;
    snprintf(f->event_name, sizeof(f->event_name), "EV_%u", seq);
    f->body_len = 4;
    f->body[0] = (uint8_t)seq; f->body[1] = (uint8_t)(seq >> 8); f->body[2] = (uint8_t)(seq >> 16); f->body[3] = (uint8_t)(seq >> 24);
}

/* Ordering (single-threaded) -------------------- */
static void test_ordering(void) {
    mtk_async_queue_t q;
    mtk_async_queue_init(&q);
    for (uint32_t i = 0; i < 5; i++) {
        mtk_async_frame_t f; frame_with_seq(&f, i);
        MTK_CHECK_EQ(mtk_async_queue_push(&q, &f), 1);
    }
    MTK_CHECK_EQ(mtk_async_queue_count(&q), 5);
    for (uint32_t i = 0; i < 5; i++) {
        mtk_async_frame_t out;
        MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
        MTK_CHECK_EQ(out.seq_or_status, i); /* strict FIFO order preserved */
        MTK_CHECK_EQ(out.correlation, i);
    }
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &(mtk_async_frame_t){0}), 0); /* empty */
}

/* Overflow / backpressure ---------------------- */
static void test_overflow_backpressure(void) {
    mtk_async_queue_t q;
    mtk_async_queue_init(&q);
    for (uint32_t i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        mtk_async_frame_t f; frame_with_seq(&f, i);
        MTK_CHECK_EQ(mtk_async_queue_push(&q, &f), 1);
    }
    /* Queue is now full: further pushes are dropped (not blocked), and
     * the drop is counted -- bounded memory, no unbounded backpressure
     * stall for a producer (a background task must never wedge waiting
     * on a slow/absent consumer). */
    mtk_async_frame_t overflow1, overflow2;
    frame_with_seq(&overflow1, 100); frame_with_seq(&overflow2, 101);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &overflow1), 0);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &overflow2), 0);
    MTK_CHECK_EQ(mtk_async_queue_dropped_count(&q), 2);
    MTK_CHECK_EQ(mtk_async_queue_count(&q), MTK_ASYNC_QUEUE_DEPTH);

    /* The frames that WERE accepted are intact, in order, uncorrupted by
     * the dropped pushes. */
    for (uint32_t i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        mtk_async_frame_t out;
        MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
        MTK_CHECK_EQ(out.seq_or_status, i);
    }
    MTK_CHECK_EQ(mtk_async_queue_count(&q), 0);
}

/* Reset / cancellation ------------------------ */
static void test_reset_cancellation(void) {
    mtk_async_queue_t q;
    mtk_async_queue_init(&q);
    for (uint32_t i = 0; i < 4; i++) {
        mtk_async_frame_t f; frame_with_seq(&f, i);
        mtk_async_queue_push(&q, &f);
    }
    MTK_CHECK_EQ(mtk_async_queue_count(&q), 4);
    mtk_async_queue_reset(&q); /* e.g. STOP or boot_epoch reset invalidating everything still queued */
    MTK_CHECK_EQ(mtk_async_queue_count(&q), 0);
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &(mtk_async_frame_t){0}), 0);

    /* The queue is reusable after a reset (a later operation on the same
     * persistent, adapter-owned queue must not be permanently wedged by
     * an earlier operation's cancellation). */
    mtk_async_frame_t f; frame_with_seq(&f, 99);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &f), 1);
    mtk_async_frame_t out;
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
    MTK_CHECK_EQ(out.seq_or_status, 99);
}

/* Real cross-thread lifetime/ownership under a real mutex ------ */
#define THREAD_TEST_N 2000

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void thread_lock(void *ctx) { (void)ctx; pthread_mutex_lock(&s_mutex); }
static void thread_unlock(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_mutex); }

typedef struct { mtk_async_queue_t *q; } producer_arg_t;

static void *producer_thread(void *arg) {
    producer_arg_t *pa = (producer_arg_t *)arg;
    for (uint32_t i = 0; i < THREAD_TEST_N; i++) {
        mtk_async_frame_t f; frame_with_seq(&f, i);
        /* A real background delivery task retries on backpressure rather
         * than dropping in this producer (unlike the single-shot
         * overflow test above) -- exercises the queue under sustained
         * concurrent pressure from both ends. */
        while (!mtk_async_queue_push(pa->q, &f)) { /* spin: consumer is draining concurrently */ }
    }
    return NULL;
}

static void test_real_cross_thread(void) {
    /* Heap-allocated (not stack-local) -- this is the exact shape of a
     * persistent, adapter-owned queue that must survive past the
     * function that kicked off the producer, which is the entire point
     * of this mechanism (mtek_router.h's SAFETY CONTRACT). */
    mtk_async_queue_t *q = malloc(sizeof(*q));
    mtk_async_queue_init(q);
    mtk_async_queue_set_lock(q, thread_lock, thread_unlock, NULL);

    pthread_t producer;
    producer_arg_t pa = { q };
    int rc = pthread_create(&producer, NULL, producer_thread, &pa);
    MTK_CHECK_EQ(rc, 0);

    uint32_t next_expected = 0;
    unsigned guard = 0;
    while (next_expected < THREAD_TEST_N && guard < 20000000u) {
        mtk_async_frame_t out;
        if (mtk_async_queue_pop(q, &out)) {
            /* Ordering holds even under real concurrent pushes: a single
             * producer + single consumer FIFO never reorders. */
            MTK_CHECK_EQ(out.seq_or_status, next_expected);
            MTK_CHECK_EQ(out.body[0], (uint8_t)next_expected);
            next_expected++;
        }
        guard++;
    }
    MTK_CHECK_EQ(next_expected, THREAD_TEST_N);

    pthread_join(producer, NULL);
    MTK_CHECK_EQ(mtk_async_queue_count(q), 0);
    free(q); /* no leak, no use-after-free: the producer has fully joined before this line */
}

/* responses/events must never be dropped or delayed by a stream flood, and pop
 * order must be priority-first (RESPONSE > EVENT > STREAM), not strict FIFO.
 * ------------- */
static void frame_of_kind(mtk_async_frame_t *f, mtk_async_frame_kind_t kind, uint32_t seq) {
    memset(f, 0, sizeof(*f));
    f->kind = kind;
    f->correlation = seq;
    f->seq_or_status = seq;
}

static void test_priority_scheduling(void) {
    mtk_async_queue_t q;
    mtk_async_queue_init(&q);

    /* A STREAM flood filling the queue can never prevent a subsequent
     * RESPONSE from being accepted -- it displaces (evicts) the
     * lowest-priority (a STREAM) occupied slot instead of being dropped
     * itself. Reproduces the exact original failure mode: under the old
     * plain-FIFO drop-newest policy, this RESPONSE push would have been
     * the one dropped. */
    for (uint32_t i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        mtk_async_frame_t f; frame_of_kind(&f, MTK_ASYNC_FRAME_STREAM, i);
        MTK_CHECK_EQ(mtk_async_queue_push(&q, &f), 1);
    }
    mtk_async_frame_t resp; frame_of_kind(&resp, MTK_ASYNC_FRAME_RESPONSE, 999);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &resp), 1); /* accepted, not dropped */
    MTK_CHECK_EQ(mtk_async_queue_count(&q), MTK_ASYNC_QUEUE_DEPTH); /* one STREAM slot was evicted to make room */

    /* pop returns the RESPONSE first, ahead of every still-queued STREAM chunk,
     * regardless of FIFO arrival order -- closing "FIFO delivery can also send a
     * stream before a queued response". */
    mtk_async_frame_t out;
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
    MTK_CHECK_EQ(out.kind, MTK_ASYNC_FRAME_RESPONSE);
    MTK_CHECK_EQ(out.correlation, 999);

    /* An EVENT pushed now also outranks every remaining STREAM. */
    mtk_async_frame_t ev; frame_of_kind(&ev, MTK_ASYNC_FRAME_EVENT, 998);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &ev), 1);
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
    MTK_CHECK_EQ(out.kind, MTK_ASYNC_FRAME_EVENT);
    MTK_CHECK_EQ(out.correlation, 998);

    /* The remaining slots drain as STREAM, in their original FIFO order
     * (one was evicted above -- whichever was pushed first, seq 0). */
    for (uint32_t i = 1; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
        MTK_CHECK_EQ(out.kind, MTK_ASYNC_FRAME_STREAM);
        MTK_CHECK_EQ(out.correlation, i);
    }
    MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 0); /* empty */

    /* The inverse must also hold: a queue genuinely full of RESPONSEs is
     * never displaced by a lower-priority arrival -- nothing can evict a
     * RESPONSE. */
    mtk_async_queue_reset(&q);
    for (uint32_t i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        mtk_async_frame_t f; frame_of_kind(&f, MTK_ASYNC_FRAME_RESPONSE, i);
        MTK_CHECK_EQ(mtk_async_queue_push(&q, &f), 1);
    }
    mtk_async_frame_t stream_arrival; frame_of_kind(&stream_arrival, MTK_ASYNC_FRAME_STREAM, 777);
    MTK_CHECK_EQ(mtk_async_queue_push(&q, &stream_arrival), 0); /* dropped -- nothing lower-priority to evict */
    for (uint32_t i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++) {
        MTK_CHECK_EQ(mtk_async_queue_pop(&q, &out), 1);
        MTK_CHECK_EQ(out.kind, MTK_ASYNC_FRAME_RESPONSE);
        MTK_CHECK_EQ(out.correlation, i); /* every original RESPONSE survived, in FIFO order */
    }
}

MTK_TEST_MAIN_BEGIN
    test_ordering();
    test_overflow_backpressure();
    test_reset_cancellation();
    test_priority_scheduling();
    test_real_cross_thread();
MTK_TEST_MAIN_END
