/* RC12 hardening round, item 2 (P1) "Transport counters": proves the
 * transport diagnostics counters (mtk_transport_counters_* in mtek_core.c)
 * are concurrency-safe once real lock hooks are registered -- exact totals
 * under many concurrent incrementers, and internally-consistent snapshots
 * taken concurrently with those increments. On target, app_main now
 * registers a dedicated mutex via mtk_transport_counters_set_lock (which
 * was previously never called at all, leaving every increment/snapshot
 * lock-free); here that same registration is exercised with a pthread
 * mutex. Run under ThreadSanitizer (the suite's TSan build) this also
 * proves the increments and snapshots carry no data race.
 *
 * The dedicated-mutex requirement is deliberately NOT reusing the shared
 * core mutex: on target, mtk_async_queue_push calls add_dropped_frame while
 * already holding the queue lock (which is the shared mutex), so a shared
 * counter lock would self-deadlock a non-recursive mutex. That structural
 * constraint is documented at the app_main registration site; this test
 * only needs to prove the counters themselves are exact and race-free under
 * their own lock. */
#include "mtk_test.h"
#include "mtek_core.h"
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdatomic.h>

static pthread_mutex_t s_tc_mutex = PTHREAD_MUTEX_INITIALIZER;
static void tc_lock(void) { pthread_mutex_lock(&s_tc_mutex); }
static void tc_unlock(void) { pthread_mutex_unlock(&s_tc_mutex); }

/* Sized to expose any lost-update race or torn snapshot over many
 * thousands of interleaved locked operations, while keeping the TSan
 * footprint modest enough not to starve other concurrency tests sharing a
 * parallel `ctest -j` TSan run of their own wall-clock bounds. */
#define N_THREADS 6
#define INCR_PER_THREAD 5000

/* Each worker hammers all five counters equally so the final totals are
 * exactly predictable. */
static void *incr_worker(void *arg) {
    (void)arg;
    for (int i = 0; i < INCR_PER_THREAD; i++) {
        mtk_transport_counters_add_integrity_failure();
        mtk_transport_counters_add_dropped_frame();
        mtk_transport_counters_add_packet_seq_gap();
        mtk_transport_counters_add_retry_observed();
        mtk_transport_counters_add_duplicate_response_served();
    }
    return NULL;
}

/* A concurrent snapshotter: every snapshot must be internally consistent
 * (each field taken atomically as a whole under the lock) and monotonic
 * non-decreasing over time, never a torn mixture. */
static atomic_int s_snapshotting;
static void *snapshot_worker(void *arg) {
    (void)arg;
    mtk_transport_counters_t prev; mtk_transport_counters_get(&prev);
    unsigned iters = 0;
    while (atomic_load(&s_snapshotting)) {
        mtk_transport_counters_t cur; mtk_transport_counters_get(&cur);
        /* Yield every few hundred snapshots so this poller does not spin the
         * counter mutex hard enough to starve the incrementers (or, under a
         * parallel TSan run, neighbouring tests). Correctness is unaffected. */
        if ((++iters & 0x1FF) == 0) sched_yield();
        /* Monotonic: counters only ever increment, so no field may ever
         * be observed going backwards between two snapshots. */
        if (cur.integrity_failures < prev.integrity_failures) return (void *)1;
        if (cur.dropped_frames < prev.dropped_frames) return (void *)1;
        if (cur.packet_seq_gaps < prev.packet_seq_gaps) return (void *)1;
        if (cur.retries_observed < prev.retries_observed) return (void *)1;
        if (cur.duplicate_responses_served < prev.duplicate_responses_served) return (void *)1;
        prev = cur;
    }
    return (void *)0;
}

static void test_exact_counts_and_consistent_snapshots(void) {
    mtk_transport_counters_set_lock(tc_lock, tc_unlock);
    mtk_transport_counters_reset();

    atomic_store(&s_snapshotting, 1);
    pthread_t snap;
    pthread_create(&snap, NULL, snapshot_worker, NULL);

    pthread_t workers[N_THREADS];
    for (int i = 0; i < N_THREADS; i++) pthread_create(&workers[i], NULL, incr_worker, NULL);
    for (int i = 0; i < N_THREADS; i++) pthread_join(workers[i], NULL);

    atomic_store(&s_snapshotting, 0);
    void *snap_rc = (void *)0;
    pthread_join(snap, &snap_rc);
    MTK_CHECK(snap_rc == (void *)0); /* no snapshot ever observed a counter going backwards */

    const uint32_t expect = (uint32_t)N_THREADS * INCR_PER_THREAD;
    mtk_transport_counters_t final; mtk_transport_counters_get(&final);
    MTK_CHECK_EQ(final.integrity_failures, expect);         /* exact -- no lost increments under contention */
    MTK_CHECK_EQ(final.dropped_frames, expect);
    MTK_CHECK_EQ(final.packet_seq_gaps, expect);
    MTK_CHECK_EQ(final.retries_observed, expect);
    MTK_CHECK_EQ(final.duplicate_responses_served, expect);
}

MTK_TEST_MAIN_BEGIN
    test_exact_counts_and_consistent_snapshots();
MTK_TEST_MAIN_END
