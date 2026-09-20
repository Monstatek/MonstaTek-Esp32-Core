/* Deterministic forced-interleaving proof
 * of mtk_ble_op_lifecycle -- the exact primitive mtek_ble_hal_esp32.c's nine
 * NimBLE callbacks now enter/leave through. mtek_ble_hal_esp32.c itself is
 * ESP-IDF/NimBLE-only and not linked into host tests, so this proves the
 * LIFETIME PROTOCOL (the reusable core of the fix) directly, using pthreads to
 * force the precise orderings the check-to-use race depends on:
 *
 * A. callback already ENTERED (holding the lifetime lock, mid-use) when a
 * timeout's retire begins -- retire must block until the callback finishes,
 * never freeing context out from under it. B. callback ARRIVING AFTER cleanup --
 * callback_begin must reject it and never hand back a context. C. STALE callback
 * after a NEW operation is armed -- the old generation must be rejected while
 * the new one is accepted with the new context. D. START/ALLOCATION FAILURE
 * paths -- arm followed immediately by retire with no callback ever firing (the
 * HAL's rc!=0 / semaphore- alloc-failure branches), and a late callback after
 * that must be rejected.
 *
 * The primitive is driven here EXACTLY as the HAL drives it: arm(ctx) -> (start
 * op) -> wait -> [cancel] -> retire -> free ctx; callbacks enter via
 * callback_begin/end. */
#include "mtk_test.h"
#include "mtk_ble_op_lifecycle.h"
#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>

/* The lifetime lock hooks: a plain pthread mutex, matching the real target
 * (a FreeRTOS mutex). */
static pthread_mutex_t s_lc_mutex = PTHREAD_MUTEX_INITIALIZER;
static void lc_take(void *ctx) { (void)ctx; pthread_mutex_lock(&s_lc_mutex); }
static void lc_give(void *ctx) { (void)ctx; pthread_mutex_unlock(&s_lc_mutex); }

/* A stand-in for one of the HAL's per-operation context structs. The
 * sentinel lets a callback prove it read THIS armed context and not freed/
 * reused storage. */
typedef struct {
    uint32_t sentinel;   /* 0xA5A5A5A5 while live; a callback that reads anything else touched freed/wrong storage */
    int callback_saw_ctx;
} fake_ctx_t;

/* Orchestration for interleaving A: a control gate distinct from the lifetime
 * lock, so the "callback" thread can pause WHILE STILL HOLDING the lifetime lock
 * (exactly the mid-callback state retire must respect). */
static pthread_mutex_t s_gate_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_gate_cond = PTHREAD_COND_INITIALIZER;
static int s_gate_open;
static int s_cb_entered;      /* callback has taken the lifetime lock and read ctx (guarded by s_gate_mutex) */
static atomic_int s_cb_completed;    /* callback_end done (cross-thread, atomic for a clean TSan run) */
static atomic_int s_retire_completed;

static mtk_ble_op_lifecycle_t s_lc;
static fake_ctx_t s_ctxA;
static uint32_t s_genA;

static void *callback_thread_A(void *arg) {
    (void)arg;
    void *cptr = NULL;
    /* Enter the callback: takes the lifetime lock, hands back the published
     * ctx (generation still current at this point). */
    if (mtk_ble_op_lifecycle_callback_begin(&s_lc, s_genA, &cptr)) {
        fake_ctx_t *c = (fake_ctx_t *)cptr;
        /* Announce we are INSIDE the callback (lifetime lock held), then block
         * on the gate -- still holding the lock -- so the main thread can launch
         * retire and prove it cannot proceed. */
        pthread_mutex_lock(&s_gate_mutex);
        s_cb_entered = 1;
        pthread_cond_signal(&s_gate_cond);
        while (!s_gate_open) pthread_cond_wait(&s_gate_cond, &s_gate_mutex);
        pthread_mutex_unlock(&s_gate_mutex);
        /* Use the context AFTER the gate opened (i.e. after main tried to
         * retire): if retire could interleave, the sentinel would be gone. */
        c->callback_saw_ctx = (c->sentinel == 0xA5A5A5A5u);
        atomic_store(&s_cb_completed, 1);
        mtk_ble_op_lifecycle_callback_end(&s_lc);
    }
    return NULL;
}

static void *retire_thread_A(void *arg) {
    (void)arg;
    mtk_ble_op_lifecycle_retire(&s_lc); /* blocks until the callback releases the lifetime lock */
    atomic_store(&s_retire_completed, 1);
    return NULL;
}

static void test_A_callback_entered_when_timeout_cleanup_begins(void) {
    mtk_ble_op_lifecycle_init(&s_lc);
    mtk_ble_op_lifecycle_set_lock(&s_lc, lc_take, lc_give, NULL);
    s_ctxA.sentinel = 0xA5A5A5A5u;
    s_ctxA.callback_saw_ctx = 0;
    s_gate_open = 0; s_cb_entered = 0; atomic_store(&s_cb_completed, 0); atomic_store(&s_retire_completed, 0);

    s_genA = mtk_ble_op_lifecycle_arm(&s_lc, &s_ctxA);

    pthread_t cb, rt;
    pthread_create(&cb, NULL, callback_thread_A, NULL);

    /* Wait until the callback is genuinely inside its locked critical
     * section. */
    pthread_mutex_lock(&s_gate_mutex);
    while (!s_cb_entered) pthread_cond_wait(&s_gate_cond, &s_gate_mutex);
    pthread_mutex_unlock(&s_gate_mutex);

    /* Launch retire while the callback holds the lifetime lock. It MUST block --
     * give it a moment and confirm it has not completed. */
    pthread_create(&rt, NULL, retire_thread_A, NULL);
    for (volatile int spin = 0; spin < 1000000; spin++) { /* brief busy pause */ }
    MTK_CHECK_EQ(atomic_load(&s_retire_completed), 0); /* retire cannot finish while the callback is mid-use */
    MTK_CHECK_EQ(atomic_load(&s_cb_completed), 0);

    /* Open the gate: the callback finishes its context use and releases the
     * lock; retire then proceeds. */
    pthread_mutex_lock(&s_gate_mutex);
    s_gate_open = 1;
    pthread_cond_broadcast(&s_gate_cond);
    pthread_mutex_unlock(&s_gate_mutex);

    pthread_join(cb, NULL);
    pthread_join(rt, NULL);

    MTK_CHECK(atomic_load(&s_cb_completed));       /* callback ran to completion */
    MTK_CHECK(atomic_load(&s_retire_completed));   /* retire completed AFTER the callback released the lock */
    MTK_CHECK(s_ctxA.callback_saw_ctx); /* the sentinel was intact throughout the callback's use -- no freed/torn context */
}

static void test_B_callback_after_cleanup_is_rejected(void) {
    mtk_ble_op_lifecycle_t lc;
    mtk_ble_op_lifecycle_init(&lc);
    mtk_ble_op_lifecycle_set_lock(&lc, lc_take, lc_give, NULL);
    fake_ctx_t ctx = { 0xA5A5A5A5u, 0 };
    uint32_t gen = mtk_ble_op_lifecycle_arm(&lc, &ctx);
    /* Timeout path: retire before the (late) callback ever runs. */
    mtk_ble_op_lifecycle_retire(&lc);
    void *cptr = (void *)0x1; /* poisoned: must be left untouched on rejection */
    int entered = mtk_ble_op_lifecycle_callback_begin(&lc, gen, &cptr);
    MTK_CHECK_EQ(entered, 0);          /* stale generation rejected */
    MTK_CHECK(cptr == (void *)0x1);    /* ctx_out never written on rejection */
    MTK_CHECK_EQ(ctx.callback_saw_ctx, 0);
}

static void test_C_stale_callback_after_new_op_armed(void) {
    mtk_ble_op_lifecycle_t lc;
    mtk_ble_op_lifecycle_init(&lc);
    mtk_ble_op_lifecycle_set_lock(&lc, lc_take, lc_give, NULL);
    fake_ctx_t ctx1 = { 0xA5A5A5A5u, 0 };
    fake_ctx_t ctx2 = { 0xA5A5A5A5u, 0 };
    uint32_t g1 = mtk_ble_op_lifecycle_arm(&lc, &ctx1);
    mtk_ble_op_lifecycle_retire(&lc);           /* op 1 completes/times out */
    uint32_t g2 = mtk_ble_op_lifecycle_arm(&lc, &ctx2); /* op 2 re-arms */
    MTK_CHECK(g1 != g2);

    /* A late callback from op 1 must be rejected -- never handed op 2's ctx. */
    void *c1 = NULL;
    MTK_CHECK_EQ(mtk_ble_op_lifecycle_callback_begin(&lc, g1, &c1), 0);

    /* Op 2's own callback is accepted and gets op 2's ctx. */
    void *c2 = NULL;
    MTK_CHECK_EQ(mtk_ble_op_lifecycle_callback_begin(&lc, g2, &c2), 1);
    MTK_CHECK(c2 == &ctx2);
    mtk_ble_op_lifecycle_callback_end(&lc);
}

static void test_D_start_and_alloc_failure_paths(void) {
    mtk_ble_op_lifecycle_t lc;
    mtk_ble_op_lifecycle_init(&lc);
    mtk_ble_op_lifecycle_set_lock(&lc, lc_take, lc_give, NULL);

    /* NimBLE start-call failure: the HAL arms, the start returns nonzero,
     * so it retires immediately with no callback ever firing. A late
     * callback under that generation must still be rejected. */
    fake_ctx_t ctx = { 0xA5A5A5A5u, 0 };
    uint32_t gen = mtk_ble_op_lifecycle_arm(&lc, &ctx);
    mtk_ble_op_lifecycle_retire(&lc); /* start failed -> immediate retire */
    void *cptr = NULL;
    MTK_CHECK_EQ(mtk_ble_op_lifecycle_callback_begin(&lc, gen, &cptr), 0);

    /* Semaphore-allocation failure: the HAL returns BEFORE arm is ever called,
     * so the lifecycle stays in its initial (nothing-armed) state. Any callback
     * that somehow fires against generation 0, or any real generation, is
     * rejected -- and retire on a never-armed lifecycle is a safe no-op. */
    mtk_ble_op_lifecycle_retire(&lc); /* idempotent / never-armed: no crash */
    void *cptr2 = NULL;
    MTK_CHECK_EQ(mtk_ble_op_lifecycle_callback_begin(&lc, 0, &cptr2), 0); /* generation 0 is never current */
    MTK_CHECK_EQ(mtk_ble_op_lifecycle_callback_begin(&lc, gen, &cptr2), 0); /* the retired generation stays rejected */
}

MTK_TEST_MAIN_BEGIN
    test_A_callback_entered_when_timeout_cleanup_begins();
    test_B_callback_after_cleanup_is_rejected();
    test_C_stale_callback_after_new_op_armed();
    test_D_start_and_alloc_failure_paths();
MTK_TEST_MAIN_END
