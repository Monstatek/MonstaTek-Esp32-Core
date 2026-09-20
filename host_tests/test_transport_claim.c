/* mtk_transport_claim_try is the cross-transport (UART vs. SPI) boot-session
 * exclusivity latch that makes real AUTO selection across all three physical
 * adapters safe -- "Exactly one profile may dispatch after selection" even
 * though UART0 and the shared SPI bus run their own physical loops fully
 * concurrently before that point. Proves single-threaded first-wins semantics,
 * idempotency for the winner, permanent rejection of every other contender, and
 * -- the actual concurrency property this exists for -- that exactly one of two
 * genuinely concurrent pthread callers ever wins when both race the same claim
 * at once, with never a double-win. */
#include "mtk_test.h"
#include "mtek_transport_select.h"
#include <pthread.h>

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static void thread_lock(void) { pthread_mutex_lock(&s_mutex); }
static void thread_unlock(void) { pthread_mutex_unlock(&s_mutex); }

static void test_single_threaded_first_wins(void) {
    mtk_transport_claim_set_lock(thread_lock, thread_unlock);
    mtk_transport_claim_reset();
    MTK_CHECK_EQ(mtk_transport_claim_get(), MTK_PUBLIC_ADAPTER_NONE);

    /* First claimant wins. */
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_FACTORY_UART), 1);
    MTK_CHECK_EQ(mtk_transport_claim_get(), MTK_PUBLIC_ADAPTER_FACTORY_UART);

    /* A different adapter arriving after is permanently rejected. */
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_NATIVE_SPI), 0);
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_COMPAT_C3_SPI), 0);
    MTK_CHECK_EQ(mtk_transport_claim_get(), MTK_PUBLIC_ADAPTER_FACTORY_UART); /* unchanged */

    /* The winner re-attempting (e.g. a second command line on the same
     * adapter) is idempotently still a win, never demoted. */
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_FACTORY_UART), 1);

    /* Rejected contenders stay rejected even after the winner re-confirms. */
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_NATIVE_SPI), 0);
}

static void test_reset_allows_a_new_boot_session(void) {
    mtk_transport_claim_set_lock(thread_lock, thread_unlock);
    mtk_transport_claim_reset();
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_NATIVE_SPI), 1);
    mtk_transport_claim_reset(); /* e.g. a fresh boot */
    MTK_CHECK_EQ(mtk_transport_claim_get(), MTK_PUBLIC_ADAPTER_NONE);
    MTK_CHECK_EQ(mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_COMPAT_C3_SPI), 1);
}

/* Real concurrent race: exactly one of two genuinely simultaneous pthread
 * callers ever wins, never both, never neither. -------- */
#define RACE_ITERS 500

typedef struct { mtk_public_adapter_t which; int wins; } racer_arg_t;

static void *racer(void *arg) {
    racer_arg_t *r = (racer_arg_t *)arg;
    r->wins = mtk_transport_claim_try(r->which);
    return NULL;
}

static void test_concurrent_race_exactly_one_winner(void) {
    mtk_transport_claim_set_lock(thread_lock, thread_unlock);
    for (int i = 0; i < RACE_ITERS; i++) {
        mtk_transport_claim_reset();
        racer_arg_t a = { MTK_PUBLIC_ADAPTER_FACTORY_UART, 0 };
        racer_arg_t b = { MTK_PUBLIC_ADAPTER_NATIVE_SPI, 0 };
        pthread_t ta, tb;
        MTK_CHECK_EQ(pthread_create(&ta, NULL, racer, &a), 0);
        MTK_CHECK_EQ(pthread_create(&tb, NULL, racer, &b), 0);
        pthread_join(ta, NULL);
        pthread_join(tb, NULL);
        /* Exactly one of the two genuinely raced calls won -- never both
         * (a double-dispatch hazard, the exact defect this primitive
         * exists to prevent) and never neither (every call above always
         * either wins or is rejected against a claim that IS set). */
        MTK_CHECK_EQ(a.wins + b.wins, 1);
        mtk_public_adapter_t final = mtk_transport_claim_get();
        MTK_CHECK(final == MTK_PUBLIC_ADAPTER_FACTORY_UART || final == MTK_PUBLIC_ADAPTER_NATIVE_SPI);
        MTK_CHECK_EQ(final == MTK_PUBLIC_ADAPTER_FACTORY_UART, a.wins);
        MTK_CHECK_EQ(final == MTK_PUBLIC_ADAPTER_NATIVE_SPI, b.wins);
    }
}

MTK_TEST_MAIN_BEGIN
    test_single_threaded_first_wins();
    test_reset_allows_a_new_boot_session();
    test_concurrent_race_exactly_one_winner();
MTK_TEST_MAIN_END
