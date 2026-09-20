/* Clean-room implementation from MonstaTek contract. Central dispatch point
 * every adapter calls into after constructing a canonical request context: looks
 * the opcode up in the generated registry, applies the profile's
 * capability-state gate, and -- only if SUPPORTED -- hands off to the owning
 * service's registered handler. */
#pragma once
#include "mtek_core.h"
#include "mtek_opcode_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A service handler receives the still-undecoded request payload and is
 * responsible for its own decode (via mtk_decode against op->req_desc),
 * business logic, and exactly one emit_response call (plus zero or more
 * emit_event/emit_stream calls for ACCEPTED_ASYNC operations). */
typedef void (*mtk_service_dispatch_fn)(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                         const uint8_t *req_bytes, size_t req_len);

void mtk_router_init(void);
typedef enum {
    MTK_REGISTER_OK = 0,
    MTK_REGISTER_INVALID_HANDLER,
    MTK_REGISTER_DUPLICATE,
    MTK_REGISTER_FULL,
} mtk_register_result_t;
/* Startup-only. Failure leaves the registry unchanged. */
mtk_register_result_t mtk_router_register(uint16_t service_id, mtk_service_dispatch_fn fn);

/* Optional async runner: 0 means ownership of fn(arg) was accepted; a
 * nonzero result means fn was not scheduled. Deferred requests are copied
 * into a bounded pool before scheduling. Pool/spawn failure emits NO_MEMORY.
 *
 * Only ACCEPTED_ASYNC requests with dispatch_mode=DEFER_ALLOWED may be
 * deferred. This mode is a caller promise: sink.user and everything its
 * callbacks access must survive the entire async operation. The router
 * copies pointers, not their storage. A borrowed synchronous capture must
 * select INLINE; factory UART does so even for its persistent event sink.
 * Wire/capability profile does not determine execution eligibility.
 * Configure runner and pool locks before concurrent dispatch begins. */
typedef int (*mtk_async_runner_fn)(void (*fn)(void *arg), void *arg);
void mtk_router_set_async_runner(mtk_async_runner_fn runner);

/* True only while the CALLING THREAD is itself executing inside a
 * deferred async worker's handler call (thread-local, not a global "is a
 * runner registered anywhere" flag -- with up to 4 genuinely concurrent
 * deferred operations, a global flag would be wrong the moment more than
 * one worker is active at once). A handler for a genuinely open-ended,
 * "run until stopped" ACCEPTED_ASYNC operation (e.g. DEAUTH_START
 * count=0 -- mtek_wifi_logic.c) can query this to decide whether it is
 * safe to block indefinitely waiting for a concurrent STOP: even with a
 * runner registered, an INLINE dispatch is never deferred
 * (the factory UART adapter explicitly selects INLINE) and this reports 0 for
 * it, exactly like the fully-synchronous case (no runner registered at
 * all, matching every host test that does not itself register one) --
 * in both, nothing else could ever dispatch a concurrent STOP, so
 * blocking indefinitely would hang the caller permanently rather than
 * implement "until stopped". Only reports 1 for code genuinely running
 * on a background worker task, where the transport's own request-
 * dispatch loop remains free to receive and act on that STOP. */
int mtk_router_running_on_worker(void);

/* Optional lock hooks for the async pool's allocate/release bookkeeping
 * (ESP32 target glue only). If registered, `lock`/`unlock` bracket every
 * pool-slot allocation and release, since the release (in async_trampoline,
 * running on the worker task) and the allocation (in mtk_router_dispatch,
 * running on whichever task/ISR context calls it) can race otherwise.
 * Defaults to no-op, which is only safe when mtk_router_dispatch is never
 * called from more than one thread of control at a time (true for every
 * host test, which is single-threaded, and for a target build that has
 * not registered an async runner at all). */
typedef void (*mtk_router_lock_fn)(void);
void mtk_router_set_lock(mtk_router_lock_fn lock, mtk_router_lock_fn unlock);

/* Looks up (service_id, opcode), applies the capability-state closure rule
 * (DISABLED/UNAVAILABLE/UNSUPPORTED capability -> wire status UNSUPPORTED,
 * no side effect, opcode never reaches the service), and otherwise
 * dispatches. Always calls ctx->sink.emit_response exactly once itself
 * when it rejects before dispatch; a dispatched call's response is the
 * service handler's own responsibility. */
void mtk_router_dispatch(mtk_request_ctx_t *ctx, uint16_t service_id, uint16_t opcode,
                          const uint8_t *req_bytes, size_t req_len);

#ifdef __cplusplus
}
#endif
