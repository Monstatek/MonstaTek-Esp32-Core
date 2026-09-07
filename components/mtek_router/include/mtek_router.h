/* Clean-room implementation from MonstaTek contract
 * (002-service-registry.md Sec 3.0a capability-state/wire-status closure).
 * Central dispatch point every adapter calls into after constructing a
 * canonical request context: looks the opcode up in the generated
 * registry, applies the profile's capability-state gate, and -- only if
 * SUPPORTED -- hands off to the owning service's registered handler. */
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
void mtk_router_register(uint16_t service_id, mtk_service_dispatch_fn fn);

/* Optional async execution hook (ESP32 target glue only; never registered
 * by host tests, which keep the deterministic synchronous-call model).
 * When set, every ACCEPTED_ASYNC opcode's handler runs via `runner`
 * (expected to spawn a bounded FreeRTOS worker task and call `fn(arg)` on
 * it, returning 0 on success or nonzero if task creation itself failed)
 * instead of on the calling thread, so a long-running scan/deauth/
 * capture/connect never stalls the adapter's own request-dispatch loop
 * and STOP/status/other requests on the same transport remain
 * servicable while it runs. mtk_router_dispatch copies the request bytes
 * and request context into a bounded internal pool (matching the
 * 4-in-flight-request core budget) before invoking `runner`, since the
 * BORROWED request payload is otherwise only valid for the synchronous
 * duration of the call (002-canonical-core-contract.md Sec 2 `payload`
 * row). Pool exhaustion, and a `runner` that reports it failed to spawn a
 * task, are both checked failures: the caller receives NO_MEMORY
 * synchronously, never a silent drop.
 *
 * SAFETY CONTRACT (binding on every caller of mtk_router_dispatch, not
 * just this header): `ctx->sink.user` -- and anything the sink callbacks
 * dereference through it -- must remain valid for the *entire* async
 * operation's lifetime once this runner is registered, not merely for
 * the synchronous duration of the dispatch call. Every sink function
 * pointer in `ctx->sink` is copied by value into the pool slot and
 * invoked later, from the worker task, with that same `user` pointer.
 * A caller whose sink target is a stack-local object that goes out of
 * scope when the calling function returns (the pattern every adapter in
 * this tree currently uses: build a stack-local capture struct, call
 * mtk_router_dispatch, synchronously read the capture struct's fields
 * immediately afterward) MUST NOT be reached by this path -- doing so is
 * a use-after-return. This is precisely why no adapter in this codebase
 * currently registers a runner: each would need to switch from
 * synchronous stack-local capture to a persistent, adapter-owned sink
 * object (e.g. a per-session outbound queue) before enabling this
 * mechanism safely. See docs/PROVENANCE.md. */
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
 * runner registered, a FACTORY_UART-profile dispatch is never deferred
 * (mtk_router_dispatch's own transport-aware gate) and this reports 0 for
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
