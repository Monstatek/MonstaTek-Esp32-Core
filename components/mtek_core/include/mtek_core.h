/* Clean-room implementation from MonstaTek contract
 * (002-canonical-core-contract.md Sec 2-6). Transport-neutral canonical
 * request context, response/event sink, and operation-token lifecycle
 * every adapter (factory UART, native SPI v1, Legacy SPI Compatibility) constructs a
 * request into and every service dispatches against -- independent of
 * which adapter originated the request. */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "mtek_schema_codec.h"
#include "mtek_schema_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* "profile" field */
typedef enum {
    MTK_PROFILE_FACTORY_UART = 1,
    MTK_PROFILE_NATIVE_SPI = 2,
    MTK_PROFILE_COMPAT_SPI = 3,
    MTK_PROFILE_HOST_ADAPTER = 4,
} mtk_profile_t;

/* Sec 3: the service's only way to produce output. A service never writes
 * to a transport; it calls exactly one of these against the response_route
 * captured in the request context. */
/* OK means accepted locally, not acknowledged by the peer. Responses must
 * not be truncated. Events (including terminal events) remain best effort;
 * streams may be truncated/dropped. Callbacks never retry or wait for space.
 * Queue priority remains RESPONSE > EVENT > STREAM. */
typedef enum {
    MTK_EMIT_OK = 0,
    MTK_EMIT_ENCODING_FAILED,
    MTK_EMIT_CAPACITY_FAILED,
    MTK_EMIT_TRUNCATED,
    MTK_EMIT_DROPPED,
} mtk_emit_result_t;

typedef struct mtk_sink {
    void *user;
    mtk_emit_result_t (*emit_response)(void *user, uint32_t correlation, uint8_t status,
                           const void *body, const mtk_struct_desc_t *body_desc);
    /* Escape hatch for the one opcode (CAPTURE_POLL_READ,) whose response is a
     * true tagged union where the EMPTY variant transmits zero body bytes -- not
     * encodable by the generic struct-desc walker, which always walks every
     * field of a struct. `body` is already-encoded wire bytes. */
    mtk_emit_result_t (*emit_response_raw)(void *user, uint32_t correlation, uint8_t status,
                               const uint8_t *body, size_t body_len);
    /* correlation_or_zero: originating operation's token, or 0 for a
     * session-scoped event with no originating request . */
    mtk_emit_result_t (*emit_event)(void *user, uint32_t correlation_or_zero, const char *event_name,
                        const void *body, const mtk_struct_desc_t *body_desc);
    mtk_emit_result_t (*emit_stream)(void *user, uint32_t session_token, uint32_t sequence,
                         const uint8_t *chunk, size_t len);
} mtk_sink_t;

typedef enum {
    MTK_DISPATCH_INLINE = 0,
    MTK_DISPATCH_DEFER_ALLOWED,
} mtk_dispatch_mode_t;

/* Sec 2: canonical request context. An adapter that cannot populate every
 * required field rejects the request at the adapter boundary and never
 * constructs a partial context. */
typedef struct mtk_request_ctx {
    mtk_profile_t profile;
    uint32_t correlation;      /* token: adapter-scoped request correlation */
    uint32_t boot_epoch;
    uint8_t authorization_level;  /* Sec 10 of : Phase 1 always 0 */
    mtk_sink_t sink;
    /* P0 correction (follow-up read-only audit, "genuine peer-session
     * ownership"): 0 means "not session-scoped" (every adapter except
     * native SPI -- Legacy SPI Compatibility and factory UART have no peer-reboot concept
     * of their own, matching mtk_core.h's own established boot_epoch doc
     * comment) -- never fenced. Native SPI stamps this with mtk_core_
     * session_generation() at the moment it actually dispatches a
     * request (mtek_spi_native_dispatch.c), NOT a wire field and NOT the
     * same concept as boot_epoch: boot_epoch identifies the CURRENT ESP
     * boot session for operation-token scoping (§3.4 below); this field
     * identifies WHICH peer session originated this specific request, so
     * a request queued for deferred (async-pool) dispatch before a peer
     * reboot, whose worker has not yet actually started running its
     * handler by the time the reboot is detected, can be recognized as
     * stale and refused (mtk_router.c's own async_trampoline checks this
     * immediately before invoking the handler) rather than minting a
     * brand-new operation indistinguishable from a legitimate request
     * from the NEW peer session. */
    uint32_t session_generation;
    /* DEFER_ALLOWED requires sink.user and all callback state to outlive
     * the entire async operation. Zero initialization selects inline. */
    mtk_dispatch_mode_t dispatch_mode;
} mtk_request_ctx_t;

/* Sec 4.2 ACCEPTED_ASYNC operation lifecycle ------------ */

typedef enum {
    MTK_OPS_FREE = 0,   /* slot unused */
    MTK_OPS_ACCEPTED,
    MTK_OPS_RUNNING,
    MTK_OPS_STOPPING,
    MTK_OPS_COMPLETED,  /* terminal */
    MTK_OPS_FAILED,     /* terminal */
    MTK_OPS_TIMED_OUT,  /* terminal */
    MTK_OPS_STOPPED,    /* terminal */
} mtk_op_state_t;

static inline int mtk_op_state_is_terminal(mtk_op_state_t s) {
    return s == MTK_OPS_COMPLETED || s == MTK_OPS_FAILED || s == MTK_OPS_TIMED_OUT || s == MTK_OPS_STOPPED;
}

typedef struct mtk_operation_record {
    uint32_t token;             /* 0 = free */
    uint32_t boot_epoch;
    uint16_t service_id;
    uint16_t opcode;
    mtk_op_state_t state;
    uint8_t final_status;       /* mtk_status_t, valid once terminal */
    uint64_t terminal_at_ms;    /* monotonic ms at which state became terminal */
    uint64_t created_at_ms;
    void *service_ctx;          /* opaque, owned by the service that minted this token */
} mtk_operation_record_t;

/* Optional lock hooks (ESP32 target glue only; default no-op, safe for every
 * host test which is single-threaded and for a target build with no async runner
 * registered). When registered, `lock`/`unlock` bracket every mutation and
 * lookup of the operation-token table below -- required once the router's async
 * runner can genuinely run several FreeRTOS workers concurrently against this
 * shared table ("Shared runtime state is not concurrency-safe"). Mirrors
 * mtk_router_ set_lock/mtk_async_queue_set_lock's own established pattern. */
typedef void (*mtk_core_lock_fn)(void);
void mtk_core_set_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock);

/* Fixed 8-slot table (core_budgets.max_operation_tokens). now_ms is supplied
 * by the caller (portable: no OS clock dependency in this component). */
void mtk_core_init(uint32_t boot_epoch);
uint32_t mtk_core_boot_epoch(void);
void mtk_core_reset(uint32_t new_boot_epoch); /* Sec 8: invalidates every token/cursor/etc from the prior epoch */

/* A SEPARATE, transport-facing counter from boot_epoch -- deliberately never
 * touched by mtk_core_reset/a global core reset, and never mistaken for "the ESP
 * itself rebooted" (mtk_request_ctx_t's own session_generation field doc comment
 * explains the distinction in full). Only native SPI's own peer-reboot detection
 * ever bumps this (mtek_spi_native_dispatch.c); every other adapter's requests
 * carry session_generation==0 (never fenced) and are completely unaffected by
 * it. Starts at 1 (0 is reserved as the "not session-scoped" sentinel) at
 * mtk_core_init and is NOT reset by mtk_core_reset -- a peer session boundary
 * and an ESP boot/epoch-reset boundary are two independent events by design. */
uint32_t mtk_core_session_generation(void);
uint32_t mtk_core_bump_session_generation(void); /* returns the NEW (post-bump) generation */

/* A worker can win mtk_op_claim_finalization/mtk_op_transition_by_token for its
 * own token an instant BEFORE a concurrent peer-session reset bumps
 * mtk_core_session_generation (and sweeps/invalidates old-session state), then
 * publish shared results or emit an event an instant AFTER -- a real, narrow
 * window neither the op-table win nor the arbiter-ownership checks close, since
 * both only ever check state at ONE moment, not "is my session still the current
 * one right up until I actually publish."
 *
 * Further correction (follow-up read-only audit, "one P0 race remains"): an
 * EARLIER version of this fix was a single point-in-time re-check
 * (`mtk_op_confirm_still_current_session`) taken right before publish -- but a
 * single check only proves the generation had not YET changed at the instant it
 * ran; a reset could still land in the (necessarily nonzero) window between that
 * check RETURNING and the caller's own subsequent state-write/event-emit
 * statements actually executing. No single re-check, however late, can close a
 * window that exists AFTER it returns. The only way to close it to zero width is
 * to make "check" and "publish" mutually exclusive with "bump" via a shared
 * lock, so this is now a genuine begin/end GUARD around the whole publish, held
 * across mtk_op_begin_publish_guard returning true and the caller's publish,
 * backed by a lock ALSO acquired (briefly, only around the bump itself) by
 * mtk_core_bump_session_generation -- so a bump literally cannot complete while
 * a guard is open, and a guard cannot open while a bump is in progress. This
 * lock is DELIBERATELY separate from mtk_core_set_lock's own lock (mirroring
 * mtek_capture_service.h's own mtek_capture_set_lock precedent for the identical
 * reason): the guard is held across the caller's own sink emit_event call, which
 * can itself reach back into a queue-backed sink's lock -- often the SAME
 * physical lock as mtk_core_set_lock's own (see main/app_main.c's own
 * shared_lock_v wiring) -- so reusing that lock here would risk a real
 * self-deadlock the instant a worker tried to emit while holding it.
 *
 * Usage: a handler that has just won its own op-table finalization for `token`
 * calls mtk_op_begin_publish_guard, passing the session_ generation its own
 * request ctx was stamped with at admission time (mtk_request_ctx_t's own
 * session_generation field; 0 for every non- native-SPI adapter, which is never
 * fenced and always succeeds here). If it returns 1 (still current), the caller
 * does its ENTIRE publish (state write(s) + event emission) then calls
 * mtk_op_end_publish_guard -- never anything slow or blocking in between, and
 * never a second, nested acquisition of this same lock. If it returns 0, the
 * lock has already been released and the caller must skip publish/emit entirely
 * (the op-table transition it already won still stands; do not undo it). */
int mtk_op_begin_publish_guard(uint32_t session_generation);
void mtk_op_end_publish_guard(void);

/* Optional lock hooks for the publish guard above (default no-op, safe
 * for every single-threaded host test and matching every other lock's
 * own established registration pattern in this codebase). MUST be
 * registered with a lock object DISTINCT from mtk_core_set_lock's own
 * (see mtk_op_begin_publish_guard's own doc comment for why). */
void mtk_op_set_publish_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock);

/* Test-only synchronization seam (always NULL/no-op unless a host test
 * explicitly registers one -- a single function-pointer check costs nothing in
 * production and is never used to gate or alter behavior). Called by
 * mtk_op_begin_publish_guard, above, right before it returns 1 (still holding
 * the publish-guard lock) -- letting a test force a real, controlled pause while
 * a worker holds the guard, so a concurrent mtk_core_bump_session_generation
 * call (e.g. from a real peer-session reset) can be proven to genuinely BLOCK
 * until the paused worker finishes its own publish and releases the guard,
 * rather than merely hoping a reset lands in some narrow window. */
typedef void (*mtk_op_won_hook_t)(uint32_t token);
void mtk_op_set_won_hook(mtk_op_won_hook_t hook);

/* Session-admission guard: the SAME mutual-exclusion property as the publish
 * guard above (backed by the identical lock also acquired by
 * mtk_core_bump_session_generation), but for the OPPOSITE end of a token-backed
 * producer's lifecycle -- admission, not publish. Diagnosed gap this closes: a
 * producer handler previously acquired its arbiter class with a placeholder
 * token 0, minted the real operation token afterward, then replaced the
 * placeholder via mtk_arbiter_force_ transfer -- a real, externally-observable
 * window in which the arbiter reports an active, token-backed class owned by
 * token 0, no operation record for it exists yet, and a peer-session reset
 * landing in that exact window can neither reject the request as stale (it
 * already appears admitted) nor cancel it by token (none has been minted yet).
 * Serializing admission against mtk_core_bump_session_ generation the same way
 * publish already is closes both halves: a reset now either completes strictly
 * before admission begins (so the caller's own final session-generation check
 * below sees it and refuses the request as stale) or strictly after the real
 * token is fully published and the ACCEPTED response sent (so it can cancel that
 * exact token instead).
 *
 * Usage: a handler calls mtk_op_begin_admission_guard(ctx->session_ generation)
 * BEFORE acquiring its arbiter class. If it returns 0, the request's own
 * originating session has already been superseded by a reset -- respond
 * MTK_STATUS_NOT_READY and return; nothing has been allocated or acquired yet,
 * so there is nothing to unwind. If it returns 1, the caller holds the guard
 * through: minting the operation token (mtk_op_alloc_id), publishing arbiter
 * ownership under the REAL token in the SAME arbiter call
 * (mtk_arbiter_acquire(class, id.token) -- never acquire(class, 0) followed by a
 * later force_transfer, which is exactly the half-published sequence this guard
 * exists to close; see mtk_op_discard_unpublished below for the
 * rejected-acquisition case), the initial RUNNING transition, committing that
 * operation's COMPLETE cancellation-visible initial state (whatever a peer-reset
 * canceller or a STATUS query reads about it -- e.g. deauth's s_deauth,
 * handshake's s_hs -- under that service's own existing field lock; never
 * partially initialized), and the single synchronous result (ACCEPTED, or a
 * guarded allocation/acquisition/immediate-HAL-failure response) -- then calls
 * mtk_op_end_admission_guard. Never anything slow or blocking in between (the
 * same discipline mtk_op_begin_publish_guard's own doc comment requires), and
 * never call this while already holding the publish guard or vice versa (same
 * lock, non-recursive). Every path that successfully opened the guard, including
 * a rejection, must publish its one result INSIDE the guard, before calling
 * mtk_op_end_admission_guard -- releasing the guard first would let a HELLO
 * blocked on the same lock proceed, clear native-transport pending/queue state,
 * and only then see this worker's response land as a stale, unmatched arrival.
 *
 * Nested-lock rule (M3 correction, following a real, reproducible AB-BA deadlock
 * a full-suite ASan/UBSan run caught between mtek_capture_ logic.c's own
 * admission and channel-hop-tick paths): this guard's lock (pub_lock, the SAME
 * lock mtk_op_begin_publish_guard and mtk_core_bump_session_generation share) is
 * never the OUTERMOST lock in a call that also needs a service's own
 * action/lease lock (e.g. mtek_capture_logic.c's cap_action_lock). Any handler
 * that may need both must acquire its own service lock FIRST, then this guard,
 * exactly mirroring the corresponding periodic-tick/background path's own order
 * (e.g. mtek_capture_channel_hop_tick: cap_action_lock, then this guard's lock
 * via mtk_op_begin_publish_guard, then the service's field lock). Reversing that
 * order in only one of the two paths -- this guard first, the service lock
 * second -- creates exactly the cycle this correction closes. A handler with no
 * such service action/lease lock is unaffected and needs no reordering.
 *
 * Deliberately does NOT invoke mtk_op_set_won_hook's hook -- that seam is
 * specific to the publish guard's own purpose (proving a bump blocks across an
 * open PUBLISH), and reusing it here would conflate two distinct guarded regions
 * under one signal. Use mtk_op_set_admission_
 * hook/mtk_op_set_admission_prepublish_hook (below) to pause deterministically
 * inside THIS guard instead. */
int mtk_op_begin_admission_guard(uint32_t session_generation);
void mtk_op_end_admission_guard(void);

/* Test-only synchronization seam for the admission guard above, distinct
 * from mtk_op_set_won_hook (see mtk_op_begin_admission_guard's own doc
 * comment for why these must not be conflated). Called right before
 * mtk_op_begin_admission_guard returns 1, still holding the guard's
 * lock -- i.e. before the token even exists ("pre-publication"). */
void mtk_op_set_admission_hook(mtk_op_won_hook_t hook);

/* A second, distinct test-only seam at the OPPOSITE end of the same guard
 * span ("post-publication, pre-unlock"): called at the top of
 * mtk_op_end_admission_guard, after the caller has already minted the
 * token, published arbiter ownership, committed cancellation-visible
 * state, and emitted its one synchronous result, but still holding
 * pub_lock (never the arbiter or core-table lock). Lets a test assert a
 * coherent snapshot, matching op identity/state, and the already-
 * published response deterministically, instead of guessing when a
 * detached async worker reaches that point. */
void mtk_op_set_admission_prepublish_hook(mtk_op_won_hook_t hook);

/* Frees an operation record's slot immediately, without requiring it to first
 * reach a terminal state (mtk_op_evict's own normal precondition, below) -- the
 * one deliberate exception to that rule. Valid ONLY for a record that has never
 * been exposed beyond the calling thread: never published as an arbiter owner,
 * never included in an ACCEPTED response, never observable by any other worker.
 * This is exactly the shape of a record minted by mtk_op_alloc_id inside an open
 * admission guard whose following mtk_arbiter_acquire(class, id.token) call was
 * then rejected (MTK_ARB_GRANT_BUSY/GUARDED-not-taken) -- the token was minted
 * but the arbiter never published it, so nothing outside this call stack can
 * possibly know it exists yet, and it is safe to reclaim the slot immediately
 * rather than leaving a live, never-terminal record sitting in the table until a
 * future GC pass. Callers must still hold the same admission guard under which
 * the record was minted when calling this (never after
 * mtk_op_end_admission_guard has already released it). No-op if token is 0 or
 * the record cannot be found. */
void mtk_op_discard_unpublished(uint32_t token, uint32_t boot_epoch);

/* Mints a new nonzero token and an ACCEPTED-state record, or NULL with
 * *out_no_memory=1 if the 8-token budget is exhausted and no terminal record is
 * available to evict . */
#ifdef MTK_ENABLE_TEST_OPCODES
/* Test-only raw table access. Production services use identities/snapshots. */
mtk_operation_record_t *mtk_op_alloc(uint16_t service_id, uint16_t opcode, uint64_t now_ms, int *out_no_memory);
mtk_operation_record_t *mtk_op_find(uint32_t token, uint32_t boot_epoch);
#endif

/* Immutable operation identity, copied while the allocation lock is held.
 * Services retain this value, never a pointer into the reusable record table.
 * token == 0 signals allocation failure; out_no_memory is optional. */
typedef struct {
    uint32_t token;      /* 0 = allocation failed */
    uint32_t boot_epoch;
} mtk_op_id_t;
mtk_op_id_t mtk_op_alloc_id(uint16_t service_id, uint16_t opcode, uint64_t now_ms, int *out_no_memory);
/* ("STOP and worker paths can both restore the radio, release the arbiter, and
 * emit lifecycle output"): returns 1 if THIS call is the one that actually
 * performed the transition (the record was found and was not already terminal),
 * 0 otherwise (NULL rec, or already terminal -- idempotent, no-op, exactly as
 * before). Callers whose cleanup (radio-mode restore, arbiter release,
 * terminal-event emission) must happen EXACTLY ONCE per operation -- not once
 * per code path that happens to notice terminal state -- must gate that whole
 * cleanup sequence on this return value, not on a separate, racy
 * `!mtk_op_state_is_terminal(...)` check of their own (the two natural competing
 * callers, e.g. a STOP request and the worker's own natural completion, must
 * linearize on the SAME atomic decision, not each make their own
 * separately-timed check against a value that can change between their check and
 * their own subsequent unconditional cleanup). */
#ifdef MTK_ENABLE_TEST_OPCODES
int mtk_op_transition(mtk_operation_record_t *rec, mtk_op_state_t new_state, uint8_t status, uint64_t now_ms);
#endif
/* Sweeps 60s-expired terminal records back to FREE . */
void mtk_op_gc(uint64_t now_ms);

/* "mtk_op_find returns a table pointer after releasing the core mutex... Service
 * code then directly reads rec->state, final_status, and tokens across worker,
 * STOP, Wi-Fi callback, and tick tasks." A service that only briefly uses a
 * freshly-`mtk_op_find`-ed pointer within the same call frame (never retaining
 * it past that call) is fine as-is -- the hazard is specifically a session that
 * RETAINS a `mtk_operation_record_t *` across multiple separate calls/ticks/
 * callbacks (e.g. deauth/handshake/capture/signal-meter/GATT session state,
 * mtek_wifi_logic.c/mtek_ble_logic.c/mtek_capture_logic.c), since the lock is
 * released the instant mtk_op_find returns and the slot can legitimately be
 * reused for a completely different token by the time a LATER call dereferences
 * that same stale pointer. Such sessions must retain the TOKEN (a plain
 * uint32_t, immune to reuse -- a stale token simply fails to re-match on its
 * next fresh lookup, per mtk_op_find's own boot_epoch check) and re-look-up
 * fresh, under lock, every time -- these two atomic (single lock acquisition
 * covering the whole find+use) helpers make that the easy, safe default instead
 * of a two-step find-then-separately-lock-again pattern that would still leave a
 * gap. */

/* Atomic locked snapshot: finds the record for (token, boot_epoch) and
 * copies its CURRENT fields into *out under the same lock acquisition
 * that found it -- no live pointer, no gap for a concurrent transition to
 * race the read. Returns 1 if found, 0 if not (already evicted, wrong
 * epoch, or token stale/zero). */
int mtk_op_snapshot(uint32_t token, uint32_t boot_epoch, mtk_operation_record_t *out);

/* Atomic locked transition by token: finds the record for (token, boot_epoch)
 * and transitions it in the SAME lock acquisition, closing the gap a separate
 * mtk_op_find + mtk_op_transition(rec,...) pair would leave (the record could be
 * reused in between). Same "did I win the linearization" return semantics as
 * mtk_op_transition above: 1 if THIS call actually performed the transition, 0
 * if not found OR already terminal. */
int mtk_op_transition_by_token(uint32_t token, uint32_t boot_epoch, mtk_op_state_t new_state, uint8_t status, uint64_t now_ms);

/* Atomically claims the single cleanup/finalization path. Exactly one contender
 * can move a live operation to STOPPING; later contenders see STOPPING/terminal
 * and return 0. The winner performs cleanup and then uses
 * mtk_op_transition_by_token for the terminal state. */
int mtk_op_claim_finalization(uint32_t token, uint32_t boot_epoch);

/* Cross-operation-token family validation (item 1)
 * ------------------------------------- The generic (token,
 * boot_epoch)-addressed APIs above are correct for a handler acting on a token
 * IT ITSELF minted (a START handler's own RUNNING transition, a worker's own
 * natural-completion tail, native-SPI peer invalidation). They are NOT
 * sufficient for a feature-specific handler that receives a token FROM THE
 * REQUEST BODY -- a STOP/STATUS/READ/ session-info/stats opcode. Such a handler
 * must additionally prove the token belongs to ITS OWN operation family: the
 * record was minted by the expected service AND by the expected originating
 * START opcode. Without that check, an AP_SCAN_STOP given a live STA_SCAN token
 * would find it, transition it, restore the radio, release the arbiter, and emit
 * a terminal event for an operation the caller never named -- a systemic
 * cross-operation-token defect.
 *
 * These family-aware variants fold the (service_id, start_opcode) check into the
 * SAME single lock acquisition as the find + action, so a wrong- family token is
 * rejected atomically with zero side effect on any operation, service, HAL,
 * event, or arbiter state -- there is no window between "is this the right
 * family" and "act". `expected_start_opcode` is the opcode of the START that
 * mints this family's token (e.g. AP_SCAN_STOP/ STATUS/RESULTS_PAGE/DETAILS all
 * pass AP_SCAN_START's opcode). Each returns 0 (family mismatch, wrong epoch,
 * evicted, or -- for the action variants -- already terminal/stopping) exactly
 * as its generic counterpart returns 0, so a caller maps a 0 to its existing
 * NOT_FOUND / no-op path unchanged.
 *
 * GET_OPERATION_STATUS (service 0x0000) is INTENTIONALLY generic and keeps using
 * mtk_op_snapshot: it is defined to report any operation's status regardless of
 * family. */

/* Pure predicate: 1 iff a record for (token, boot_epoch) exists AND its
 * service_id == expected_service_id AND its minting opcode ==
 * expected_start_opcode. No mutation, no side effect. */
int mtk_op_validate_family(uint32_t token, uint32_t boot_epoch,
                           uint16_t expected_service_id, uint16_t expected_start_opcode);

/* Family-gated mtk_op_snapshot: copies the record into *out only if the
 * family matches; otherwise returns 0 and leaves *out untouched. */
int mtk_op_snapshot_family(uint32_t token, uint32_t boot_epoch,
                           uint16_t expected_service_id, uint16_t expected_start_opcode,
                           mtk_operation_record_t *out);

/* Family-gated mtk_op_transition_by_token: transitions only if the family
 * matches AND (as the generic form) the record is not already terminal.
 * Returns 1 iff THIS call performed the transition. */
int mtk_op_transition_by_token_family(uint32_t token, uint32_t boot_epoch,
                                      uint16_t expected_service_id, uint16_t expected_start_opcode,
                                      mtk_op_state_t new_state, uint8_t status, uint64_t now_ms);

/* Family-gated mtk_op_claim_finalization: claims the single finalization
 * path only if the family matches. Returns 1 iff THIS call won the claim. */
int mtk_op_claim_finalization_family(uint32_t token, uint32_t boot_epoch,
                                     uint16_t expected_service_id, uint16_t expected_start_opcode);

/* Immediately frees a TERMINAL record's slot (rather than waiting out
 * MTK_OP_RETENTION_MS), so a subsequent mtk_op_snapshot/
 * mtk_op_transition_by_token/mtk_op_find for (token, boot_epoch) reports "not
 * found" right away. Returns 1 if this call actually evicted the slot, 0 if not
 * found OR found but NOT YET terminal (never evicts a live operation -- a caller
 * must finalize it first, e.g. via mtk_op_claim_ finalization +
 * mtk_op_transition_by_token, and only then evict). Used by native SPI's own
 * peer-session invalidation (SPI_PROTOCOL_V1.md: a changed peer boot_epoch
 * "invalidates... operation tokens for that peer") to make a cancelled
 * operation's token genuinely unusable immediately, not merely
 * terminal-but-still-reportable for its normal retention window. */
int mtk_op_evict(uint32_t token, uint32_t boot_epoch);

/* Sweeps every one of the fixed 8 table slots and evicts (mtk_op_evict's own
 * semantics) every TERMINAL one regardless of which token/epoch it carries --
 * never touches a live (non-terminal) record. Used by native SPI's own
 * peer-session invalidation, AFTER every per-service cancel path has already
 * finalized whatever single operation could have been genuinely live (only one
 * radio-owning class can ever be active at a time), to also evict every OTHER,
 * merely-retained terminal record left over from earlier in the same now-ended
 * peer session (e.g. a completed AP_SCAN from before the most recent DEAUTH) --
 * these carry no live HAL resource to release (that already happened when they
 * went terminal), only their own table slot to free. Returns the number of slots
 * evicted. */
int mtk_op_evict_all_terminal(void);

#define MTK_OP_RETENTION_MS MTK_BUDGET_OPERATION_RECORD_RETENTION_MS

/* Terminal events use the ordinary adapter event sink on a best-effort
 * basis. There is no dedicated per-operation delivery reserve or guaranteed
 * terminal delivery; operation status remains independently queryable. */

/* Transport diagnostics counters (GET_TRANSPORT_COUNTERS, service 0x0005 opcode
 * 0x0001) -- and "RETRY/latest-eight duplicate cache": these five fields are the
 * schema's own frozen response shape (mtk_get_transport_counters_resp_t); this
 * is where every adapter that can observe one of these events reports it, and
 * where the diagnostics service handler (mtek_capture_logic.c) reads the live
 * totals instead of a hard-zeroed stub. Global (not per-adapter): meaningful
 * because at most one transport adapter can ever be the boot session's
 * dispatching winner at a time (mtk_transport_claim_try), so these are
 * unambiguously "this boot session's" counters regardless of which adapter is
 * active. Monotonic for the life of the boot session (never reset on a peer
 * boot-epoch change -- these are diagnostics about the ESP's own operation, not
 * peer-session-scoped state like the duplicate/reassembly caches epoch-reset
 * invalidates). */
typedef struct {
    uint32_t integrity_failures;
    uint32_t dropped_frames;
    uint32_t packet_seq_gaps;
    uint32_t retries_observed;
    uint32_t duplicate_responses_served;
} mtk_transport_counters_t;

void mtk_transport_counters_set_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock);
void mtk_transport_counters_reset(void);
void mtk_transport_counters_add_integrity_failure(void);
void mtk_transport_counters_add_dropped_frame(void);
void mtk_transport_counters_add_packet_seq_gap(void);
void mtk_transport_counters_add_retry_observed(void);
void mtk_transport_counters_add_duplicate_response_served(void);
void mtk_transport_counters_get(mtk_transport_counters_t *out);

#ifdef __cplusplus
}
#endif
