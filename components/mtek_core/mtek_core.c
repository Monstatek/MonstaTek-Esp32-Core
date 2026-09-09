/* Clean-room implementation from MonstaTek contract (002-canonical-core-contract.md). */
#include "mtek_core.h"
#include "mtek_codec_api.h"
#include <string.h>

static uint32_t s_boot_epoch;
static uint32_t s_next_token_seq;
static mtk_operation_record_t s_ops[MTK_BUDGET_MAX_OPERATION_TOKENS];
/* P0 correction (follow-up read-only audit, "genuine peer-session
 * ownership"): see mtek_core.h's own doc comment on mtk_core_session_
 * generation/mtk_core_bump_session_generation -- deliberately separate
 * from s_boot_epoch above, never touched by anything but native SPI's
 * own peer-reboot detection. */
static uint32_t s_session_generation;

/* Terminal-event reserve: indexed 1:1 with s_ops[] above (same slot i =
 * same operation), so it is cleared/reserved in lockstep with the op
 * table's own alloc/evict/reset -- see mtek_core.h's own doc comment. */
typedef struct {
    uint8_t pending;
    uint32_t token;
    char event_name[MTK_TERMINAL_EVENT_NAME_MAX];
    uint8_t body[MTK_TERMINAL_EVENT_BODY_MAX];
    size_t body_len;
} mtk_terminal_event_slot_t;
static mtk_terminal_event_slot_t s_terminal[MTK_BUDGET_TERMINAL_EVENT_RESERVE];

static mtk_core_lock_fn s_lock, s_unlock;
void mtk_core_set_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock) { s_lock = lock; s_unlock = unlock; }
static void core_lock(void) { if (s_lock) s_lock(); }
static void core_unlock(void) { if (s_unlock) s_unlock(); }

/* Deterministic, host-testable boot_epoch generator: nonzero, distinct per
 * mtk_core_reset() call. A target build seeds the first call from a real
 * hardware RNG (Sec 3.4: "native: random nonzero u32"); the seed is
 * supplied by the caller so this component has no direct HAL dependency. */
void mtk_core_init(uint32_t boot_epoch) {
    s_boot_epoch = boot_epoch ? boot_epoch : 1;
    s_next_token_seq = 0;
    memset(s_ops, 0, sizeof(s_ops));
    memset(s_terminal, 0, sizeof(s_terminal));
    s_session_generation = 1; /* 0 is reserved as the "not session-scoped" sentinel */
}

uint32_t mtk_core_boot_epoch(void) { return s_boot_epoch; }

void mtk_core_reset(uint32_t new_boot_epoch) {
    mtk_core_init(new_boot_epoch);
}

uint32_t mtk_core_session_generation(void) {
    core_lock();
    uint32_t g = s_session_generation;
    core_unlock();
    return g;
}

/* P0 correction (follow-up read-only audit, "one P0 race remains"): a
 * lock DISTINCT from s_lock/s_unlock above -- see mtk_op_begin_publish_
 * guard's own doc comment (mtek_core.h) for the full self-deadlock
 * rationale. Acquired here, briefly, around the bump itself (never
 * across anything slow/blocking), so that a bump cannot complete while
 * a worker's own mtk_op_begin_publish_guard()...mtk_op_end_publish_
 * guard() region is open, and a guard cannot open while a bump is
 * itself in progress -- making "check the generation" and "the
 * generation actually changes" genuinely mutually exclusive rather than
 * two independently-timed events with a gap between them. */
static mtk_core_lock_fn s_pub_lock, s_pub_unlock;
void mtk_op_set_publish_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock) { s_pub_lock = lock; s_pub_unlock = unlock; }
static void pub_lock(void) { if (s_pub_lock) s_pub_lock(); }
static void pub_unlock(void) { if (s_pub_unlock) s_pub_unlock(); }

uint32_t mtk_core_bump_session_generation(void) {
    pub_lock();
    core_lock();
    s_session_generation++;
    if (s_session_generation == 0) s_session_generation = 1; /* never let it wrap to the reserved sentinel */
    uint32_t g = s_session_generation;
    core_unlock();
    pub_unlock();
    return g;
}

static mtk_op_won_hook_t s_op_won_hook;
void mtk_op_set_won_hook(mtk_op_won_hook_t hook) { s_op_won_hook = hook; }

int mtk_op_begin_publish_guard(uint32_t session_generation) {
    pub_lock();
    if (session_generation != 0 && session_generation != mtk_core_session_generation()) {
        pub_unlock();
        return 0;
    }
    if (s_op_won_hook) s_op_won_hook(session_generation); /* test seam -- still holding pub_lock */
    return 1; /* caller now owns the lock until it calls mtk_op_end_publish_guard() */
}

void mtk_op_end_publish_guard(void) {
    pub_unlock();
}

/* Distinct test seam from s_op_won_hook -- see mtk_op_begin_admission_
 * guard's own doc comment (mtek_core.h) for why the two must stay
 * separate. */
static mtk_op_won_hook_t s_admission_hook;
void mtk_op_set_admission_hook(mtk_op_won_hook_t hook) { s_admission_hook = hook; }

/* M2 TSan-harness-correction round: a SECOND, distinct test seam, firing
 * at the opposite end of the same guard span -- mtk_op_begin_admission_
 * guard's own hook fires the instant the guard is acquired (before the
 * token even exists); this one fires in mtk_op_end_admission_guard,
 * immediately before pub_unlock -- i.e. after every producer site has
 * already minted the real token, published arbiter ownership, committed
 * its complete cancellation-visible initial state, transitioned to
 * RUNNING, and emitted its single synchronous result (ACCEPTED or a
 * guarded allocation/acquisition/HAL failure), but while a concurrent
 * mtk_core_bump_session_generation is still genuinely blocked on the same
 * pub_lock. This is what lets a test assert "coherent expected class +
 * nonzero token, the matching op identity/state, and the already-
 * published response" deterministically, instead of guessing when a
 * detached async worker might have reached that point. Still holds only
 * pub_lock -- never the arbiter or core-table lock -- exactly like the
 * begin-side hook; a test seam only, never used for any runtime
 * decision. */
static mtk_op_won_hook_t s_admission_prepublish_hook;
void mtk_op_set_admission_prepublish_hook(mtk_op_won_hook_t hook) { s_admission_prepublish_hook = hook; }

int mtk_op_begin_admission_guard(uint32_t session_generation) {
    pub_lock();
    if (session_generation != 0 && session_generation != mtk_core_session_generation()) {
        pub_unlock();
        return 0;
    }
    if (s_admission_hook) s_admission_hook(session_generation); /* test seam -- still holding pub_lock */
    return 1; /* caller now owns the lock until it calls mtk_op_end_admission_guard() */
}

void mtk_op_end_admission_guard(void) {
    if (s_admission_prepublish_hook) s_admission_prepublish_hook(mtk_core_session_generation()); /* test seam -- still holding pub_lock */
    pub_unlock();
}

void mtk_op_discard_unpublished(uint32_t token, uint32_t boot_epoch) {
    if (token == 0) return;
    core_lock();
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) {
            memset(&s_ops[i], 0, sizeof(s_ops[i]));
            memset(&s_terminal[i], 0, sizeof(s_terminal[i]));
            break;
        }
    }
    core_unlock();
}

static uint32_t alloc_token(void) {
    uint32_t t;
    do {
        s_next_token_seq++;
        t = s_next_token_seq;
    } while (t == 0);
    return t;
}

static mtk_operation_record_t *find_free_or_evict_locked(uint64_t now_ms) {
    (void)now_ms;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == 0) return &s_ops[i];
    }
    /* Evict the oldest terminal record (Sec 4.2.1); never a RUNNING one. */
    int oldest_idx = -1;
    uint64_t oldest_at = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (mtk_op_state_is_terminal(s_ops[i].state)) {
            if (oldest_idx < 0 || s_ops[i].terminal_at_ms < oldest_at) {
                oldest_idx = (int)i;
                oldest_at = s_ops[i].terminal_at_ms;
            }
        }
    }
    if (oldest_idx < 0) return NULL;
    memset(&s_ops[oldest_idx], 0, sizeof(s_ops[oldest_idx]));
    memset(&s_terminal[oldest_idx], 0, sizeof(s_terminal[oldest_idx])); /* this slot's reserve is about to belong to a different token */
    return &s_ops[oldest_idx];
}

/* Internal, lock-already-held variant, used by mtk_op_alloc (which holds
 * the lock across its whole gc+allocate sequence, since the two must be
 * atomic together: a concurrent alloc on another worker must never see a
 * slot this gc pass just freed race ahead of this allocation). */
static void op_gc_locked(uint64_t now_ms) {
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token != 0 && mtk_op_state_is_terminal(s_ops[i].state)) {
            if (now_ms >= s_ops[i].terminal_at_ms &&
                (now_ms - s_ops[i].terminal_at_ms) >= MTK_OP_RETENTION_MS) {
                memset(&s_ops[i], 0, sizeof(s_ops[i]));
                memset(&s_terminal[i], 0, sizeof(s_terminal[i]));
            }
        }
    }
}

mtk_operation_record_t *mtk_op_alloc(uint16_t service_id, uint16_t opcode, uint64_t now_ms, int *out_no_memory) {
    core_lock();
    op_gc_locked(now_ms);
    mtk_operation_record_t *slot = find_free_or_evict_locked(now_ms);
    if (!slot) {
        if (out_no_memory) *out_no_memory = 1;
        core_unlock();
        return NULL;
    }
    if (out_no_memory) *out_no_memory = 0;
    memset(slot, 0, sizeof(*slot));
    memset(&s_terminal[slot - s_ops], 0, sizeof(s_terminal[0])); /* fresh token admits a fresh, empty reserve slot -- see mtek_core.h's own doc comment */
    slot->token = alloc_token();
    slot->boot_epoch = s_boot_epoch;
    slot->service_id = service_id;
    slot->opcode = opcode;
    slot->state = MTK_OPS_ACCEPTED;
    slot->created_at_ms = now_ms;
    core_unlock();
    return slot;
}

mtk_op_id_t mtk_op_alloc_id(uint16_t service_id, uint16_t opcode, uint64_t now_ms, int *out_no_memory) {
    mtk_op_id_t id = {0, 0};
    mtk_operation_record_t *rec = mtk_op_alloc(service_id, opcode, now_ms, out_no_memory);
    /* The one legitimate, same-expression use of the raw pointer mtk_op_
     * alloc returns: copied out immediately, nothing else executes in
     * between (mtk_op_alloc has already unlocked, but nothing has run yet
     * that could reuse a BRAND NEW, not-yet-externally-visible,
     * not-yet-terminal record's own slot). The caller never sees `rec`
     * itself. */
    if (rec) { id.token = rec->token; id.boot_epoch = rec->boot_epoch; }
    return id;
}

mtk_operation_record_t *mtk_op_find(uint32_t token, uint32_t boot_epoch) {
    if (token == 0) return NULL;
    core_lock();
    mtk_operation_record_t *found = NULL;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token) {
            /* Sec 4.2 stale-epoch rule -- checked and returned while still
             * holding the lock, since the slot's boot_epoch/token fields
             * must be read atomically together with the linear scan. */
            found = (s_ops[i].boot_epoch == boot_epoch) ? &s_ops[i] : NULL;
            break;
        }
    }
    core_unlock();
    return found;
}

/* NOTE on the returned mtk_operation_record_t* from mtk_op_alloc/mtk_op_find:
 * the pointer itself is a stable slot address for the life of the boot
 * session (the table is a fixed static array, slots are never moved), so
 * callers may retain it across an async operation's lifetime; but every
 * READ or WRITE of the fields it points to that can race a concurrent
 * gc/alloc/transition on another worker must go through mtk_op_transition
 * (below) or be re-validated via a fresh mtk_op_find -- never dereference
 * a stale record's fields without the lock once more than one FreeRTOS
 * worker can be executing service handlers concurrently. */
int mtk_op_transition(mtk_operation_record_t *rec, mtk_op_state_t new_state, uint8_t status, uint64_t now_ms) {
    if (!rec) return 0;
    core_lock();
    if (mtk_op_state_is_terminal(rec->state)) { core_unlock(); return 0; } /* idempotent: terminal is sticky, no re-transition */
    rec->state = new_state;
    rec->final_status = status;
    if (mtk_op_state_is_terminal(new_state)) rec->terminal_at_ms = now_ms;
    core_unlock();
    return 1;
}

void mtk_op_gc(uint64_t now_ms) {
    core_lock();
    op_gc_locked(now_ms);
    core_unlock();
}

/* Locked helper: returns the index of the record for (token, boot_epoch),
 * or -1. Caller must hold core_lock(). token==0 never matches. */
static int find_op_index_locked(uint32_t token, uint32_t boot_epoch) {
    if (token == 0) return -1;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) return (int)i;
    }
    return -1;
}

/* RC12 blocker round, item 1: the family membership test -- a record
 * "belongs to" a family iff it was minted by the expected service AND the
 * expected originating START opcode. `s_ops[idx].opcode` is exactly the
 * opcode that called mtk_op_alloc_id when the token was minted (a START). */
static int op_index_family_matches(int idx, uint16_t expected_service_id, uint16_t expected_start_opcode) {
    return idx >= 0 &&
           s_ops[idx].service_id == expected_service_id &&
           s_ops[idx].opcode == expected_start_opcode;
}

int mtk_op_snapshot(uint32_t token, uint32_t boot_epoch, mtk_operation_record_t *out) {
    if (token == 0) return 0;
    core_lock();
    int idx = find_op_index_locked(token, boot_epoch);
    int found = 0;
    if (idx >= 0) { *out = s_ops[idx]; found = 1; }
    core_unlock();
    return found;
}

int mtk_op_validate_family(uint32_t token, uint32_t boot_epoch,
                           uint16_t expected_service_id, uint16_t expected_start_opcode) {
    core_lock();
    int idx = find_op_index_locked(token, boot_epoch);
    int ok = op_index_family_matches(idx, expected_service_id, expected_start_opcode);
    core_unlock();
    return ok;
}

int mtk_op_snapshot_family(uint32_t token, uint32_t boot_epoch,
                           uint16_t expected_service_id, uint16_t expected_start_opcode,
                           mtk_operation_record_t *out) {
    core_lock();
    int idx = find_op_index_locked(token, boot_epoch);
    int found = 0;
    if (op_index_family_matches(idx, expected_service_id, expected_start_opcode)) {
        *out = s_ops[idx];
        found = 1;
    }
    core_unlock();
    return found;
}

int mtk_op_transition_by_token_family(uint32_t token, uint32_t boot_epoch,
                                      uint16_t expected_service_id, uint16_t expected_start_opcode,
                                      mtk_op_state_t new_state, uint8_t status, uint64_t now_ms) {
    core_lock();
    int idx = find_op_index_locked(token, boot_epoch);
    int won = 0;
    if (op_index_family_matches(idx, expected_service_id, expected_start_opcode) &&
        !mtk_op_state_is_terminal(s_ops[idx].state)) {
        s_ops[idx].state = new_state;
        s_ops[idx].final_status = status;
        if (mtk_op_state_is_terminal(new_state)) s_ops[idx].terminal_at_ms = now_ms;
        won = 1;
    }
    core_unlock();
    return won;
}

int mtk_op_claim_finalization_family(uint32_t token, uint32_t boot_epoch,
                                     uint16_t expected_service_id, uint16_t expected_start_opcode) {
    core_lock();
    int idx = find_op_index_locked(token, boot_epoch);
    int won = 0;
    if (op_index_family_matches(idx, expected_service_id, expected_start_opcode) &&
        !mtk_op_state_is_terminal(s_ops[idx].state) && s_ops[idx].state != MTK_OPS_STOPPING) {
        s_ops[idx].state = MTK_OPS_STOPPING;
        won = 1;
    }
    core_unlock();
    return won;
}

int mtk_op_transition_by_token(uint32_t token, uint32_t boot_epoch, mtk_op_state_t new_state, uint8_t status, uint64_t now_ms) {
    if (token == 0) return 0;
    core_lock();
    int won = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) {
            if (!mtk_op_state_is_terminal(s_ops[i].state)) {
                s_ops[i].state = new_state;
                s_ops[i].final_status = status;
                if (mtk_op_state_is_terminal(new_state)) s_ops[i].terminal_at_ms = now_ms;
                won = 1;
            }
            break;
        }
    }
    core_unlock();
    return won;
}

int mtk_op_claim_finalization(uint32_t token, uint32_t boot_epoch) {
    if (token == 0) return 0;
    core_lock();
    int won = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) {
            if (!mtk_op_state_is_terminal(s_ops[i].state) && s_ops[i].state != MTK_OPS_STOPPING) {
                s_ops[i].state = MTK_OPS_STOPPING;
                won = 1;
            }
            break;
        }
    }
    core_unlock();
    return won;
}

int mtk_op_evict(uint32_t token, uint32_t boot_epoch) {
    if (token == 0) return 0;
    core_lock();
    int evicted = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) {
            if (mtk_op_state_is_terminal(s_ops[i].state)) {
                memset(&s_ops[i], 0, sizeof(s_ops[i]));
                memset(&s_terminal[i], 0, sizeof(s_terminal[i])); /* this slot's reserve is about to belong to a different token */
                evicted = 1;
            }
            break;
        }
    }
    core_unlock();
    return evicted;
}

int mtk_op_evict_all_terminal(void) {
    core_lock();
    int count = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token != 0 && mtk_op_state_is_terminal(s_ops[i].state)) {
            memset(&s_ops[i], 0, sizeof(s_ops[i]));
            memset(&s_terminal[i], 0, sizeof(s_terminal[i]));
            count++;
        }
    }
    core_unlock();
    return count;
}

int mtk_op_stage_terminal_event(uint32_t token, uint32_t boot_epoch, const char *event_name,
                                 const void *body, const mtk_struct_desc_t *desc) {
    if (token == 0) return 0;
    uint8_t encoded[MTK_TERMINAL_EVENT_BODY_MAX];
    size_t encoded_len = 0;
    if (body && desc) {
        if (mtk_encode(desc, body, encoded, sizeof(encoded), &encoded_len) != MTK_CODEC_OK) return 0;
    }
    core_lock();
    int ok = 0;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_ops[i].token == token && s_ops[i].boot_epoch == boot_epoch) {
            mtk_terminal_event_slot_t *slot = &s_terminal[i];
            slot->pending = 1;
            slot->token = token;
            size_t name_len = event_name ? strnlen(event_name, MTK_TERMINAL_EVENT_NAME_MAX - 1) : 0;
            memset(slot->event_name, 0, sizeof(slot->event_name));
            if (event_name) memcpy(slot->event_name, event_name, name_len);
            memcpy(slot->body, encoded, encoded_len);
            slot->body_len = encoded_len;
            ok = 1;
            break;
        }
    }
    core_unlock();
    return ok;
}

int mtk_op_poll_terminal_event(uint32_t *out_token, char *out_name, size_t name_cap,
                                uint8_t *out_body, size_t body_cap, size_t *out_body_len) {
    core_lock();
    int found = -1;
    for (unsigned i = 0; i < MTK_BUDGET_MAX_OPERATION_TOKENS; i++) {
        if (s_terminal[i].pending) { found = (int)i; break; }
    }
    if (found < 0) { core_unlock(); return 0; }
    mtk_terminal_event_slot_t *slot = &s_terminal[found];
    *out_token = slot->token;
    size_t name_len = strnlen(slot->event_name, sizeof(slot->event_name));
    if (name_len >= name_cap) name_len = name_cap - 1;
    memcpy(out_name, slot->event_name, name_len);
    out_name[name_len] = '\0';
    size_t body_len = slot->body_len > body_cap ? body_cap : slot->body_len;
    memcpy(out_body, slot->body, body_len);
    *out_body_len = body_len;
    slot->pending = 0;
    core_unlock();
    return 1;
}

/* ---- Transport diagnostics counters ------------------------------------ */
static mtk_transport_counters_t s_transport_counters;
static mtk_core_lock_fn s_tc_lock, s_tc_unlock;
static void tc_lock(void) { if (s_tc_lock) s_tc_lock(); }
static void tc_unlock(void) { if (s_tc_unlock) s_tc_unlock(); }

void mtk_transport_counters_set_lock(mtk_core_lock_fn lock, mtk_core_lock_fn unlock) {
    s_tc_lock = lock; s_tc_unlock = unlock;
}
void mtk_transport_counters_reset(void) {
    tc_lock();
    memset(&s_transport_counters, 0, sizeof(s_transport_counters));
    tc_unlock();
}
void mtk_transport_counters_add_integrity_failure(void) { tc_lock(); s_transport_counters.integrity_failures++; tc_unlock(); }
void mtk_transport_counters_add_dropped_frame(void) { tc_lock(); s_transport_counters.dropped_frames++; tc_unlock(); }
void mtk_transport_counters_add_packet_seq_gap(void) { tc_lock(); s_transport_counters.packet_seq_gaps++; tc_unlock(); }
void mtk_transport_counters_add_retry_observed(void) { tc_lock(); s_transport_counters.retries_observed++; tc_unlock(); }
void mtk_transport_counters_add_duplicate_response_served(void) { tc_lock(); s_transport_counters.duplicate_responses_served++; tc_unlock(); }
void mtk_transport_counters_get(mtk_transport_counters_t *out) {
    tc_lock();
    *out = s_transport_counters;
    tc_unlock();
}
