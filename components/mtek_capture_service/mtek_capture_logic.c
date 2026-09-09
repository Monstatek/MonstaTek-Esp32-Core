/* Clean-room implementation from MonstaTek contract
 * (002-capture-diagnostics-service.md, 002-resource-arbiter.md). Portable:
 * no ESP-IDF dependency, host-testable; reuses mtek_wifi_hal_t's
 * promiscuous-mode primitives (mtek_wifi_get_hal()) since MonstaShark
 * capture and WPA handshake capture share the same underlying radio
 * resource (RADIO_OWNER_WIFI, arbiter class M), never active together. */
#include "mtek_capture_service.h"
#include "mtek_wifi_service.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include <string.h>
#include <stdint.h>

/* RC12 blocker round, item 1: capture token family (service 0x0004),
 * every token minted by CAPTURE_START (0x0001). Passed to mtk_op_*_family
 * so a non-capture token handed to a capture STOP/STATUS/session-info/stats
 * handler is rejected NOT_FOUND with no side effect. */
#define CAPTURE_SERVICE_ID    0x0004
#define CAPTURE_START_OPCODE  0x0001

#define CAPTURE_RING_SLOTS 32

static uint64_t (*s_now_ms)(void);
void mtek_capture_service_init(uint64_t (*now_ms_fn)(void)) { s_now_ms = now_ms_fn; }
static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

/* RC7 independent audit item 11 "the global session is raced between the
 * Wi-Fi callback and control/tick tasks" -- see this function's own
 * declaration doc comment (mtek_capture_service.h) for the full
 * rationale. A DEDICATED mutex (not the one shared core/arbiter/router/
 * async-queue registration use elsewhere in this tree), specifically so
 * frame_cb can safely call emit_stream/emit_event while still holding
 * this lock without any risk of a recursive acquisition on a non-
 * recursive FreeRTOS mutex -- emit_stream/emit_event ultimately reach a
 * DIFFERENT mutex (the async_queue's own, for a queue-backed sink) or
 * none at all (a stack-local sink), never this one. */
static mtk_capture_lock_fn s_cap_lock, s_cap_unlock;
void mtek_capture_set_lock(mtk_capture_lock_fn lock, mtk_capture_lock_fn unlock) { s_cap_lock = lock; s_cap_unlock = unlock; }
static void cap_lock(void) { if (s_cap_lock) s_cap_lock(); }
static void cap_unlock(void) { if (s_cap_unlock) s_cap_unlock(); }

/* P0 correction (RC11 round 10, item 1): a dedicated action/lease lock,
 * genuinely distinct from s_cap_lock/s_cap_unlock above -- see its own
 * doc comment (mtek_capture_service.h's mtek_capture_set_action_lock) for
 * the full rationale. Only mtek_capture_channel_hop_tick and
 * handle_capture_start ever acquire it; never frame_cb, never anything a
 * blocking HAL call could re-enter, so it is safe to hold across the real
 * hal->set_channel() round-trip. */
static mtk_capture_action_lock_fn s_cap_action_lock, s_cap_action_unlock;
void mtek_capture_set_action_lock(mtk_capture_action_lock_fn lock, mtk_capture_action_lock_fn unlock) { s_cap_action_lock = lock; s_cap_action_unlock = unlock; }
static void cap_action_lock(void) { if (s_cap_action_lock) s_cap_action_lock(); }
static void cap_action_unlock(void) { if (s_cap_action_unlock) s_cap_action_unlock(); }

/* P0 correction (this round, item 4): see mtek_capture_service.h's own doc
 * comment on mtek_capture_service_mark_tick_task_failed. Ready by default;
 * app_main.c marks this false, once, if ble_tick_task's own xTaskCreate
 * fails -- never re-armed for the rest of this boot session. */
static uint8_t s_tick_task_ready = 1;
void mtek_capture_service_mark_tick_task_failed(void) { s_tick_task_ready = 0; }

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}

typedef struct {
    uint32_t sequence;
    uint64_t timestamp_us;
    uint8_t link_type;
    uint8_t channel;
    int8_t rssi;
    uint8_t flags;
    uint16_t original_len;
    uint16_t captured_len;
    uint8_t data[1000];
} capture_record_t;

/* RC7 independent audit P0 "Shared operation/session state remains
 * data-racy": TOKEN, not a retained `mtk_operation_record_t *` -- same
 * ABA-hazard rationale as mtek_wifi_logic.c's s_deauth/s_hs (this
 * session's own struct doc comments). frame_cb/hop_tick re-resolve to a
 * live record via mtk_op_find each time; handlers keyed on a caller-
 * supplied token compare it against s_cap.token before trusting any of
 * this struct's other fields, never assuming a fresh mtk_op_find match
 * means "this is MY session". */
static struct {
    uint32_t token;
    /* RC11 promiscuous-mode audit follow-up #3 "store the operation
     * record's core boot_epoch in long-lived handshake and capture
     * sessions, not the transport context's epoch": mtk_core-internal
     * epoch (rec->boot_epoch at handle_capture_start's own alloc time),
     * deliberately separate from ctx.boot_epoch below (the CALLING
     * TRANSPORT's own link/peer epoch, which can change independently --
     * e.g. a peer HELLO mid-session, mtek_spi_native_dispatch.c -- while
     * this long-lived, callback-driven session is still active). Every
     * frame_cb/mtek_capture_channel_hop_tick lookup below uses THIS field,
     * never ctx.boot_epoch, matching mtek_wifi_logic.c's own identical
     * fix for s_hs and its handle_deauth_start's own established
     * rationale for its background send loop. */
    uint32_t boot_epoch;
    /* P0 correction (follow-up read-only audit, "next focused P0 session-
     * publication closure round", requirement 3 "store and validate the
     * originating session generation for every long-lived callback/
     * tick/session object"): mtk_request_ctx_t's own session_generation
     * field (0 for every non-native-SPI adapter -- never fenced),
     * captured once at handle_capture_start time. frame_cb and
     * mtek_capture_channel_hop_tick -- both of which run long after this
     * call returns, from contexts with no request ctx of their own --
     * validate against THIS stored value via mtk_op_begin_publish_guard,
     * never a caller's own ctx. */
    uint32_t session_generation;
    mtk_request_ctx_t ctx;
    uint8_t mode; /* 0=PUSH, 1=POLL */
    uint16_t snap_len;
    uint32_t started_boot_us;
    /* RC7 independent audit item 11 "duration_ms is ignored": the
     * requested session duration (0 = run until explicitly stopped,
     * matching every other List B operation's own count=0/duration=0
     * "unbounded" convention in this tree) and the wall-clock start time
     * (now_ms() units, distinct from started_boot_us's own boot-relative
     * microsecond field above) needed to enforce it -- checked by
     * mtek_capture_channel_hop_tick below (this session's own periodic
     * driver, despite the hop-specific name; renamed in its own doc
     * comment, not its symbol, to avoid an unrelated header churn). */
    uint32_t duration_ms;
    uint64_t started_wall_ms;
    uint32_t next_sequence;
    uint32_t total_frames, dropped_frames, truncated_frames, credit_stalls;
    uint32_t credit_bytes;
    capture_record_t ring[CAPTURE_RING_SLOTS];
    unsigned ring_head, ring_count; /* POLL-mode unread queue */
    mtk_channelplan_t channel_plan;
    mtk_capturefilter_t filter;
    /* RC5 independent audit P1 "MonstaShark capture path is incomplete":
     * real channel hopping state for channel_plan.mode==1 -- see
     * mtek_capture_channel_hop_tick below. */
    uint8_t hop_active;
    uint8_t hop_current_channel;
    uint64_t hop_last_switch_ms;
    uint32_t filtered_frames; /* frames seen but excluded by filter.filter_bssid -- never counted as captured/dropped/truncated */
    /* RC8 independent audit P0-1 "Eliminate target stack overflow paths":
     * frame_cb below is a real Wi-Fi driver RX callback (registered via
     * hal->promisc_start), invoked directly from the Wi-Fi driver's own
     * internal context -- a stack budget this project does not configure
     * and cannot assume is as generous as its own application tasks'
     * (spi_runtime_task/uart_repl_task, 12,288 bytes each). Real ELF
     * measurement (-fstack-usage) showed frame_cb's own frame at 3,008
     * bytes, dominated by three per-call locals: a full capture_record_t
     * (~1,020 bytes, `data[1000]`), and two 952-byte PUSH-mode STREAM
     * fragment-header/chunk buffers. All three move here -- frame_cb is
     * already strictly single-threaded per invocation (serialized under
     * cap_lock for the record, and the STREAM-building section runs only
     * after cap_lock is released but still only from this one callback,
     * never re-entrantly with itself) -- eliminating essentially all of
     * frame_cb's own stack footprint rather than merely relocating one
     * piece of it. */
    capture_record_t frame_scratch;
    uint8_t stream_chunk1[8 + 944];
    uint8_t stream_chunk2[8 + 944];
} s_cap;

/* Hand-written field table for CAPTURE_POLL_READ's RECORD variant only
 * (002-capture-diagnostics-service.md Sec 3.6): the tagged-union response
 * shape (EMPTY transmits zero body bytes) isn't representable by the
 * generic generated-struct codec, so this one opcode's response is
 * assembled directly (mtek_core.h emit_response_raw) instead of forcing a
 * one-off extension onto the schema generator for a single message. */
static const mtk_field_desc_t s_poll_record_fields[] = {
    { "sequence", MTK_F_U32, offsetof(capture_record_t, sequence), 0, 0, 0, NULL, MTK_F_U8 },
    { "timestamp_us", MTK_F_U64, offsetof(capture_record_t, timestamp_us), 0, 0, 0, NULL, MTK_F_U8 },
    { "link_type", MTK_F_U8, offsetof(capture_record_t, link_type), 0, 0, 0, NULL, MTK_F_U8 },
    { "channel", MTK_F_U8, offsetof(capture_record_t, channel), 0, 0, 0, NULL, MTK_F_U8 },
    { "rssi", MTK_F_I8, offsetof(capture_record_t, rssi), 0, 0, 0, NULL, MTK_F_U8 },
    { "flags", MTK_F_U8, offsetof(capture_record_t, flags), 0, 0, 0, NULL, MTK_F_U8 },
    { "original_len", MTK_F_U16, offsetof(capture_record_t, original_len), 0, 0, 0, NULL, MTK_F_U8 },
    { "captured_len", MTK_F_U16, offsetof(capture_record_t, captured_len), 0, 0, 0, NULL, MTK_F_U8 },
};
static const mtk_struct_desc_t s_poll_record_desc = {
    s_poll_record_fields, sizeof(s_poll_record_fields) / sizeof(s_poll_record_fields[0]), sizeof(capture_record_t)
};

/* True if `filter` is unset (all-zero BSSID -- capture everything, the
 * documented "no filter" sentinel). */
static int filter_is_empty(const mtk_capturefilter_t *filter) {
    for (int i = 0; i < 6; i++) if (filter->filter_bssid.b[i]) return 0;
    return 1;
}

/* RC5 independent audit P1 "MonstaShark capture path is incomplete":
 * "capture filters ... are stored but not applied". A set BSSID filter
 * matches any of the three 802.11 header address fields (addr1/addr2/
 * addr3, the same convention mtek_wifi_hal_esp32.c's own station-target
 * scan already uses) so a filtered session captures traffic FROM, TO, or
 * belonging to the target AP's BSS, not just one specific direction. */
static int frame_matches_filter(const uint8_t *frame, uint16_t len, const mtk_capturefilter_t *filter) {
    if (filter_is_empty(filter)) return 1;
    if (len < 16 + 6) return 0; /* too short to carry addr3 -- cannot match a specific BSSID */
    return memcmp(frame + 4, filter->filter_bssid.b, 6) == 0
        || memcmp(frame + 10, filter->filter_bssid.b, 6) == 0
        || memcmp(frame + 16, filter->filter_bssid.b, 6) == 0;
}

static void frame_cb(void *user, const uint8_t *frame, uint16_t len, int8_t rssi, uint8_t channel) {
    /* P0 correction (follow-up read-only audit, "Round 8: final concurrency
     * and resource-failure closure", item 3 "close promiscuous-callback
     * session ABA"): `user` is now this exact registration's own
     * immutable token, captured once at handle_capture_start's own
     * promisc_start call -- previously always NULL, carrying no identity
     * of its own at all. See hs_frame_cb's own doc comment (mtek_wifi_
     * logic.c) for the full rationale: esp32_promisc_service can
     * legitimately copy (cb, user) together, under its own mutex, an
     * instant BEFORE a concurrent STOP runs -- and a brand-new
     * CAPTURE_START can then reinitialize s_cap for a wholly different
     * operation before this already-copied, now-stale call actually
     * executes. Reading s_cap.token/boot_epoch fresh (the previous
     * design) would validate this stale call against the NEW session's
     * own identity instead of refusing it. */
    uint32_t registered_token = (uint32_t)(uintptr_t)user;
    /* P0 correction (Round 8, item 3 "put every s_cap token/epoch/session
     * read under cap_lock"): this snapshot was previously read completely
     * UNLOCKED -- a genuine data race against cap_lock-protected writers
     * (handle_capture_start's own reinit, capture_teardown). */
    cap_lock();
    uint32_t current_token = s_cap.token;
    uint32_t boot_epoch = s_cap.boot_epoch;
    uint32_t session_generation = s_cap.session_generation;
    cap_unlock();
    if (current_token != registered_token) return; /* s_cap has moved on to a different (or no) session since THIS callback was armed */
    /* RC11 promiscuous-mode audit follow-up #2/#4 "reject STOPPING as well
     * as terminal" / "replace remaining callback-side mtk_op_find pointer
     * dereferences with atomic mtk_op_snapshot reads": a snapshot copy,
     * never a retained pointer -- and STOPPING (a concurrent capture_
     * teardown already claimed finalization and may be mid-cleanup,
     * hal->promisc_stop possibly already in flight) must reject a frame
     * exactly like a terminal state does, not just as a fast-path filter
     * but because delivering into a session already being torn down is
     * never correct regardless of how racy the actual window is. */
    mtk_operation_record_t cap_snap;
    if (!mtk_op_snapshot(registered_token, boot_epoch, &cap_snap)) return;
    if (mtk_op_state_is_terminal(cap_snap.state) || cap_snap.state == MTK_OPS_STOPPING) return;

    /* P0 correction (follow-up read-only audit, "next focused P0 session-
     * publication closure round"): the op-table check above only proves
     * this session was not YET finalized at that exact instant -- a
     * concurrent peer-session reset (which itself finalizes an active
     * capture via mtek_capture_cancel_active_for_peer_reset -> capture_
     * teardown) can still land in the window between that check and the
     * shared-state publish (ring/counters) and queue emission
     * (emit_stream) below. mtk_op_begin_publish_guard, acquired here and
     * held across ALL of that -- never across any HAL call, per its own
     * "final validation, shared-state publication, and queue emission
     * only" contract -- closes the window. Skipped entirely (no state
     * touched at all) if the session has already gone stale by this
     * point. */
    if (!mtk_op_begin_publish_guard(session_generation)) return;

    cap_lock();
    /* P0 correction (Round 8, item 3): re-validate the token identity
     * again, immediately before mutating any shared state -- the window
     * between the checks above and this lock acquisition is small but
     * real. */
    if (s_cap.token != registered_token) { cap_unlock(); mtk_op_end_publish_guard(); return; }
    if (!frame_matches_filter(frame, len, &s_cap.filter)) { s_cap.filtered_frames++; cap_unlock(); mtk_op_end_publish_guard(); return; }
    uint16_t cap_len = len > s_cap.snap_len ? s_cap.snap_len : len;
    uint8_t truncated = (cap_len < len);
    s_cap.total_frames++;
    if (truncated) s_cap.truncated_frames++;

    /* RC8 independent audit P0-1: s_cap.frame_scratch, not a stack-local
     * -- see s_cap's own doc comment. Safe to read again below AFTER
     * cap_unlock() (same established pattern this function already uses
     * for `sink`/`token`): frame_cb is invoked strictly one frame at a
     * time by the Wi-Fi driver, never concurrently with itself, so
     * nothing else can overwrite this scratch record in between. */
    capture_record_t *rec = &s_cap.frame_scratch;
    memset(rec, 0, sizeof(*rec));
    rec->sequence = s_cap.next_sequence++;
    rec->timestamp_us = (uint64_t)now_ms() * 1000ull;
    rec->link_type = 0; /* IEEE80211 */
    rec->channel = channel;
    rec->rssi = rssi;
    rec->flags = truncated ? 0x01 : 0x00;
    rec->original_len = len;
    rec->captured_len = cap_len;
    memcpy(rec->data, frame, cap_len);

    /* RC7 independent audit item 11 "PUSH mode bypasses the required
     * common ring and drops a frame immediately when credit is absent":
     * every captured frame -- PUSH or POLL -- now also lands in the
     * shared bounded ring (drop-oldest on overflow, same policy either
     * mode already used on its own), so a POLL_READ issued mid-PUSH-
     * session (or after a PUSH session stalls on missing credit) can
     * still retrieve recently-captured frames instead of nothing. PUSH
     * mode's own credit-gated STREAM delivery below is unaffected --
     * this is in ADDITION to it, not instead of it. */
    if (s_cap.ring_count >= CAPTURE_RING_SLOTS) {
        s_cap.dropped_frames++;
    } else {
        s_cap.ring[(s_cap.ring_head + s_cap.ring_count) % CAPTURE_RING_SLOTS] = *rec;
        s_cap.ring_count++;
    }
    if (s_cap.mode == 1) { /* POLL: the ring above IS this mode's own delivery path -- done. */
        cap_unlock();
        mtk_op_end_publish_guard();
        return;
    }

    /* PUSH: credit-gated STREAM delivery, fragmented per
     * 002-capture-diagnostics-service.md Sec 3.7 (25-byte first-fragment
     * header / 896B chunk; 8-byte continuation header / 944B chunk). */
    unsigned needed = 25 + (cap_len > 896 ? 896 : cap_len);
    if (cap_len > 896) needed += 8 + (cap_len - 896);
    if (s_cap.credit_bytes < needed) {
        /* RC7 independent audit item 11: "drops a frame immediately when
         * credit is absent" -- the frame is NOT actually lost any more
         * (it already landed in the common ring above, retrievable via
         * POLL_READ); only the STREAM delivery itself is skipped for
         * lack of credit. credit_stalls still counts this real condition
         * for diagnostics. dropped_frames is intentionally NOT
         * double-incremented here -- the ring-overflow branch above is
         * the only place that increments it, matching its own single,
         * well-defined meaning ("a frame this session could not retain
         * anywhere"), not "a frame PUSH couldn't stream this instant". */
        s_cap.credit_stalls++;
        cap_unlock();
        mtk_op_end_publish_guard();
        return;
    }
    s_cap.credit_bytes -= needed;
    uint32_t token = cap_snap.token;
    mtk_sink_t sink = s_cap.ctx.sink;
    cap_unlock(); /* release before the sink call below -- see this file's own lock doc comment */

    /* RC8 independent audit P0-1: s_cap.stream_chunk1/2, not stack-locals
     * -- see s_cap's own doc comment. Zero-init preserved exactly as
     * before (byte 24 reserved/padding -- RC5 independent audit P1
     * "MonstaShark capture path is incomplete" -- must never carry stale
     * memory onto the wire; a persistent field could carry a stale value
     * from a PRIOR frame's own write if not re-zeroed every call, so the
     * explicit memset below is required here, unlike a stack-local that
     * started zeroed by construction every time). */
    uint8_t *chunk = s_cap.stream_chunk1; memset(chunk, 0, 8 + 944);
    unsigned first_len = cap_len > 896 ? 896 : cap_len;
    uint8_t fragment_count = (cap_len > 896) ? 2 : 1;
    chunk[0]=(uint8_t)rec->sequence; chunk[1]=(uint8_t)(rec->sequence>>8); chunk[2]=(uint8_t)(rec->sequence>>16); chunk[3]=(uint8_t)(rec->sequence>>24);
    chunk[4] = 0; chunk[5] = fragment_count;
    for (int i = 0; i < 8; i++) chunk[6+i] = (uint8_t)(rec->timestamp_us >> (8*i));
    chunk[14] = rec->link_type; chunk[15] = rec->channel; chunk[16] = (uint8_t)rec->rssi; chunk[17] = rec->flags;
    chunk[18] = (uint8_t)rec->original_len; chunk[19] = (uint8_t)(rec->original_len>>8);
    chunk[20] = (uint8_t)rec->captured_len; chunk[21] = (uint8_t)(rec->captured_len>>8);
    chunk[22] = (uint8_t)first_len; chunk[23] = (uint8_t)(first_len>>8);
    /* chunk[24] stays 0 (reserved), per the zero-init above. */
    memcpy(chunk + 25, rec->data, first_len);
    sink.emit_stream(sink.user, token, rec->sequence, chunk, 25 + first_len);
    if (cap_len > 896) {
        unsigned rem = cap_len - 896;
        uint8_t *c2 = s_cap.stream_chunk2; memset(c2, 0, 8 + 944);
        c2[0]=(uint8_t)rec->sequence; c2[1]=(uint8_t)(rec->sequence>>8); c2[2]=(uint8_t)(rec->sequence>>16); c2[3]=(uint8_t)(rec->sequence>>24);
        c2[4] = 1; c2[5] = fragment_count;
        c2[6] = (uint8_t)rem; c2[7] = (uint8_t)(rem>>8);
        memcpy(c2 + 8, rec->data + 896, rem);
        sink.emit_stream(sink.user, token, rec->sequence + 1, c2, 8 + rem);
    }
    mtk_op_end_publish_guard();
}

/* RC7 independent audit item 11 "CREDIT addition is unsynchronized and
 * can overflow" (the overflow half -- the synchronization half is the
 * caller's own responsibility now that native SPI's CREDIT class calls
 * this from the same locked dispatch path as every other shared-state
 * mutation, mtek_spi_native_dispatch.c) + RC7 independent audit P0
 * "Native CREDIT and CANCEL are not implemented" (native SPI's own new
 * CREDIT handler needs to know whether the grant was actually applied to
 * report an honest wire status, not just fire-and-forget): saturating
 * add (never wraps past UINT32_MAX) and a real applied/not-applied
 * return, matching this service's own "session-token-scoped, explicit
 * status" requirement for the new native SPI CREDIT class. Returns 1 if
 * `token` matched the currently active capture session (credit applied),
 * 0 otherwise (unknown/mismatched/no active session -- a safe no-op, not
 * an error, matching the pre-existing UART/Bedge callers' own
 * fire-and-forget usage of this same function). */
uint8_t mtek_capture_grant_credit(uint32_t token, uint32_t bytes) {
    cap_lock();
    if (s_cap.token == 0 || s_cap.token != token) { cap_unlock(); return 0; }
    uint32_t sum = s_cap.credit_bytes + bytes;
    s_cap.credit_bytes = (sum < s_cap.credit_bytes) ? 0xFFFFFFFFu : sum; /* saturate, never wrap */
    cap_unlock();
    return 1;
}

/* Forward declaration: handle_capture_start's own new promisc_start
 * failure path (RC8 independent audit P0-9) needs this; full definition
 * (and its own doc comment) is further down, also called from
 * handle_capture_stop and mtek_capture_channel_hop_tick.
 *
 * RC11 independent correction order P0 "replace unsafe operation-record
 * pointer reads with atomic snapshots": takes (token, boot_epoch) rather
 * than a retained `mtk_operation_record_t *` -- mtek_capture_channel_hop_
 * tick's own caller context only ever has a locked mtk_op_snapshot's
 * token/epoch fields to offer in the first place (it must never retain a
 * live pointer across its own periodic-tick call boundary -- the same
 * ABA-hazard rationale as s_deauth/s_hs elsewhere in this tree), and
 * handle_capture_start/_stop's own already-valid token/epoch pair is
 * exactly as safe to pass here directly, with no pointer dereference
 * inside this function needed at all. */
static void capture_teardown(uint32_t token, uint32_t epoch, uint8_t reason, uint8_t status);

static void handle_capture_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    if (req.snap_len == 0 || req.snap_len > 1000) { respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT); return; }
    /* P0 correction (this round, item 4): a nonzero duration (auto-stop) or
     * hop-mode channel plan is only ever enforced by mtek_capture_channel_
     * hop_tick, itself only ever called from main/app_main.c's own
     * ble_tick_task -- refuse honestly, before acquiring any resource, if
     * that task never started. An unbounded, non-hopping capture needs no
     * tick at all (frame_cb alone drives it) and is unaffected. */
    if (!s_tick_task_ready && (req.duration_ms != 0 || req.channel_plan.mode == 1)) {
        respond_empty(ctx, MTK_STATUS_NOT_READY); return;
    }

    /* Diagnosed ownership-publication fix (TSan-exposed): admission is now
     * held under the same lock that serializes mtk_core_bump_session_
     * generation, from final session validation through arbiter ownership
     * publication and the ACCEPTED response, so a peer-session reset can
     * never observe a token-backed class owned by a not-yet-real token --
     * see mtk_op_begin_admission_guard's own doc comment (mtek_core.h).
     *
     * M3 lock-order correction (real, reproducible AB-BA deadlock caught
     * by a full-suite ASan/UBSan run): this file's own global nested-lock
     * order is `cap_action_lock -> pub_lock -> cap_lock` -- exactly
     * mtek_capture_channel_hop_tick's own established order (cap_action_
     * lock at its own top, mtk_op_begin_publish_guard i.e. pub_lock next,
     * cap_lock innermost). The action lease is acquired HERE, before the
     * admission guard even begins, so this call can never hold pub_lock
     * while waiting for the action lease that a paused hop tick already
     * holds -- the previous order (pub_lock first, action lease second)
     * was the reverse of hop_tick's own and formed the deadlock's other
     * half. See mtek_core.h's own admission-guard doc comment for the
     * full nested-lock contract this establishes for every Core path. */
    cap_action_lock();
    if (!mtk_op_begin_admission_guard(ctx->session_generation)) {
        cap_action_unlock();
        respond_empty(ctx, MTK_STATUS_NOT_READY);
        return;
    }
    int no_mem = 0;
    /* Release-tooling-round P0 correction (independent audit): the
     * identity is copied out atomically at mint time -- no raw record
     * pointer is retained past this point, including across the ACCEPTED
     * response and the promisc_start call below (which can synchronously
     * deliver frames via frame_cb on the fake HAL, or race a real target's
     * own callback). */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) {
        respond_empty(ctx, MTK_STATUS_NO_MEMORY);
        mtk_op_end_admission_guard();
        cap_action_unlock();
        return;
    }
    /* Publishes the REAL token in the SAME arbiter call -- never
     * acquire(class, 0) followed by a later force_transfer(class, token),
     * exactly the half-published sequence this guard exists to close. */
    if (mtk_arbiter_acquire(MTK_ARB_M, id.token) != MTK_ARB_GRANT_OK) {
        mtk_op_discard_unpublished(id.token, id.boot_epoch);
        respond_empty(ctx, MTK_STATUS_BUSY);
        mtk_op_end_admission_guard();
        cap_action_unlock();
        return;
    }
    mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_RUNNING, MTK_STATUS_OK, now_ms());

    /* RC8 independent audit P0-3 "synchronize service session state":
     * s_cap is shared, concurrently-read singleton state -- frame_cb (a
     * real Wi-Fi driver RX callback that can start firing the instant
     * hal->promisc_start below arms it, possibly before this function
     * even returns) and mtek_capture_channel_hop_tick (a separate,
     * periodic tick context) both read it under cap_lock. Initializing
     * it here without the same lock let either of those observe a
     * partially-written struct -- a real, not merely theoretical, data
     * race. Locked for the whole init, matching handle_ap_scan_start/etc
     * elsewhere in this tree that mutate shared session state before
     * anything else can observe it. */
    uint8_t fixed_channel = (req.channel_plan.mode == 0) ? req.channel_plan.channel : 1;
    /* P0 correction (RC11 round 10, item 1 "close the capture-hop pre-HAL
     * race completely"): the action lease (acquired above, before the
     * admission guard) is what actually excludes a hop tick's own HAL
     * action/commit for the whole span from here through cap_unlock below
     * -- if a hop tick for the session this call is about to replace is
     * currently mid-action (already past its own final identity
     * validation, about to call or still inside hal->set_channel()), this
     * reinit genuinely cannot proceed until that tick's own action fully
     * completes (commits or discovers it is stale and bails) and releases
     * the SAME lease, so this reinit can never race a HAL round-trip
     * already in flight for the session being replaced. The lease is
     * released right after cap_unlock below -- never held across the
     * ACCEPTED response, mtk_op_end_admission_guard, or this function's
     * own later hal->promisc_start() call, none of which need it. */
    cap_lock();
    memset(&s_cap, 0, sizeof(s_cap));
    s_cap.token = id.token; s_cap.boot_epoch = id.boot_epoch; s_cap.ctx = *ctx; s_cap.mode = req.mode; s_cap.snap_len = req.snap_len;
    s_cap.session_generation = ctx->session_generation;
    s_cap.started_boot_us = (uint32_t)(now_ms() * 1000);
    s_cap.duration_ms = req.duration_ms; s_cap.started_wall_ms = now_ms();
    s_cap.channel_plan = req.channel_plan; s_cap.filter = req.filter;
    /* RC5 independent audit P1 "MonstaShark capture path is incomplete":
     * real hop-mode state (mode==1) -- mtek_capture_channel_hop_tick
     * below actually switches channel once hop_dwell_ms elapses, driven
     * by a target-side periodic tick (matching mtek_ble_signal_meter_
     * tick/mtek_ble_gatt_tick's own established pattern -- see
     * mtek_capture_service.h). ESP32-C6 is 2.4GHz-only, matching every
     * other opcode in this tree that rejects band==1/5GHz, so hopping
     * cycles the 13 2.4GHz channels regardless of channel_plan.band. */
    if (req.channel_plan.mode == 1) {
        s_cap.hop_active = 1;
        s_cap.hop_current_channel = fixed_channel; /* 1, just armed above */
        s_cap.hop_last_switch_ms = now_ms();
    }
    cap_unlock();
    /* Released here, still holding pub_lock (the admission guard) a
     * moment longer -- releasing the OUTER lock (cap_action_lock) before
     * the INNER one (pub_lock) is safe for deadlock-freedom (only
     * ACQUISITION order can form a cycle, never release order) precisely
     * because this function never re-acquires cap_action_lock afterward;
     * any hop tick that acquires it the instant it is freed here can only
     * ever block on pub_lock next, which this call unconditionally
     * releases moments later via mtk_op_end_admission_guard below --
     * never the other way around. */
    cap_action_unlock();

    mtk_capture_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_capture_start_resp_t_desc);
    mtk_op_end_admission_guard();

    /* RC8 independent audit P0-9 "Correct raw-radio channel and monitor-
     * mode entry behavior": the accept response above is already sent by
     * this point (an ACCEPTED_ASYNC operation's own established shape --
     * "accepted" means the request was understood and an operation
     * object created, not that the underlying radio operation is
     * guaranteed to succeed), so a promisc_start failure here cannot
     * retroactively change it -- but it must not be silently discarded
     * either, leaving a capture session marked RUNNING forever with no
     * real frame delivery ever possible. Torn down immediately (same
     * cleanup path a real STOP would run) with a real terminal event and
     * status, exactly as if it had failed moments after a successful
     * start rather than never having truly started at all. */
    const mtk_wifi_hal_t *hal = mtek_wifi_get_hal();
    /* P0 correction (Round 8, item 3): `user` is this exact registration's
     * own immutable token, not NULL -- see frame_cb's own doc comment for
     * the full ABA-hazard rationale. */
    int promisc_rc = hal && hal->promisc_start ? hal->promisc_start(fixed_channel, frame_cb, (void *)(uintptr_t)id.token) : -1;
    if (promisc_rc != 0) {
        capture_teardown(id.token, id.boot_epoch, 2 /* START_FAILED -- distinct from USER_REQUEST=0/DURATION_ELAPSED=1 */, MTK_STATUS_IO_ERROR);
    }
}

/* P0 correction (this round, item 3): test-only pause hook -- see its own
 * doc comment in mtek_capture_service.h. Always NULL in production. */
static mtk_capture_hop_tick_pause_hook_t s_hop_tick_pause_hook;
void mtek_capture_set_hop_tick_pause_hook(mtk_capture_hop_tick_pause_hook_t hook) { s_hop_tick_pause_hook = hook; }

/* RC7 independent audit item 11 "duration_ms is ignored" + "channel-event
 * drop count is hard-coded zero": despite the hop-specific name (kept
 * as-is -- see mtek_capture_service.h's own doc comment on why this
 * symbol was not renamed), this is this whole session's own general
 * periodic driver, called regardless of whether hop mode is active. */
void mtek_capture_channel_hop_tick(uint64_t now) {
    /* P0 correction (follow-up read-only audit, "Round 8: final concurrency
     * and resource-failure closure", item 3 "put every s_cap token/epoch/
     * session read under cap_lock"): token/boot_epoch were previously read
     * completely UNLOCKED here -- a genuine data race against cap_lock-
     * protected writers (handle_capture_start's own reinit, frame_cb's own
     * mutations, capture_teardown). Folded into the SAME lock acquisition
     * as the rest of this tick's own snapshot, immediately below.
     *
     * P0 correction (this round, item 3 "capture channel-hop session-
     * replacement race"): session_generation is now captured HERE, as part
     * of this SAME identity snapshot, tied to snap_token -- previously it
     * was re-read fresh from s_cap much later (immediately before the
     * publish guard call), by which point a replacement session could
     * already occupy s_cap, handing the guard call a generation value that
     * belongs to the WRONG session entirely. */
    uint32_t snap_token, snap_boot_epoch, duration_ms, hop_active_u32, session_generation;
    uint64_t started_wall_ms, hop_last_switch_ms;
    uint16_t dwell;
    cap_lock();
    snap_token = s_cap.token; snap_boot_epoch = s_cap.boot_epoch;
    duration_ms = s_cap.duration_ms;
    started_wall_ms = s_cap.started_wall_ms;
    hop_active_u32 = s_cap.hop_active;
    dwell = s_cap.channel_plan.hop_dwell_ms ? s_cap.channel_plan.hop_dwell_ms : 250;
    hop_last_switch_ms = s_cap.hop_last_switch_ms;
    session_generation = s_cap.session_generation;
    cap_unlock();
    uint8_t hop_active = (uint8_t)hop_active_u32;

    mtk_operation_record_t snap;
    if (!mtk_op_snapshot(snap_token, snap_boot_epoch, &snap) ||
        mtk_op_state_is_terminal(snap.state) || snap.state == MTK_OPS_STOPPING) return;

    /* duration_ms: 0 means "run until stopped" (this tree's own
     * established convention for an unbounded List B operation, matching
     * deauth's count=0). A real elapsed-time check against the session's
     * own wall-clock start time, not a placeholder. */
    if (duration_ms != 0 && now >= started_wall_ms && (now - started_wall_ms) >= duration_ms) {
        capture_teardown(snap.token, snap.boot_epoch, 1 /* DURATION_ELAPSED -- distinct from USER_REQUEST's own reason=0 */, MTK_STATUS_OK);
        return;
    }
    if (!hop_active) return;
    if (now < hop_last_switch_ms || now - hop_last_switch_ms < dwell) return;

    /* P0 correction (RC11 round 10, item 1 "close the capture-hop pre-HAL
     * race completely"): acquired BEFORE the final identity re-validation
     * below and held across the ENTIRE window this item's own requirement
     * names -- final immutable-token validation, the real hal->
     * set_channel() HAL action, and the post-HAL state/event commit
     * re-validation. handle_capture_start blocks on this SAME lease before
     * reinitializing s_cap, so a replacement session's own reinit
     * literally cannot happen anywhere inside this window -- not merely
     * detected-after-the-fact by a state-commit re-check (round 9's own
     * fix), but structurally excluded for the whole HAL action too.
     * Released before this tick's own emit_event call below (never held
     * across that external call) and on every early-return path. */
    cap_action_lock();

    /* P0 correction (round 9, item 3): every gate below re-checks
     * s_cap.token == snap_token -- the identity THIS invocation captured at
     * entry -- in addition to hop_active. Checking hop_active alone cannot
     * distinguish "my own session is still hopping" from "a completely
     * different, newer session that also happens to be hop-mode now
     * occupies s_cap" (handle_capture_start's own memset-and-reinit on
     * every new session reuses the same static storage); checking
     * hop_active alone also cannot detect a plain STOP with no
     * replacement, which capture_teardown now clears explicitly (round 10,
     * item 1) precisely so this check catches that case too. */
    cap_lock();
    if (s_cap.token != snap_token || !s_cap.hop_active) { cap_unlock(); cap_action_unlock(); return; } /* a concurrent STOP/teardown (of THIS session) or a session replacement won this race instead */
    uint8_t new_channel = (uint8_t)((s_cap.hop_current_channel % 13) + 1); /* 1..13 round-robin -- not yet committed */
    cap_unlock();

    /* Test-only: lets a deterministic test pause exactly here -- the
     * FINAL identity validation above has just passed, this lease is held,
     * and nothing below has acted on it yet. This is exactly the point a
     * real target's scheduler could preempt this tick for an arbitrarily
     * long time; while paused, a concurrent STOP can still complete (it
     * needs no lease of its own) but a concurrent CAPTURE_START attempting
     * to replace this session genuinely BLOCKS on cap_action_lock until
     * this tick resumes and releases it below -- see mtek_capture_
     * service.h's own doc comment. */
    if (s_hop_tick_pause_hook) s_hop_tick_pause_hook();

    /* P0 correction (follow-up read-only audit, "next focused P0 session-
     * publication closure round"): the real radio channel switch below
     * is a HAL round-trip that must never be held under the publish
     * guard (requirement 5: the guard covers only final validation,
     * shared-state publication, and queue emission) -- it also happens
     * regardless of whether this session later turns out to be stale,
     * matching mtek_wifi_restore_and_release's own "the radio state must
     * be dealt with honestly regardless" precedent elsewhere in this
     * tree. cap_action_lock (held since before the final validation
     * above) is what actually excludes a replacement session's own reinit
     * for the whole duration of this call, not merely the identity check
     * immediately before it. */
    const mtk_wifi_hal_t *hal = mtek_wifi_get_hal();
    if (hal && hal->set_channel) hal->set_channel(new_channel);

    if (!mtk_op_begin_publish_guard(session_generation)) { cap_action_unlock(); return; }
    cap_lock();
    if (s_cap.token != snap_token || !s_cap.hop_active) { cap_unlock(); mtk_op_end_publish_guard(); cap_action_unlock(); return; } /* a concurrent STOP/teardown or session replacement won this race instead, in the window since the check above */
    s_cap.hop_current_channel = new_channel;
    s_cap.hop_last_switch_ms = now;
    uint32_t token = s_cap.token; /* == snap_token, proven by the check just above */
    /* RC7 independent audit item 11 "channel-event drop count is
     * hard-coded zero": the real running dropped-frame total at the
     * moment of this hop, not a placeholder. */
    uint32_t drop_count = s_cap.dropped_frames;
    mtk_sink_t sink = s_cap.ctx.sink;
    cap_unlock();
    cap_action_unlock();
    mtk_capture_channel_event_ev_t ev = {0};
    ev.operation_token = token; ev.channel = new_channel; ev.visible_drop_count = drop_count;
    sink.emit_event(sink.user, token, "CAPTURE_CHANNEL_EVENT", &ev, &mtk_capture_channel_event_ev_t_desc);
    mtk_op_end_publish_guard();
}

/* Release-tooling-round P0 correction (independent audit, "peer
 * session invalidation"): see mtek_capture_service.h's own doc comment on
 * this function's declaration for the full rationale. capture_teardown
 * itself already gates on mtk_op_claim_finalization, so this is a safe
 * no-op if the active operation somehow already finalized between the
 * arbiter check and this call. */
mtk_op_id_t mtek_capture_cancel_active_for_peer_reset(void) {
    mtk_op_id_t id = {0, 0};
    /* Coherent-reader fix (diagnosed alongside the admission-guard fix):
     * class and token read from ONE lock acquisition -- two separate
     * mtk_arbiter_active_class()/mtk_arbiter_active_token() calls could
     * observe a class from one ownership moment and a token from a later
     * one if a concurrent worker changed ownership in between. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    if (snap.cls != MTK_ARB_M) return id;
    uint32_t tok = snap.token;
    uint32_t epoch = mtk_core_boot_epoch();
    capture_teardown(tok, epoch, 3 /* PEER_RESET -- distinct from USER_REQUEST=0/DURATION_ELAPSED=1/START_FAILED=2 */, MTK_STATUS_OK);
    id.token = tok; id.boot_epoch = epoch;
    return id;
}

/* RC7 independent audit P0 "Shared operation/session state remains
 * data-racy": `rec` is gated on mtk_op_transition's own return value
 * (the linearization point), not a separate, racy pre-check -- matching
 * mtek_wifi_logic.c's own deauth/handshake fix, for the same reason
 * (defensive: capture has no second natural-completion path today, but
 * this closes the same class of bug structurally rather than relying on
 * that remaining true). */
static void capture_teardown(uint32_t token, uint32_t epoch, uint8_t reason, uint8_t status) {
    if (!mtk_op_claim_finalization(token, epoch)) return;
    /* RC8 independent audit P0-3 "synchronize service session state":
     * sink/context/counters copied out under the lock HERE, BEFORE
     * releasing MTK_ARB_M below (mtek_wifi_restore_and_release) -- not
     * after, as the previous ordering did.
     *
     * P0 correction (follow-up read-only audit, "next focused P0 session-
     * publication closure round"): the earlier ordering's own doc
     * comment already warned "a concurrent handle_capture_start could
     * already have reset s_cap for a brand-new session ... by the time
     * an unlocked read here would have run" -- but the locked read still
     * happened AFTER the arbiter release that actually permits a new
     * session to start, so once that release genuinely lets a brand-new
     * handle_capture_start race in and reinitialize s_cap, THIS call's
     * own "locked" read could still pick up the NEW session's own ctx/
     * sink/counters while reporting THIS (old) token -- the exact "old
     * callback must never read/modify a newly initialized session's
     * state" hazard this round closes. session_generation is the
     * ORIGINATING session's own, stored once at handle_capture_start
     * time -- never a caller's own ctx, which this function does not
     * always have (mtek_capture_channel_hop_tick's own duration-elapsed
     * call site has none). */
    cap_lock();
    uint32_t total_frames = s_cap.total_frames, dropped_frames = s_cap.dropped_frames, truncated_frames = s_cap.truncated_frames;
    mtk_sink_t sink = s_cap.ctx.sink;
    uint32_t session_generation = s_cap.session_generation;
    /* P0 correction (RC11 round 10, item 1 "teardown/STOP must coordinate
     * with the same ownership mechanism"): explicitly cleared here, under
     * the SAME field-level lock a hop tick's own identity re-validation
     * reads under -- a tick that revalidates `s_cap.token == snap_token`
     * for THIS exact token, strictly after this STOP has already run,
     * must still see its session as no longer legitimately hopping (not
     * only a token mismatch, which only detects a REPLACEMENT, never a
     * plain stop with no replacement yet). This does not by itself need
     * mtek_capture_set_action_lock's own lease -- teardown never touches
     * hop_current_channel or calls set_channel, so it cannot race the
     * tick's own HAL action; only a REPLACEMENT session's own reinit
     * (handle_capture_start) needs to coordinate with that lease. */
    s_cap.hop_active = 0;
    cap_unlock();
    const mtk_wifi_hal_t *hal = mtek_wifi_get_hal();
    if (hal && hal->promisc_stop) hal->promisc_stop();
    /* RC10 independent correction order P0 "restore is not actually
     * failure-atomic": restore_sta_mode now reports success/failure --
     * a failure here means the radio's own state is not confirmed safe,
     * even if the capture's own reason for stopping was a clean user
     * request; downgrade the reported status rather than claim a clean
     * stop the HAL itself could not confirm. */
    int restore_rc = mtek_wifi_restore_and_release(MTK_ARB_M, token);
    uint8_t final_status = (restore_rc == 0) ? status : MTK_STATUS_IO_ERROR;
    mtk_op_transition_by_token(token, epoch, MTK_OPS_STOPPED, final_status, now_ms());
    /* mtk_op_begin_publish_guard, held across the whole publish, closes
     * the window between winning finalization above and actually
     * emitting -- see its own doc comment in mtek_core.h. restore_sta_
     * mode already ran above, OUTSIDE any guard. */
    if (mtk_op_begin_publish_guard(session_generation)) {
        mtk_capture_stopped_ev_t ev = {0};
        ev.operation_token = token;
        ev.status = final_status; ev.reason = reason;
        ev.total_frames = total_frames; ev.dropped_frames = dropped_frames; ev.truncated_frames = truncated_frames;
        sink.emit_event(sink.user, ev.operation_token, "CAPTURE_STOPPED", &ev, &mtk_capture_stopped_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_capture_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                 const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t precheck_snap;
    /* RC12 item 1: family gate (CAPTURE_START) -- a non-capture token is
     * rejected NOT_FOUND before capture_teardown could finalize a foreign
     * operation or release its arbiter class. */
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE, &precheck_snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    capture_teardown(req.operation_token, ctx->boot_epoch, 0 /* USER_REQUEST */, MTK_STATUS_OK);
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_capture_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status; r.final_reason = 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_capture_stop_resp_t_desc);
}

static void handle_capture_status(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_capture_status_resp_t r;
    r.state = (uint8_t)snap.state;
    cap_lock();
    r.mode = s_cap.mode; r.frames_captured = s_cap.total_frames;
    r.frames_dropped = s_cap.dropped_frames; r.buffered = (uint8_t)(s_cap.ring_count > 255 ? 255 : s_cap.ring_count);
    cap_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_capture_status_resp_t_desc);
}

static void handle_capture_session_info(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                         const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_session_info_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_capture_session_info_resp_t r; memset(&r, 0, sizeof(r));
    r.session_id = snap.token;
    cap_lock();
    r.mode = s_cap.mode; r.snap_len = s_cap.snap_len;
    r.channel_plan = s_cap.channel_plan; r.filter = s_cap.filter; r.started_at_boot_us = s_cap.started_boot_us;
    cap_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_capture_session_info_resp_t_desc);
}

static void handle_capture_stats(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_stats_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_capture_stats_resp_t r;
    cap_lock();
    r.total_frames = s_cap.total_frames; r.dropped_frames = s_cap.dropped_frames;
    r.truncated_frames = s_cap.truncated_frames; r.credit_stalls = s_cap.credit_stalls;
    cap_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_capture_stats_resp_t_desc);
}

static void handle_capture_poll_read(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                      const uint8_t *req_bytes, size_t req_len) {
    mtk_capture_poll_read_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) { respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR); return; }

    /* RC12 item 1: family gate (CAPTURE_START). The s_cap.token match below
     * already rejects any token this service did not mint, but this makes
     * the family check explicit and uniform with every other token-
     * addressed handler -- a foreign token is rejected NOT_FOUND before any
     * s_cap interaction. During an active (mode==1) capture the operation
     * record is RUNNING (non-terminal, never evicted), so a legitimate poll
     * always passes. */
    if (!mtk_op_validate_family(req.operation_token, ctx->boot_epoch, CAPTURE_SERVICE_ID, CAPTURE_START_OPCODE)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }

    uint8_t out[1 + 25 + 1000];
    cap_lock();
    /* P0 correction (follow-up read-only audit, "Round 8: final concurrency
     * and resource-failure closure", item 3 "put every s_cap token/epoch/
     * session read under cap_lock"): token/mode were previously read
     * completely UNLOCKED here -- a genuine data race against cap_lock-
     * protected writers (handle_capture_start's own reinit, frame_cb's own
     * ring writes, capture_teardown). Folded into the SAME lock
     * acquisition as the ring read below. */
    if (s_cap.token == 0 || s_cap.token != req.operation_token) {
        cap_unlock();
        respond_empty(ctx, MTK_STATUS_NOT_FOUND);
        return;
    }
    if (s_cap.mode != 1) {
        cap_unlock();
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    if (s_cap.ring_count == 0) {
        cap_unlock();
        out[0] = 0; /* EMPTY: zero body bytes beyond the tag */
        ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, 1);
        return;
    }
    /* Encoded while still holding the lock -- pure computation, no
     * external call, avoiding a second ~1KB capture_record_t stack copy
     * (docs/RESOURCE_BUDGET.md's own heightened scrutiny on this exact
     * class of stack cost this round). */
    capture_record_t *rec = &s_cap.ring[s_cap.ring_head];
    s_cap.ring_head = (s_cap.ring_head + 1) % CAPTURE_RING_SLOTS;
    s_cap.ring_count--;
    out[0] = 1; /* RECORD */
    size_t enc_len = 0;
    mtk_encode(&s_poll_record_desc, rec, out + 1, sizeof(out) - 1, &enc_len);
    /* data field (bytes(max=1000), u16 length prefix) is appended manually:
     * the hand-written field table above covers only the fixed fields. */
    size_t off = 1 + enc_len;
    out[off++] = (uint8_t)rec->captured_len; out[off++] = (uint8_t)(rec->captured_len >> 8);
    memcpy(out + off, rec->data, rec->captured_len); off += rec->captured_len;
    cap_unlock();
    ctx->sink.emit_response_raw(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, out, off);
}

/* ---- diagnostics (service_id 0x0005) ---------------------------------- */

static void handle_get_transport_counters(mtk_request_ctx_t *ctx) {
    /* RC7 independent audit item 3 "packet-sequence diagnostics" /
     * "RETRY/latest-eight duplicate cache": real, live counters
     * (mtk_transport_counters_get, mtek_core.h) instead of a hard-zeroed
     * stub -- see its own doc comment for who increments each field. */
    mtk_transport_counters_t c; mtk_transport_counters_get(&c);
    mtk_get_transport_counters_resp_t r; memset(&r, 0, sizeof(r));
    r.integrity_failures = c.integrity_failures;
    r.dropped_frames = c.dropped_frames;
    r.packet_seq_gaps = c.packet_seq_gaps;
    r.retries_observed = c.retries_observed;
    r.duplicate_responses_served = c.duplicate_responses_served;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_transport_counters_resp_t_desc);
}
static void handle_get_radio_resource_state(mtk_request_ctx_t *ctx) {
    mtk_get_radio_resource_state_resp_t r; memset(&r, 0, sizeof(r));
    /* Coherent-reader fix: one snapshot instead of three separate class
     * reads plus a separate token read -- resource-state output must
     * never combine values from different ownership moments. */
    mtk_arbiter_snapshot_t snap = mtk_arbiter_snapshot();
    r.wifi_owner_class = (uint8_t)snap.cls;
    r.ble_owner_class = (uint8_t)snap.cls;
    if (snap.cls != MTK_ARB_NONE) { r.active_operation_tokens.count = 1; r.active_operation_tokens.items[0] = snap.token; }
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_radio_resource_state_resp_t_desc);
}
static void handle_get_queue_watermarks(mtk_request_ctx_t *ctx) {
    mtk_get_queue_watermarks_resp_t r; memset(&r, 0, sizeof(r));
    cap_lock();
    r.capture_ring_high_water_pct = (uint8_t)((s_cap.ring_count * 100) / CAPTURE_RING_SLOTS);
    cap_unlock();
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_queue_watermarks_resp_t_desc);
}
static void handle_get_fault_summary(mtk_request_ctx_t *ctx) {
    mtk_get_fault_summary_resp_t r; memset(&r, 0, sizeof(r));
    r.redacted = 1;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_fault_summary_resp_t_desc);
}

static void mtek_capture_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: handle_capture_start(ctx, op, req_bytes, req_len); return;
        case 0x0002: handle_capture_stop(ctx, op, req_bytes, req_len); return;
        case 0x0003: handle_capture_status(ctx, op, req_bytes, req_len); return;
        case 0x0004: handle_capture_session_info(ctx, op, req_bytes, req_len); return;
        case 0x0005: handle_capture_stats(ctx, op, req_bytes, req_len); return;
        case 0x0006: handle_capture_poll_read(ctx, op, req_bytes, req_len); return;
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

static void mtek_diagnostics_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                       const uint8_t *req_bytes, size_t req_len) {
    (void)req_bytes; (void)req_len;
    switch (op->opcode) {
        case 0x0001: handle_get_transport_counters(ctx); return;
        case 0x0002: handle_get_radio_resource_state(ctx); return;
        case 0x0003: handle_get_queue_watermarks(ctx); return;
        case 0x0004: handle_get_fault_summary(ctx); return;
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

void mtek_capture_service_register(void) {
    mtk_router_register(0x0004, mtek_capture_dispatch);
    mtk_router_register(0x0005, mtek_diagnostics_dispatch);
}
