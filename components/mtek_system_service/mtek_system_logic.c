/* Clean-room implementation from MonstaTek contract (002-system-service.md,
 * 002-service-registry.md). Portable: no ESP-IDF dependency, host-testable. */
#include "mtek_system_service.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_arbiter.h"
#include <string.h>

/* RC12 blocker round, item 1: system-service token families (service
 * 0x0000). TIME_SYNC_STOP consumes a token minted by TIME_SYNC_START. */
#define SYSTEM_SERVICE_ID       0x0000
#define TIME_SYNC_START_OPCODE  0x0008
/* Test-only opcodes (see mtek_system_dispatch's own #ifdef block): the
 * async fixture START and its paired generic STOP. Compiled ONLY when
 * MTK_ENABLE_TEST_OPCODES is defined (host tests define it via
 * host_tests/CMakeLists.txt; the ESP32 target never does), so these are
 * genuinely not present in the shipped firmware. */
#define TEST_ASYNC_START_OPCODE 0x00F0
#define TEST_ASYNC_STOP_OPCODE  0x00F1

#define MTK_SET_STR(field, cstr) do { \
    size_t _n = strlen(cstr); \
    if (_n > sizeof((field).data)) _n = sizeof((field).data); \
    (field).len = (uint16_t)_n; \
    memcpy((field).data, (cstr), _n); \
} while (0)

static mtk_system_build_info_t s_info;
static uint64_t (*s_now_ms)(void);
static int (*s_sta_query)(void);
static void (*s_reset_hook)(uint32_t);
static uint8_t s_reset_reason;

void mtek_system_set_sta_query(int (*fn)(void)) { s_sta_query = fn; }
void mtek_system_set_reset_hook(void (*fn)(uint32_t)) { s_reset_hook = fn; }
void mtek_system_set_reset_reason(uint8_t reason) { s_reset_reason = reason; }

void mtek_system_service_init(const mtk_system_build_info_t *info, uint64_t (*now_ms_fn)(void)) {
    s_info = *info;
    s_now_ms = now_ms_fn;
}

static uint64_t now_ms(void) { return s_now_ms ? s_now_ms() : 0; }

static void respond(mtk_request_ctx_t *ctx, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, body, desc);
}
static void respond_empty(mtk_request_ctx_t *ctx, uint8_t status) {
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, status, NULL, NULL);
}

/* 002-service-registry.md Sec 7: the six fixed service namespaces, echoed
 * back verbatim by GET_VERSION.service_summary so a peer can learn every
 * implemented service's version in one round trip. */
static const struct { uint16_t id; uint8_t major, minor; } s_registry_versions[] = {
    {0x0000, 1, 0}, {0x0001, 1, 0}, {0x0002, 1, 0}, {0x0003, 1, 0}, {0x0004, 1, 0}, {0x0005, 1, 0},
};

static void handle_get_version(mtk_request_ctx_t *ctx) {
    mtk_get_version_resp_t r; memset(&r, 0, sizeof(r));
    r.product_major = s_info.product_major;
    r.product_minor = s_info.product_minor;
    r.product_patch = s_info.product_patch;
    MTK_SET_STR(r.build_id, s_info.build_id ? s_info.build_id : "dev");
    r.build_dirty = s_info.build_dirty ? 1 : 0;
    r.build_epoch_s = s_info.build_epoch_s;
    r.build_epoch_reproducible = s_info.build_epoch_reproducible ? 1 : 0;
    r.target_chip = s_info.target_chip;
    MTK_SET_STR(r.hw_compat_id, s_info.hw_compat_id ? s_info.hw_compat_id : "unknown");
    MTK_SET_STR(r.esp_idf_version, s_info.esp_idf_version ? s_info.esp_idf_version : "unknown");
    r.protocol_major = 1;
    r.protocol_minor = 0;
    unsigned n = sizeof(s_registry_versions) / sizeof(s_registry_versions[0]);
    r.service_summary.count = n;
    for (unsigned i = 0; i < n; i++) {
        r.service_summary.items[i].service_id = s_registry_versions[i].id;
        r.service_summary.items[i].service_major = s_registry_versions[i].major;
        r.service_summary.items[i].service_minor = s_registry_versions[i].minor;
    }
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_version_resp_t_desc);
}

static void handle_get_limits(mtk_request_ctx_t *ctx) {
    mtk_get_limits_resp_t r; memset(&r, 0, sizeof(r));
    r.max_inflight_requests = MTK_BUDGET_MAX_INFLIGHT_REQUESTS;
    r.max_reassembly_contexts = MTK_BUDGET_MAX_REASSEMBLY_CONTEXTS;
    r.max_control_payload = MTK_BUDGET_MAX_CONTROL_PAYLOAD_BYTES;
    r.max_operation_tokens = MTK_BUDGET_MAX_OPERATION_TOKENS;
    /* Advertised as 0: there is no dedicated non-starvable terminal-event
     * reserve. Terminal events (*_STOPPED / *_COMPLETE) share the ordinary
     * best-effort priority queue with progress events, so delivery is not
     * guaranteed under sustained progress-event backpressure. GET_LIMITS
     * must report the reserve the firmware actually implements (0), not a
     * capability it does not provide (see docs/RESOURCE_BUDGET.md). */
    r.max_terminal_event_reserve = MTK_BUDGET_TERMINAL_EVENT_RESERVE;
    r.max_progress_event_queue = MTK_BUDGET_MAX_PROGRESS_EVENT_QUEUE;
    r.max_cursors = MTK_BUDGET_MAX_ACTIVE_CURSORS;
    r.max_owned_buffers = MTK_BUDGET_MAX_OWNED_BUFFERS;
    r.operation_record_retention_ms = MTK_BUDGET_OPERATION_RECORD_RETENTION_MS;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_limits_resp_t_desc);
}

extern const mtk_opcode_entry_t mtk_opcode_table[MTK_OPCODE_COUNT];

/* RC8 independent audit P0-8 "Make capabilities truthful for the exact
 * build": the generated registry's own cap_native/cap_factory_uart/
 * cap_compat_c3 fields declare several Wi-Fi opcode families as
 * MTK_CAP_SUPPORTED, but mtek_wifi_logic.c's own dispatch switch
 * unconditionally answers every one of them with MTK_STATUS_UNSUPPORTED
 * -- true today regardless of each family's own Kconfig module gate
 * (CONFIG_MTEK_MODULE_*), since not one of them has real radio-behavior
 * logic implemented yet this session (see that switch's own doc comment,
 * mtek_wifi_logic.c). GET_CAPABILITIES must never promise behavior the
 * image actually rejects -- this overlay corrects exactly (and only)
 * those opcodes to MTK_CAP_UNSUPPORTED, leaving the registry's own
 * static table (still the source of truth for every opcode this
 * overlay does not name) untouched. Update this list the moment any one
 * of these families gets a real implementation -- see
 * test_opcode_registry.c's own property test, which fails loud if this
 * list and mtek_wifi_logic.c's own dispatch switch ever drift apart. */
static int opcode_is_unimplemented_optional_wifi_module(uint16_t service_id, uint16_t opcode) {
    if (service_id != 0x0001) return 0;
    switch (opcode) {
        case 0x000D: case 0x000E: case 0x000F: /* BEACON_START/STOP/STATUS */
        case 0x0019: case 0x001A: case 0x001B: /* SOFTAP_START/STOP/STA_LIST */
        case 0x001C: case 0x001D: /* PROBE_FLOOD_START/STOP */
        case 0x001E: case 0x0026: /* PMKID_CAPTURE_START/STOP */
        case 0x001F: case 0x0020: /* KARMA_START/STOP */
        case 0x0022: case 0x0023: case 0x0024: case 0x0025: /* CAPTIVE_PORTAL_START/STOP/GET_CREDENTIALS/GET_DIAGNOSTICS */
            return 1;
        default:
            return 0;
    }
}

/* RC12 hardening round, item 5 (P1) "TIME_SYNC capability truthfulness":
 * this candidate's own handle_time_sync_start below has no SNTP client
 * wired in and ALWAYS completes FAILED/IO_ERROR, so advertising
 * TIME_SYNC_START as SUPPORTED was a dishonest capability claim. Round 9
 * (item 6) had DEFERRED the downgrade because TIME_SYNC_START was then the
 * ONLY MTK_LC_ACCEPTED_ASYNC + MTK_ARB_NONE opcode in the registry and
 * five host tests borrowed exactly that arbiter-free async shape as their
 * generic op-table/async-pool test vehicle -- downgrading would have
 * regressed all five at the protocol level.
 *
 * RC12 removed that coupling directly: those tests now register a test-
 * ONLY opcode of the same shape in the opcode overlay (mtek_opcode_
 * overlay.h, empty in production; host_tests/support/mtk_test_async_
 * fixture.h), routed by mtek_system_dispatch to this SAME handler, so they
 * still exercise the real production async machinery. With the tests no
 * longer dependent on it, TIME_SYNC_START's generated capability_state is
 * now the truthful MTK_CAP_UNSUPPORTED for native and Mtek Compatibility/C3
 * (factory-UART was already MTK_CAP_UNAVAILABLE). Restore SUPPORTED only
 * when a real SNTP client exists. No cap_for OVERLAY is needed for this --
 * the generated registry field itself now carries the honest value, so
 * cap_for returns it directly like every other opcode. */
static mtk_capability_state_t cap_for(const mtk_opcode_entry_t *op, mtk_profile_t profile) {
    if (opcode_is_unimplemented_optional_wifi_module(op->service_id, op->opcode)) return MTK_CAP_UNSUPPORTED;
    switch (profile) {
        case MTK_PROFILE_FACTORY_UART: return op->cap_factory_uart;
        case MTK_PROFILE_COMPAT_C3_SPI: return op->cap_compat_c3;
        default: return op->cap_native;
    }
}

static void handle_get_capabilities(mtk_request_ctx_t *ctx, const uint8_t *req_bytes, size_t req_len) {
    mtk_get_capabilities_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(&mtk_get_capabilities_req_t_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    uint8_t max_items = req.max_items ? req.max_items : 32;
    if (max_items > 32) max_items = 32;
    mtk_get_capabilities_resp_t r; memset(&r, 0, sizeof(r));
    uint16_t i = req.start_index;
    uint32_t n = 0;
    for (; i < MTK_OPCODE_COUNT && n < max_items; i++, n++) {
        const mtk_opcode_entry_t *op = &mtk_opcode_table[i];
        r.entries.items[n].service_id = op->service_id;
        r.entries.items[n].opcode = op->opcode;
        r.entries.items[n].capability_id = 0;
        r.entries.items[n].service_major = 1;
        r.entries.items[n].service_minor = 0;
        r.entries.items[n].state = (uint8_t)cap_for(op, ctx->profile);
        r.entries.items[n].max_concurrent = 1;
        r.entries.items[n].max_payload = 0;
        r.entries.items[n].dependencies.count = 0;
    }
    r.entries.count = n;
    r.next_index = (i < MTK_OPCODE_COUNT) ? i : 0;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_capabilities_resp_t_desc);
}

static void handle_get_operation_status(mtk_request_ctx_t *ctx, const uint8_t *req_bytes, size_t req_len) {
    mtk_get_operation_status_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(&mtk_get_operation_status_req_t_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot(req.operation_token, ctx->boot_epoch, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_get_operation_status_resp_t r; memset(&r, 0, sizeof(r));
    r.state = (uint8_t)snap.state;
    r.owning_service_id = snap.service_id;
    r.owning_operation = snap.opcode;
    r.elapsed_ms = (uint32_t)(now_ms() - snap.created_at_ms);
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_operation_status_resp_t_desc);
}

static void handle_reset_intent(mtk_request_ctx_t *ctx, const uint8_t *req_bytes, size_t req_len) {
    mtk_reset_intent_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(&mtk_reset_intent_req_t_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    uint32_t delay = req.delay_ms > 60000 ? 60000 : req.delay_ms;
    mtk_reset_intent_resp_t r; r.accepted_delay_ms = delay;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_reset_intent_resp_t_desc);
    mtk_reset_scheduled_ev_t ev; ev.operation_token = 0; ev.fires_at_boot_ms = (uint32_t)now_ms() + delay;
    ctx->sink.emit_event(ctx->sink.user, 0, "RESET_SCHEDULED", &ev, &mtk_reset_scheduled_ev_t_desc);
    if (s_reset_hook) s_reset_hook(delay);
}

static void handle_get_reset_reason(mtk_request_ctx_t *ctx) {
    mtk_get_reset_reason_resp_t r; r.reason = s_reset_reason;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_get_reset_reason_resp_t_desc);
}

static void handle_time_sync_start(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                    const uint8_t *req_bytes, size_t req_len) {
    mtk_time_sync_start_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    if (req.timeout_ms != 0 && (req.timeout_ms < 1000 || req.timeout_ms > 60000)) {
        respond_empty(ctx, MTK_STATUS_INVALID_ARGUMENT);
        return;
    }
    if (!s_sta_query || !s_sta_query()) { respond_empty(ctx, MTK_STATUS_NOT_READY); return; }
    int no_mem = 0;
    /* Release-tooling-round P0 correction (independent audit, "the
     * same slot-reuse/ABA hazard remains through every production
     * mtk_op_alloc() call site"): the identity (token, boot_epoch) is
     * copied out atomically at mint time (mtk_op_alloc_id's own doc
     * comment) -- no raw record pointer is ever held past this point,
     * including across the ACCEPTED response below and the terminal
     * transition after it. */
    mtk_op_id_t id = mtk_op_alloc_id(op->service_id, op->opcode, now_ms(), &no_mem);
    if (id.token == 0) { respond_empty(ctx, MTK_STATUS_NO_MEMORY); return; }
    mtk_time_sync_start_resp_t r; r.operation_token = id.token;
    respond(ctx, MTK_STATUS_ACCEPTED, &r, &mtk_time_sync_start_resp_t_desc);
    /* No SNTP client is wired into this candidate (no network access during
     * this build/session); the operation always completes FAILED/IO_ERROR
     * per its own contracted non-success terminal shape (all-zero time
     * fields) rather than fabricating a successful sync.
     *
     * P0 correction (follow-up read-only audit, "Round 8: final concurrency
     * and resource-failure closure", item 1): TIME_SYNC_START is ACCEPTED_
     * ASYNC and this tail runs on a deferred worker exactly like deauth's
     * own natural-completion tail -- previously it transitioned the token
     * and emitted TIME_SYNC_RESULT unconditionally, with no check that a
     * concurrent TIME_SYNC_STOP/peer-session reset had not already claimed
     * this token, and no guard against a reset landing between winning the
     * transition and actually publishing. `won` mirrors deauth_finalize's
     * own convention (mtk_op_transition_by_token's return value IS the
     * atomic "did I just win this transition" check -- STOP or a
     * concurrent finalize can only have transitioned it once); mtk_op_
     * begin_publish_guard, held across the whole publish, closes the
     * remaining reset-race window (see its own doc comment in
     * mtek_core.h). ctx is this same, still-in-scope request's own
     * context -- no separate long-lived session struct is needed here,
     * matching deauth's own established pattern. */
    int won = mtk_op_transition_by_token(id.token, id.boot_epoch, MTK_OPS_FAILED, MTK_STATUS_IO_ERROR, now_ms());
    if (won && mtk_op_begin_publish_guard(ctx->session_generation)) {
        mtk_time_sync_result_ev_t ev; memset(&ev, 0, sizeof(ev));
        ev.operation_token = id.token;
        ev.status = MTK_STATUS_IO_ERROR;
        ctx->sink.emit_event(ctx->sink.user, id.token, "TIME_SYNC_RESULT", &ev, &mtk_time_sync_result_ev_t_desc);
        mtk_op_end_publish_guard();
    }
}

static void handle_time_sync_stop(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                   const uint8_t *req_bytes, size_t req_len) {
    mtk_time_sync_stop_req_t req; memset(&req, 0, sizeof(req));
    if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
        respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
        return;
    }
    /* RC12 item 1: family gate. The real TIME_SYNC_STOP (0x0009) consumes a
     * TIME_SYNC_START (0x0008) token; the test-only STOP (0x00F1, compiled
     * only under MTK_ENABLE_TEST_OPCODES) consumes a test-START (0x00F0)
     * token. A token from any other family is rejected NOT_FOUND with no
     * transition. */
    uint16_t expected_start_opcode = TIME_SYNC_START_OPCODE;
#ifdef MTK_ENABLE_TEST_OPCODES
    if (op->opcode == TEST_ASYNC_STOP_OPCODE) expected_start_opcode = TEST_ASYNC_START_OPCODE;
#endif
    mtk_operation_record_t snap;
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, SYSTEM_SERVICE_ID, expected_start_opcode, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_op_transition_by_token_family(req.operation_token, ctx->boot_epoch, SYSTEM_SERVICE_ID, expected_start_opcode, MTK_OPS_STOPPED, MTK_STATUS_OK, now_ms());
    if (!mtk_op_snapshot_family(req.operation_token, ctx->boot_epoch, SYSTEM_SERVICE_ID, expected_start_opcode, &snap)) { respond_empty(ctx, MTK_STATUS_NOT_FOUND); return; }
    mtk_time_sync_stop_resp_t r; r.final_state = (uint8_t)snap.state; r.final_status = snap.final_status;
    respond(ctx, MTK_STATUS_OK, &r, &mtk_time_sync_stop_resp_t_desc);
}

static void mtek_system_dispatch(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                                  const uint8_t *req_bytes, size_t req_len) {
    switch (op->opcode) {
        case 0x0001: { /* PING */
            mtk_ping_req_t req; memset(&req, 0, sizeof(req));
            if (mtk_decode(op->req_desc, &req, req_bytes, req_len, NULL) != MTK_CODEC_OK) {
                respond_empty(ctx, MTK_STATUS_PROTOCOL_ERROR);
                return;
            }
            mtk_ping_resp_t r; r.nonce = req.nonce;
            respond(ctx, MTK_STATUS_OK, &r, &mtk_ping_resp_t_desc);
            return;
        }
        case 0x0002: handle_get_version(ctx); return;
        case 0x0003: handle_get_limits(ctx); return;
        case 0x0004: handle_get_capabilities(ctx, req_bytes, req_len); return;
        case 0x0005: handle_get_operation_status(ctx, req_bytes, req_len); return;
        case 0x0006: handle_reset_intent(ctx, req_bytes, req_len); return;
        case 0x0007: handle_get_reset_reason(ctx); return;
        case 0x0008: handle_time_sync_start(ctx, op, req_bytes, req_len); return;
        case 0x0009: handle_time_sync_stop(ctx, op, req_bytes, req_len); return;
#ifdef MTK_ENABLE_TEST_OPCODES
        /* RC12 hardening round, item 5 (P1) + blocker round item 1: a test-
         * only generic arbiter-free ACCEPTED_ASYNC vehicle (0x00F0 START)
         * and its paired generic SYNCHRONOUS STOP (0x00F1), routed to the
         * SAME production handlers as TIME_SYNC_START/STOP so host tests
         * exercise the real async worker/publish-guard/pool/family machinery
         * rather than a divergent reimplementation. These opcodes only exist
         * in the test-only opcode overlay (mtek_opcode_overlay.h) AND this
         * routing is compiled out entirely of the ESP32 target
         * (MTK_ENABLE_TEST_OPCODES is defined only for the host-test build,
         * never the target) -- so they are genuinely not present in the
         * shipped firmware, satisfying the blocker round's requirement that
         * TIME_SYNC_STOP be UNSUPPORTED on native while any generic STOP the
         * tests need lives in a test-only fixture. The handlers are fully
         * opcode-generic (they key off op->service_id/op->opcode/op->req_desc,
         * never a hardcoded 0x0008/0x0009). See mtk_test_async_fixture.h. */
        case TEST_ASYNC_START_OPCODE: handle_time_sync_start(ctx, op, req_bytes, req_len); return;
        case TEST_ASYNC_STOP_OPCODE:  handle_time_sync_stop(ctx, op, req_bytes, req_len); return;
#endif
        default: respond_empty(ctx, MTK_STATUS_UNSUPPORTED); return;
    }
}

mtk_register_result_t mtek_system_service_register(void) {
    return mtk_router_register(0x0000, mtek_system_dispatch);
}
