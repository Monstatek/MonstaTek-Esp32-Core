/* Clean-room implementation from MonstaTek contract. */
#include "mtek_router.h"
#include <stddef.h>
#include <string.h>

/* system, wifi, ble, gatt, capture, diagnostics, espnow. Sized with one
 * spare slot so adding a service is a table-capacity change made
 * deliberately rather than a silent MTK_REGISTER_FULL at boot. */
#define MTK_ROUTER_MAX_SERVICES 8
/* Matches the core contract's 4-in-flight-request budget. */
#define MTK_ROUTER_ASYNC_POOL_SIZE 4
#define MTK_ROUTER_ASYNC_MAX_PAYLOAD 2048

typedef struct { uint16_t service_id; mtk_service_dispatch_fn fn; } mtk_router_entry_t;
static mtk_router_entry_t s_services[MTK_ROUTER_MAX_SERVICES];
static unsigned s_service_count;

void mtk_router_init(void) { s_service_count = 0; }

mtk_register_result_t mtk_router_register(uint16_t service_id, mtk_service_dispatch_fn fn) {
    if (!fn) return MTK_REGISTER_INVALID_HANDLER;
    for (unsigned i = 0; i < s_service_count; i++) {
        if (s_services[i].service_id == service_id) return MTK_REGISTER_DUPLICATE;
    }
    if (s_service_count >= MTK_ROUTER_MAX_SERVICES) return MTK_REGISTER_FULL;
    s_services[s_service_count].service_id = service_id;
    s_services[s_service_count].fn = fn;
    s_service_count++;
    return MTK_REGISTER_OK;
}

static mtk_service_dispatch_fn find_service(uint16_t service_id) {
    for (unsigned i = 0; i < s_service_count; i++) {
        if (s_services[i].service_id == service_id) return s_services[i].fn;
    }
    return NULL;
}

/* DISABLED/UNAVAILABLE/UNSUPPORTED are capability-discovery states only; a
 * request against any of them returns the legal wire status UNSUPPORTED with no
 * side effect. */
static mtk_capability_state_t cap_for_profile(const mtk_opcode_entry_t *op, mtk_profile_t profile) {
    switch (profile) {
        case MTK_PROFILE_FACTORY_UART: return op->cap_factory_uart;
        case MTK_PROFILE_COMPAT_SPI: return op->cap_compat_spi;
        case MTK_PROFILE_NATIVE_SPI:
        case MTK_PROFILE_HOST_ADAPTER:
        default: return op->cap_native;
    }
}

static mtk_async_runner_fn s_async_runner;
static mtk_router_lock_fn s_lock, s_unlock;

void mtk_router_set_async_runner(mtk_async_runner_fn runner) { s_async_runner = runner; }
void mtk_router_set_lock(mtk_router_lock_fn lock, mtk_router_lock_fn unlock) { s_lock = lock; s_unlock = unlock; }

typedef struct {
    uint8_t in_use;
    mtk_service_dispatch_fn fn;
    mtk_request_ctx_t ctx;
    const mtk_opcode_entry_t *op;
    uint8_t req_bytes[MTK_ROUTER_ASYNC_MAX_PAYLOAD];
    size_t req_len;
} mtk_router_async_slot_t;

static mtk_router_async_slot_t s_async_pool[MTK_ROUTER_ASYNC_POOL_SIZE];

static mtk_router_async_slot_t *async_pool_alloc(void) {
    if (s_lock) s_lock();
    mtk_router_async_slot_t *slot = NULL;
    for (unsigned i = 0; i < MTK_ROUTER_ASYNC_POOL_SIZE; i++) {
        if (!s_async_pool[i].in_use) { slot = &s_async_pool[i]; slot->in_use = 1; break; }
    }
    if (s_unlock) s_unlock();
    return slot;
}

static void async_pool_release(mtk_router_async_slot_t *slot) {
    if (s_lock) s_lock();
    slot->in_use = 0;
    if (s_unlock) s_unlock();
}

/* Thread-local (not global): with up to 4 genuinely concurrent deferred
 * operations, a plain global flag would be wrong the instant more than one
 * worker is executing at once (thread A finishing and clearing it while thread
 * B's own handler is still legitimately running). Each worker thread sets this
 * only around its own slot->fn call below, so mtk_router_running_on_worker
 * always answers for "the thread calling it right now", which is exactly what a
 * handler like DEAUTH_START's count=0 "run until stopped" loop needs to know
 * before deciding it is safe to block indefinitely (see mtek_wifi_logic.c). */
static _Thread_local int t_running_on_worker;

static void async_trampoline(void *arg) {
    mtk_router_async_slot_t *slot = (mtk_router_async_slot_t *)arg;
    /* This slot was captured (mtk_router_dispatch, below) BEFORE this worker
     * thread ever actually started running -- a real async runner (e.g. a
     * FreeRTOS task pool) may not schedule this trampoline until well after the
     * calling thread returned, and a native-SPI peer reboot detected in that
     * window must not let a request queued under the OLD peer session mint a
     * brand-new operation that would be indistinguishable from one legitimately
     * created by the NEW session. session_generation==0 (every non- native-SPI
     * adapter) is never fenced.
     *
     * A stale request must release its slot WITHOUT emitting any response/event
     * into the shared native-SPI queue. The request's own request_id may already
     * have been reused by a brand-new request minted under the new session by
     * the time this stale worker finally runs; a response emitted here would be
     * indistinguishable from a real answer to that new request and could corrupt
     * it. The stale requester already lost its session on the peer side and
     * cannot observe any answer, so slot->fn is simply never invoked and the
     * slot is freed in silence -- no operation is minted, no arbiter class is
     * ever acquired, and no frame is ever queued under this request_id. */
    if (slot->ctx.session_generation != 0 && slot->ctx.session_generation != mtk_core_session_generation()) {
        async_pool_release(slot);
        return;
    }
    t_running_on_worker = 1;
    slot->fn(&slot->ctx, slot->op, slot->req_bytes, slot->req_len);
    t_running_on_worker = 0;
    async_pool_release(slot);
}

int mtk_router_running_on_worker(void) { return t_running_on_worker; }

void mtk_router_dispatch(mtk_request_ctx_t *ctx, uint16_t service_id, uint16_t opcode,
                          const uint8_t *req_bytes, size_t req_len) {
    const mtk_opcode_entry_t *op = mtk_opcode_find(service_id, opcode);
    if (!op) {
        /* Unknown/out-of-range opcode: never routed at all (SPI_PROTOCOL_V1.md). */
        ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_PROTOCOL_ERROR, NULL, NULL);
        return;
    }
    mtk_capability_state_t cap = cap_for_profile(op, ctx->profile);
    if (cap != MTK_CAP_SUPPORTED) {
        ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_UNSUPPORTED, NULL, NULL);
        return;
    }
    mtk_service_dispatch_fn fn = find_service(service_id);
    if (!fn) {
        ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_UNSUPPORTED, NULL, NULL);
        return;
    }

    /* Only an explicitly persistent sink may be deferred. Adapters choose
     * execution eligibility independently of their wire/capability profile;
     * factory UART continues to select inline dispatch. */
    if (s_async_runner && op->lifecycle == MTK_LC_ACCEPTED_ASYNC &&
        ctx->dispatch_mode == MTK_DISPATCH_DEFER_ALLOWED) {
        if (req_len > MTK_ROUTER_ASYNC_MAX_PAYLOAD) {
            ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_OVERFLOW, NULL, NULL);
            return;
        }
        mtk_router_async_slot_t *slot = async_pool_alloc();
        if (!slot) {
            /* Checked allocation failure, per task requirement -- never a
             * silently dropped or serialized-behind request. */
            ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_NO_MEMORY, NULL, NULL);
            return;
        }
        slot->fn = fn;
        slot->ctx = *ctx;
        slot->op = op;
        if (req_len) memcpy(slot->req_bytes, req_bytes, req_len);
        slot->req_len = req_len;
        if (s_async_runner(async_trampoline, slot) != 0) {
            /* Task creation itself failed (checked, per task requirement):
             * release the slot and report the failure synchronously --
             * never leave a "phantom" accepted operation with no worker
             * ever running it. */
            async_pool_release(slot);
            ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_NO_MEMORY, NULL, NULL);
        }
        return;
    }

    fn(ctx, op, req_bytes, req_len);
}
