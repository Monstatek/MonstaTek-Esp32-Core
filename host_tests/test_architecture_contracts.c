#include "mtk_test.h"
#include "mtek_router.h"
#include "mtek_async_queue.h"
#include "mtek_async_sink.h"
#include "mtk_fake_sink.h"
#include <string.h>

static unsigned calls, replacements, scheduled;
static void (*pending_fn)(void *);
static void *pending_arg;
static void handler(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                    const uint8_t *bytes, size_t len) {
    calls++;
    ctx->sink.emit_response(ctx->sink.user, ctx->correlation, MTK_STATUS_OK, NULL, NULL);
}
static void replacement(mtk_request_ctx_t *ctx, const mtk_opcode_entry_t *op,
                        const uint8_t *bytes, size_t len) { replacements++; }
static int defer(void (*fn)(void *), void *arg) {
    scheduled++;
    pending_fn = fn;
    pending_arg = arg;
    return 0;
}

MTK_TEST_MAIN_BEGIN
    mtk_core_init(42);
    mtk_router_init();
    mtk_router_set_async_runner(NULL);
    MTK_CHECK_EQ(mtk_router_register(1, NULL), MTK_REGISTER_INVALID_HANDLER);
    MTK_CHECK_EQ(mtk_router_register(1, handler), MTK_REGISTER_OK);
    MTK_CHECK_EQ(mtk_router_register(1, replacement), MTK_REGISTER_DUPLICATE);
    for (unsigned i = 2; i <= 6; i++)
        MTK_CHECK_EQ(mtk_router_register((uint16_t)i, handler), MTK_REGISTER_OK);
    MTK_CHECK_EQ(mtk_router_register(7, handler), MTK_REGISTER_FULL);
    MTK_CHECK_EQ(mtk_router_register(1, replacement), MTK_REGISTER_DUPLICATE);

    /* The same harmless fixture handler proves execution mode is independent
     * of profile. No radio service is registered or invoked by this test. */
    static mtk_fake_sink_state_t sink;
    mtk_request_ctx_t ctx = {0};
    ctx.profile = MTK_PROFILE_HOST_ADAPTER;
    ctx.boot_epoch = 42;
    ctx.sink = mtk_fake_sink_make(&sink);
    mtk_router_set_async_runner(defer);
    mtk_router_dispatch(&ctx, 1, 1, NULL, 0);
    MTK_CHECK_EQ(calls, 1);
    MTK_CHECK_EQ(scheduled, 0); /* zero initialization is INLINE */
    MTK_CHECK_EQ(replacements, 0); /* duplicate did not replace the handler */
    ctx.profile = MTK_PROFILE_FACTORY_UART;
    ctx.dispatch_mode = MTK_DISPATCH_DEFER_ALLOWED;
    mtk_router_dispatch(&ctx, 1, 1, NULL, 0);
    MTK_CHECK_EQ(calls, 1);
    MTK_CHECK_EQ(scheduled, 1);
    MTK_CHECK(pending_fn != NULL);
    pending_fn(pending_arg);
    MTK_CHECK_EQ(calls, 2);
    mtk_router_set_async_runner(NULL);

    static mtk_async_queue_t queue;
    static mtk_async_frame_t frame;
    static uint8_t large[MTK_ASYNC_FRAME_MAX_BODY + 1];
    memset(large, 0x5a, sizeof(large));
    mtk_async_queue_init(&queue);
    MTK_CHECK_EQ(mtk_async_sink_resp_raw(&queue, 7, MTK_STATUS_OK, large, sizeof(large)),
                 MTK_EMIT_CAPACITY_FAILED);
    MTK_CHECK_EQ(mtk_async_queue_count(&queue), 0); /* no partial response */
    MTK_CHECK_EQ(mtk_async_sink_stream(&queue, 7, 2, large, sizeof(large)), MTK_EMIT_TRUNCATED);
    MTK_CHECK(mtk_async_queue_pop(&queue, &frame));
    MTK_CHECK_EQ(frame.body_len, MTK_ASYNC_FRAME_MAX_BODY);
    MTK_CHECK_EQ(frame.body[0], 0x5a);

    static const mtk_field_desc_t bool_field = { .type = MTK_F_BOOL };
    static const mtk_struct_desc_t bool_desc = { &bool_field, 1, 1 };
    uint8_t invalid_bool = 2;
    MTK_CHECK_EQ(mtk_async_sink_resp(&queue, 7, MTK_STATUS_OK, &invalid_bool, &bool_desc),
                 MTK_EMIT_ENCODING_FAILED);
    MTK_CHECK_EQ(mtk_async_sink_event(&queue, 7, "DONE", &invalid_bool, &bool_desc),
                 MTK_EMIT_ENCODING_FAILED);
    MTK_CHECK_EQ(mtk_async_queue_count(&queue), 0);
    for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++)
        MTK_CHECK_EQ(mtk_async_sink_resp(&queue, i, MTK_STATUS_OK, NULL, NULL), MTK_EMIT_OK);
    MTK_CHECK_EQ(mtk_async_sink_resp(&queue, 9, MTK_STATUS_OK, NULL, NULL),
                 MTK_EMIT_CAPACITY_FAILED);
    MTK_CHECK_EQ(mtk_async_sink_event(&queue, 9, "DONE", NULL, NULL), MTK_EMIT_CAPACITY_FAILED);
    MTK_CHECK_EQ(mtk_async_sink_stream(&queue, 9, 1, large, 1), MTK_EMIT_CAPACITY_FAILED);
    MTK_CHECK_EQ(mtk_async_queue_count(&queue), MTK_ASYNC_QUEUE_DEPTH);
    MTK_CHECK_EQ(mtk_async_queue_dropped_count(&queue), 3);

    int no_memory = 0;
    mtk_op_id_t id = mtk_op_alloc_id(1, 1, 0, &no_memory);
    mtk_operation_record_t snapshot;
    MTK_CHECK(id.token != 0);
    MTK_CHECK_EQ(id.boot_epoch, 42);
    MTK_CHECK(mtk_op_snapshot(id.token, id.boot_epoch, &snapshot));
    MTK_CHECK_EQ(snapshot.token, id.token);
MTK_TEST_MAIN_END
