/* Executes the actual SPI task against a simulated slave driver. No radio
 * handlers, USB device, or physical peripheral are accessed. */
#include "mtk_test.h"
#include "spi_runtime_test_platform.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_async_sink.h"
#include "mtek_core.h"
#include <setjmp.h>
#include <string.h>

static mtk_async_queue_t *output_queue;
static mtk_spi_native_dispatch_ctx_t *native_ctx;
static void observe_notify(mtk_async_queue_t *q, mtk_async_notify_fn fn, void *ctx) {
    output_queue = q;
    mtk_async_queue_set_notify(q, fn, ctx);
}
static void observe_init(mtk_spi_native_dispatch_ctx_t *ctx, uint32_t epoch) {
    native_ctx = ctx;
    mtek_spi_native_dispatch_init(ctx, epoch);
}
#define mtk_async_queue_set_notify observe_notify
#define mtek_spi_native_dispatch_init observe_init
#include "../main/mtek_spi_runtime.c"
#undef mtk_async_queue_set_notify
#undef mtek_spi_native_dispatch_init

static jmp_buf finished;
static spi_slave_interface_config_t callbacks;
static spi_slave_transaction_t *armed;
static unsigned transaction, polls, phase, wakes;
static int task_cookie, dataready;
static uint32_t clock_ms;
static uint8_t tx_snapshot[MTK_SPI_NATIVE_CELL_SIZE], ack_snapshot[MTK_COMPAT_CELL_SIZE];
static unsigned queue_lock_depth, notifications;
static void test_queue_lock(void *ctx) { MTK_CHECK_EQ(queue_lock_depth, 0); queue_lock_depth++; }
static void test_queue_unlock(void *ctx) { MTK_CHECK_EQ(queue_lock_depth, 1); queue_lock_depth--; }
static void test_queue_notify(void *ctx) {
    MTK_CHECK_EQ(queue_lock_depth, 0); /* callback must be outside the lock */
    MTK_CHECK(mtk_async_queue_count((mtk_async_queue_t *)ctx) > 0);
    notifications++;
}

esp_err_t gpio_config(const gpio_config_t *cfg) { return ESP_OK; }
esp_err_t gpio_set_level(int pin, int level) {
    if (pin == PIN_DATAREADY) dataready = level;
    return ESP_OK;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return &task_cookie; }
void xTaskNotifyGive(TaskHandle_t task) { MTK_CHECK(task == &task_cookie); wakes++; }
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *wake) {
    xTaskNotifyGive(task); *wake = pdFALSE;
}
int64_t esp_timer_get_time(void) { return (int64_t)clock_ms * 1000; }
void vTaskDelay(TickType_t ticks) { MTK_CHECK(ticks > 0); clock_ms += ticks * 10; }
void vTaskDelete(TaskHandle_t task) { MTK_CHECK(0); longjmp(finished, 1); }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *task) { return pdPASS; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return &task_cookie; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { return pdTRUE; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks) { return pdTRUE; }

esp_err_t spi_slave_initialize(int host, const spi_bus_config_t *bus,
                              const spi_slave_interface_config_t *cfg, int dma) {
    callbacks = *cfg;
    MTK_CHECK_EQ(cfg->queue_size, 1);
    return ESP_OK;
}
esp_err_t spi_slave_queue_trans(int host, const spi_slave_transaction_t *t, uint32_t ticks) {
    MTK_CHECK(armed == NULL); /* never requeue while the driver owns buffers */
    transaction++;
    if (transaction > 7) { MTK_CHECK(0); longjmp(finished, 1); }
    MTK_CHECK_EQ(t->length, (transaction <= 5 ? MTK_COMPAT_CELL_SIZE : MTK_SPI_NATIVE_CELL_SIZE) * 8);
    if (transaction == 2) memcpy(ack_snapshot, t->tx_buffer, sizeof(ack_snapshot));
    if (transaction >= 3 && transaction <= 5)
        MTK_CHECK(memcmp(ack_snapshot, t->tx_buffer, sizeof(ack_snapshot)) == 0);
    if (transaction == 7) {
        mtk_spi_native_header_t hdr;
        const uint8_t *body;
        MTK_CHECK_EQ(mtk_spi_native_parse_cell(t->tx_buffer, &hdr, &body), MTK_SPI_PARSE_OK);
        MTK_CHECK_EQ(hdr.msg_class, MTK_SPI_CLASS_EVENT);
        MTK_CHECK_EQ(dataready, 1);
        longjmp(finished, 1);
    }
    armed = (spi_slave_transaction_t *)t;
    memcpy(tx_snapshot, t->tx_buffer, sizeof(tx_snapshot));
    polls = 0;
    callbacks.post_setup_cb(armed);
    return ESP_OK;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
    MTK_CHECK(ticks > 0);
    MTK_CHECK(armed != NULL);
    MTK_CHECK(memcmp(armed->tx_buffer, tx_snapshot, sizeof(tx_snapshot)) == 0);
    if (transaction == 6 && phase == 0) {
        /* Output arrives with an IDLE cell already armed and the master silent. */
        MTK_CHECK_EQ(dataready, 0);
        MTK_CHECK_EQ(mtk_async_sink_event(output_queue, 7, "READY", NULL, NULL), MTK_EMIT_OK);
        native_ctx->inbound.active = 1;
        native_ctx->inbound.last_seen_ms = clock_ms;
        phase = 1;
    } else if (transaction == 6 && phase == 1) {
        MTK_CHECK_EQ(dataready, 1);
        MTK_CHECK_EQ(native_ctx->inbound.active, 1);
        clock_ms += MTK_SPI_NATIVE_REASM_TIMEOUT_MS;
        phase = 2;
    } else MTK_CHECK(0);
    uint32_t pending = wakes; wakes = 0;
    return pending;
}
esp_err_t spi_slave_get_trans_result(int host, spi_slave_transaction_t **out, uint32_t ticks) {
    MTK_CHECK(armed != NULL);
    MTK_CHECK_EQ(ticks, 0);
    MTK_CHECK(memcmp(armed->tx_buffer, tx_snapshot, sizeof(tx_snapshot)) == 0);
    polls++;
    if (transaction == 6) {
        if (phase < 2) return ESP_ERR_TIMEOUT;
        MTK_CHECK_EQ(native_ctx->inbound.active, 0); /* expiry without RX traffic */
        MTK_CHECK_EQ(dataready, 1); /* queue wake was not lost */
        MTK_CHECK(polls >= 3);
    }
    mtk_spi_native_header_t hdr = {0};
    hdr.magic = MTK_SPI_NATIVE_MAGIC;
    hdr.major = MTK_SPI_NATIVE_MAJOR;
    hdr.minor = MTK_SPI_NATIVE_MINOR;
    hdr.msg_class = transaction == 1 ? MTK_SPI_CLASS_HELLO : MTK_SPI_CLASS_IDLE;
    hdr.boot_epoch = 123;
    hdr.packet_seq = transaction;
    /* Only full transactions replace RX bytes. Short ones retain stale RX
     * data to prove the runtime rejects the physical transfer first. */
    if (transaction == 1 || transaction >= 5)
        mtk_spi_native_build_cell(&hdr, NULL, armed->rx_buffer);
    armed->trans_len = transaction == 2 ? 0 :
                       transaction == 3 ? 4095 :
                       transaction == 4 ? 4088 : armed->length;
    callbacks.post_trans_cb(armed);
    *out = armed;
    armed = NULL;
    return ESP_OK;
}

MTK_TEST_MAIN_BEGIN
    mtk_core_init(123);
    mtk_arbiter_init();
    mtk_router_init();
    mtk_transport_claim_reset();
    mtk_transport_counters_reset();
    if (setjmp(finished) == 0) spi_runtime_task(NULL);
    MTK_CHECK_EQ(transaction, 7);
    MTK_CHECK_EQ(phase, 2);

    /* Expiry uses elapsed milliseconds across counter wrap, independently
     * of polling rate and without requiring another request. */
    native_ctx->inbound.active = 1;
    native_ctx->inbound.last_seen_ms = UINT32_MAX - 1000u;
    for (unsigned i = 0; i < 2500; i++)
        mtek_spi_native_dispatch_tick(native_ctx, UINT32_MAX - 1000u);
    MTK_CHECK_EQ(native_ctx->inbound.active, 1);
    mtek_spi_native_dispatch_tick(native_ctx, 998);
    MTK_CHECK_EQ(native_ctx->inbound.active, 1);
    mtek_spi_native_dispatch_tick(native_ctx, 999);
    MTK_CHECK_EQ(native_ctx->inbound.active, 0);

    static mtk_async_queue_t queue;
    static mtk_async_frame_t frame;
    mtk_async_queue_init(&queue);
    mtk_async_queue_set_lock(&queue, test_queue_lock, test_queue_unlock, NULL);
    mtk_async_queue_set_notify(&queue, test_queue_notify, &queue);
    frame.kind = MTK_ASYNC_FRAME_RESPONSE;
    for (unsigned i = 0; i < MTK_ASYNC_QUEUE_DEPTH; i++)
        MTK_CHECK(mtk_async_queue_push(&queue, &frame));
    MTK_CHECK_EQ(notifications, MTK_ASYNC_QUEUE_DEPTH);
    MTK_CHECK_EQ(mtk_async_queue_push(&queue, &frame), 0);
    MTK_CHECK_EQ(notifications, MTK_ASYNC_QUEUE_DEPTH); /* no wake for a rejected push */
MTK_TEST_MAIN_END
