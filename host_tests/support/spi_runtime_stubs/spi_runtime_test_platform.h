#pragma once
#include <stddef.h>
#include <stdint.h>
#define CONFIG_MTEK_ADAPTER_NATIVE_SPI 1
#define CONFIG_MTEK_ADAPTER_COMPAT_SPI 0
#define IRAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGE(tag, ...) ((void)(tag))
typedef int esp_err_t;
typedef int BaseType_t;
typedef unsigned TickType_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 1
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((ms) / 10)
#define portYIELD_FROM_ISR() ((void)0)
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 0
typedef struct {
    uint64_t pin_bit_mask;
    int mode, pull_up_en, pull_down_en, intr_type;
} gpio_config_t;
typedef struct {
    int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num;
    size_t max_transfer_sz;
} spi_bus_config_t;
typedef struct spi_slave_transaction_t {
    size_t length, trans_len;
    const void *tx_buffer;
    void *rx_buffer, *user;
} spi_slave_transaction_t;
typedef struct {
    int spics_io_num, flags, queue_size, mode;
    void (*post_setup_cb)(spi_slave_transaction_t *);
    void (*post_trans_cb)(spi_slave_transaction_t *);
} spi_slave_interface_config_t;
esp_err_t gpio_config(const gpio_config_t *);
esp_err_t gpio_set_level(int, int);
esp_err_t spi_slave_initialize(int, const spi_bus_config_t *, const spi_slave_interface_config_t *, int);
esp_err_t spi_slave_queue_trans(int, const spi_slave_transaction_t *, uint32_t);
esp_err_t spi_slave_get_trans_result(int, spi_slave_transaction_t **, uint32_t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void xTaskNotifyGive(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
void vTaskDelay(TickType_t);
void vTaskDelete(TaskHandle_t);
BaseType_t xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
BaseType_t xSemaphoreGive(SemaphoreHandle_t);
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t);
int64_t esp_timer_get_time(void);
