/* OpenThread Radio Co-Processor runtime for the mtkcore-154 variant.
 *
 * The ESP32-C6 is the radio; the Thread stack itself runs on the host. Core
 * therefore builds OpenThread in RADIO_MODE_NATIVE with an RCP host
 * connection, which links only the radio/Spinel layers rather than the full
 * Thread application stack -- a deliberate architectural boundary, not a
 * size compromise: the host owns network state, Core owns the radio.
 *
 * The same boundary is what makes the variant protocol-neutral. Spinel is
 * simply the framing Thread hosts already speak; a host-side Zigbee stack
 * consumes the radio through the raw 802.15.4 service (namespace 0x0007)
 * instead, over the same hardware and the same arbiter lease. Neither path
 * requires an on-device protocol stack.
 *
 * Ownership follows the service's existing model: RCP holds the IEEE154
 * arbiter class for as long as it runs, so it cannot overlap the raw radio
 * service, a capture session, Wi-Fi or ESP-NOW. Startup failure unwinds
 * fully; stop, peer reset and transport loss all funnel through one teardown.
 */
#include "sdkconfig.h"
#if CONFIG_MTEK_IEEE802154_ENABLED && CONFIG_OPENTHREAD_ENABLED
/* Built only in RCP mode; in raw mode the raw HAL owns the driver callbacks. */

#include "mtek_ieee802154_hal.h"
#include "esp_openthread.h"
#include "esp_openthread_types.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "mtk_154_rcp";

/* Spinel link to the host: the M1's existing STM32<->ESP32 SPI wires.
 *
 * The production M1 routes exactly six signals between the STM32 and the
 * ESP32-C6 -- SPI MOSI/MISO/CLK/CS plus HANDSHAKE and DATA_READY (the board's
 * own ESP32 configuration; RESET is not wired). No second UART exists between
 * the two parts, so Spinel has to travel over those same wires.
 *
 * The ESP32-C6 has one general-purpose SPI peripheral (SPI2; SPI0/SPI1 serve
 * flash), and both Core's canonical transport and OpenThread's RCP host
 * connection drive an SPI slave on it. They therefore cannot both run in one
 * image: in this variant OpenThread owns the link and the STM32 speaks Spinel
 * directly, which is exactly how a dedicated radio co-processor behaves.
 * main/mtek_spi_runtime.c leaves the peripheral alone when this variant is
 * built. */
#define MTK_RCP_SPI_HOST      SPI2_HOST
#define MTK_RCP_SPI_PIN_MOSI  12
#define MTK_RCP_SPI_PIN_MISO  13
#define MTK_RCP_SPI_PIN_SCLK  7
#define MTK_RCP_SPI_PIN_CS    15
/* Spinel's own flow-control signal, carried on the same line the Core
 * transport uses for DATA_READY so no additional routing is required. */
#define MTK_RCP_SPI_PIN_INTR  6

static TaskHandle_t s_rcp_task;
static SemaphoreHandle_t s_rcp_exited;   /* signalled when the mainloop task returns */
static volatile uint8_t s_rcp_running;
static volatile uint8_t s_rcp_init_done; /* esp_openthread_init succeeded: deinit is owed */

/* Runs the OpenThread mainloop. It only returns once the stack is being torn
 * down, so the task signals its own exit rather than being deleted from
 * outside -- deleting a task inside the stack would strand its locks. */
static void rcp_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "OpenThread RCP mainloop entered");
    esp_err_t err = esp_openthread_launch_mainloop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_openthread_launch_mainloop returned %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "OpenThread RCP mainloop exited");
    s_rcp_running = 0;
    if (s_rcp_exited) xSemaphoreGive(s_rcp_exited);
    s_rcp_task = NULL;
    vTaskDelete(NULL);
}

int mtek_154_rcp_start(void) {
    if (s_rcp_running) return 0;   /* already serving this host */
    if (!s_rcp_exited) {
        s_rcp_exited = xSemaphoreCreateBinary();
        if (!s_rcp_exited) { ESP_LOGE(TAG, "xSemaphoreCreateBinary failed -- RCP refuses"); return -1; }
    }

    esp_openthread_platform_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* The C6's own radio: this image IS the co-processor, it does not talk to
     * a separate one. */
    cfg.radio_config.radio_mode = RADIO_MODE_NATIVE;
    /* Spinel to the STM32 over the board's existing SPI wires. */
    cfg.host_config.host_connection_mode = HOST_CONNECTION_MODE_RCP_SPI;
    cfg.host_config.spi_slave_config.host_device = MTK_RCP_SPI_HOST;
    cfg.host_config.spi_slave_config.bus_config.mosi_io_num = MTK_RCP_SPI_PIN_MOSI;
    cfg.host_config.spi_slave_config.bus_config.miso_io_num = MTK_RCP_SPI_PIN_MISO;
    cfg.host_config.spi_slave_config.bus_config.sclk_io_num = MTK_RCP_SPI_PIN_SCLK;
    cfg.host_config.spi_slave_config.bus_config.quadwp_io_num = -1;
    cfg.host_config.spi_slave_config.bus_config.quadhd_io_num = -1;
    cfg.host_config.spi_slave_config.slave_config.spics_io_num = MTK_RCP_SPI_PIN_CS;
    cfg.host_config.spi_slave_config.slave_config.queue_size = 5;
    /* CPOL=0, CPHA=1, matching the mode the M1's Core transport already uses
     * on these wires. */
    cfg.host_config.spi_slave_config.slave_config.mode = 1;
    cfg.host_config.spi_slave_config.intr_pin = MTK_RCP_SPI_PIN_INTR;
    /* An RCP keeps no Thread dataset of its own: network state belongs to the
     * host, so no storage partition is claimed here. */
    cfg.port_config.storage_partition_name = NULL;
    cfg.port_config.netif_queue_size = 10;
    cfg.port_config.task_queue_size = 10;

    esp_err_t err = esp_openthread_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_openthread_init returned %s", esp_err_to_name(err));
        return -1;
    }
    s_rcp_init_done = 1;
    s_rcp_running = 1;

    /* Unwound completely if the task cannot be created: a half-initialised
     * stack with no mainloop would own the radio and serve nobody. */
    if (xTaskCreate(rcp_task, "mtek_ot_rcp", 4096, NULL, 5, &s_rcp_task) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(mtek_ot_rcp) failed -- unwinding OpenThread init");
        s_rcp_running = 0;
        esp_openthread_deinit();
        s_rcp_init_done = 0;
        return -1;
    }
    return 0;
}

/* Always safe to call, including when start never succeeded or the mainloop
 * already exited on its own (host link lost, co-processor reset). */
void mtek_154_rcp_stop(void) {
    if (!s_rcp_init_done) return;
    uint8_t was_running = s_rcp_running;
    s_rcp_running = 0;
    /* deinit is what makes the blocking mainloop return; the task then
     * signals and deletes itself. */
    esp_err_t err = esp_openthread_deinit();
    if (err != ESP_OK) ESP_LOGW(TAG, "esp_openthread_deinit returned %s", esp_err_to_name(err));
    if (was_running && s_rcp_exited) {
        /* Bounded: a mainloop that refuses to exit must not wedge the caller,
         * and the radio lease is released either way. */
        if (xSemaphoreTake(s_rcp_exited, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "OpenThread RCP mainloop did not exit within its deadline");
        }
    }
    s_rcp_init_done = 0;
    /* esp_openthread_deinit releases the SPI slave it installed, so the
     * peripheral is free again for a later RCP session. */
}

int mtek_154_rcp_is_running(void) { return s_rcp_running ? 1 : 0; }

/* In RCP mode OpenThread owns the radio driver outright, so the raw radio
 * entry points are genuinely unavailable rather than merely idle. They are
 * present so the service links, and each refuses honestly; capability
 * reporting marks the raw opcodes UNAVAILABLE in this image, so a host
 * never discovers them in the first place. */
static int rcp_mode_start(uint8_t channel, uint8_t promiscuous, mtk_hal_154_rx_cb_t cb, void *user) {
    (void)channel; (void)promiscuous; (void)cb; (void)user;
    return -1;
}
static void rcp_mode_stop(void) { }
static void rcp_mode_service(void) { }
static int rcp_mode_set_channel(uint8_t channel) { (void)channel; return -1; }
static int rcp_mode_transmit(const uint8_t *d, uint8_t l, uint8_t c) { (void)d; (void)l; (void)c; return -1; }
static int rcp_mode_energy_scan(uint8_t ch, uint16_t dwell, int8_t *out) { (void)ch; (void)dwell; (void)out; return -1; }
static int rcp_mode_stats(uint32_t *tx, uint32_t *fail) { *tx = 0; *fail = 0; return 0; }

static const mtk_ieee802154_hal_t s_rcp_hal_impl = {
    rcp_mode_start, rcp_mode_stop, rcp_mode_service, rcp_mode_set_channel,
    rcp_mode_transmit, rcp_mode_energy_scan, rcp_mode_stats,
    mtek_154_rcp_start, mtek_154_rcp_stop, mtek_154_rcp_is_running,
};

const mtk_ieee802154_hal_t *mtek_ieee802154_hal_esp32_get(void) {
    ESP_LOGI(TAG, "ESP32-C6 802.15.4 in OpenThread RCP mode (host-tested; hardware behavior not validated)");
    return &s_rcp_hal_impl;
}

int mtek_ieee802154_hal_esp32_init(void) { return 0; }

#endif /* CONFIG_MTEK_IEEE802154_ENABLED && CONFIG_OPENTHREAD_ENABLED */
