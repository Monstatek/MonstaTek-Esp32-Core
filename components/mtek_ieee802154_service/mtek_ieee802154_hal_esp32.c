/* ESP-IDF-backed IEEE 802.15.4 HAL for the ESP32-C6.
 *
 * The driver's completion callbacks (esp_ieee802154_receive_done,
 * transmit_done/failed, energy_detect_done) are documented as running in ISR
 * context, so none of them touch service state directly: received frames are
 * copied into a FreeRTOS queue from the ISR and handed to the portable layer
 * from esp32_154_service, which the application tick calls on an ordinary
 * task. This is the same deferred-delivery discipline the Wi-Fi HAL's
 * promiscuous path already uses.
 *
 * Compiled into the dedicated 802.15.4 variant only. The universal image
 * leaves CONFIG_MTEK_IEEE802154_ENABLED unset, so this translation unit is
 * empty there and the radio driver is never linked -- measured evidence
 * showed 802.15.4 does not fit alongside the full Wi-Fi/BLE image. */
#include "sdkconfig.h"
#if CONFIG_MTEK_IEEE802154_ENABLED

#include "mtek_ieee802154_hal.h"
#include "esp_ieee802154.h"
#include "esp_ieee802154_types.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "mtk_154";

#define I154_RX_QUEUE_LEN 16

typedef struct {
    uint64_t timestamp_us;
    uint8_t channel;
    int8_t rssi;
    uint8_t lqi;
    uint8_t len;
    uint8_t data[MTK_154_MAX_PHY_LEN];
} i154_rx_msg_t;

static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_ed_done;     /* energy-detect completion */
static mtk_hal_154_rx_cb_t s_cb;
static void *s_cb_user;
static volatile int8_t s_ed_result;
static uint32_t s_transmitted, s_tx_failures;
static uint8_t s_started;

static void h_lock(void)   { if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void h_unlock(void) { if (s_mutex) xSemaphoreGive(s_mutex); }

/* ---- driver callbacks (ISR context) ----------------------------------- */

void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info) {
    /* frame[0] is the PHY length; the driver replaces the FCS with RSSI/LQI,
     * which frame_info reports directly. Nothing here may block or touch
     * service state: copy, post, and hand the buffer straight back. */
    if (s_rx_queue && frame && frame_info) {
        i154_rx_msg_t msg;
        uint8_t len = frame[0];
        if (len > MTK_154_MAX_PHY_LEN) len = MTK_154_MAX_PHY_LEN;
        msg.timestamp_us = frame_info->timestamp;
        msg.channel = frame_info->channel;
        msg.rssi = frame_info->rssi;
        msg.lqi = frame_info->lqi;
        msg.len = len;
        if (len) memcpy(msg.data, frame + 1, len);
        BaseType_t hp_woken = pdFALSE;
        /* Zero wait: a full queue drops the frame (counted by the portable
         * ring's own accounting) rather than stalling the radio ISR. */
        (void)xQueueSendFromISR(s_rx_queue, &msg, &hp_woken);
        if (hp_woken == pdTRUE) portYIELD_FROM_ISR();
    }
    esp_ieee802154_receive_handle_done(frame);
}

void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                   esp_ieee802154_frame_info_t *ack_frame_info) {
    (void)frame;
    s_transmitted++;
    /* An ACK frame is owned by the driver until handed back. */
    if (ack) esp_ieee802154_receive_handle_done(ack);
    (void)ack_frame_info;
}

void esp_ieee802154_transmit_failed(const uint8_t *frame, esp_ieee802154_tx_error_t error) {
    (void)frame; (void)error;
    s_tx_failures++;
}

void esp_ieee802154_energy_detect_done(int8_t power) {
    s_ed_result = power;
    if (s_ed_done) {
        BaseType_t hp_woken = pdFALSE;
        xSemaphoreGiveFromISR(s_ed_done, &hp_woken);
        if (hp_woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

/* ---- HAL ---------------------------------------------------------------- */

int mtek_ieee802154_hal_esp32_init(void) {
    int ok = 1;
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) { ESP_LOGE(TAG, "xSemaphoreCreateMutex failed -- 802.15.4 will refuse"); ok = 0; }
    s_ed_done = xSemaphoreCreateBinary();
    if (!s_ed_done) { ESP_LOGE(TAG, "xSemaphoreCreateBinary failed -- energy scan will refuse"); ok = 0; }
    s_rx_queue = xQueueCreate(I154_RX_QUEUE_LEN, sizeof(i154_rx_msg_t));
    if (!s_rx_queue) { ESP_LOGE(TAG, "xQueueCreate failed -- 802.15.4 will refuse"); ok = 0; }
    return ok ? 0 : -1;
}

static int esp32_154_start(uint8_t channel, uint8_t promiscuous, mtk_hal_154_rx_cb_t cb, void *user) {
    if (!s_mutex || !s_rx_queue) {
        ESP_LOGE(TAG, "esp32_154_start: required runtime resources missing -- refusing");
        return -1;
    }
    esp_err_t err = esp_ieee802154_enable();
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_ieee802154_enable returned %s", esp_err_to_name(err)); return -1; }
    /* Unwound in reverse on any partial failure, so a caller that gets -1 can
     * assume the radio is not receiving. */
    err = esp_ieee802154_set_channel(channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ieee802154_set_channel(%u) returned %s", channel, esp_err_to_name(err));
        esp_ieee802154_disable();
        return -1;
    }
    /* Promiscuous mode disables address filtering, which is what a sniffer
     * needs; a protocol consumer leaves it off so the driver filters for it. */
    err = esp_ieee802154_set_promiscuous(promiscuous ? true : false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ieee802154_set_promiscuous returned %s", esp_err_to_name(err));
        esp_ieee802154_disable();
        return -1;
    }
    err = esp_ieee802154_receive();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ieee802154_receive returned %s", esp_err_to_name(err));
        esp_ieee802154_disable();
        return -1;
    }
    h_lock();
    s_cb = cb; s_cb_user = user;
    s_transmitted = 0; s_tx_failures = 0;
    s_started = 1;
    h_unlock();
    xQueueReset(s_rx_queue);
    return 0;
}

static void esp32_154_stop(void) {
    h_lock();
    uint8_t was_started = s_started;
    s_started = 0;
    s_cb = NULL; s_cb_user = NULL;
    h_unlock();
    if (was_started) {
        esp_err_t err = esp_ieee802154_disable();
        if (err != ESP_OK) ESP_LOGW(TAG, "esp_ieee802154_disable returned %s", esp_err_to_name(err));
    }
    if (s_rx_queue) xQueueReset(s_rx_queue);
}

static void esp32_154_service(void) {
    if (!s_rx_queue) return;
    i154_rx_msg_t msg;
    /* Bounded per call so one tick cannot monopolise the calling task. */
    for (unsigned i = 0; i < I154_RX_QUEUE_LEN; i++) {
        if (xQueueReceive(s_rx_queue, &msg, 0) != pdTRUE) break;
        h_lock();
        mtk_hal_154_rx_cb_t cb = s_cb;
        void *user = s_cb_user;
        h_unlock();
        if (cb) cb(user, msg.timestamp_us, msg.channel, msg.rssi, msg.lqi, msg.data, msg.len);
    }
}

static int esp32_154_set_channel(uint8_t channel) {
    esp_err_t err = esp_ieee802154_set_channel(channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_ieee802154_set_channel(%u) returned %s", channel, esp_err_to_name(err));
        return -1;
    }
    /* Retuning leaves the receiver armed on the new channel. */
    err = esp_ieee802154_receive();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_ieee802154_receive after retune returned %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int esp32_154_transmit(const uint8_t *data, uint8_t len, uint8_t cca) {
    /* The driver takes a PSDU buffer whose first octet is the PHY length. */
    static uint8_t tx_buf[1 + MTK_154_MAX_PHY_LEN];
    if (len == 0 || len > MTK_154_MAX_PHY_LEN) return -1;
    tx_buf[0] = len;
    memcpy(tx_buf + 1, data, len);
    esp_err_t err = esp_ieee802154_transmit(tx_buf, cca ? true : false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_ieee802154_transmit returned %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int esp32_154_energy_scan(uint8_t channel, uint16_t dwell_ms, int8_t *peak_rssi_out) {
    if (!s_ed_done) return -1;
    esp_err_t err = esp_ieee802154_set_channel(channel);
    if (err != ESP_OK) { ESP_LOGW(TAG, "energy scan: set_channel(%u) %s", channel, esp_err_to_name(err)); return -1; }
    /* Drain any stale completion before arming, so a previous scan's result
     * can never be read as this one's. */
    (void)xSemaphoreTake(s_ed_done, 0);
    err = esp_ieee802154_energy_detect((uint32_t)dwell_ms * 1000u);
    if (err != ESP_OK) { ESP_LOGW(TAG, "esp_ieee802154_energy_detect returned %s", esp_err_to_name(err)); return -1; }
    /* Bounded wait: a driver that never reports completion must not hang the
     * caller. The margin covers scheduling latency on top of the dwell. */
    if (xSemaphoreTake(s_ed_done, pdMS_TO_TICKS((uint32_t)dwell_ms + 200u)) != pdTRUE) {
        ESP_LOGW(TAG, "energy scan on channel %u did not complete within its deadline", channel);
        return -1;
    }
    *peak_rssi_out = s_ed_result;
    /* Leave the receiver armed again for whatever follows. */
    (void)esp_ieee802154_receive();
    return 0;
}

static int esp32_154_stats(uint32_t *transmitted_out, uint32_t *tx_failures_out) {
    h_lock();
    *transmitted_out = s_transmitted;
    *tx_failures_out = s_tx_failures;
    h_unlock();
    return 0;
}

static const mtk_ieee802154_hal_t s_hal_impl = {
    esp32_154_start, esp32_154_stop, esp32_154_service, esp32_154_set_channel,
    esp32_154_transmit, esp32_154_energy_scan, esp32_154_stats,
};

const mtk_ieee802154_hal_t *mtek_ieee802154_hal_esp32_get(void) {
    ESP_LOGI(TAG, "ESP32-C6 IEEE 802.15.4 HAL ready (host-tested; hardware behavior not validated)");
    return &s_hal_impl;
}

#endif /* CONFIG_MTEK_IEEE802154_ENABLED -- radio compiled out of this variant */
