/* ESP-IDF-backed ESP-NOW HAL. Receives are queued in the driver callback and
 * delivered from mtek_espnow_service_tick's drain (esp32_espnow_service), so
 * portable service state is never mutated from a driver task -- the same
 * discipline the Wi-Fi HAL's promiscuous path uses. */
#include "mtek_espnow_hal.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "mtk_espnow";

#define ESPNOW_RX_QUEUE_LEN 16
#define ESPNOW_MAX_PAYLOAD  250

typedef struct {
    uint8_t src[6];
    int8_t rssi;
    uint8_t len;
    uint8_t data[ESPNOW_MAX_PAYLOAD];
} espnow_rx_msg_t;

static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_mutex;
static mtk_hal_espnow_rx_cb_t s_cb;
static void *s_cb_user;
static uint32_t s_sent_ok, s_send_fail;
static uint8_t s_started;

static void en_lock(void)   { if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void en_unlock(void) { if (s_mutex) xSemaphoreGive(s_mutex); }

static void espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    (void)info;
    en_lock();
    if (status == ESP_NOW_SEND_SUCCESS) s_sent_ok++; else s_send_fail++;
    en_unlock();
}

/* Zero timeout: the driver callback must never block because the drain has
 * fallen behind. A dropped frame is counted by the portable layer's own ring
 * accounting rather than stalling the Wi-Fi task. */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!s_rx_queue || len <= 0) return;
    espnow_rx_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    if (info && info->src_addr) memcpy(msg.src, info->src_addr, 6);
    msg.rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
    msg.len = (uint8_t)(len > ESPNOW_MAX_PAYLOAD ? ESPNOW_MAX_PAYLOAD : len);
    memcpy(msg.data, data, msg.len);
    (void)xQueueSend(s_rx_queue, &msg, 0);
}

int mtek_espnow_hal_esp32_init(void) {
    int ok = 1;
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) { ESP_LOGE(TAG, "xSemaphoreCreateMutex failed -- ESP-NOW will refuse"); ok = 0; }
    s_rx_queue = xQueueCreate(ESPNOW_RX_QUEUE_LEN, sizeof(espnow_rx_msg_t));
    if (!s_rx_queue) { ESP_LOGE(TAG, "xQueueCreate failed -- ESP-NOW will refuse"); ok = 0; }
    return ok ? 0 : -1;
}

static int esp32_espnow_start(uint8_t channel, mtk_hal_espnow_rx_cb_t cb, void *user) {
    if (!s_mutex || !s_rx_queue) {
        ESP_LOGE(TAG, "esp32_espnow_start: required runtime resources missing -- refusing");
        return -1;
    }
    /* ESP-NOW requires a started Wi-Fi driver in station mode; the channel is
     * fixed here because ESP-NOW peers must agree on it. */
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_wifi_set_mode(STA) returned %s", esp_err_to_name(err)); return -1; }
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) { ESP_LOGE(TAG, "esp_wifi_start returned %s", esp_err_to_name(err)); return -1; }
    err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_wifi_set_channel(%u) returned %s", channel, esp_err_to_name(err)); return -1; }

    err = esp_now_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_now_init returned %s", esp_err_to_name(err)); return -1; }
    /* Unwound in reverse on any partial failure, so a caller that gets -1 can
     * assume ESP-NOW is not running. */
    if (esp_now_register_recv_cb(espnow_recv_cb) != ESP_OK ||
        esp_now_register_send_cb(espnow_send_cb) != ESP_OK) {
        ESP_LOGE(TAG, "esp_now callback registration failed -- unwinding");
        esp_now_deinit();
        return -1;
    }
    en_lock();
    s_cb = cb; s_cb_user = user;
    s_sent_ok = 0; s_send_fail = 0;
    s_started = 1;
    en_unlock();
    xQueueReset(s_rx_queue);
    return 0;
}

static void esp32_espnow_stop(void) {
    en_lock();
    uint8_t was_started = s_started;
    s_started = 0;
    s_cb = NULL; s_cb_user = NULL;
    en_unlock();
    if (was_started) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        /* Forgets every peer, which is the documented teardown contract. */
        esp_now_deinit();
    }
    if (s_rx_queue) xQueueReset(s_rx_queue);
}

static void esp32_espnow_service(void) {
    if (!s_rx_queue) return;
    espnow_rx_msg_t msg;
    /* Bounded per call so one tick cannot monopolise the calling task. */
    for (unsigned i = 0; i < ESPNOW_RX_QUEUE_LEN; i++) {
        if (xQueueReceive(s_rx_queue, &msg, 0) != pdTRUE) break;
        en_lock();
        mtk_hal_espnow_rx_cb_t cb = s_cb;
        void *user = s_cb_user;
        en_unlock();
        if (cb) {
            mtk_hal_mac6_t src;
            memcpy(src.b, msg.src, 6);
            cb(user, src, msg.rssi, msg.data, msg.len);
        }
    }
}

static int esp32_espnow_add_peer(mtk_hal_mac6_t peer, uint8_t channel, uint8_t encrypt,
                                  const uint8_t *lmk, uint8_t lmk_len) {
    esp_now_peer_info_t info;
    memset(&info, 0, sizeof(info));
    memcpy(info.peer_addr, peer.b, 6);
    info.channel = channel;
    info.ifidx = WIFI_IF_STA;
    info.encrypt = encrypt ? true : false;
    if (encrypt && lmk_len == 16) memcpy(info.lmk, lmk, 16);
    esp_err_t err = esp_now_add_peer(&info);
    /* The key must not linger in this frame after the driver has copied it. */
    memset(&info, 0, sizeof(info));
    if (err == ESP_OK) return 0;
    if (err == ESP_ERR_ESPNOW_FULL) return 1;   /* caller maps this to OVERFLOW */
    if (err == ESP_ERR_ESPNOW_EXIST) return 0;  /* already present is success */
    ESP_LOGW(TAG, "esp_now_add_peer returned %s", esp_err_to_name(err));
    return -1;
}

static int esp32_espnow_send(mtk_hal_mac6_t peer, const uint8_t *data, uint8_t len) {
    esp_err_t err = esp_now_send(peer.b, data, len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send returned %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

static int esp32_espnow_stats(uint32_t *sent_ok_out, uint32_t *send_fail_out) {
    en_lock();
    *sent_ok_out = s_sent_ok;
    *send_fail_out = s_send_fail;
    en_unlock();
    return 0;
}

static const mtk_espnow_hal_t s_hal_impl = {
    esp32_espnow_start, esp32_espnow_stop, esp32_espnow_service,
    esp32_espnow_add_peer, esp32_espnow_send, esp32_espnow_stats,
};

const mtk_espnow_hal_t *mtek_espnow_hal_esp32_get(void) {
    ESP_LOGI(TAG, "ESP32-C6 ESP-NOW HAL ready (host-tested; hardware behavior not validated)");
    return &s_hal_impl;
}
