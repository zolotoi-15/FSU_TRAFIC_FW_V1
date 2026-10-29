#include "lora_transport.h"
#include "lora.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "lora_tx";
#define PKT_MAGIC     0x4C
#define PKT_MAX       240
#define ACK_TIMEOUT   1500   // мс
#define MAX_RETRIES   3

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5 };

static lora_rx_cb_t s_cmd_cb = NULL;
static lora_rx_cb_t s_cfg_cb = NULL;
static lora_rx_cb_t s_status_cb = NULL;
static lora_evt_cb_t s_evt_cb = NULL;

static SemaphoreHandle_t s_ack_sem = NULL;
static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_ready = false;

// Отправка одного пакета с ACK
static esp_err_t send_packet(uint8_t type, uint8_t seq,
                             const char *payload, size_t len) {
    if (len > PKT_MAX - 4) return ESP_ERR_INVALID_SIZE;

    uint8_t buf[PKT_MAX];
    buf[0] = PKT_MAGIC;
    buf[1] = type;
    buf[2] = seq;
    buf[3] = (uint8_t)len;
    memcpy(&buf[4], payload, len);

    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        // очищаем семафор перед отправкой
        xSemaphoreTake(s_ack_sem, 0);

        esp_err_t err = lora_send_packet(buf, 4 + len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send failed, retry %d", retry);
            continue;
        }

        // ждём ACK
        if (xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(ACK_TIMEOUT)) == pdTRUE &&
            s_last_ack_seq == seq) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "ACK timeout for seq=%u, retry %d", seq, retry);
    }
    return ESP_FAIL;
}

// Фрагментация длинных сообщений: разбиваем на куски по PKT_MAX-8
static esp_err_t send_fragmented(uint8_t type, const char *data, size_t len) {
    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);
    size_t sent = 0;
    while (sent < len) {
        size_t chunk = (len - sent > PKT_MAX - 8) ? (PKT_MAX - 8) : (len - sent);
        // первый байт payload — флаг: 0x01 = не последний, 0x00 = последний
        uint8_t flag = (sent + chunk < len) ? 0x01 : 0x00;
        uint8_t buf[PKT_MAX];
        buf[0] = flag;
        memcpy(&buf[1], data + sent, chunk);
        esp_err_t err = send_packet(type, seq, (const char*)buf, chunk + 1);
        if (err != ESP_OK) return err;
        sent += chunk;
        seq++;
    }
    return ESP_OK;
}

// Публичные обёртки
esp_err_t lora_transport_send_cmd(const char *cmd, size_t len)
    { return send_fragmented(T_CMD, cmd, len); }
esp_err_t lora_transport_send_cfg(const char *json, size_t len)
    { return send_fragmented(T_CFG, json, len); }
esp_err_t lora_transport_send_status(const char *json, size_t len)
    { return send_fragmented(T_STATUS, json, len); }
esp_err_t lora_transport_send_evt(const char *evt, size_t len)
    { return send_fragmented(T_EVT, evt, len); }

// Приём: обработка входящего пакета (вызывается из callback espp)
static void on_lora_rx(const espp::Sx126x::RxPacket &pkt) {
    if (pkt.data.size() < 4) return;
    const uint8_t *d = pkt.data.data();
    if (d[0] != PKT_MAGIC) return;

    uint8_t type = d[1];
    uint8_t seq  = d[2];
    uint8_t len  = d[3];

    if (type == T_ACK) {
        s_last_ack_seq = seq;
        xSemaphoreGive(s_ack_sem);
        return;
    }

    // Для остальных типов сразу отправляем ACK
    uint8_t ack[4] = { PKT_MAGIC, T_ACK, seq, 0 };
    lora_send_packet(ack, 4);

    // Фрагментация: собираем куски
    static char accum[8192];
    static size_t accum_len = 0;
    if (len < 1) return;
    uint8_t flag = d[4];
    size_t chunk = len - 1;
    if (accum_len + chunk < sizeof(accum)) {
        memcpy(&accum[accum_len], &d[5], chunk);
        accum_len += chunk;
    }
    if (flag == 0x00) {
        accum[accum_len] = '\0';
        switch (type) {
            case T_CMD:    if (s_cmd_cb)    s_cmd_cb(accum, accum_len);    break;
            case T_CFG:    if (s_cfg_cb)    s_cfg_cb(accum, accum_len);    break;
            case T_STATUS: if (s_status_cb) s_status_cb(accum, accum_len); break;
            case T_EVT:    if (s_evt_cb)    s_evt_cb(accum, accum_len);    break;
        }
        accum_len = 0;
    }
}

esp_err_t lora_transport_init(void) {
    if (!lora_init()) return ESP_FAIL;
    s_ack_sem = xSemaphoreCreateBinary();
    if (!s_ack_sem) return ESP_ERR_NO_MEM;

    // Регистрируем callback приёма
    extern void lora_set_rx_callback(void (*cb)(const espp::Sx126x::RxPacket&));
    lora_set_rx_callback(on_lora_rx);

    // Стартуем непрерывный приём
    lora_receive();
    s_ready = true;
    ESP_LOGI(TAG, "LoRa transport ready");
    return ESP_OK;
}

bool lora_transport_is_ready(void) { return s_ready; }