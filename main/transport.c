/* ============================================================
   transport.c — выбор канала: LoRa (основной) или TCP (резервный).
   Инкапсулирует пакетный протокол LoRa с ACK и фрагментацией.
   TCP вызывается через node_link_* без изменений.
   ============================================================ */

#include "transport.h"
#include "lora.h"
#include "node_link.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "transport";

/* ============================================================
   Пакетный протокол LoRa
   ============================================================ */
#define PKT_MAGIC     0x4C
#define PKT_MAX       240
#define ACK_TIMEOUT   800
#define MAX_RETRIES   3

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5 };

static SemaphoreHandle_t s_ack_sem = NULL;
static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_use_lora = false;
static volatile bool     s_inited   = false;

static transport_rx_cb_t s_cmd_cb    = NULL;
static transport_rx_cb_t s_cfg_cb    = NULL;
static transport_rx_cb_t s_status_cb = NULL;
static transport_rx_cb_t s_evt_cb    = NULL;

/* ============================================================
   LoRa: отправка одного пакета с ACK и ретраями
   ============================================================ */
static esp_err_t lora_send_packet_ack(uint8_t type, uint8_t seq,
                                      const uint8_t *payload, size_t len)
{
    if (len > PKT_MAX - 4) return ESP_ERR_INVALID_SIZE;

    uint8_t buf[PKT_MAX];
    buf[0] = PKT_MAGIC;
    buf[1] = type;
    buf[2] = seq;
    buf[3] = (uint8_t)len;
    memcpy(&buf[4], payload, len);

    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        /* очищаем семафор перед отправкой */
        xSemaphoreTake(s_ack_sem, 0);

        if (lora_send_packet(buf, (uint8_t)(4 + len)) != ESP_OK) {
            ESP_LOGW(TAG, "LoRa tx fail type=%u seq=%u retry=%d",
                     type, seq, retry);
            continue;
        }

        if (xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(ACK_TIMEOUT)) == pdTRUE &&
            s_last_ack_seq == seq) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "LoRa ACK timeout type=%u seq=%u retry=%d",
                 type, seq, retry);
    }
    return ESP_FAIL;
}

/* ============================================================
   LoRa: фрагментация длинного сообщения
   ============================================================ */
static esp_err_t lora_send_fragmented(uint8_t type, const char *data, size_t len)
{
    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);
    size_t sent = 0;

    while (sent < len) {
        size_t chunk = (len - sent > PKT_MAX - 8) ? (PKT_MAX - 8) : (len - sent);
        uint8_t flag = (sent + chunk < len) ? 0x01 : 0x00;  /* 1 = продолжение */

        uint8_t buf[PKT_MAX];
        buf[0] = flag;
        memcpy(&buf[1], data + sent, chunk);

        esp_err_t err = lora_send_packet_ack(type, seq, buf, chunk + 1);
        if (err != ESP_OK) return err;

        sent += chunk;
        seq++;
    }
    return ESP_OK;
}

/* ============================================================
   LoRa: приём пакета (вызывается из callback драйвера)
   ============================================================ */
static void lora_raw_handler(const uint8_t *data, size_t len)
{
    if (len < 4 || data[0] != PKT_MAGIC) return;

    uint8_t type = data[1];
    uint8_t seq  = data[2];
    uint8_t plen = data[3];

    /* ACK для наших исходящих пакетов */
    if (type == T_ACK) {
        s_last_ack_seq = seq;
        xSemaphoreGive(s_ack_sem);
        return;
    }

    /* На входящий пакет сразу отвечаем ACK */
    uint8_t ack[4] = { PKT_MAGIC, T_ACK, seq, 0 };
    lora_send_packet(ack, 4);

    /* Сборка фрагментов */
    static char accum[8192];
    static size_t accum_len = 0;

    if (len < 5 || plen < 1) return;
    uint8_t flag  = data[4];
    size_t  chunk = plen - 1;

    if (accum_len + chunk < sizeof(accum)) {
        memcpy(&accum[accum_len], &data[5], chunk);
        accum_len += chunk;
    }

    if (flag == 0x00) {  /* последний фрагмент */
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

/* ============================================================
   TCP callbacks — просто пробрасываем в transport cb
   ============================================================ */
static void tcp_cmd_cb(const char *cmd, size_t len)
    { if (s_cmd_cb) s_cmd_cb(cmd, len); }
static void tcp_cfg_cb(const char *json, size_t len)
    { if (s_cfg_cb) s_cfg_cb(json, len); }
static void tcp_status_cb(const char *json, size_t len)
    { if (s_status_cb) s_status_cb(json, len); }
static void tcp_evt_cb(const char *evt, size_t len)
    { if (s_evt_cb) s_evt_cb(evt, len); }

/* ============================================================
   Public API
   ============================================================ */
esp_err_t transport_init(void)
{
    if (s_inited) return ESP_OK;
    s_inited = true;

    s_ack_sem = xSemaphoreCreateBinary();
    if (!s_ack_sem) {
        ESP_LOGE(TAG, "ack_sem alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* TCP-callback'и регистрируем всегда — это резерв */
    node_link_set_cmd_cb(tcp_cmd_cb);
    node_link_set_cfg_cb(tcp_cfg_cb);
    node_link_set_cb(tcp_status_cb);
    node_link_set_evt_cb(tcp_evt_cb);

    /* Пытаемся поднять LoRa как основной канал */
    if (lora_init() == ESP_OK) {
        lora_set_rx_callback(lora_raw_handler);
        lora_receive();
        s_use_lora = true;
        ESP_LOGI(TAG, "Transport = LoRa (primary), TCP = fallback");
    } else {
        s_use_lora = false;
        ESP_LOGW(TAG, "LoRa init failed → Transport = TCP");
    }

    return ESP_OK;
}

bool transport_is_lora(void) { return s_use_lora; }
bool transport_is_tcp(void)  { return !s_use_lora; }

void transport_force_lora(void)
{
    if (!s_use_lora && s_inited) {
        ESP_LOGW(TAG, "force → LoRa");
        s_use_lora = true;
    }
}

void transport_force_tcp(void)
{
    if (s_use_lora) {
        ESP_LOGW(TAG, "force → TCP");
        s_use_lora = false;
    }
}

/* ============================================================
   Отправка: выбор канала
   ============================================================ */
esp_err_t transport_send_cmd(const char *cmd, size_t len)
{
    if (s_use_lora) return lora_send_fragmented(T_CMD, cmd, len);
    return node_link_server_send_cmd(cmd, len);
}

esp_err_t transport_send_cfg(const char *json, size_t len)
{
    if (s_use_lora) return lora_send_fragmented(T_CFG, json, len);
    return node_link_server_send_config(json, len);
}

esp_err_t transport_send_status(const char *json, size_t len)
{
    if (s_use_lora) return lora_send_fragmented(T_STATUS, json, len);
    return node_link_client_send(json, len);
}

esp_err_t transport_send_evt(const char *evt, size_t len)
{
    if (s_use_lora) return lora_send_fragmented(T_EVT, evt, len);
    return node_link_client_send_evt(evt, len);
}

/* ============================================================
   Установка callback'ов
   ============================================================ */
void transport_set_cmd_cb(transport_rx_cb_t cb)    { s_cmd_cb = cb; }
void transport_set_cfg_cb(transport_rx_cb_t cb)    { s_cfg_cb = cb; }
void transport_set_status_cb(transport_rx_cb_t cb) { s_status_cb = cb; }
void transport_set_evt_cb(transport_rx_cb_t cb)    { s_evt_cb = cb; }