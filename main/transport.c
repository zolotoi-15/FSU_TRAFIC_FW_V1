/* ============================================================
   transport.c — LoRa-транспорт (единственный канал).

   === FIX ===
   - мьютекс s_accum_mtx защищает статические буферы.
   - флаг s_tx_busy: не отправляем новый пакет, пока
     предыдущий transmit не завершён (защита от переполнения
     очереди SX126x и гонки с ack_tx_task).
   - transport_is_tx_idle() для node_send_task.
   ============================================================ */

#include "transport.h"
#include "lora.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "transport";

#ifndef TRANSPORT_LOG_TX
#define TRANSPORT_LOG_TX  1
#endif
#ifndef TRANSPORT_LOG_RX
#define TRANSPORT_LOG_RX  1
#endif
#ifndef TRANSPORT_LOG_PAYLOAD_MAX
#define TRANSPORT_LOG_PAYLOAD_MAX  64
#endif

#define PKT_MAGIC         0x4C
#define PKT_MAX           240
#define ACK_TIMEOUT       1500
#define ACK_TIMEOUT_CFG   4000
#define MAX_RETRIES       3

#define ACCUM_CFG_MAX     8192
#define ACCUM_CMD_MAX     2048
#define ACCUM_STATUS_MAX  4096
#define ACCUM_EVT_MAX     1024

#define CFG_NEW_BURST_MS  10000
#define CFG_MIN_VALID_LEN 3000

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5 };

static SemaphoreHandle_t s_ack_sem = NULL;
static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_use_lora = false;
static volatile bool     s_inited   = false;

static QueueHandle_t     s_ack_queue = NULL;

/* === FIX: мьютекс, защищающий статические буферы накопления === */
static SemaphoreHandle_t s_accum_mtx = NULL;
/* === FIX: флаг «идёт передача» === */
static volatile bool     s_tx_busy  = false;
/* === END FIX === */

static transport_rx_cb_t s_cmd_cb    = NULL;
static transport_rx_cb_t s_cfg_cb    = NULL;
static transport_rx_cb_t s_status_cb = NULL;
static transport_rx_cb_t s_evt_cb    = NULL;

static char   s_accum_cmd[ACCUM_CMD_MAX];       static size_t s_accum_cmd_len = 0;
static char   s_accum_cfg[ACCUM_CFG_MAX];       static size_t s_accum_cfg_len = 0;
static char   s_accum_status[ACCUM_STATUS_MAX]; static size_t s_accum_status_len = 0;
static char   s_accum_evt[ACCUM_EVT_MAX];       static size_t s_accum_evt_len = 0;

static uint32_t s_last_cfg_frag_ms = 0;
static uint8_t  s_last_cfg_seq     = 0xFF;

/* ============================================================
   Публичные аксессоры.
   ============================================================ */
bool transport_is_cfg_rx_in_progress(void)
{
    if (!s_accum_mtx) return s_accum_cfg_len > 0;
    bool v = false;
    if (xSemaphoreTake(s_accum_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        v = s_accum_cfg_len > 0;
        xSemaphoreGive(s_accum_mtx);
    }
    return v;
}

void transport_reset_cfg_accum(void)
{
    if (!s_accum_mtx) {
        s_accum_cfg_len = 0;
        s_last_cfg_frag_ms = 0;
        s_last_cfg_seq = 0xFF;
        ESP_LOGW(TAG, "transport_reset_cfg_accum: mutex NULL, direct reset");
        return;
    }
    if (xSemaphoreTake(s_accum_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_accum_cfg_len = 0;
        s_last_cfg_frag_ms = 0;
        s_last_cfg_seq = 0xFF;
        xSemaphoreGive(s_accum_mtx);
        ESP_LOGI(TAG, "transport_reset_cfg_accum: len=0");
    } else {
        ESP_LOGW(TAG, "transport_reset_cfg_accum: mutex timeout");
    }
}

/* === FIX: публичный геттер «TX свободен» === */
bool transport_is_tx_idle(void)
{
    return !s_tx_busy;
}
/* === END FIX === */

/* ============================================================ */
static const char *pkt_type_str(uint8_t t)
{
    switch (t) {
        case T_CMD:    return "CMD";
        case T_CFG:    return "CFG";
        case T_STATUS: return "STATUS";
        case T_EVT:    return "EVT";
        case T_ACK:    return "ACK";
        default:       return "?";
    }
}

static void log_hex_ascii(const char *prefix, const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;
    size_t n = len;
    bool truncated = false;
    if (n > TRANSPORT_LOG_PAYLOAD_MAX) {
        n = TRANSPORT_LOG_PAYLOAD_MAX;
        truncated = true;
    }
    char hex[TRANSPORT_LOG_PAYLOAD_MAX * 3 + 1];
    char asc[TRANSPORT_LOG_PAYLOAD_MAX + 1];
    size_t h = 0, a = 0;
    for (size_t i = 0; i < n; i++) {
        h += snprintf(hex + h, sizeof(hex) - h, "%02X ", data[i]);
        asc[a++] = isprint(data[i]) ? (char)data[i] : '.';
    }
    hex[h] = '\0';
    asc[a] = '\0';
    if (truncated) {
        ESP_LOGI(TAG, "%s len=%u (first %u): %s| %s...",
                 prefix, (unsigned)len, (unsigned)n, hex, asc);
    } else {
        ESP_LOGI(TAG, "%s len=%u: %s| %s",
                 prefix, (unsigned)len, hex, asc);
    }
}

static void ack_tx_task(void *arg)
{
    ESP_LOGI(TAG, "ack_tx_task started");
    uint8_t seq;
    while (1) {
        if (xQueueReceive(s_ack_queue, &seq, portMAX_DELAY) == pdTRUE) {
            /* === FIX: не отправляем ACK, если TX занят === */
            if (s_tx_busy) {
                ESP_LOGW(TAG, "ack_tx: TX busy, skip ACK seq=%u", seq);
                continue;
            }
            /* === END FIX === */
            uint8_t ack[4] = { PKT_MAGIC, T_ACK, seq, 0 };
            esp_err_t err = lora_send_packet(ack, 4);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ack_tx: send ACK seq=%u failed: %s",
                         seq, esp_err_to_name(err));
            }
        }
    }
}

static void lora_poll_task(void *arg)
{
    ESP_LOGI(TAG, "lora_poll_task started (50 ms period)");
    while (1) {
        lora_poll();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

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

#if TRANSPORT_LOG_TX
    {
        char prefix[48];
        snprintf(prefix, sizeof(prefix), "TX %s seq=%u",
                 pkt_type_str(type), seq);
        log_hex_ascii(prefix, payload, len);
    }
#endif

    if (type == T_STATUS || type == T_CMD) {
        esp_err_t err = lora_send_packet(buf, (uint8_t)(4 + len));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX %s seq=%u failed: %s",
                     pkt_type_str(type), seq, esp_err_to_name(err));
        }
        return err;
    }

    int ack_timeout = (type == T_CFG) ? ACK_TIMEOUT_CFG : ACK_TIMEOUT;

    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        xSemaphoreTake(s_ack_sem, 0);

        if (lora_send_packet(buf, (uint8_t)(4 + len)) != ESP_OK) {
            ESP_LOGW(TAG, "LoRa tx fail type=%u seq=%u retry=%d",
                     type, seq, retry);
            continue;
        }

        if (xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(ack_timeout)) == pdTRUE &&
            s_last_ack_seq == seq) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "LoRa ACK timeout type=%u seq=%u retry=%d (timeout=%d ms)",
                 type, seq, retry, ack_timeout);
    }
    return ESP_FAIL;
}

static esp_err_t lora_send_fragmented(uint8_t type, const char *data, size_t len)
{
    /* === FIX: берём флаг TX busy, чтобы node_send_task не наложился === */
    s_tx_busy = true;
    /* === END FIX === */

    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);
    size_t sent = 0;
    int frag_idx = 0;
    int frag_total = (len + (PKT_MAX - 8) - 1) / (PKT_MAX - 8);

#if TRANSPORT_LOG_TX
    ESP_LOGI(TAG, "TX %s (fragmented): total %u bytes, %d fragments",
             pkt_type_str(type), (unsigned)len, frag_total);
#endif

    while (sent < len) {
        size_t chunk = (len - sent > PKT_MAX - 8) ? (PKT_MAX - 8) : (len - sent);
        uint8_t flag = (sent + chunk < len) ? 0x01 : 0x00;

        uint8_t buf[PKT_MAX];
        buf[0] = flag;
        memcpy(&buf[1], data + sent, chunk);

#if TRANSPORT_LOG_TX
        ESP_LOGI(TAG, "TX %s frag %d/%d seq=%u len=%u",
                 pkt_type_str(type), frag_idx + 1, frag_total,
                 seq, (unsigned)(chunk + 1));
#endif

        esp_err_t err = lora_send_packet_ack(type, seq, buf, chunk + 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "TX %s frag %d/%d FAILED: %s",
                     pkt_type_str(type), frag_idx + 1, frag_total,
                     esp_err_to_name(err));
            s_tx_busy = false;   /* FIX: сброс флага */
            return err;
        }

        sent += chunk;
        seq++;
        frag_idx++;
    }

    s_tx_busy = false;           /* FIX: сброс флага */
    return ESP_OK;
}

/* ============================================================
   lora_raw_handler — приём пакета.
   ============================================================ */
static void lora_raw_handler(const uint8_t *data, size_t len)
{
    if (len < 4 || data[0] != PKT_MAGIC) {
#if TRANSPORT_LOG_RX
        ESP_LOGW(TAG, "RX junk (len=%u, magic=0x%02X)",
                 (unsigned)len, len > 0 ? data[0] : 0);
#endif
        return;
    }

    uint8_t type = data[1];
    uint8_t seq  = data[2];
    uint8_t plen = data[3];

    if (type == T_ACK) {
#if TRANSPORT_LOG_RX
        ESP_LOGI(TAG, "RX ACK seq=%u", seq);
#endif
        s_last_ack_seq = seq;
        xSemaphoreGive(s_ack_sem);
        return;
    }

    if (s_ack_queue) {
        xQueueSend(s_ack_queue, &seq, 0);
    }

    /* === FIX: захватываем мьютекс накопления === */
    if (s_accum_mtx &&
        xSemaphoreTake(s_accum_mtx, pdMS_TO_TICKS(20)) != pdTRUE) {
        ESP_LOGW(TAG, "RX %s seq=%u drop frag (accum mutex busy)",
                 pkt_type_str(type), seq);
        return;
    }

    char  *accum = NULL;
    size_t *accum_len = NULL;
    size_t max_len = 0;

    switch (type) {
        case T_CMD:    accum = s_accum_cmd;    accum_len = &s_accum_cmd_len;    max_len = sizeof(s_accum_cmd);    break;
        case T_CFG:    accum = s_accum_cfg;    accum_len = &s_accum_cfg_len;    max_len = sizeof(s_accum_cfg);    break;
        case T_STATUS: accum = s_accum_status; accum_len = &s_accum_status_len; max_len = sizeof(s_accum_status); break;
        case T_EVT:    accum = s_accum_evt;    accum_len = &s_accum_evt_len;    max_len = sizeof(s_accum_evt);    break;
        default:
            if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
            return;
    }

    if (len < 5 || plen < 1) {
        if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
        return;
    }
    uint8_t flag  = data[4];
    size_t  chunk = plen - 1;

    if (type == T_CFG && flag == 0x01) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (*accum_len > 0 && (now - s_last_cfg_frag_ms) > CFG_NEW_BURST_MS) {
            ESP_LOGW(TAG, "RX CFG new burst (%u ms gap), reset accum (%u)",
                     (unsigned)(now - s_last_cfg_frag_ms),
                     (unsigned)*accum_len);
            *accum_len = 0;
            s_last_cfg_seq = 0xFF;
        }
        s_last_cfg_frag_ms = now;

        if (seq == s_last_cfg_seq && *accum_len > 0) {
            ESP_LOGW(TAG, "CFG frag seq=%u duplicate — ignore (accum=%u)",
                     seq, (unsigned)*accum_len);
            if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
            return;
        }
        s_last_cfg_seq = seq;
    }

    if (type == T_CFG && flag == 0x01 && *accum_len > 4096) {
        ESP_LOGW(TAG, "RX CFG accum overflow (%u), reset",
                 (unsigned)*accum_len);
        *accum_len = 0;
        s_last_cfg_seq = 0xFF;
    }

#if TRANSPORT_LOG_RX
    {
        char prefix[48];
        snprintf(prefix, sizeof(prefix), "RX %s seq=%u flag=%u",
                 pkt_type_str(type), seq, flag);
        log_hex_ascii(prefix, &data[5], chunk);
    }
#endif

    if (type == T_CFG) {
        ESP_LOGI(TAG, "CFG frag: seq=%u flag=%u plen=%u chunk=%u accum_before=%u",
                 seq, flag, plen, (unsigned)chunk, (unsigned)*accum_len);
    }

    if (*accum_len + chunk < max_len) {
        memcpy(&accum[*accum_len], &data[5], chunk);
        *accum_len += chunk;
    } else {
        ESP_LOGW(TAG, "RX %s overflow, reset", pkt_type_str(type));
        *accum_len = 0;
        if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
        return;
    }

    if (flag == 0x00) {
        if (*accum_len == 0) {
            ESP_LOGW(TAG, "RX %s last frag but accum empty — ignore",
                     pkt_type_str(type));
            if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
            return;
        }
        accum[*accum_len] = '\0';

        if (type == T_CFG && *accum_len < CFG_MIN_VALID_LEN) {
            ESP_LOGW(TAG, "RX CFG too short (%u < %u) — ignore",
                     (unsigned)*accum_len, CFG_MIN_VALID_LEN);
            *accum_len = 0;
            s_last_cfg_frag_ms = 0;
            s_last_cfg_seq = 0xFF;
            if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
            return;
        }

#if TRANSPORT_LOG_RX
        ESP_LOGI(TAG, "RX %s assembled: %u bytes",
                 pkt_type_str(type), (unsigned)*accum_len);
#endif

        if (type == T_CFG) {
            size_t tail_n = *accum_len > 32 ? 32 : *accum_len;
            size_t off = *accum_len - tail_n;
            ESP_LOG_BUFFER_HEXDUMP("CFG_TAIL_RX",
                                   accum + off, tail_n, ESP_LOG_ERROR);
        }

        /* Копируем данные локально, чтобы вызвать callback ВНЕ мьютекса */
        char   local_buf[ACCUM_CFG_MAX];
        size_t local_len = *accum_len;
        if (local_len > sizeof(local_buf)) local_len = sizeof(local_buf);
        memcpy(local_buf, accum, local_len);
        local_buf[local_len] = '\0';

        *accum_len = 0;
        s_last_cfg_frag_ms = 0;
        s_last_cfg_seq = 0xFF;

        if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);

        switch (type) {
            case T_CMD:    if (s_cmd_cb)    s_cmd_cb(local_buf, local_len);    break;
            case T_CFG:    if (s_cfg_cb)    s_cfg_cb(local_buf, local_len);    break;
            case T_STATUS: if (s_status_cb) s_status_cb(local_buf, local_len); break;
            case T_EVT:    if (s_evt_cb)    s_evt_cb(local_buf, local_len);    break;
        }
        return;
    }

    if (s_accum_mtx) xSemaphoreGive(s_accum_mtx);
}

esp_err_t transport_init(void)
{
    if (s_inited) return ESP_OK;
    s_inited = true;

    s_ack_sem = xSemaphoreCreateBinary();
    if (!s_ack_sem) {
        ESP_LOGE(TAG, "ack_sem alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_ack_queue = xQueueCreate(8, sizeof(uint8_t));
    if (!s_ack_queue) {
        ESP_LOGE(TAG, "ack_queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_accum_mtx = xSemaphoreCreateMutex();
    if (!s_accum_mtx) {
        ESP_LOGE(TAG, "accum_mtx alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (lora_init() == ESP_OK) {
        lora_set_rx_callback(lora_raw_handler);
        lora_receive();
        s_use_lora = true;

        xTaskCreate(lora_poll_task, "lora_poll", 12288, NULL, 3, NULL);  /* FIX: стек 12K */
        xTaskCreate(ack_tx_task,    "ack_tx",    4096, NULL, 3, NULL);

        ESP_LOGI(TAG, "Transport = LoRa only (no TCP fallback)");
        ESP_LOGI(TAG, "Transport log: TX=%d RX=%d max_payload=%d",
                 TRANSPORT_LOG_TX, TRANSPORT_LOG_RX,
                 TRANSPORT_LOG_PAYLOAD_MAX);
        return ESP_OK;
    }

    s_use_lora = false;
    ESP_LOGE(TAG, "LoRa init failed → NO transport (TCP disabled)");
    return ESP_FAIL;
}

bool transport_is_lora(void) { return s_use_lora; }
bool transport_is_tcp(void)  { return false; }

void transport_force_lora(void)
{
    if (s_inited && !s_use_lora) {
        ESP_LOGW(TAG, "force → LoRa (not implemented, LoRa always primary)");
    }
}

void transport_force_tcp(void)
{
    ESP_LOGW(TAG, "transport_force_tcp: TCP disabled (no Wi-Fi)");
}

esp_err_t transport_send_cmd(const char *cmd, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_cmd: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_CMD, cmd, len);
}

esp_err_t transport_send_cfg(const char *json, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_cfg: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_CFG, json, len);
}

esp_err_t transport_send_status(const char *json, size_t len)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) {
        ESP_LOGD(TAG, "transport_send_status: ignored on ROOT");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_status: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_STATUS, json, len);
}

esp_err_t transport_send_evt(const char *evt, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_evt: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_EVT, evt, len);
}

void transport_set_cmd_cb(transport_rx_cb_t cb)    { s_cmd_cb = cb; }
void transport_set_cfg_cb(transport_rx_cb_t cb)    { s_cfg_cb = cb; }
void transport_set_status_cb(transport_rx_cb_t cb) { s_status_cb = cb; }
void transport_set_evt_cb(transport_rx_cb_t cb)    { s_evt_cb = cb; }