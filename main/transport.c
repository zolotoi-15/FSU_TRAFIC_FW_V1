/* ============================================================
   transport.c — LoRa-транспорт (единственный канал).
   T_CMD / T_STATUS — fire-and-forget.
   T_CFG / T_EVT — ACK 1500 мс (CFG) / 800 мс (EVT).
   === FIX: CFG_NEW_BURST_MS = 2000 ===
   === FIX: ACK_TIMEOUT_CFG = 1500 ===
   === FIX: игнорировать flag=0 при accum_len==0 ===
   === FIX: игнорировать CFG < 3000 байт ===
   === FIX: s_last_cfg_frag_ms = 0 при завершении ===
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
#define ACK_TIMEOUT       800
#define ACK_TIMEOUT_CFG   1500     /* было 800 */
#define MAX_RETRIES       3

#define ACCUM_CFG_MAX     8192
#define ACCUM_CMD_MAX     2048
#define ACCUM_STATUS_MAX  4096
#define ACCUM_EVT_MAX     1024

#define CFG_NEW_BURST_MS  2000     /* было 1500 */
#define CFG_MIN_VALID_LEN 3000     /* минимальный размер CFG */

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5 };

static SemaphoreHandle_t s_ack_sem = NULL;
static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_use_lora = false;
static volatile bool     s_inited   = false;

static QueueHandle_t     s_ack_queue = NULL;

static transport_rx_cb_t s_cmd_cb    = NULL;
static transport_rx_cb_t s_cfg_cb    = NULL;
static transport_rx_cb_t s_status_cb = NULL;
static transport_rx_cb_t s_evt_cb    = NULL;

static char   s_accum_cmd[ACCUM_CMD_MAX];       static size_t s_accum_cmd_len = 0;
static char   s_accum_cfg[ACCUM_CFG_MAX];       static size_t s_accum_cfg_len = 0;
static char   s_accum_status[ACCUM_STATUS_MAX]; static size_t s_accum_status_len = 0;
static char   s_accum_evt[ACCUM_EVT_MAX];       static size_t s_accum_evt_len = 0;

static uint32_t s_last_cfg_frag_ms = 0;

void transport_reset_cfg_accum(void)
{
    s_accum_cfg_len = 0;
    s_last_cfg_frag_ms = 0;
    ESP_LOGI(TAG, "transport_reset_cfg_accum: len=0");
}

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
            return err;
        }

        sent += chunk;
        seq++;
        frag_idx++;
    }
    return ESP_OK;
}

/* === FIX: лог RX junk, игнорирование flag=0 при accum_len==0, CFG < 3000 === */
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

    char  *accum = NULL;
    size_t *accum_len = NULL;
    size_t max_len = 0;

    switch (type) {
        case T_CMD:    accum = s_accum_cmd;    accum_len = &s_accum_cmd_len;    max_len = sizeof(s_accum_cmd);    break;
        case T_CFG:    accum = s_accum_cfg;    accum_len = &s_accum_cfg_len;    max_len = sizeof(s_accum_cfg);    break;
        case T_STATUS: accum = s_accum_status; accum_len = &s_accum_status_len; max_len = sizeof(s_accum_status); break;
        case T_EVT:    accum = s_accum_evt;    accum_len = &s_accum_evt_len;    max_len = sizeof(s_accum_evt);    break;
        default: return;
    }

    if (len < 5 || plen < 1) return;
    uint8_t flag  = data[4];
    size_t  chunk = plen - 1;

    /* === FIX: пауза > 2000 мс между фрагментами CFG → новый CFG → сброс === */
    if (type == T_CFG && flag == 0x01) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (*accum_len > 0 && (now - s_last_cfg_frag_ms) > CFG_NEW_BURST_MS) {
            ESP_LOGW(TAG, "RX CFG new burst (%u ms gap), reset accum (%u)",
                     (unsigned)(now - s_last_cfg_frag_ms),
                     (unsigned)*accum_len);
            *accum_len = 0;
        }
        s_last_cfg_frag_ms = now;
    }

    if (type == T_CFG && flag == 0x01 && *accum_len > 4096) {
        ESP_LOGW(TAG, "RX CFG accum overflow (%u), reset",
                 (unsigned)*accum_len);
        *accum_len = 0;
    }

#if TRANSPORT_LOG_RX
    {
        char prefix[48];
        snprintf(prefix, sizeof(prefix), "RX %s seq=%u flag=%u",
                 pkt_type_str(type), seq, flag);
        log_hex_ascii(prefix, &data[5], chunk);
    }
#endif

    if (*accum_len + chunk < max_len) {
        memcpy(&accum[*accum_len], &data[5], chunk);
        *accum_len += chunk;
    } else {
        ESP_LOGW(TAG, "RX %s overflow, reset", pkt_type_str(type));
        *accum_len = 0;
        return;
    }

    if (flag == 0x00) {
        /* === FIX: последний фрагмент без первого — игнорировать === */
        if (*accum_len == 0) {
            ESP_LOGW(TAG, "RX %s last frag but accum empty — ignore",
                     pkt_type_str(type));
            return;
        }
        accum[*accum_len] = '\0';

        /* === FIX: CFG слишком короткий — игнорировать === */
        if (type == T_CFG && *accum_len < CFG_MIN_VALID_LEN) {
            ESP_LOGW(TAG, "RX CFG too short (%u < %u) — ignore",
                     (unsigned)*accum_len, CFG_MIN_VALID_LEN);
            *accum_len = 0;
            s_last_cfg_frag_ms = 0;
            return;
        }

#if TRANSPORT_LOG_RX
        ESP_LOGI(TAG, "RX %s assembled: %u bytes",
                 pkt_type_str(type), (unsigned)*accum_len);
#endif
        switch (type) {
            case T_CMD:    if (s_cmd_cb)    s_cmd_cb(accum, *accum_len);    break;
            case T_CFG:    if (s_cfg_cb)    s_cfg_cb(accum, *accum_len);    break;
            case T_STATUS: if (s_status_cb) s_status_cb(accum, *accum_len); break;
            case T_EVT:    if (s_evt_cb)    s_evt_cb(accum, *accum_len);    break;
        }
        *accum_len = 0;
        s_last_cfg_frag_ms = 0;   /* === FIX: сброс после завершения === */
    }
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

    if (lora_init() == ESP_OK) {
        lora_set_rx_callback(lora_raw_handler);
        lora_receive();
        s_use_lora = true;

        xTaskCreate(lora_poll_task, "lora_poll", 8192, NULL, 3, NULL);
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