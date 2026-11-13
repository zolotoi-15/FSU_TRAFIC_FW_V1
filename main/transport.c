/* ============================================================
   transport.c — LoRa + BLE транспорт.

   === SIMPLIFIED-NO-CFG ===
       T_CFG, retry_worker, cfg:reset/cfg:ok/cfg:req убраны.

   === FIX PKT-48 ===
       PKT_MAX = 48, FRAG_DATA_MAX = 41.

   === FIX STATUS-NOACK-AGAIN ===
       STATUS/EVT/CHANNELS — noack. ACK только T_CMD на последнем
       фрагменте.

   === FIX T_CHANNELS ===
       T_CHANNELS = 6 — снимок 16 каналов NODE (1.4 КБ → 35
       фрагментов). noack.

   === FIX BLE-TRANSPORT ===
       Единая точка transport_handle_raw_rx для LoRa RX и BLE RX.
       BLE-сервер (ble_transport.c) вызывает transport_handle_raw_rx
       при Write в CFG/CMD — те же байты, что и по LoRa. Логика
       фрагментации/ACK/accum — общая.

       transport_send_raw — отправка одного пакета через BLE (если
       подключён) или LoRa. Основной путь — lora_send_fragmented*
       (LoRa), BLE используется для конфигурации (ble_transport.c
       сам шлёт JSON в RESP, не через transport_send_raw).

   === FIX RX-TYPE-RANGE-GUARD ===
       s_last_done_seq[] и s_next_frag_idx[] имеют размер 7 и
       индексируются по type (1..6). Битый пакет с type=0 или
       type>6 приводил к записи за границы массива (UB). Проверка
       типа — ДО любых обращений к массивам.

   === FIX SEND-RAW-SIZE-GUARD ===
       transport_send_raw не фрагментирует — при len > PKT_MAX-4
       уходит только первый кусок (memcpy переполняет buf). Явный
       лог + ESP_ERR_INVALID_SIZE.
   ============================================================ */

#include "transport.h"
#include "lora.h"
#include "config_manager.h"
#include "root_wd.h"
#include "ble_transport.h"
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
#define PKT_MAX           48
#define FRAG_HDR_SIZE     3
#define FRAG_DATA_MAX     (PKT_MAX - 4 - FRAG_HDR_SIZE)   /* 41 */

#define FRAG_GAP_MS       300
#define FRAG_GAP_STATUS_EXTRA_MS  150

#define TX_IDLE_GUARD_MS  50

#define ACK_TIMEOUT       800
#define ACK_TIMEOUT_CMD   2000
#define MAX_RETRIES       3
#define MAX_RETRIES_CMD   2

#define TOTAL_BUDGET_OTHER_MS     30000

#define ACCUM_CMD_MAX     2048
#define ACCUM_STATUS_MAX  4096
#define ACCUM_EVT_MAX     1024
#define ACCUM_CHANNELS_MAX 4096

#define ACK_QUEUE_LEN     64

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5, T_CHANNELS = 6 };

/* ============================================================
   State
   ============================================================ */
static SemaphoreHandle_t s_tx_ack_sem     = NULL;
static SemaphoreHandle_t s_rx_ack_pending = NULL;
static QueueHandle_t     s_ack_queue      = NULL;

static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_use_lora     = false;
static volatile bool     s_inited       = false;
static volatile bool     s_tx_busy      = false;

static volatile uint32_t s_last_rx_ms   = 0;

static transport_rx_cb_t s_cmd_cb    = NULL;
static transport_rx_cb_t s_status_cb = NULL;
static transport_rx_cb_t s_evt_cb    = NULL;
static transport_rx_cb_t s_channels_cb = NULL;
static transport_ack_cb_t s_ack_cb   = NULL;

static char   s_accum_cmd[ACCUM_CMD_MAX];
static char   s_accum_status[ACCUM_STATUS_MAX];
static char   s_accum_evt[ACCUM_EVT_MAX];
static char   s_accum_channels[ACCUM_CHANNELS_MAX];
static volatile size_t s_accum_channels_len = 0;

static volatile size_t s_accum_cmd_len    = 0;
static volatile size_t s_accum_status_len = 0;
static volatile size_t s_accum_evt_len    = 0;

static volatile uint8_t s_next_frag_idx[7] = {0, 0, 0, 0, 0, 0, 0};
static uint8_t s_last_done_seq[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

extern volatile uint32_t s_last_node_msg_ms;

/* ============================================================
   SIMPLIFIED-NO-CFG: stub-функции
   ============================================================ */
void transport_reset_cfg_accum(void) { }
void transport_clear_cfg_expected(void) { }
bool transport_is_cfg_rx_in_progress(void) { return false; }
esp_err_t transport_send_retry_request(uint8_t type, uint8_t start_frag_idx)
{
    (void)type; (void)start_frag_idx;
    return ESP_ERR_NOT_SUPPORTED;
}

static const char *pkt_type_str(uint8_t t)
{
    switch (t) {
        case T_CMD:      return "CMD";
        case T_CFG:      return "CFG";
        case T_STATUS:   return "STATUS";
        case T_EVT:      return "EVT";
        case T_ACK:      return "ACK";
        case T_CHANNELS: return "CHANNELS";
        default:         return "?";
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

    bool important = (strncmp(prefix, "RX EVT", 6) == 0);
    if (important) {
        if (truncated) {
            ESP_LOGI(TAG, "%s len=%u (first %u): %s| %s...",
                     prefix, (unsigned)len, (unsigned)n, hex, asc);
        } else {
            ESP_LOGI(TAG, "%s len=%u: %s| %s",
                     prefix, (unsigned)len, hex, asc);
        }
    } else {
        if (truncated) {
            ESP_LOGD(TAG, "%s len=%u (first %u): %s| %s...",
                     prefix, (unsigned)len, (unsigned)n, hex, asc);
        } else {
            ESP_LOGD(TAG, "%s len=%u: %s| %s",
                     prefix, (unsigned)len, hex, asc);
        }
    }
}

/* ============================================================
   ack_tx_task
   ============================================================ */
static void ack_tx_task(void *arg)
{
    ESP_LOGI(TAG, "ack_tx_task started (waits for TX idle)");
    uint8_t seq;
    while (1) {
        if (xQueueReceive(s_ack_queue, &seq, portMAX_DELAY) == pdTRUE) {
            for (int w = 0; w < 50; w++) {
                if (!lora_is_tx_active()) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            uint8_t ack[4] = { PKT_MAGIC, T_ACK, seq, 0 };
            esp_err_t err = lora_send_packet(ack, 4);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ack_tx: send ACK seq=%u failed: %s",
                         seq, esp_err_to_name(err));
            }
        }
    }
}

/* ============================================================
   lora_poll_task
   ============================================================ */
static void lora_poll_task(void *arg)
{
    ESP_LOGI(TAG, "lora_poll_task started (10 ms period, stack=24K)");
    while (1) {
        lora_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void wait_tx_idle(void)
{
    for (int w = 0; w < 40; w++) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_last_rx_ms == 0 || (now - s_last_rx_ms) >= TX_IDLE_GUARD_MS) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
   lora_send_packet_ack
   ============================================================ */
static esp_err_t lora_send_packet_ack(uint8_t type, uint8_t seq,
                                      const uint8_t *payload, size_t len,
                                      bool need_ack)
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
        snprintf(prefix, sizeof(prefix), "TX %s seq=%u%s",
                 pkt_type_str(type), seq, need_ack ? "" : " (no-ack)");
        if (type == T_EVT) {
            if (len > 0) {
                char asc[TRANSPORT_LOG_PAYLOAD_MAX + 1];
                size_t n = len < TRANSPORT_LOG_PAYLOAD_MAX
                           ? len : TRANSPORT_LOG_PAYLOAD_MAX;
                for (size_t i = 0; i < n; i++)
                    asc[i] = isprint(payload[i]) ? (char)payload[i] : '.';
                asc[n] = '\0';
                ESP_LOGI(TAG, "%s len=%u: %s", prefix, (unsigned)len, asc);
            }
        } else {
            log_hex_ascii(prefix, payload, len);
        }
    }
#endif

    if (!need_ack) {
        wait_tx_idle();
        s_tx_busy = true;
        esp_err_t err = lora_send_packet(buf, (uint8_t)(4 + len));
        s_tx_busy = false;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX %s seq=%u (no-ack) failed: %s",
                     pkt_type_str(type), seq, esp_err_to_name(err));
        }
        return err;
    }

    int max_retries;
    int ack_timeout;
    if (type == T_CMD) {
        max_retries = MAX_RETRIES_CMD;
        ack_timeout = ACK_TIMEOUT_CMD;
    } else {
        max_retries = MAX_RETRIES;
        ack_timeout = ACK_TIMEOUT;
    }

    for (int retry = 0; retry < max_retries; retry++) {
        s_last_ack_seq = seq;
        xSemaphoreTake(s_tx_ack_sem, 0);

        wait_tx_idle();
        s_tx_busy = true;
        if (lora_send_packet(buf, (uint8_t)(4 + len)) != ESP_OK) {
            s_tx_busy = false;
            ESP_LOGW(TAG, "LoRa tx fail type=%u seq=%u retry=%d",
                     type, seq, retry);
            continue;
        }

        if (xSemaphoreTake(s_tx_ack_sem, pdMS_TO_TICKS(ack_timeout)) == pdTRUE) {
            s_tx_busy = false;
            return ESP_OK;
        }
        s_tx_busy = false;
        ESP_LOGW(TAG, "LoRa ACK timeout type=%u seq=%u retry=%d (timeout=%d ms)",
                 type, seq, retry, ack_timeout);
    }
    return ESP_FAIL;
}

/* ============================================================
   lora_send_fragmented
   ============================================================ */
static esp_err_t lora_send_fragmented(uint8_t type, const char *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t total_budget_ms = TOTAL_BUDGET_OTHER_MS;
    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);

    size_t sent = 0;
    int frag_idx = 0;
    int frag_total = (len + FRAG_DATA_MAX - 1) / FRAG_DATA_MAX;

    while (sent < len) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - start_ms) > total_budget_ms) {
            ESP_LOGE(TAG, "TX %s budget exhausted", pkt_type_str(type));
            return ESP_ERR_TIMEOUT;
        }

        size_t chunk = (len - sent > FRAG_DATA_MAX) ? FRAG_DATA_MAX : (len - sent);
        uint8_t flag = (sent + chunk < len) ? 0x01 : 0x00;
        uint8_t buf[PKT_MAX];
        buf[0] = flag;
        buf[1] = (uint8_t)frag_idx;
        buf[2] = (uint8_t)frag_total;
        memcpy(&buf[3], data + sent, chunk);

        size_t payload_len = FRAG_HDR_SIZE + chunk;

        bool need_ack;
        if (type == T_EVT || type == T_STATUS || type == T_CHANNELS) {
            need_ack = false;
        } else {
            need_ack = (flag == 0x00);
        }

        esp_err_t err = lora_send_packet_ack(type, seq, buf, payload_len, need_ack);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "TX %s frag %d/%d FAILED: %s",
                     pkt_type_str(type), frag_idx + 1, frag_total,
                     esp_err_to_name(err));
            return err;
        }

        sent += chunk;
        seq++;
        frag_idx++;
        if (sent < len) {
            uint32_t gap = FRAG_GAP_MS;
            if (type == T_STATUS || type == T_CHANNELS) gap += FRAG_GAP_STATUS_EXTRA_MS;
            vTaskDelay(pdMS_TO_TICKS(gap));
        }
    }
    return ESP_OK;
}

static esp_err_t lora_send_fragmented_noack(uint8_t type,
                                            const char *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t total_budget_ms = TOTAL_BUDGET_OTHER_MS;
    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);

    size_t sent = 0;
    int frag_idx = 0;
    int frag_total = (len + FRAG_DATA_MAX - 1) / FRAG_DATA_MAX;
    /* Для больших пакетов (CHANNELS = 30 фрагментов) нужен больший gap */
    uint32_t base_gap = FRAG_GAP_MS;
    if (frag_total >= 10) base_gap = 300;
    if (frag_total >= 20) base_gap = 400;

    while (sent < len) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - start_ms) > total_budget_ms) {
            ESP_LOGE(TAG, "TX %s no-ack budget exhausted", pkt_type_str(type));
            return ESP_ERR_TIMEOUT;
        }

        size_t chunk = (len - sent > FRAG_DATA_MAX) ? FRAG_DATA_MAX : (len - sent);
        uint8_t flag = (sent + chunk < len) ? 0x01 : 0x00;
        uint8_t buf[PKT_MAX];
        buf[0] = flag;
        buf[1] = (uint8_t)frag_idx;
        buf[2] = (uint8_t)frag_total;
        memcpy(&buf[3], data + sent, chunk);

        size_t payload_len = FRAG_HDR_SIZE + chunk;
        esp_err_t err = lora_send_packet_ack(type, seq, buf, payload_len, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "TX %s no-ack frag %d/%d FAILED: %s",
                     pkt_type_str(type), frag_idx + 1, frag_total,
                     esp_err_to_name(err));
            return err;
        }

        sent += chunk;
        seq++;
        frag_idx++;
        if (sent < len) {
            uint32_t gap = base_gap;
            if (type == T_STATUS) gap += FRAG_GAP_STATUS_EXTRA_MS;
            vTaskDelay(pdMS_TO_TICKS(gap));
        }
    }
    return ESP_OK;
}

/* ============================================================
   lora_raw_handler — RX (LoRa). Делегирует в общий
   transport_handle_raw_rx.
   ============================================================ */
static void lora_raw_handler(const uint8_t *data, size_t len)
{
    transport_handle_raw_rx(data, len);
}

/* ============================================================
   transport_handle_raw_rx — общая обработка (LoRa + BLE)
   === FIX BLE-TRANSPORT ===
   === FIX RX-TYPE-RANGE-GUARD ===
   ============================================================ */
void transport_handle_raw_rx(const uint8_t *data, size_t len)
{
    if (len < 4 || data[0] != PKT_MAGIC) {
#if TRANSPORT_LOG_RX
        ESP_LOGW(TAG, "RX junk (len=%u, magic=0x%02X)",
                 (unsigned)len, len > 0 ? data[0] : 0);
#endif
        return;
    }

    s_last_rx_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t type = data[1];
    uint8_t seq  = data[2];
    uint8_t plen = data[3];

    if (type == T_ACK) {
#if TRANSPORT_LOG_RX
        ESP_LOGI(TAG, "RX ACK seq=%u (last sent %u)", seq, s_last_ack_seq);
#endif
        xSemaphoreGive(s_tx_ack_sem);
        if (s_ack_cb) s_ack_cb(seq, T_ACK);
        return;
    }

    /* === FIX RX-TYPE-RANGE-GUARD ===
       s_last_done_seq[] и s_next_frag_idx[] имеют размер 7 и
       индексируются по type (1..6). Если пришёл битый пакет с
       type=0 или type>6 — раньше мы писали за границы массива
       (UB → corrupt соседних переменных). Проверяем ДО любых
       обращений. */
    if (type < T_CMD || type > T_CHANNELS) {
#if TRANSPORT_LOG_RX
        ESP_LOGW(TAG, "RX unknown type=%u (len=%u) → drop",
                 type, (unsigned)len);
#endif
        return;
    }
    /* === END FIX RX-TYPE-RANGE-GUARD === */

    if (type == T_STATUS || type == T_EVT || type == T_CHANNELS) {
        if (g_config.role == DEVICE_ROLE_MASTER) {
            s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    }

    char  *accum = NULL;
    volatile size_t *accum_len = NULL;
    size_t max_len = 0;

    switch (type) {
        case T_CMD:    accum = s_accum_cmd;    accum_len = &s_accum_cmd_len;    max_len = sizeof(s_accum_cmd);    break;
        case T_STATUS: accum = s_accum_status; accum_len = &s_accum_status_len; max_len = sizeof(s_accum_status); break;
        case T_EVT:    accum = s_accum_evt;    accum_len = &s_accum_evt_len;    max_len = sizeof(s_accum_evt);    break;
        case T_CHANNELS:
            accum = s_accum_channels;
            accum_len = &s_accum_channels_len;
            max_len = sizeof(s_accum_channels);
            break;
        default:
            return;
    }

    if (len < 4 + FRAG_HDR_SIZE || plen < FRAG_HDR_SIZE) return;
    uint8_t flag       = data[4];
    uint8_t frag_idx   = data[5];
    uint8_t frag_total = data[6];
    size_t  chunk      = plen - FRAG_HDR_SIZE;

    if (flag == 0x00 && frag_idx + 1 == frag_total) {
        if (seq == s_last_done_seq[type]) {
            ESP_LOGD(TAG, "RX %s duplicate last (seq=%u) → drop",
                     pkt_type_str(type), seq);
            return;
        }
        s_last_done_seq[type] = seq;
    }

    if ((type == T_CMD || type == T_EVT || type == T_STATUS || type == T_CHANNELS) && flag == 0x00) {
        if (s_ack_queue) {
            if (xQueueSend(s_ack_queue, &seq, 0) != pdTRUE) {
                ESP_LOGW(TAG, "ack_queue FULL (seq=%u), ACK dropped", seq);
            }
        }
    }

    if (chunk > FRAG_DATA_MAX || frag_total == 0 || frag_idx >= frag_total) {
#if TRANSPORT_LOG_RX
        ESP_LOGW(TAG, "RX %s invalid frag hdr: idx=%u total=%u chunk=%u",
                 pkt_type_str(type), frag_idx, frag_total, (unsigned)chunk);
#endif
        return;
    }

#if TRANSPORT_LOG_RX
    {
        char prefix[48];
        snprintf(prefix, sizeof(prefix), "RX %s seq=%u frag %u/%u",
                 pkt_type_str(type), seq, frag_idx + 1, frag_total);
        log_hex_ascii(prefix, &data[4 + FRAG_HDR_SIZE], chunk);
    }
#endif

    if (frag_idx == 0) {
        *accum_len = 0;
        s_next_frag_idx[type] = 1;
    } else {
        if (frag_idx != s_next_frag_idx[type]) {
            static uint32_t s_last_gap_warn_ms = 0;
            uint32_t now_gap = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (s_last_gap_warn_ms == 0 ||
                (now_gap - s_last_gap_warn_ms) >= 5000) {
                s_last_gap_warn_ms = now_gap;
                ESP_LOGW(TAG, "RX %s frag gap: expected idx=%u, got %u "
                              "(total=%u) → drop",
                         pkt_type_str(type), s_next_frag_idx[type],
                         frag_idx, frag_total);
            }
            *accum_len = 0;
            s_next_frag_idx[type] = 0;
            return;
        }
        s_next_frag_idx[type] = frag_idx + 1;
    }

    if (*accum_len + chunk < max_len) {
        memcpy(&accum[*accum_len], &data[4 + FRAG_HDR_SIZE], chunk);
        *accum_len += chunk;
    } else {
        ESP_LOGW(TAG, "RX %s overflow, reset", pkt_type_str(type));
        *accum_len = 0;
        s_next_frag_idx[type] = 0;
        return;
    }

    if (flag == 0x00) {
        if (frag_idx + 1 != frag_total) {
            ESP_LOGW(TAG, "RX %s incomplete: got %u/%u frags → drop",
                     pkt_type_str(type), frag_idx + 1, frag_total);
            *accum_len = 0;
            s_next_frag_idx[type] = 0;
            return;
        }

        accum[*accum_len] = '\0';
        s_next_frag_idx[type] = 0;

#if TRANSPORT_LOG_RX
        if (type == T_EVT || type == T_CHANNELS) {
            ESP_LOGI(TAG, "RX %s assembled: %u bytes (frags %u/%u)",
                     pkt_type_str(type), (unsigned)*accum_len,
                     frag_idx + 1, frag_total);
        } else {
            ESP_LOGD(TAG, "RX %s assembled: %u bytes (frags %u/%u)",
                     pkt_type_str(type), (unsigned)*accum_len,
                     frag_idx + 1, frag_total);
        }
#endif

        switch (type) {
            case T_CMD:
                if (s_cmd_cb) s_cmd_cb(accum, *accum_len);
                break;
            case T_STATUS:
                if (s_status_cb) s_status_cb(accum, *accum_len);
                break;
            case T_EVT:
                if (s_evt_cb) s_evt_cb(accum, *accum_len);
                break;
            case T_CHANNELS:
                if (s_channels_cb) s_channels_cb(accum, *accum_len);
                break;
            default: break;
        }
        *accum_len = 0;
    }
}

/* === FIX BLE-TRANSPORT ===
   transport_send_raw — отправка одного пакета через BLE (если
   подключён) или LoRa. Основной путь — lora_send_fragmented*,
   эта функция для совместимости. */
esp_err_t transport_send_raw(uint8_t type, const uint8_t *payload, size_t len)
{
    if (!payload || len == 0) return ESP_ERR_INVALID_ARG;
    if (type < T_CMD || type > T_CHANNELS) return ESP_ERR_INVALID_ARG;

    /* === FIX SEND-RAW-SIZE-GUARD ===
       transport_send_raw не делает фрагментации — если len >
       PKT_MAX-4, уйдёт только первый кусок (memcpy переполнит
       buf). Явный лог + ошибка, чтобы было видно, что пакет
       слишком большой. */
    if (len > PKT_MAX - 4) {
        ESP_LOGW(TAG, "transport_send_raw: len=%u > PKT_MAX-4=%u "
                      "(no fragmentation), drop",
                 (unsigned)len, (unsigned)(PKT_MAX - 4));
        return ESP_ERR_INVALID_SIZE;
    }
    /* === END FIX SEND-RAW-SIZE-GUARD === */

    uint8_t buf[PKT_MAX];
    buf[0] = PKT_MAGIC;
    buf[1] = type;
    buf[2] = (uint8_t)(esp_timer_get_time() & 0xFF);
    buf[3] = (uint8_t)len;
    memcpy(&buf[4], payload, len);

    if (transport_is_ble_active()) {
        /* === FIX BLE-NAME-CLASH === */
        if (fsu_ble_send_raw(buf, 4 + len)) return ESP_OK;
        /* === END FIX BLE-NAME-CLASH === */
    }

    return lora_send_packet(buf, (uint8_t)(4 + len));
}

bool transport_is_ble_active(void)
{
    /* === FIX BLE-NAME-CLASH === */
    return fsu_ble_is_connected();
    /* === END FIX BLE-NAME-CLASH === */
}
/* === END FIX BLE-TRANSPORT === */

/* ============================================================
   transport_init
   ============================================================ */
esp_err_t transport_init(void)
{
    if (s_inited) return ESP_OK;
    s_inited = true;

    s_tx_ack_sem = xSemaphoreCreateBinary();
    if (!s_tx_ack_sem) {
        ESP_LOGE(TAG, "tx_ack_sem alloc failed");
        return ESP_ERR_NO_MEM;
    }
    s_rx_ack_pending = xSemaphoreCreateBinary();
    if (!s_rx_ack_pending) {
        ESP_LOGE(TAG, "rx_ack_pending alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_ack_queue = xQueueCreate(ACK_QUEUE_LEN, sizeof(uint8_t));
    if (!s_ack_queue) {
        ESP_LOGE(TAG, "ack_queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (lora_init() == ESP_OK) {
        lora_set_rx_callback(lora_raw_handler);
        lora_receive();
        s_use_lora = true;
        xTaskCreate(lora_poll_task, "lora_poll", 24576, NULL, 3, NULL);
        xTaskCreate(ack_tx_task,    "ack_tx",    4096,  NULL, 5, NULL);
        ESP_LOGI(TAG, "Transport = LoRa + BLE, PKT_MAX=%d "
                 "(frag data=%d, hdr=%d, gap=%d ms, "
                 "STATUS/CHANNELS-NOACK, T_CMD-ACK-ON-LAST)",
                 PKT_MAX, FRAG_DATA_MAX, FRAG_HDR_SIZE, FRAG_GAP_MS);
        return ESP_OK;
    }

    s_use_lora = false;
    ESP_LOGE(TAG, "LoRa init failed -> NO transport");
    return ESP_FAIL;
}

bool transport_is_lora(void) { return s_use_lora; }
bool transport_is_tcp(void)  { return false; }

void transport_force_lora(void) { }
void transport_force_tcp(void)  { }

/* ============================================================
   send-функции
   ============================================================ */
esp_err_t transport_send_cmd(const char *cmd, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented(T_CMD, cmd, len);
}

esp_err_t transport_send_cmd_noack(const char *cmd, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented_noack(T_CMD, cmd, len);
}

esp_err_t transport_send_cfg(const char *json, size_t len)
{
    (void)json; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t transport_send_status(const char *json, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented_noack(T_STATUS, json, len);
}

esp_err_t transport_send_status_noack(const char *json, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented_noack(T_STATUS, json, len);
}

esp_err_t transport_send_evt(const char *evt, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented(T_EVT, evt, len);
}

esp_err_t transport_send_evt_noack(const char *evt, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented_noack(T_EVT, evt, len);
}

esp_err_t transport_send_channels(const char *json, size_t len)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    return lora_send_fragmented_noack(T_CHANNELS, json, len);
}

/* ============================================================
   Регистрация callbacks
   ============================================================ */
void transport_set_cmd_cb(transport_rx_cb_t cb)    { s_cmd_cb = cb; }
void transport_set_cfg_cb(transport_rx_cb_t cb)    { (void)cb; }
void transport_set_status_cb(transport_rx_cb_t cb) { s_status_cb = cb; }
void transport_set_evt_cb(transport_rx_cb_t cb)    { s_evt_cb = cb; }
void transport_set_ack_cb(transport_ack_cb_t cb)   { s_ack_cb = cb; }
void transport_set_channels_cb(transport_rx_cb_t cb) { s_channels_cb = cb; }

bool transport_is_tx_idle(void)
{
    if (s_tx_busy) return false;
    if (!s_ack_queue) return true;
    return (uxQueueMessagesWaiting(s_ack_queue) == 0);
}