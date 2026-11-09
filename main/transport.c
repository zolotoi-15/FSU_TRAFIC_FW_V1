/* ============================================================
   transport.c — LoRa-транспорт.

   === FIX FRAG-DEDUP ===
       Убрана проверка s_last_rx_seq[type] для всех фрагментов.

   === FIX RETRY-ONLY-CFG-CMD ===
       retry имеет смысл только для CFG и CMD.

   === FIX RETRY-NOACK ===
       retry и cfg:req отправляются no-ack.

   === FIX RETRY-CMD-BUF ===
       ROOT сохраняет последний CMD-буфер (sync:, traffic:start).

   === FIX FRAG-GAP-100 ===
       FRAG_GAP_MS 60 → 100. При gap 60 мс NODE терял фрагменты
       STATUS (RX STATUS frag gap: ... → drop).

   === FIX ACK-ANY-SEQ ===
       ACK принимается независимо от s_last_ack_seq. Раньше при
       несовпадении seq ACK игнорировался, из-за чего T_STATUS/T_CMD
       ретраили до таймаута.
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
#define PKT_MAX           32
#define FRAG_HDR_SIZE     3
#define FRAG_DATA_MAX     (PKT_MAX - 4 - FRAG_HDR_SIZE)

/* === FIX FRAG-GAP-100 === */
#define FRAG_GAP_MS       100
/* === END FIX FRAG-GAP-100 === */

#define ACK_TIMEOUT       800
#define ACK_TIMEOUT_CFG   1500
#define ACK_TIMEOUT_CMD   2000
#define MAX_RETRIES       3
#define MAX_RETRIES_CFG   3
#define MAX_RETRIES_CMD   2

#define CFG_NEW_BURST_MS          4000
#define CFG_EXPECTED_TIMEOUT_MS   15000
#define TOTAL_BUDGET_CFG_MS       90000
#define TOTAL_BUDGET_OTHER_MS     30000

#define ACCUM_CFG_MAX     8192
#define ACCUM_CMD_MAX     2048
#define ACCUM_STATUS_MAX  4096
#define ACCUM_EVT_MAX     1024

#define ACK_QUEUE_LEN     64
#define CFG_QUEUE_LEN     2

#define TX_IDLE_GUARD_MS  200

/* === FIX RETRY-CMD-BUF: буферы для досыла === */
static char  *s_last_cfg_buf     = NULL;
static size_t s_last_cfg_buf_len = 0;
static uint8_t s_last_cfg_seq_base = 0;

static char  *s_last_cmd_buf     = NULL;
static size_t s_last_cmd_buf_len = 0;
static uint8_t s_last_cmd_seq_base = 0;

static SemaphoreHandle_t s_buf_mtx = NULL;
/* === END FIX RETRY-CMD-BUF === */

static volatile uint32_t s_last_rx_ms = 0;

typedef struct {
    char   *data;
    size_t  len;
} cfg_msg_t;

enum { T_CMD = 1, T_CFG = 2, T_STATUS = 3, T_EVT = 4, T_ACK = 5 };

/* === FIX ACK-SEM-SPLIT === */
static SemaphoreHandle_t s_tx_ack_sem = NULL;
static SemaphoreHandle_t s_rx_ack_pending = NULL;
/* === END FIX ACK-SEM-SPLIT === */

static volatile uint8_t  s_last_ack_seq = 0xFF;
static volatile bool     s_use_lora = false;
static volatile bool     s_inited   = false;
static volatile bool     s_tx_busy  = false;
static volatile bool     s_cfg_expected    = false;
static volatile uint32_t s_cfg_expected_ms = 0;

static QueueHandle_t     s_ack_queue = NULL;
static QueueHandle_t     s_cfg_queue = NULL;

static transport_rx_cb_t s_cmd_cb    = NULL;
static transport_rx_cb_t s_cfg_cb    = NULL;
static transport_rx_cb_t s_status_cb = NULL;
static transport_rx_cb_t s_evt_cb    = NULL;
static transport_ack_cb_t s_ack_cb   = NULL;

static char   s_accum_cmd[ACCUM_CMD_MAX];
static char   s_accum_cfg[ACCUM_CFG_MAX];
static char   s_accum_status[ACCUM_STATUS_MAX];
static char   s_accum_evt[ACCUM_EVT_MAX];

static volatile size_t s_accum_cmd_len    = 0;
static volatile size_t s_accum_cfg_len    = 0;
static volatile size_t s_accum_status_len = 0;
static volatile size_t s_accum_evt_len    = 0;

static volatile uint8_t s_next_frag_idx[6] = {0, 0, 0, 0, 0, 0};

/* === FIX FRAG-DEDUP: только для последнего фрагмента === */
static uint8_t s_last_done_seq[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/* === END FIX FRAG-DEDUP === */

static uint32_t s_last_cfg_frag_ms = 0;

extern volatile uint16_t s_fault_yellow_mask_from_root;
extern volatile bool     s_fault_mask_valid;

void transport_reset_cfg_accum(void)
{
    s_accum_cfg_len = 0;
    s_last_cfg_frag_ms = 0;
    s_next_frag_idx[T_CFG] = 0;
    ESP_LOGI(TAG, "transport_reset_cfg_accum: len=0");
}

/* === FIX CFG-EXPECTED-CLEAR === */
void transport_clear_cfg_expected(void)
{
    s_cfg_expected = false;
    ESP_LOGI(TAG, "cfg_expected=false (CFG received)");
}
/* === END FIX CFG-EXPECTED-CLEAR === */

bool transport_is_cfg_rx_in_progress(void)
{
    if (s_accum_cfg_len > 0) return true;
    if (s_cfg_expected) {
        uint32_t elapsed = (xTaskGetTickCount() - s_cfg_expected_ms)
                           * portTICK_PERIOD_MS;
        if (elapsed > CFG_EXPECTED_TIMEOUT_MS) {
            s_cfg_expected = false;
            return false;
        }
        return true;
    }
    return false;
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

/* === FIX TX-BUSY: ack_tx_task === */
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
/* === END FIX TX-BUSY === */

static void cfg_handler_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_handler_task started (stack=24K)");
    cfg_msg_t msg;
    while (1) {
        if (xQueueReceive(s_cfg_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.data && s_cfg_cb) {
                ESP_LOGI(TAG, "cfg_handler: parse %u bytes", (unsigned)msg.len);
                s_cfg_cb(msg.data, msg.len);
            }
            if (msg.data) free(msg.data);
        }
    }
}

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
    ESP_LOGD(TAG, "wait_tx_idle: guard timeout (RX still active)");
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
        log_hex_ascii(prefix, payload, len);
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
    if (type == T_CFG) {
        max_retries = MAX_RETRIES_CFG;
        ack_timeout = ACK_TIMEOUT_CFG;
    } else if (type == T_CMD) {
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
   === FIX RETRY-CMD-BUF: сохраняем CFG и CMD для досыла ===
   ============================================================ */
static esp_err_t lora_send_fragmented(uint8_t type, const char *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    if ((type == T_CFG || type == T_CMD) && s_buf_mtx) {
        if (xSemaphoreTake(s_buf_mtx, pdMS_TO_TICKS(200)) == pdTRUE) {
            char  **pbuf = (type == T_CFG) ? &s_last_cfg_buf : &s_last_cmd_buf;
            size_t *plen = (type == T_CFG) ? &s_last_cfg_buf_len
                                           : &s_last_cmd_buf_len;
            uint8_t *pseq = (type == T_CFG) ? &s_last_cfg_seq_base
                                            : &s_last_cmd_seq_base;
            if (*pbuf) { free(*pbuf); *pbuf = NULL; }
            *pbuf = malloc(len);
            if (*pbuf) {
                memcpy(*pbuf, data, len);
                *plen = len;
                *pseq = (uint8_t)(esp_timer_get_time() & 0xFF);
            } else {
                *plen = 0;
            }
            xSemaphoreGive(s_buf_mtx);
        }
    }

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t total_budget_ms = (type == T_CFG) ? TOTAL_BUDGET_CFG_MS
                                               : TOTAL_BUDGET_OTHER_MS;
    uint8_t seq = (uint8_t)(esp_timer_get_time() & 0xFF);
    if (type == T_CFG && s_last_cfg_seq_base) seq = s_last_cfg_seq_base;
    if (type == T_CMD && s_last_cmd_seq_base) seq = s_last_cmd_seq_base;

    size_t sent = 0;
    int frag_idx = 0;
    int frag_total = (len + FRAG_DATA_MAX - 1) / FRAG_DATA_MAX;

#if TRANSPORT_LOG_TX
    ESP_LOGI(TAG, "TX %s (fragmented): total %u bytes, %d fragments "
             "(frag_data=%d, gap=%d ms)",
             pkt_type_str(type), (unsigned)len, frag_total,
             FRAG_DATA_MAX, FRAG_GAP_MS);
#endif

    while (sent < len) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - start_ms) > total_budget_ms) {
            ESP_LOGE(TAG, "TX %s budget exhausted (%lu ms), abort at frag %d/%d",
                     pkt_type_str(type), (unsigned long)(now_ms - start_ms),
                     frag_idx, frag_total);
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

#if TRANSPORT_LOG_TX
        ESP_LOGI(TAG, "TX %s frag %d/%d seq=%u len=%u",
                 pkt_type_str(type), frag_idx + 1, frag_total,
                 seq, (unsigned)payload_len);
#endif

        esp_err_t err = lora_send_packet_ack(type, seq, buf, payload_len,
                                             flag == 0x00);
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
            vTaskDelay(pdMS_TO_TICKS(FRAG_GAP_MS));
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

    ESP_LOGI(TAG, "TX %s (no-ack, fragmented): total %u bytes, %d fragments",
             pkt_type_str(type), (unsigned)len, frag_total);

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
        if (sent < len) vTaskDelay(pdMS_TO_TICKS(FRAG_GAP_MS));
    }
    return ESP_OK;
}

static void parse_fault_yellow_mask(const char *buf, size_t len)
{
    if (!buf || len < 26) return;
    if (strncmp(buf, "fault:yellow_blink:", 19) != 0) return;
    const char *p = buf + 19;
    if (p + 7 > buf + len) return;
    if (p[0] != '0' || (p[1] != 'x' && p[1] != 'X')) return;
    if (p[6] != ':') return;
    char hex[5];
    memcpy(hex, p + 2, 4);
    hex[4] = '\0';
    char *end = NULL;
    unsigned long v = strtoul(hex, &end, 16);
    if (end != hex + 4) return;
    s_fault_yellow_mask_from_root = (uint16_t)v;
    s_fault_mask_valid            = true;
    ESP_LOGW(TAG, "parsed fault_yellow_mask=0x%04X from ROOT",
             (unsigned)s_fault_yellow_mask_from_root);
}

/* ============================================================
   retry_worker_task — досыл CFG/CMD с idx.
   ============================================================ */
typedef struct {
    uint8_t type;
    uint8_t start_idx;
} retry_req_t;

static void retry_worker_task(void *arg)
{
    retry_req_t req = *(retry_req_t *)arg;
    free(arg);

    if (req.type != T_CFG && req.type != T_CMD) {
        ESP_LOGW(TAG, "retry_worker: type=%u not supported", req.type);
        vTaskDelete(NULL);
        return;
    }

    char  *buf = NULL;
    size_t len = 0;
    uint8_t seq_base = 0;

    if (s_buf_mtx &&
        xSemaphoreTake(s_buf_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
        char  **pbuf = (req.type == T_CFG) ? &s_last_cfg_buf : &s_last_cmd_buf;
        size_t *plen = (req.type == T_CFG) ? &s_last_cfg_buf_len
                                           : &s_last_cmd_buf_len;
        uint8_t *pseq = (req.type == T_CFG) ? &s_last_cfg_seq_base
                                            : &s_last_cmd_seq_base;
        if (*pbuf && *plen > 0) {
            len = *plen;
            buf = malloc(len);
            if (buf) {
                memcpy(buf, *pbuf, len);
                seq_base = *pseq;
            }
        }
        xSemaphoreGive(s_buf_mtx);
    }

    if (!buf || len == 0) {
        ESP_LOGW(TAG, "retry_worker: no %s buffer to resend",
                 pkt_type_str(req.type));
        vTaskDelete(NULL);
        return;
    }

    int frag_total = (len + FRAG_DATA_MAX - 1) / FRAG_DATA_MAX;
    if (req.start_idx >= frag_total) {
        ESP_LOGW(TAG, "retry_worker: start_idx=%u >= total=%d",
                 req.start_idx, frag_total);
        free(buf);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "retry_worker: resending %s frags %u..%d (seq_base=%u)",
             pkt_type_str(req.type), req.start_idx, frag_total - 1, seq_base);

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    size_t off = (size_t)req.start_idx * FRAG_DATA_MAX;
    uint8_t seq = seq_base + (uint8_t)req.start_idx;
    int frag_idx = req.start_idx;

    while (off < len) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t budget = (req.type == T_CFG) ? TOTAL_BUDGET_CFG_MS
                                              : TOTAL_BUDGET_OTHER_MS;
        if ((now_ms - start_ms) > budget) {
            ESP_LOGE(TAG, "retry_worker: budget exhausted at frag %d/%d",
                     frag_idx, frag_total);
            break;
        }

        size_t chunk = (len - off > FRAG_DATA_MAX) ? FRAG_DATA_MAX : (len - off);
        uint8_t flag = (off + chunk < len) ? 0x01 : 0x00;
        uint8_t pbuf[PKT_MAX];
        pbuf[0] = flag;
        pbuf[1] = (uint8_t)frag_idx;
        pbuf[2] = (uint8_t)frag_total;
        memcpy(&pbuf[3], buf + off, chunk);
        size_t payload_len = FRAG_HDR_SIZE + chunk;

        ESP_LOGI(TAG, "retry_worker: TX %s frag %d/%d seq=%u",
                 pkt_type_str(req.type), frag_idx + 1, frag_total, seq);

        esp_err_t err = lora_send_packet_ack(req.type, seq, pbuf, payload_len,
                                             flag == 0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "retry_worker: frag %d/%d FAILED: %s",
                     frag_idx + 1, frag_total, esp_err_to_name(err));
            break;
        }

        off += chunk;
        seq++;
        frag_idx++;
        if (off < len) vTaskDelay(pdMS_TO_TICKS(FRAG_GAP_MS));
    }

    free(buf);
    vTaskDelete(NULL);
}

esp_err_t transport_send_retry_request(uint8_t type, uint8_t start_frag_idx)
{
    if (!s_use_lora) return ESP_ERR_INVALID_STATE;
    if (!s_buf_mtx) return ESP_ERR_INVALID_STATE;
    if (type != T_CFG && type != T_CMD) return ESP_ERR_INVALID_ARG;

    retry_req_t *req = malloc(sizeof(*req));
    if (!req) return ESP_ERR_NO_MEM;
    req->type = type;
    req->start_idx = start_frag_idx;

    BaseType_t rc = xTaskCreate(retry_worker_task, "retry_worker",
                                8192, req, 4, NULL);
    if (rc != pdPASS) {
        free(req);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ============================================================
   lora_raw_handler

   === FIX FRAG-DEDUP ===
   === FIX RETRY-ONLY-CFG-CMD ===
   === FIX RETRY-NOACK ===
   === FIX ACK-ANY-SEQ ===
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

    s_last_rx_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t type = data[1];
    uint8_t seq  = data[2];
    uint8_t plen = data[3];

    if (type == T_ACK) {
#if TRANSPORT_LOG_RX
        ESP_LOGI(TAG, "RX ACK seq=%u (last sent %u)", seq, s_last_ack_seq);
#endif
        /* === FIX ACK-ANY-SEQ ===
           Принимаем ACK по любому seq. Раньше при несовпадении seq
           (ACK на no-ack ping с seq=15, а s_last_ack_seq=154 от STATUS)
           ACK игнорировался, и T_STATUS/T_CMD ретраили до таймаута. */
        xSemaphoreGive(s_tx_ack_sem);
        /* === END FIX ACK-ANY-SEQ === */
        if (s_ack_cb) s_ack_cb(seq, T_ACK);
        return;
    }

    char  *accum = NULL;
    volatile size_t *accum_len = NULL;
    size_t max_len = 0;

    switch (type) {
        case T_CMD:    accum = s_accum_cmd;    accum_len = &s_accum_cmd_len;    max_len = sizeof(s_accum_cmd);    break;
        case T_CFG:    accum = s_accum_cfg;    accum_len = &s_accum_cfg_len;    max_len = sizeof(s_accum_cfg);    break;
        case T_STATUS: accum = s_accum_status; accum_len = &s_accum_status_len; max_len = sizeof(s_accum_status); break;
        case T_EVT:    accum = s_accum_evt;    accum_len = &s_accum_evt_len;    max_len = sizeof(s_accum_evt);    break;
        default: return;
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

    if ((type == T_CMD || type == T_CFG || type == T_EVT || type == T_STATUS)
        && flag == 0x00) {
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
            ESP_LOGW(TAG, "RX %s frag gap: expected idx=%u, got %u (total=%u) → drop",
                     pkt_type_str(type), s_next_frag_idx[type],
                     frag_idx, frag_total);

            *accum_len = 0;
            s_next_frag_idx[type] = 0;

            if (type == T_CFG || type == T_CMD) {
                char retry_cmd[32];
                snprintf(retry_cmd, sizeof(retry_cmd), "retry:%u:%u",
                         type, s_next_frag_idx[type]);
                transport_send_evt_noack(retry_cmd, strlen(retry_cmd));
            }
            return;
        }
        s_next_frag_idx[type] = frag_idx + 1;
    }

    if (type == T_CFG && frag_idx == 0) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_last_cfg_frag_ms != 0 &&
            (now - s_last_cfg_frag_ms) > CFG_NEW_BURST_MS) {
            ESP_LOGW(TAG, "RX CFG new burst (%u ms gap)",
                     (unsigned)(now - s_last_cfg_frag_ms));
        }
        s_last_cfg_frag_ms = now;
    }

    if (type == T_CFG && *accum_len > 4096) {
        ESP_LOGW(TAG, "RX CFG accum overflow (%u), reset",
                 (unsigned)*accum_len);
        *accum_len = 0;
        s_next_frag_idx[type] = 0;
        return;
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
        ESP_LOGI(TAG, "RX %s assembled: %u bytes (frags %u/%u)",
                 pkt_type_str(type), (unsigned)*accum_len,
                 frag_idx + 1, frag_total);
#endif

        switch (type) {
            case T_CMD:
                if (*accum_len == 9 && memcmp(accum, "cfg:reset", 9) == 0) {
                    s_cfg_expected    = true;
                    s_cfg_expected_ms = xTaskGetTickCount();
                    ESP_LOGI(TAG, "cfg:reset detected → cfg_expected=true");
                }
                if (*accum_len >= 26 &&
                    strncmp(accum, "fault:yellow_blink:", 19) == 0) {
                    parse_fault_yellow_mask(accum, *accum_len);
                }

                if (strncmp(accum, "retry:", 6) == 0) {
                    int r_type = 0, r_idx = 0;
                    if (sscanf(accum, "retry:%d:%d", &r_type, &r_idx) == 2 &&
                        r_idx >= 0 && r_idx < 255 &&
                        (r_type == T_CFG || r_type == T_CMD)) {
                        ESP_LOGW(TAG, "RX retry request: type=%d, "
                                      "start_idx=%d → resend",
                                 r_type, r_idx);
                        transport_send_retry_request((uint8_t)r_type,
                                                     (uint8_t)r_idx);
                    } else {
                        ESP_LOGD(TAG, "RX retry ignored: type=%d idx=%d",
                                 r_type, r_idx);
                    }
                    break;
                }

                if (s_cmd_cb) s_cmd_cb(accum, *accum_len);
                break;
            case T_CFG:
                if (s_cfg_queue) {
                    cfg_msg_t msg;
                    msg.data = malloc(*accum_len + 1);
                    if (msg.data) {
                        memcpy(msg.data, accum, *accum_len);
                        msg.data[*accum_len] = '\0';
                        msg.len = *accum_len;
                        if (xQueueSend(s_cfg_queue, &msg, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "cfg_queue FULL, drop CFG (%u bytes)",
                                     (unsigned)msg.len);
                            free(msg.data);
                        }
                    } else {
                        ESP_LOGE(TAG, "cfg_queue: malloc failed, drop CFG");
                    }
                } else if (s_cfg_cb) {
                    s_cfg_cb(accum, *accum_len);
                }
                break;
            case T_STATUS:
                if (s_status_cb) s_status_cb(accum, *accum_len);
                break;
            case T_EVT:
                if (s_evt_cb) s_evt_cb(accum, *accum_len);
                break;
        }
        *accum_len = 0;
    }
}

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

    s_buf_mtx = xSemaphoreCreateMutex();
    if (!s_buf_mtx) {
        ESP_LOGE(TAG, "buf_mtx alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_ack_queue = xQueueCreate(ACK_QUEUE_LEN, sizeof(uint8_t));
    if (!s_ack_queue) {
        ESP_LOGE(TAG, "ack_queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_cfg_queue = xQueueCreate(CFG_QUEUE_LEN, sizeof(cfg_msg_t));
    if (!s_cfg_queue) {
        ESP_LOGE(TAG, "cfg_queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (lora_init() == ESP_OK) {
        lora_set_rx_callback(lora_raw_handler);
        lora_receive();
        s_use_lora = true;
        xTaskCreate(lora_poll_task, "lora_poll", 24576, NULL, 3, NULL);
        xTaskCreate(ack_tx_task,    "ack_tx",    4096,  NULL, 5, NULL);
        xTaskCreate(cfg_handler_task, "cfg_handler", 24576, NULL, 4, NULL);
        ESP_LOGI(TAG, "Transport = LoRa only, PKT_MAX=%d "
                 "(frag data=%d, hdr=%d, gap=%d ms) — "
                 "FIX FRAG-GAP-100 / ACK-ANY-SEQ / RETRY-CMD-BUF",
                 PKT_MAX, FRAG_DATA_MAX, FRAG_HDR_SIZE, FRAG_GAP_MS);
        return ESP_OK;
    }

    s_use_lora = false;
    ESP_LOGE(TAG, "LoRa init failed -> NO transport (TCP disabled)");
    return ESP_FAIL;
}

bool transport_is_lora(void) { return s_use_lora; }
bool transport_is_tcp(void)  { return false; }

void transport_force_lora(void)
{
    if (s_inited && !s_use_lora) {
        ESP_LOGW(TAG, "force -> LoRa (not implemented, LoRa always primary)");
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

esp_err_t transport_send_cmd_noack(const char *cmd, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_cmd_noack: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented_noack(T_CMD, cmd, len);
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
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_status: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_STATUS, json, len);
}

esp_err_t transport_send_status_noack(const char *json, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_status_noack: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented_noack(T_STATUS, json, len);
}

esp_err_t transport_send_evt(const char *evt, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_evt: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented(T_EVT, evt, len);
}

esp_err_t transport_send_evt_noack(const char *evt, size_t len)
{
    if (!s_use_lora) {
        ESP_LOGW(TAG, "transport_send_evt_noack: LoRa not ready");
        return ESP_ERR_INVALID_STATE;
    }
    return lora_send_fragmented_noack(T_EVT, evt, len);
}

void transport_set_cmd_cb(transport_rx_cb_t cb)    { s_cmd_cb = cb; }
void transport_set_cfg_cb(transport_rx_cb_t cb)    { s_cfg_cb = cb; }
void transport_set_status_cb(transport_rx_cb_t cb) { s_status_cb = cb; }
void transport_set_evt_cb(transport_rx_cb_t cb)    { s_evt_cb = cb; }
void transport_set_ack_cb(transport_ack_cb_t cb)   { s_ack_cb = cb; }

bool transport_is_tx_idle(void)
{
    if (s_tx_busy) return false;
    if (!s_ack_queue) return true;
    return (uxQueueMessagesWaiting(s_ack_queue) == 0);
}