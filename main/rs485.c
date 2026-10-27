#include "rs485.h"
#include "board_pins.h"
#include "shift_register.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "rs485";
static bool     s_inited = false;
static uint32_t s_baud   = 9600;

/* ---------- Фоновый приём ---------- */
#define RS485_RX_BUF_SIZE     4096        /* UART driver ring */
#define RS485_FRAME_BUF_SIZE  1024        /* накопитель кадра */
#define RS485_TASK_STACK      4096
#define RS485_TASK_PRIO       6
#define RS485_DEFAULT_IDLE_MS 20          /* Modbus RTU: 3.5 символа ~ 1 мс @9600,
                                             но 20 мс безопаснее */

static TaskHandle_t       s_rx_task       = NULL;
static volatile bool      s_rx_run        = false;
static rs485_byte_cb_t    s_byte_cb       = NULL;
static rs485_frame_cb_t   s_frame_cb      = NULL;
static uint32_t           s_idle_ms       = RS485_DEFAULT_IDLE_MS;
static SemaphoreHandle_t  s_cb_mutex      = NULL;

static rs485_stats_t      s_stats         = {0};
static SemaphoreHandle_t  s_stats_mutex   = NULL;

/* Управление RE через shift register (не трогает остальные выходы) */
void rs485_set_re(bool receive_enable)
{
    sr_set_output(RS485_RE_SR_PIN, receive_enable ? 1 : 0);
    esp_rom_delay_us(50);
}

esp_err_t rs485_init(uint32_t baudrate)
{
    if (s_inited) return ESP_OK;
    if (baudrate == 0) baudrate = 9600;
    s_baud = baudrate;

    if (!s_cb_mutex)    s_cb_mutex    = xSemaphoreCreateMutex();
    if (!s_stats_mutex) s_stats_mutex = xSemaphoreCreateMutex();
    if (!s_cb_mutex || !s_stats_mutex) return ESP_ERR_NO_MEM;

    /* Изначально RE=1 (приём) — безопасное состояние */
    sr_set_output(RS485_RE_SR_PIN, 1);

    uart_config_t cfg = {
        .baud_rate  = baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(RS485_UART_NUM,
                                        RS485_RX_BUF_SIZE, 2048,
                                        0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(RS485_UART_NUM, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(RS485_UART_NUM, RS485_DI, RS485_RO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    uart_flush_input(RS485_UART_NUM);
    s_inited = true;
    ESP_LOGI(TAG, "RS485 init: RO=%d DI=%d RE=SR%d baud=%lu",
             RS485_RO, RS485_DI, RS485_RE_SR_PIN, (unsigned long)baudrate);
    return ESP_OK;
}

esp_err_t rs485_set_baudrate(uint32_t baudrate)
{
    if (baudrate == 0) return ESP_ERR_INVALID_ARG;
    s_baud = baudrate;
    esp_err_t err = uart_set_baudrate(RS485_UART_NUM, baudrate);
    if (err == ESP_OK) ESP_LOGI(TAG, "baud=%lu", (unsigned long)baudrate);
    return err;
}

esp_err_t rs485_send(const uint8_t *data, size_t len)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    /* RE=0 → драйвер включён */
    rs485_set_re(false);

    int written = uart_write_bytes(RS485_UART_NUM, data, len);
    if (written != (int)len) {
        rs485_set_re(true);
        if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
            s_stats.tx_errors++;
            xSemaphoreGive(s_stats_mutex);
        }
        ESP_LOGW(TAG, "write partial %d/%u", written, (unsigned)len);
        return ESP_FAIL;
    }
    uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(1000));

    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
        s_stats.tx_bytes += len;
        xSemaphoreGive(s_stats_mutex);
    }

    /* RE=1 → приём */
    rs485_set_re(true);
    return ESP_OK;
}

int rs485_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (!s_inited || !buf || len == 0) return -1;
    sr_set_output(RS485_RE_SR_PIN, 1);
    int n = uart_read_bytes(RS485_UART_NUM, buf, len,
                            pdMS_TO_TICKS(timeout_ms));
    return n;
}

/* ---------- Фоновая задача приёма ---------- */
static void rs485_rx_task(void *arg)
{
    ESP_LOGI(TAG, "RS485 rx task started (idle=%lu ms)",
             (unsigned long)s_idle_ms);

    uint8_t *frame_buf = malloc(RS485_FRAME_BUF_SIZE);
    if (!frame_buf) {
        ESP_LOGE(TAG, "no mem for frame_buf");
        s_rx_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    size_t   frame_len    = 0;
    TickType_t last_byte  = xTaskGetTickCount();
    uint8_t  chunk[256];

    /* Убеждаемся, что RE=1 (приём) */
    sr_set_output(RS485_RE_SR_PIN, 1);

    while (s_rx_run) {
        int n = uart_read_bytes(RS485_UART_NUM, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(20));
        if (n > 0) {
            last_byte = xTaskGetTickCount();

            if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                s_stats.rx_bytes += n;
                xSemaphoreGive(s_stats_mutex);
            }

            /* byte callback — на каждый байт */
            rs485_byte_cb_t bcb = s_byte_cb;
            if (bcb) {
                for (int i = 0; i < n; i++) bcb(chunk[i]);
            }

            /* frame accumulator */
            if (s_frame_cb) {
                size_t space = RS485_FRAME_BUF_SIZE - frame_len;
                size_t copy  = (n <= (int)space) ? (size_t)n : space;
                memcpy(frame_buf + frame_len, chunk, copy);
                frame_len += copy;
                if (copy < (size_t)n) {
                    /* overflow — форсируем отправку, чтобы не потерять */
                    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                        s_stats.rx_overflows++;
                        xSemaphoreGive(s_stats_mutex);
                    }
                    ESP_LOGW(TAG, "frame overflow, flushing %u bytes",
                             (unsigned)frame_len);
                    rs485_frame_cb_t fcb = s_frame_cb;
                    if (fcb && frame_len) fcb(frame_buf, frame_len);
                    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                        s_stats.rx_frames++;
                        xSemaphoreGive(s_stats_mutex);
                    }
                    frame_len = 0;
                    /* досыпаем остаток */
                    size_t rest = (size_t)n - copy;
                    if (rest > RS485_FRAME_BUF_SIZE) rest = RS485_FRAME_BUF_SIZE;
                    memcpy(frame_buf, chunk + copy, rest);
                    frame_len = rest;
                }
            }
        } else {
            /* idle-таймаут: если накопился кадр и прошло s_idle_ms — отдаём */
            if (s_frame_cb && frame_len > 0) {
                TickType_t now = xTaskGetTickCount();
                uint32_t elapsed_ms = (now - last_byte) * portTICK_PERIOD_MS;
                if (elapsed_ms >= s_idle_ms) {
                    rs485_frame_cb_t fcb = s_frame_cb;
                    if (fcb) fcb(frame_buf, frame_len);
                    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                        s_stats.rx_frames++;
                        xSemaphoreGive(s_stats_mutex);
                    }
                    frame_len = 0;
                }
            }
        }
    }

    free(frame_buf);
    s_rx_task = NULL;
    ESP_LOGI(TAG, "RS485 rx task stopped");
    vTaskDelete(NULL);
}

esp_err_t rs485_rx_task_start(void)
{
    if (s_rx_task) return ESP_OK;
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    s_rx_run = true;
    if (xTaskCreate(rs485_rx_task, "rs485_rx", RS485_TASK_STACK,
                    NULL, RS485_TASK_PRIO, &s_rx_task) != pdPASS) {
        s_rx_run = false;
        s_rx_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void rs485_rx_task_stop(void)
{
    if (!s_rx_task) return;
    s_rx_run = false;
    /* задача сама удалится через ~20 мс */
}

void rs485_set_byte_cb(rs485_byte_cb_t cb)
{
    if (s_cb_mutex && xSemaphoreTake(s_cb_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_byte_cb = cb;
        xSemaphoreGive(s_cb_mutex);
    } else {
        s_byte_cb = cb;
    }
    rs485_rx_task_start();
}

void rs485_set_frame_cb(rs485_frame_cb_t cb, uint32_t idle_timeout_ms)
{
    if (s_cb_mutex && xSemaphoreTake(s_cb_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_frame_cb = cb;
        s_idle_ms  = idle_timeout_ms ? idle_timeout_ms : RS485_DEFAULT_IDLE_MS;
        xSemaphoreGive(s_cb_mutex);
    } else {
        s_frame_cb = cb;
        s_idle_ms  = idle_timeout_ms ? idle_timeout_ms : RS485_DEFAULT_IDLE_MS;
    }
    rs485_rx_task_start();
}

void rs485_get_stats(rs485_stats_t *out)
{
    if (!out) return;
    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_stats;
        xSemaphoreGive(s_stats_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

void rs485_reset_stats(void)
{
    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&s_stats, 0, sizeof(s_stats));
        xSemaphoreGive(s_stats_mutex);
    }
}