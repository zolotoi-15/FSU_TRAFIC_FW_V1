/* ============================================================
   can_bus.c — TWAI (CAN) драйвер.

   === FIX CAN-STATE-CALLBACK ===
       on_state_change_cb теперь заполняет s_stats.last_state и
       инкрементирует bus_off_count / error_passive_count при
       ПЕРЕХОДЕ состояния, а не каждые 500 мс в polling.
       Раньше error_passive_count рос на каждый опрос в PASSIVE
       (120 за минуту вместо 1 при реальном переходе).
   ============================================================ */

#include "can_bus.h"
#include "board_pins.h"
#include "shift_register.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

static const char *TAG = "can_bus";
static bool s_inited  = false;
static bool s_started = false;
static twai_node_handle_t s_node = NULL;
static QueueHandle_t      s_rx_queue = NULL;
static TaskHandle_t       s_rx_task  = NULL;
static volatile bool      s_rx_run   = false;

static can_rx_cb_t        s_rx_cb = NULL;
static SemaphoreHandle_t  s_cb_mutex = NULL;
static can_bus_stats_t    s_stats = {0};
static SemaphoreHandle_t  s_stats_mutex = NULL;

/* Backoff для bus-off */
static uint32_t s_recovery_backoff_ms = 0;
static uint32_t s_last_recovery_ms    = 0;

/* === FIX CAN-STATE-CALLBACK ===
   Кэш последнего состояния — чтобы инкрементировать счётчики
   только при ПЕРЕХОДЕ. */
static volatile uint32_t s_last_known_state = 0xFFFFFFFF;
/* === END FIX CAN-STATE-CALLBACK === */

typedef struct { uint32_t id; uint8_t dlc; uint8_t extd; uint8_t data[8]; } can_frame_t;
#define CAN_RX_QUEUE_LEN  32
#define CAN_RX_TASK_STACK 4096
#define CAN_RX_TASK_PRIO  6

static bool IRAM_ATTR on_rx_done_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx) {
    can_frame_t frm;
    twai_frame_t rx_frame = { .buffer = frm.data, .buffer_len = sizeof(frm.data) };
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        frm.id   = rx_frame.header.id;
        frm.dlc  = rx_frame.buffer_len;
        frm.extd = rx_frame.header.ide ? 1 : 0;
        if (frm.dlc > 8) frm.dlc = 8;
        BaseType_t hp = pdFALSE;
        xQueueSendFromISR(s_rx_queue, &frm, &hp);
        return (hp == pdTRUE);
    }
    return false;
}
static bool IRAM_ATTR on_error_cb(twai_node_handle_t handle,
                                   const twai_error_event_data_t *edata,
                                   void *user_ctx) { return false; }

/* === FIX CAN-STATE-CALLBACK ===
   Заполняем статистику при переходе состояния в ISR.
   Используем portMUX spinlock (нельзя брать mutex из ISR). */
static portMUX_TYPE s_state_mtx = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR on_state_change_cb(twai_node_handle_t handle,
                                          const twai_state_change_event_data_t *edata,
                                          void *user_ctx) {
    twai_error_state_t new_state = edata->new_sta;

    portENTER_CRITICAL_ISR(&s_state_mtx);
    uint32_t prev = s_last_known_state;
    s_last_known_state = (uint32_t)new_state;
    s_stats.last_state = (uint32_t)new_state;

    if ((uint32_t)new_state != prev) {
        if (new_state == TWAI_ERROR_BUS_OFF) {
            s_stats.bus_off_count++;
        } else if (new_state == TWAI_ERROR_PASSIVE) {
            s_stats.error_passive_count++;
        }
    }
    portEXIT_CRITICAL_ISR(&s_state_mtx);

    if ((uint32_t)new_state != prev) {
        if (new_state == TWAI_ERROR_BUS_OFF) {
            ESP_EARLY_LOGE(TAG, "state → BUS_OFF");
        } else if (new_state == TWAI_ERROR_PASSIVE) {
            ESP_EARLY_LOGW(TAG, "state → PASSIVE");
        }
    }
    return false;
}
/* === END FIX CAN-STATE-CALLBACK === */

esp_err_t can_bus_set_standby(bool standby) {
    sr_set_output(CAN_RS_SR_PIN, standby ? 1 : 0);
    esp_rom_delay_us(50);
    return ESP_OK;
}

esp_err_t can_bus_init(uint32_t bitrate) {
    if (s_inited) return ESP_OK;
    if (bitrate == 0) bitrate = CAN_BITRATE_DEFAULT;
    if (!s_cb_mutex)    s_cb_mutex    = xSemaphoreCreateMutex();
    if (!s_stats_mutex) s_stats_mutex = xSemaphoreCreateMutex();
    if (!s_cb_mutex || !s_stats_mutex) return ESP_ERR_NO_MEM;
    if (!s_rx_queue) {
        s_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_frame_t));
        if (!s_rx_queue) return ESP_ERR_NO_MEM;
    }
    sr_set_output(CAN_RS_SR_PIN, 0);
    twai_onchip_node_config_t cfg = {
        .io_cfg = { .tx = CAN_TX, .rx = CAN_RX, .quanta_clk_out = -1, .bus_off_indicator = -1 },
        .bit_timing = { .bitrate = bitrate },
        .fail_retry_cnt = -1,
        .tx_queue_depth = 16,
    };
    esp_err_t err = twai_new_node_onchip(&cfg, &s_node);
    if (err != ESP_OK) { ESP_LOGE(TAG, "twai_new_node_onchip: %s", esp_err_to_name(err)); return err; }
    twai_event_callbacks_t cbs = {
        .on_rx_done = on_rx_done_cb,
        .on_error   = on_error_cb,
        .on_state_change = on_state_change_cb,
    };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) { twai_node_delete(s_node); s_node = NULL; return err; }
    s_inited = true;
    ESP_LOGI(TAG, "CAN init: TX=%d RX=%d RS=SR%d bitrate=%lu",
             CAN_TX, CAN_RX, CAN_RS_SR_PIN, (unsigned long)bitrate);
    return ESP_OK;
}

esp_err_t can_bus_start(void) {
    if (!s_inited || !s_node) return ESP_ERR_INVALID_STATE;
    if (s_started) return ESP_OK;
    esp_err_t err = twai_node_enable(s_node);
    if (err == ESP_OK) { s_started = true; ESP_LOGI(TAG, "CAN started"); }
    return err;
}
esp_err_t can_bus_stop(void) {
    if (!s_started || !s_node) return ESP_OK;
    esp_err_t err = twai_node_disable(s_node);
    if (err == ESP_OK) { s_started = false; ESP_LOGI(TAG, "CAN stopped"); }
    return err;
}

esp_err_t can_bus_send(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended) {
    if (!s_started || !s_node) return ESP_ERR_INVALID_STATE;
    if (dlc > 8) dlc = 8;
    twai_frame_t tx = {
        .header = { .id = id, .ide = extended ? 1 : 0 },
        .buffer = (uint8_t *)data,
        .buffer_len = dlc,
    };
    esp_err_t err = twai_node_transmit(s_node, &tx, pdMS_TO_TICKS(100));
    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
        if (err == ESP_OK) s_stats.tx_frames++;
        else               s_stats.tx_errors++;
        xSemaphoreGive(s_stats_mutex);
    }
    if (err != ESP_OK) ESP_LOGW(TAG, "tx id=0x%lX failed: %s",
                                (unsigned long)id, esp_err_to_name(err));
    return err;
}

static void can_rx_task(void *arg) {
    ESP_LOGI(TAG, "CAN rx dispatcher started");
    can_frame_t frm;
    while (s_rx_run) {
        if (xQueueReceive(s_rx_queue, &frm, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                s_stats.rx_frames++;
                xSemaphoreGive(s_stats_mutex);
            }
            can_rx_cb_t cb = s_rx_cb;
            if (cb) cb(frm.id, frm.data, frm.dlc, frm.extd != 0);
        }

        /* === FIX CAN-STATE-CALLBACK ===
           last_state / bus_off_count / error_passive_count теперь
           обновляются в on_state_change_cb. Здесь только recovery
           и backoff — по polling-таймауту. */
        if (s_node) {
            twai_node_status_t st;
            if (twai_node_get_info(s_node, &st, NULL) == ESP_OK) {
                if (st.state == TWAI_ERROR_BUS_OFF) {
                    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
                    if (s_recovery_backoff_ms == 0) s_recovery_backoff_ms = 500;
                    if ((now - s_last_recovery_ms) >= s_recovery_backoff_ms) {
                        ESP_LOGE(TAG, "BUS-OFF, recover (backoff=%lu ms)",
                                 (unsigned long)s_recovery_backoff_ms);
                        twai_node_recover(s_node);
                        s_last_recovery_ms = now;
                        if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, 0) == pdTRUE) {
                            s_stats.recovery_count++;
                            xSemaphoreGive(s_stats_mutex);
                        }
                        s_recovery_backoff_ms *= 2;
                        if (s_recovery_backoff_ms > 30000) s_recovery_backoff_ms = 30000;
                    }
                } else {
                    s_recovery_backoff_ms = 0;
                }
            }
        }
        /* === END FIX CAN-STATE-CALLBACK === */
    }
    ESP_LOGI(TAG, "CAN rx dispatcher stopped");
    s_rx_task = NULL;
    vTaskDelete(NULL);
}

void can_bus_set_rx_cb(can_rx_cb_t cb) {
    if (s_cb_mutex && xSemaphoreTake(s_cb_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_rx_cb = cb;
        xSemaphoreGive(s_cb_mutex);
    } else {
        s_rx_cb = cb;
    }
    if (!s_rx_task) {
        s_rx_run = true;
        if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK,
                        NULL, CAN_RX_TASK_PRIO, &s_rx_task) != pdPASS) {
            s_rx_run = false;
            s_rx_task = NULL;
            ESP_LOGE(TAG, "failed to create can_rx task");
        }
    }
}

void can_bus_get_stats(can_bus_stats_t *out) {
    if (!out) return;
    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_stats;
        xSemaphoreGive(s_stats_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}
void can_bus_reset_stats(void) {
    if (s_stats_mutex && xSemaphoreTake(s_stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&s_stats, 0, sizeof(s_stats));
        xSemaphoreGive(s_stats_mutex);
    }
}