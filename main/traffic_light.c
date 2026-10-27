/* ============================================================
   traffic_light.c — публичный API и главный цикл traffic_task.
   ============================================================ */

#include "traffic_internal.h"
#include "traffic_light.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "traffic";

extern esp_err_t board_diag_en_set(uint8_t channel, bool enable);
extern void      board_diag_en_all_off(void);
extern int       board_read_iload_raw(uint8_t channel,
                                      imon_state_t *out_state,
                                      int *out_mv);

/* ============================================================
   Определения общих переменных (extern в traffic_internal.h).
   ============================================================ */
volatile traffic_state_t s_state = TRAFFIC_STATE_STOPPED;
volatile traffic_mode_t  s_mode  = TRAFFIC_MODE_CROSS_PED;
volatile uint8_t         s_phase = 0;
volatile uint32_t        s_phase_start_ms = 0;
volatile uint32_t        s_last_recovery_ms = 0;
volatile uint32_t        s_recovery_attempts = 0;
volatile bool            s_follow_root = false;

volatile bool            s_ped_request      = false;
volatile uint32_t        s_ped_request_ms   = 0;
volatile bool            s_ped_min_recall   = false;

TaskHandle_t             s_task = NULL;
SemaphoreHandle_t        s_mutex = NULL;

volatile traffic_fault_t s_fault = TRAFFIC_FAULT_NONE;
traffic_fault_info_t     s_fault_list[TRAFFIC_FAULT_MAX];
volatile uint8_t         s_fault_count = 0;
SemaphoreHandle_t        s_fault_mutex = NULL;

volatile uint32_t        s_last_fault_notify_ms = 0;
volatile uint16_t        s_blink_mask = 0;
volatile uint16_t        s_fault_yellow_mask = 0;
volatile bool            s_blink_on = false;
volatile uint32_t        s_blink_last_phase = 0;
volatile uint32_t        s_sync_delay_us = 0;
uint8_t                  s_suspect_counter[16] = {0};
uint8_t                  s_vin_off_counter[16] = {0};
uint8_t                  s_ok_streak = 0;
volatile bool            s_mux_reading = false;
volatile uint32_t        s_last_recovery_ok_ms = 0;
volatile bool            s_fault_blink_state = false;

traffic_fault_notify_cb_t s_fault_notify = NULL;
traffic_phase_change_cb_t s_phase_change_cb = NULL;

/* ============================================================
   Публичные функции — синхронизация мигания.
   ============================================================ */
bool traffic_light_get_fault_blink_state(void)
{
    return s_fault_blink_state;
}

void traffic_light_apply_fault_blink(bool on)
{
    if (s_fault != TRAFFIC_FAULT_YELLOW_BLINK) return;
    if (s_fault_blink_state == on) return;
    fault_blink_yellow(on);
}

uint32_t traffic_light_last_recovery_ok_ms(void)
{
    return s_last_recovery_ok_ms;
}

void traffic_light_set_fault_notify(traffic_fault_notify_cb_t cb)
{
    s_fault_notify = cb;
}

void traffic_light_set_phase_change_cb(traffic_phase_change_cb_t cb)
{
    s_phase_change_cb = cb;
}

void traffic_light_set_follow_root(bool en)
{
    s_follow_root = en;
    ESP_LOGI(TAG, "follow_root=%d", en);
}

void traffic_light_request_pedestrian(void)
{
    if (!g_config.traffic.pedestrian.button_enabled) {
        ESP_LOGW(TAG, "pedestrian request ignored: button disabled in config");
        return;
    }

    uint8_t mode = g_config.traffic.pedestrian.request_mode;

    if (mode == PED_REQUEST_MODE_MIN_RECALL) {
        s_ped_min_recall = true;
        ESP_LOGW(TAG, "pedestrian request: MIN_RECALL (next cycle min %lu s)",
                 (unsigned long)g_config.traffic.pedestrian.min_recall_s);
        return;
    }

    s_ped_request = true;
    s_ped_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (mode == PED_REQUEST_MODE_INSERT) {
        ESP_LOGW(TAG, "pedestrian request: INSERT (timeout %lu ms)",
                 (unsigned long)g_config.traffic.pedestrian.insert_timeout_ms);
    } else {
        ESP_LOGW(TAG, "pedestrian request: STAGE");
    }
}

bool traffic_light_ped_button_enabled(void)
{
    return g_config.traffic.pedestrian.button_enabled;
}

void traffic_light_set_sync_delay_us(uint32_t delay_us)
{
    s_sync_delay_us = delay_us;
    ESP_LOGI(TAG, "sync_delay = %lu us (%.3f ms)",
             (unsigned long)delay_us, delay_us / 1000.0);
}

uint32_t traffic_light_get_sync_delay_us(void)
{
    return s_sync_delay_us;
}

void traffic_light_set_mux_reading(bool en)
{
    s_mux_reading = en;
}

static inline uint32_t phase_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.phase_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}

/* ============================================================
   traffic_task — главный цикл.
   ============================================================ */
void traffic_task_loop(void *arg)
{
    ESP_LOGI(TAG, "Traffic task started (step=20ms)");

    uint32_t blink_last_fault = 0;
    bool     fault_blink_on   = false;

    uint8_t  last_min_green_phase = 0xFF;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        if (s_state != TRAFFIC_STATE_RUNNING) continue;

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (s_ped_request &&
            g_config.traffic.pedestrian.request_mode == PED_REQUEST_MODE_INSERT) {
            uint32_t age = now - s_ped_request_ms;
            if (age > g_config.traffic.pedestrian.insert_timeout_ms) {
                ESP_LOGW(TAG, "pedestrian INSERT timeout (%lu ms) → cancel",
                         (unsigned long)age);
                s_ped_request = false;
            }
        }

        if (s_fault != TRAFFIC_FAULT_NONE) {
            if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) {
                if (!s_follow_root) {
                    uint32_t per = g_config.traffic.phases_ms.fault_blink_period;
                    if (per < BLINK_MIN_PERIOD_MS) per = 500;
                    if (now - blink_last_fault >= per) {
                        blink_last_fault = now;
                        fault_blink_on = !fault_blink_on;
                        fault_blink_yellow(fault_blink_on);
                    }
                }
                if (!s_fault_blink_state &&
                    (now - s_last_recovery_ms >= g_config.traffic.fault.recovery_ms)) {
                    try_recovery(now, &blink_last_fault);
                }
            }
            continue;
        }

        uint16_t bm = s_blink_mask;
        uint32_t per = phase_blink_period_ms();
        if (bm && (now - s_blink_last_phase >= per)) {
            s_blink_last_phase = now;
            s_blink_on = !s_blink_on;
            for (int ch = 0; ch < 16; ch++) {
                if (bm & (1u << ch)) {
                    if (s_blink_on) sr_output_on(ch);
                    else            sr_output_off(ch);
                }
            }
        }

        if (s_follow_root) continue;

        /* FIX: stuck-check под мьютексом — чтобы парсинг конфига
           (config_load_json) не дал ложный stuck. */
        {
            bool in_running = false;
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                in_running = (s_state == TRAFFIC_STATE_RUNNING);
                if (in_running) {
                    uint32_t dur_now = effective_stage_duration_ms(s_phase);
                    uint32_t elapsed = now - s_phase_start_ms;
                    if (dur_now > 0 &&
                        elapsed > dur_now + TRAFFIC_PHASE_OVERDUE_MS) {
                        ESP_LOGE(TAG, "phase %u stuck: elapsed=%lu >> dur=%lu → FAULT",
                                 s_phase, (unsigned long)elapsed,
                                 (unsigned long)dur_now);
                        if (s_fault == TRAFFIC_FAULT_NONE) {
                            s_fault = TRAFFIC_FAULT_YELLOW_BLINK;
                            s_last_recovery_ms = now;
                            apply_fault_mode(s_fault);
                            s_fault_blink_state = false;
                            s_ok_streak = 0;
                            s_last_recovery_ok_ms = 0;
                        }
                        xSemaphoreGive(s_mutex);
                        if (s_fault_notify)
                            s_fault_notify(TRAFFIC_FAULT_YELLOW_BLINK);
                        continue;
                    }
                }
                xSemaphoreGive(s_mutex);
            }
            if (!in_running) continue;
        }

        uint32_t dur = effective_stage_duration_ms(s_phase);

        if (s_ped_request && stage_is_transport_green(s_phase)) {
            uint32_t min_green = g_config.traffic.phases_ms.green / 2;
            if (dur > min_green) {
                dur = min_green;
                if (last_min_green_phase != s_phase) {
                    last_min_green_phase = s_phase;
                    ESP_LOGI(TAG, "PED_MIN_GREEN: green %lu → %lu (request)",
                             (unsigned long)g_config.traffic.phases_ms.green,
                             (unsigned long)min_green);
                }
            }
        } else {
            last_min_green_phase = 0xFF;
        }

        /* FIX: PED_REQUEST_MODE_MIN_RECALL — плавный режим:
           ограничиваем green до min_recall_s. */
        if (s_ped_min_recall && stage_is_transport_green(s_phase)) {
            uint32_t max_green = g_config.traffic.pedestrian.min_recall_s * 1000;
            if (max_green > 0 && dur > max_green) {
                dur = max_green;
                if (last_min_green_phase != s_phase) {
                    last_min_green_phase = s_phase;
                    ESP_LOGI(TAG, "PED_MIN_RECALL: green %lu → %lu (recall %lu s)",
                             (unsigned long)g_config.traffic.phases_ms.green,
                             (unsigned long)max_green,
                             (unsigned long)g_config.traffic.pedestrian.min_recall_s);
                }
            }
        }

        if ((now - s_phase_start_ms) < dur) continue;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) continue;

        {
            const traffic_stage_dd_t *cur_st = find_stage(g_config.traffic.sequence[s_phase]);
            bool in_transport_green = false;
            if (cur_st) {
                for (int g = 0; g < cur_st->group_count; g++) {
                    if (cur_st->groups[g] == 3) { in_transport_green = true; break; }
                }
            }
            /* FIX: при активном min_recall в green — НЕ удерживаем фазу.
               Иначе s_phase_start_ms сбрасывается, урезанный dur не
               наступает, и min_recall никогда не переключает. */
            bool min_recall_hold = s_ped_min_recall && in_transport_green;
            if (in_transport_green && !s_ped_request && !min_recall_hold) {
                s_phase_start_ms = now;
                xSemaphoreGive(s_mutex);
                continue;
            }
        }

        uint8_t next_phase;

        if (s_ped_request &&
            g_config.traffic.pedestrian.request_mode == PED_REQUEST_MODE_STAGE) {
            next_phase = next_phase_smooth_ped(s_phase);
            if (next_phase == s_phase) {
                next_phase = (s_phase + 1) % g_config.traffic.sequence_count;
            }
            ESP_LOGI(TAG, "P7: fast → ped phase (seq=%u)", next_phase);
        } else {
            next_phase = (s_phase + 1) % g_config.traffic.sequence_count;
        }

        if (s_ped_request) {
            const traffic_stage_dd_t *nx = find_stage(g_config.traffic.sequence[next_phase]);
            if (nx) {
                for (int g = 0; g < nx->group_count; g++) {
                    if (nx->groups[g] == 7) {
                        s_ped_request = false;
                        ESP_LOGI(TAG, "P7: reached pedestrian phase (seq=%u)", next_phase);
                        break;
                    }
                }
            }
        }

        s_phase = next_phase;
        s_phase_start_ms = now;
        apply_stage_dd(s_phase);

        if (s_ped_min_recall && stage_is_pedestrian(s_phase)) {
            ESP_LOGW(TAG, "MIN_RECALL: ped phase min %lu s applied",
                     (unsigned long)g_config.traffic.pedestrian.min_recall_s);
            s_ped_min_recall = false;
        }

        xSemaphoreGive(s_mutex);

        ESP_LOGI(TAG, "Phase → seq=%u stage=%u dur=%lu%s",
                 s_phase, g_config.traffic.sequence[s_phase],
                 (unsigned long)stage_duration_ms(s_phase),
                 stage_is_pedestrian(s_phase) ? " (PED)" : "");
    }
}

/* ============================================================
   Init / Pins / Mode / Start-Stop / Getters.
   ============================================================ */
esp_err_t traffic_light_init(void)
{
    if (s_task != NULL) return ESP_OK;
    s_mutex = xSemaphoreCreateMutex();
    s_fault_mutex = xSemaphoreCreateMutex();
    if (!s_mutex || !s_fault_mutex) return ESP_ERR_NO_MEM;
    set_all_off();
    xTaskCreate(traffic_task_loop, "traffic_task", 8192, NULL, 4, &s_task);
    ESP_LOGI(TAG, "Traffic initialized (DD, stages=%u seq=%u ped_button=%d)",
             g_config.traffic.stage_count, g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.button_enabled);
    return ESP_OK;
}

void traffic_light_set_pins_single(const traffic_pins_single_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.single.red    = pins->pin_red;
    g_config.traffic.pins.single.yellow = pins->pin_yellow;
    g_config.traffic.pins.single.green  = pins->pin_green;
}

void traffic_light_set_pins_cross(const traffic_pins_cross_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.cross.a_north.red    = pins->a_north_red;
    g_config.traffic.pins.cross.a_north.yellow = pins->a_north_yellow;
    g_config.traffic.pins.cross.a_north.green  = pins->a_north_green;
    g_config.traffic.pins.cross.a_south.red    = pins->a_south_red;
    g_config.traffic.pins.cross.a_south.yellow = pins->a_south_yellow;
    g_config.traffic.pins.cross.a_south.green  = pins->a_south_green;
    g_config.traffic.pins.cross.b_west.red     = pins->b_west_red;
    g_config.traffic.pins.cross.b_west.yellow  = pins->b_west_yellow;
    g_config.traffic.pins.cross.b_west.green   = pins->b_west_green;
    g_config.traffic.pins.cross.b_east.red     = pins->b_east_red;
    g_config.traffic.pins.cross.b_east.yellow  = pins->b_east_yellow;
    g_config.traffic.pins.cross.b_east.green   = pins->b_east_green;
}

void traffic_light_set_pins_cross_ped(const traffic_pins_cross_ped_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.cross_ped.axis_a.red     = pins->axis_a_red;
    g_config.traffic.pins.cross_ped.axis_a.yellow  = pins->axis_a_yellow;
    g_config.traffic.pins.cross_ped.axis_a.green   = pins->axis_a_green;
    g_config.traffic.pins.cross_ped.axis_a2.red    = pins->axis_a2_red;
    g_config.traffic.pins.cross_ped.axis_a2.yellow = pins->axis_a2_yellow;
    g_config.traffic.pins.cross_ped.axis_a2.green  = pins->axis_a2_green;
    g_config.traffic.pins.cross_ped.axis_a3.red    = pins->axis_a3_red;
    g_config.traffic.pins.cross_ped.axis_a3.yellow = pins->axis_a3_yellow;
    g_config.traffic.pins.cross_ped.axis_a3.green  = pins->axis_a3_green;
    g_config.traffic.pins.cross_ped.ped_red        = pins->ped_red;
    g_config.traffic.pins.cross_ped.ped_green      = pins->ped_green;
}

esp_err_t traffic_light_set_mode(traffic_mode_t mode)
{
    if (mode != TRAFFIC_MODE_SINGLE &&
        mode != TRAFFIC_MODE_CROSS &&
        mode != TRAFFIC_MODE_CROSS_PED)
        return ESP_ERR_INVALID_ARG;
    if (s_state != TRAFFIC_STATE_STOPPED) {
        ESP_LOGW(TAG, "Cannot change mode while running");
        return ESP_ERR_INVALID_STATE;
    }
    s_mode = mode;
    g_config.traffic.mode = (uint8_t)mode;
    s_phase = 0;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    set_all_off();
    ESP_LOGI(TAG, "Mode set to %d", mode);
    return ESP_OK;
}

esp_err_t traffic_light_start(void)
{
    if (g_config.traffic.sequence_count == 0) {
        ESP_LOGE(TAG, "No sequence in config");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_mutex) {
        ESP_LOGW(TAG, "START before init — ignored");
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    s_state = TRAFFIC_STATE_RUNNING;
    s_phase = 0;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (s_fault == TRAFFIC_FAULT_NONE) {
        s_fault_count = 0;
        s_fault_yellow_mask = 0;
        s_recovery_attempts = 0;
        s_ok_streak = 0;
        s_last_recovery_ms = s_phase_start_ms;
        apply_stage_dd(0);
    } else {
        s_last_recovery_ms = s_phase_start_ms;
        apply_fault_mode(s_fault);
        s_fault_blink_state = false;
    }

    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "START (mode=%d, seq=%u, fault=%s, follow_root=%d)",
             s_mode, g_config.traffic.sequence_count,
             traffic_light_fault_str(), s_follow_root);
    return ESP_OK;
}

esp_err_t traffic_light_stop(void)
{
    if (!s_mutex) {
        s_state = TRAFFIC_STATE_STOPPED;
        set_all_off();
        ESP_LOGI(TAG, "STOP (no mutex yet)");
        return ESP_OK;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_state = TRAFFIC_STATE_STOPPED;
    s_phase = 0;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_last_recovery_ms = 0;
    s_phase_start_ms = 0;
    s_blink_mask = 0;
    s_blink_on = false;
    s_blink_last_phase = 0;
    s_last_fault_notify_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    set_all_off();
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "STOP");
    return ESP_OK;
}

esp_err_t traffic_light_pause(void)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
    s_state = TRAFFIC_STATE_PAUSED;
    ESP_LOGI(TAG, "PAUSE");
    return ESP_OK;
}

esp_err_t traffic_light_resume(void)
{
    if (s_state != TRAFFIC_STATE_PAUSED) return ESP_ERR_INVALID_STATE;
    s_state = TRAFFIC_STATE_RUNNING;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "RESUME");
    return ESP_OK;
}

esp_err_t traffic_light_reset(void)
{
    if (!s_mutex) {
        s_state = TRAFFIC_STATE_STOPPED;
        s_phase = 0;
        set_all_off();
        ESP_LOGI(TAG, "RESET (no mutex yet)");
        return ESP_OK;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_phase = 0;
    s_state = TRAFFIC_STATE_STOPPED;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_last_recovery_ms = 0;
    s_phase_start_ms = 0;
    s_blink_mask = 0;
    s_blink_on = false;
    s_blink_last_phase = 0;
    s_last_fault_notify_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    set_all_off();
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "RESET");
    return ESP_OK;
}

esp_err_t traffic_light_next_phase(void)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_phase = (s_phase + 1) % g_config.traffic.sequence_count;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    apply_stage_dd(s_phase);
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "NEXT → seq=%u", s_phase);
    return ESP_OK;
}

esp_err_t traffic_light_force_phase(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms)
{
    if (g_config.traffic.sequence_count == 0) return ESP_ERR_INVALID_STATE;
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (phase >= g_config.traffic.sequence_count)
        phase = phase % g_config.traffic.sequence_count;

    if (duration_ms > 0 && phase_ms >= duration_ms) {
        ESP_LOGW(TAG, "force_phase: pm=%lu >= dur=%lu → clamp to 0",
                 (unsigned long)phase_ms, (unsigned long)duration_ms);
        phase_ms = 0;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "force_phase: mutex timeout %d ms", MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool in_fault = (s_fault != TRAFFIC_FAULT_NONE);

    if (s_phase == phase && s_state == TRAFFIC_STATE_RUNNING) {
        s_phase_start_ms = now - phase_ms;
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }

    s_phase          = phase;
    s_phase_start_ms = now - phase_ms;
    s_state          = TRAFFIC_STATE_RUNNING;

    if (!in_fault) {
        apply_stage_dd(phase);
    }

    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "FORCE phase=%u phase_ms=%lu (fault=%s)",
             phase, (unsigned long)phase_ms,
             in_fault ? "ACTIVE" : "NONE");
    return ESP_OK;
}

/* ============================================================
   Getters.
   ============================================================ */
traffic_state_t traffic_light_get_state(void) { return s_state; }
traffic_mode_t  traffic_light_get_mode(void)  { return s_mode;  }
uint8_t         traffic_light_get_phase(void) { return s_phase; }
uint8_t         traffic_light_get_phase_count(void) { return g_config.traffic.sequence_count; }

uint32_t traffic_light_get_phase_ms(void)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return now - s_phase_start_ms;
}

uint32_t traffic_light_get_phase_duration_ms(void)
{
    return stage_duration_ms(s_phase);
}

const char *traffic_light_state_str(void)
{
    switch (s_state) {
        case TRAFFIC_STATE_RUNNING: return "RUNNING";
        case TRAFFIC_STATE_PAUSED:  return "PAUSED";
        default:                    return "STOPPED";
    }
}

traffic_fault_t traffic_light_get_fault(void) { return s_fault; }

const char *traffic_light_fault_str(void)
{
    switch (s_fault) {
        case TRAFFIC_FAULT_YELLOW_BLINK: return "YELLOW_BLINK";
        case TRAFFIC_FAULT_ALL_OFF:      return "ALL_OFF";
        default:                         return "NONE";
    }
}

uint8_t traffic_light_get_fault_count(void) { return s_fault_count; }

void traffic_light_get_fault_info(uint8_t idx, traffic_fault_info_t *out)
{
    if (!out || idx >= s_fault_count) return;
    *out = s_fault_list[idx];
}

uint16_t traffic_light_active_sr_mask(void)
{
    return traffic_light_active_sr_mask_internal();
}

uint16_t traffic_light_expected_sr_mask(void)
{
    return traffic_light_expected_sr_mask_internal();
}

bool traffic_light_test_channel(uint8_t ch)
{
    ESP_LOGI(TAG, "Testing ch%02d...", ch);
    uint16_t saved = sr_get_state();
    uint16_t yellow = 0;
    switch (s_mode) {
    case TRAFFIC_MODE_SINGLE:
        yellow = (1u << g_config.traffic.pins.single.yellow); break;
    case TRAFFIC_MODE_CROSS:
        yellow = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                 (1u << g_config.traffic.pins.cross.a_south.yellow) |
                 (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                 (1u << g_config.traffic.pins.cross.b_east.yellow); break;
    case TRAFFIC_MODE_CROSS_PED:
        yellow = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                 (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                 (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow); break;
    }
    sr_write16(saved & yellow);
    board_diag_en_all_off();
    esp_rom_delay_us(500);
    sr_output_on(ch);
    esp_rom_delay_us(50);

    imon_state_t st = IMON_STATE_UNKNOWN;
    int mv = 0;
    int il = board_read_iload_raw(ch, &st, &mv);

    sr_output_off(ch);
    sr_write16(saved);

    bool ok = (st == IMON_STATE_OK && il >= 0 &&
               mv >= g_config.traffic.fault.imin_mv);
    ESP_LOGI(TAG, "Test ch%02d: %s (V_IMON=%d, I=%d)",
             ch, ok ? "OK" : "FAIL", mv, il);
    return ok;
}