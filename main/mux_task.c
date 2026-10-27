#include "mux_task.h"
#include "app_shared.h"
#include "mux.h"
#include "mux_data.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "config_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mux_task";
#define MUX_TASK_PERIOD_MS 1000

static void mux_task(void *arg)
{
    ESP_LOGI(TAG, "MUX task started");
    while (1) {
        mux_data_t local = {0};

        uint16_t saved_sr = sr_get_state();
        bool     in_fault = (traffic_light_get_fault() != TRAFFIC_FAULT_NONE);
        uint8_t  fault_cnt = 0;
        uint8_t  fault_ch[TRAFFIC_FAULT_MAX];
        for (int i = 0; i < TRAFFIC_FAULT_MAX; i++) fault_ch[i] = 0xFF;

        if (in_fault) {
            fault_cnt = traffic_light_get_fault_count();
            if (fault_cnt > TRAFFIC_FAULT_MAX) fault_cnt = TRAFFIC_FAULT_MAX;
            for (uint8_t i = 0; i < fault_cnt; i++) {
                traffic_fault_info_t info;
                traffic_light_get_fault_info(i, &info);
                if (info.channel < 16) fault_ch[i] = info.channel;
            }
            traffic_light_set_mux_reading(true);
        }

        for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
            bool is_active = true;
            if (ch < g_config.sr.count) is_active = g_config.sr.outputs[ch].active;
            if (!is_active) {
                local.v_in[ch] = 0.0f;
                local.i_load[ch] = 0;
                local.imon_state[ch] = IMON_STATE_OK;
                local.imon_mv[ch] = 0;
                local.sr_state[ch] = sr_get_output(ch) ? 1 : 0;
                continue;
            }
            bool sr_on = sr_get_output(ch);

            float v_in = 0.0f;
            SemaphoreHandle_t mux_mtx = mux_get_mutex();
            if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
            mux_select_channel(ch);
            vTaskDelay(pdMS_TO_TICKS(5));
            v_in = mux_read_vin_mv(4) / 1000.0f;
            if (mux_mtx) xSemaphoreGive(mux_mtx);

            if (v_in < 5.0f) v_in = 0.0f;
            local.v_in[ch] = v_in;

            imon_state_t st = IMON_STATE_UNKNOWN;
            int mv_raw = 0;
            int iload  = board_read_iload_raw(ch, &st, &mv_raw);
            local.i_load[ch]     = iload;
            local.imon_state[ch] = (uint8_t)st;
            local.imon_mv[ch]    = (int16_t)(mv_raw > 32767 ? 32767 : mv_raw);
            local.sr_state[ch]   = sr_on ? 1 : 0;
        }

        if (in_fault) {
            sr_write16(saved_sr);
            traffic_light_set_mux_reading(false);
        }

        local.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        local.valid = true;
        mux_data_set(&local);
        traffic_light_check_faults(&local);
        vTaskDelay(pdMS_TO_TICKS(MUX_TASK_PERIOD_MS));
    }
}

void mux_app_task_start(void)
{
    xTaskCreate(mux_task, "mux_task", 8192, NULL, 5, NULL);
}