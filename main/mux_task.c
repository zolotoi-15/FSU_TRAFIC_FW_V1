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

static void mux_task(void *arg) {
    ESP_LOGI(TAG, "MUX task started");
    while (1) {
        mux_data_t local = {0};

        uint16_t saved_sr = sr_get_state();
        bool     in_fault = (traffic_light_get_fault() != TRAFFIC_FAULT_NONE);
        if (in_fault) traffic_light_set_mux_reading(true);

        for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
            bool is_active = true;
            if (ch < g_config.sr.count) is_active = g_config.sr.outputs[ch].active;
            if (!is_active) {
                local.v_in[ch] = 0.0f;
                local.i_load[ch] = 0;
                local.imon_state[ch] = IMON_STATE_OK;
                local.imon_mv[ch] = 0;
                local.sr_state[ch] = sr_get_output(ch) ? 1 : 0;
                local.diag_en[ch] = 0;
                continue;
            }
            bool sr_on = sr_get_output(ch);
            local.sr_state[ch] = sr_on ? 1 : 0;

            if (!sr_on) {
                local.v_in[ch] = 0.0f;
                local.i_load[ch] = 0;
                local.imon_state[ch] = IMON_STATE_OK;
                local.imon_mv[ch] = 0;
                local.diag_en[ch] = 0;
                continue;
            }

            /* DIAG_EN включаем ДО измерения (вне mux-мьютекса) */
            esp_err_t dret = board_diag_en_set(ch, true);
            local.diag_en[ch] = (dret == ESP_OK) ? 1 : 0;
            if (dret == ESP_ERR_NOT_FOUND) local.imon_state[ch] = IMON_STATE_NO_DIAG;

            esp_rom_delay_us(50);

            /* Один захват мьютекса — оба сигнала */
            SemaphoreHandle_t mux_mtx = mux_get_mutex();
            if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
            mux_select_channel(ch);
            vTaskDelay(pdMS_TO_TICKS(5));
            int mv_vin  = mux_read_sig1_avg_mv(4);   // V_IN
            int mv_imon = mux_read_sig2_avg_mv(8);   // V_IMON
            if (mux_mtx) xSemaphoreGive(mux_mtx);

            float v_in = (mv_vin >= 0) ? (mv_vin * MUX1_DIVIDER_RATIO / 1000.0f) : 0.0f;
            if (v_in < 5.0f) v_in = 0.0f;
            local.v_in[ch] = v_in;

            int iload = 0;
            imon_state_t st = IMON_STATE_UNKNOWN;
            if (mv_imon < 0) {
                st = IMON_STATE_UNKNOWN;
            } else if (mv_imon < g_config.traffic.fault.imin_mv) {
                st = IMON_STATE_NO_DIAG;
            } else if (mv_imon >= IMON_TH_FAULT_MV) {
                st = IMON_STATE_FAULT;
            } else if (mv_imon >= IMON_TH_SUSPECT_MV) {
                st = IMON_STATE_SUSPECT;
            } else {
                st = IMON_STATE_OK;
                int offset = mux_get_imon_offset_mv(ch);
                int corr = mv_imon - offset;
                if (corr < 0) corr = 0;
                iload = (int)(((float)corr * TPS27S100B_K_IMON) / TPS27S100B_R_IMON);
            }
            local.i_load[ch]     = iload;
            local.imon_state[ch] = (uint8_t)st;
            local.imon_mv[ch]    = (int16_t)(mv_imon > 32767 ? 32767 : mv_imon);

            board_diag_en_set(ch, false);
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

void mux_app_task_start(void) {
    xTaskCreate(mux_task, "mux_task", 8192, NULL, 5, NULL);
}