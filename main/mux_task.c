/* ============================================================
   mux_task.c — циклическое измерение 16 каналов (V_IN, I_LOAD,
   V_IMON, DIAG_EN, SR) + периодический лог.

   === FIX MEASURE-ALL-ALWAYS ===
       Меряем ВСЕ 16 каналов, включая !sr_on и inactive.

   === FIX MUX-READ-IN-ALL-OFF ===
       При ALL_OFF меряем каналы. При YELLOW_BLINK — пропускаем.

   === FIX CHANNEL-PERIODIC-LOG ===
       Отдельная задача ch_log_task печатает таблицу раз в 5 с.

   === FIX FAULT-CHECK-EVERY-CYCLE ===
       traffic_light_check_faults(&local) вызывается КАЖДЫЙ
       цикл (раз в MUX_TASK_PERIOD_MS), даже если данные не
       изменились. Иначе переход SHORT_GND → восстановление
       между двумя циклами мог быть не замечен.
   ============================================================ */

#include "mux_task.h"
#include "app_shared.h"
#include "mux.h"
#include "mux_data.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mux_task";
#define MUX_TASK_PERIOD_MS        1000
#define CHANNEL_LOG_PERIOD_MS     5000

static void mux_task(void *arg)
{
    ESP_LOGI(TAG, "MUX task started");

    while (1) {
        /* === FIX MUX-READ-IN-ALL-OFF ===
           Пропускаем только YELLOW_BLINK (fault_blink меняет SR,
           try_recovery дёргает DIAG_EN — mux_task мешал бы).
           ALL_OFF — меряем (NODE ждёт traffic:start, но каналы
           нужны для T_CHANNELS → ROOT). */
        if (traffic_light_get_fault() == TRAFFIC_FAULT_YELLOW_BLINK) {
            vTaskDelay(pdMS_TO_TICKS(MUX_TASK_PERIOD_MS));
            continue;
        }

        mux_data_t local = {0};

        for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
            bool is_active = true;
            if (ch < g_config.sr.count) is_active = g_config.sr.outputs[ch].active;
            local.sr_state[ch] = sr_get_output(ch) ? 1 : 0;

            if (!is_active) {
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

            esp_rom_delay_us(200);   /* tMON(on1) typ 10 мкс */

            /* Один захват мьютекса — оба сигнала */
            SemaphoreHandle_t mux_mtx = mux_get_mutex();
            if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
            mux_select_channel(ch);
            vTaskDelay(pdMS_TO_TICKS(5));
            int mv_vin  = mux_read_sig1_avg_mv(4);   // V_IN
            int mv_imon = mux_read_sig2_avg_mv(8);   // V_IMON
            if (mux_mtx) xSemaphoreGive(mux_mtx);

            float v_in = (mv_vin >= 0)
                         ? (mv_vin * MUX1_DIVIDER_RATIO / 1000.0f) : 0.0f;
            if (v_in < 1.0f) v_in = 0.0f;
            local.v_in[ch] = v_in;

            int iload = 0;
            imon_state_t st;
            if (dret == ESP_ERR_NOT_FOUND) {
                st = IMON_STATE_NO_DIAG;
                local.imon_mv[ch] = 0;
            } else {
                st = imon_classify(mv_imon, g_config.traffic.fault.imin_mv);
                local.imon_mv[ch] = (int16_t)(mv_imon > 32767 ? 32767 : mv_imon);
                if (st == IMON_STATE_OK) {
                    int offset = mux_get_imon_offset_mv(ch);
                    int corr = mv_imon - offset;
                    if (corr < 0) corr = 0;
                    iload = (int)(((float)corr * TPS27S100B_K_IMON) /
                                  TPS27S100B_R_IMON);
                }
            }
            local.i_load[ch]     = iload;
            local.imon_state[ch] = (uint8_t)st;

            board_diag_en_set(ch, false);
        }

        local.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        local.valid = true;
        mux_data_set(&local);

        /* === FIX FAULT-CHECK-EVERY-CYCLE ===
           Проверяем аварии сразу после mux_data_set. Логика
           внутри: SHORT_GND/VIN_ON_OFF ловятся даже в grace
           period; SUSPECT/OVERLOAD — только после grace.
           check_faults вызывается независимо от того,
           изменились ли данные — важно поймать переход. */
        traffic_light_check_faults(&local);
        /* === END FIX FAULT-CHECK-EVERY-CYCLE === */

        vTaskDelay(pdMS_TO_TICKS(MUX_TASK_PERIOD_MS));
    }
}

/* === FIX CHANNEL-PERIODIC-LOG === */
static void ch_log_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(CHANNEL_LOG_PERIOD_MS));
    while (1) {
        if (traffic_light_get_fault() != TRAFFIC_FAULT_YELLOW_BLINK) {
            mux_data_t md;
            if (mux_data_get(&md)) {
                ESP_LOGI(TAG, "---- channels (role=%s, fault=%s) ----",
                         g_config.role == DEVICE_ROLE_MASTER ? "ROOT" : "NODE",
                         traffic_light_fault_str());
                for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
                    bool active = true;
                    if (ch < g_config.sr.count)
                        active = g_config.sr.outputs[ch].active;
                    if (!active) continue;
                    ESP_LOGI(TAG, "ch%02d: V_IN=%5.2fV I=%4dmA V_IMON=%4dmV "
                                  "STATE=%-7s SR=%d DIAG=%d",
                             ch, md.v_in[ch], md.i_load[ch], md.imon_mv[ch],
                             imon_state_str((imon_state_t)md.imon_state[ch]),
                             md.sr_state[ch], md.diag_en[ch]);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CHANNEL_LOG_PERIOD_MS));
    }
}
/* === END FIX CHANNEL-PERIODIC-LOG === */

void mux_app_task_start(void)
{
    xTaskCreate(mux_task,   "mux_task", 8192, NULL, 5, NULL);
    xTaskCreate(ch_log_task, "ch_log",  3072, NULL, 3, NULL);
}