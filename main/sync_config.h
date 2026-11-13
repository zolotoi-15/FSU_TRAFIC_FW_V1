#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Периоды синхронизации (константы) */
#define SYNC_MIN_INTERVAL_MS      250
#define TIME_PUSH_PERIOD_MS       10000
#define HOLD_RESYNC_INTERVAL_MS   500

#define LORA_SYNC_INTERVAL_MS     300

#define SYNC_MAX_DRIFT_MS         250
#define UTC_DELTA_MAX_MS          3000

/* === FIX LEAD-MS-RUNTIME ===
   Runtime-переменная вместо #define. Значение по умолчанию
   задаётся в sync_config.c (150 мс). */
#define SYNC_LEAD_MS_DEFAULT      150
#define SYNC_LEAD_MS_MIN          0
#define SYNC_LEAD_MS_MAX          1000

extern volatile uint32_t g_sync_lead_ms;

uint32_t sync_config_get_lead_ms(void);
/* Возвращает ESP_OK, если значение в [MIN..MAX] и применено.
   Иначе — ESP_ERR_INVALID_ARG, g_sync_lead_ms не меняется. */
int      sync_config_set_lead_ms(uint32_t ms);
/* === END FIX LEAD-MS-RUNTIME === */

/* === FIX LEAD-MS-NVS ===
   Инициализация: читает lead_ms из NVS. Если в NVS пусто или
   значение вне [MIN..MAX] — использует SYNC_LEAD_MS_DEFAULT.
   Вызывать в app_main ДО traffic_light_init().
   Возвращает ESP_OK, если NVS прочитан (или пуст — это норма),
   ESP_FAIL при ошибке NVS. */
esp_err_t sync_config_init(void);

/* Стирает сохранённый lead_ms из NVS (следующий ребут —
   дефолт 150). */
esp_err_t sync_config_nvs_erase(void);
/* === END FIX LEAD-MS-NVS === */

#ifdef __cplusplus
}
#endif