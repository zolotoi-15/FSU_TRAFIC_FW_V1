#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Инициализация: если DS1307 доступен — читает время оттуда,
   иначе стартует от 2024-01-01 00:00:00 UTC (или сохранённого в NVS). */
esp_err_t time_sync_init(void);

/* Текущее время (UTC) в секундах + мс. Возвращает false, если время
   ещё не установлено (только что загрузились, RTC пуст). */
bool time_sync_get(uint32_t *unix_sec, uint16_t *ms);

/* Установить время (UTC). Если rtt_us > 0 — добавляет rtt_us/2 к ms
   (компенсация односторонней задержки). */
void time_sync_set(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us);

/* Инициализировать SNTP (только root, если есть интернет/PPP).
   При получении времени — пишет в DS1307 и в системные часы. */
esp_err_t time_sync_start_sntp(void);

/* Периодический отчёт node → root (evt:time:...). Вызывать из
   node_send_task раз в ~5 с. */
void time_sync_node_report(void);

/* root: раз в N секунд шлёт time: node'у. Вызывать из phase_sync_task. */
void time_sync_root_push(uint32_t rtt_us);

#ifdef __cplusplus
}
#endif