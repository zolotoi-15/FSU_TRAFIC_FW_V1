/* ============================================================
   time_sync.h — API синхронизации времени ROOT/NODE.

   ROOT: SNTP (при получении IP) + DS1307 → time_sync_set()
         NODE: получает time: от ROOT → time_sync_set()
   Оба: time_sync_get() возвращает текущее UTC (sec + ms).

   === FIX UTC-SYNC ===
       time_sync_get_utc_ms() — абсолютное UTC-время в ms с 1970.
       Используется в c: (end_utc_ms) и для вычисления phase_ms
       на NODE. Возвращает 0, если время не установлено.
   ============================================================ */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Инициализация. Читает DS1307 (если есть) в системные часы.
   Вызывать один раз из app_main ПОСЛЕ rtc_init(). */
esp_err_t time_sync_init(void);

/* Получить текущее UTC-время.
   Возвращает false, если время ещё не установлено (нет RTC/SNTP/ROOT).
   unix_sec — секунды с 1970, ms — миллисекунды [0..999]. */
bool time_sync_get(uint32_t *unix_sec, uint16_t *ms);

/* === FIX UTC-SYNC ===
   Абсолютное UTC-время в миллисекундах с 1970 (uint64).
   Возвращает 0, если время не установлено. */
uint64_t time_sync_get_utc_ms(void);
/* === END FIX UTC-SYNC === */

/* Установить время. rtt_us — round-trip (для компенсации задержки). */
void time_sync_set(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us);

/* SNTP (только ROOT). Запускается из on_got_ip после IP. */
esp_err_t time_sync_start_sntp(void);

/* ROOT: push time: на NODE.
   _force — игнорировать период TIME_SYNC_PUSH_PERIOD_MS (30 с). */
void time_sync_root_push(uint32_t rtt_us);
void time_sync_root_push_force(uint32_t rtt_us);

/* NODE: периодический отчёт time: в ROOT (каждые 5 с). */
void time_sync_node_report(void);

/* Диагностика: последняя дельта (root - local) и её «возраст». */
void    time_sync_note_delta_ms(int32_t delta_ms);
int32_t time_sync_get_delta_ms(void);
uint32_t time_sync_get_delta_age_ms(void);

#ifdef __cplusplus
}
#endif