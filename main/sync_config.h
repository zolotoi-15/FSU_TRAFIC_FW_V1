/* ============================================================
   sync_config.h — общие константы синхронизации ROOT/NODE.

   === FIX SYNC-INTERVAL-500 ===
       LORA_SYNC_INTERVAL_MS 1000 → 500. NODE получает c: чаще
       → отставание от ROOT меньше. Эфир: c: 72 мс × 2/с = 14%
       при SF8/BW250 — приемлемо.

   === FIX TIME-PUSH-10S ===
       TIME_PUSH_PERIOD_MS 30000 → 10000. NODE обновляет время
       чаще → дрейф кварца (±20 ppm = ±0.6 с за 30 с, ±0.2 с за
       10 с) меньше. Эфир: time: 72 мс × 0.1/с = 0.7%.

   === FIX UTC-SYNC-DISABLE ===
       Порог игнорирования end_utc_ms при плохом time_sync.
       Если |delta| > UTC_DELTA_MAX_MS — NODE применяет pm из c:
       (fallback), а не вычисляет из UTC. Иначе 2-секундная
       ошибка часов даёт 2-секундное отставание.
   ============================================================ */
#pragma once

#define SYNC_LEAD_MS              250
#define SYNC_MIN_INTERVAL_MS      250
/* === FIX TIME-PUSH-10S: 30000 → 10000 === */
#define TIME_PUSH_PERIOD_MS       10000
/* === END FIX TIME-PUSH-10S === */
#define HOLD_RESYNC_INTERVAL_MS   500

/* === FIX SYNC-INTERVAL-500: 1000 → 500 === */
#define LORA_SYNC_INTERVAL_MS     500
/* === END FIX SYNC-INTERVAL-500 === */

#define SYNC_MAX_DRIFT_MS         250

/* === FIX UTC-SYNC-DISABLE ===
   Если |delta| (root - local) > 500 мс — time_sync на NODE
   ненадёжен, end_utc_ms игнорируется. 500 мс — половина
   LORA_SYNC_INTERVAL_MS, дальше уже заметно глазу. */
#define UTC_DELTA_MAX_MS          500
/* === END FIX UTC-SYNC-DISABLE === */