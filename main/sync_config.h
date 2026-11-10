/* ============================================================
   sync_config.h — общие константы синхронизации ROOT/NODE.

   === FIX SYNC-INTERVAL-1S ===
       LORA_SYNC_INTERVAL_MS было 3000 → 1000. Окно несинхронности
       при потере c: сокращается с 3 с до 1 с. Трафик: 29 байт/с
       (ToA ~112 мс SF9/BW250) ≈ 11% эфира — приемлемо.
   ============================================================ */
#pragma once

#define SYNC_LEAD_MS              250
#define SYNC_MIN_INTERVAL_MS      250
#define TIME_PUSH_PERIOD_MS       30000
#define HOLD_RESYNC_INTERVAL_MS   500

/* Период полного sync: (ROOT → NODE) в обычном режиме.
   === FIX SYNC-INTERVAL-1S: 3000 → 1000 === */
#define LORA_SYNC_INTERVAL_MS     1000