/* ============================================================
   sync_config.h — общие константы синхронизации ROOT/NODE.

   === FIX SYNC-LEAD-CONSISTENT ===
   Раньше SYNC_LEAD_MS был разным в root_wd.c (500),
   mqtt_apply.c (700) и node_proto.c (300). ROOT и NODE
   применяли фазу с разной задержкой → рассинхрон светофоров.
   Теперь единая константа для всех модулей.

   === FIX SYNC-INTERVAL-1S ===
   LORA_SYNC_INTERVAL_MS было 3000 → 1000. Окно несинхронности
   при потере c: сокращается с 3 с до 1 с. Трафик: 29 байт/с
   (ToA ~112 мс SF9/BW250) ≈ 11% эфира — приемлемо.
   ============================================================ */
#pragma once

/* Задержка между отправкой sync: и моментом применения фазы
   на NODE. Должна покрывать: LoRa ToA (~112 мс SF9/BW250) +
   FRAG_GAP_MS (250 мс) + запас на джиттер. */
#define SYNC_LEAD_MS        250

/* Минимальный интервал между двумя sync: с одинаковой фазой
   (защита от дублей). */
#define SYNC_MIN_INTERVAL_MS  250

/* Период push time: (ROOT → NODE). */
#define TIME_PUSH_PERIOD_MS   30000

/* Период hold-resync (ROOT повторяет sync: пока NODE в hold). */
#define HOLD_RESYNC_INTERVAL_MS  500

/* Период полного sync: (ROOT → NODE) в обычном режиме.
   === FIX SYNC-INTERVAL-1S: 3000 → 1000 === */
#define LORA_SYNC_INTERVAL_MS    1000