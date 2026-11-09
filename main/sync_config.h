/* ============================================================
   sync_config.h — общие константы синхронизации ROOT/NODE.

   === FIX SYNC-LEAD-CONSISTENT ===
   Раньше SYNC_LEAD_MS был разным в root_wd.c (500),
   mqtt_apply.c (700) и node_proto.c (300). ROOT и NODE
   применяли фазу с разной задержкой → рассинхрон светофоров.
   Теперь единая константа для всех модулей.
   ============================================================ */
#pragma once

/* Задержка между отправкой sync: и моментом применения фазы
   на NODE. Должна покрывать: LoRa ToA (~112 мс SF9/BW250) +
   FRAG_GAP_MS (250 мс) + запас на джиттер.
   Используется:
     - root_wd.c:   pm_target = phase_ms + SYNC_LEAD_MS
     - mqtt_apply.c: apply_ms = now_ms + SYNC_LEAD_MS
     - node_proto.c: fallback при отсутствии поля в sync:
*/
#define SYNC_LEAD_MS        500

/* Минимальный интервал между двумя sync: с одинаковой фазой
   (защита от дублей). */
#define SYNC_MIN_INTERVAL_MS  250

/* Период push time: (ROOT → NODE). */
#define TIME_PUSH_PERIOD_MS   30000

/* Период hold-resync (ROOT повторяет sync: пока NODE в hold). */
#define HOLD_RESYNC_INTERVAL_MS  5000

/* Период полного sync: (ROOT → NODE) в обычном режиме. */
#define LORA_SYNC_INTERVAL_MS    30000