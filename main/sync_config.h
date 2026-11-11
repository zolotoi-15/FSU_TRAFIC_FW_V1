/* ============================================================
   sync_config.h — общие константы синхронизации ROOT/NODE.

   === FIX SYNC-INTERVAL-500 ===
       LORA_SYNC_INTERVAL_MS 1000 → 500 → 300.

   === FIX TIME-PUSH-10S ===
       TIME_PUSH_PERIOD_MS 30000 → 10000.

   === FIX UTC-DELTA-LOOSE ===
       UTC_DELTA_MAX_MS 500 → 5000.

   === FIX ROOT-SEND-FIRST ===
       ROOT при смене фазы сначала отправляет c: на NODE,
       ждёт lead_ms, и только потом применяет маску у себя.
       NODE и ROOT зажигают каналы одновременно.

   === FIX LEAD-MS-RUNTIME ===
       SYNC_LEAD_MS больше НЕ #define, а runtime-переменная
       g_sync_lead_ms (по умолчанию 150 мс). Меняется:
         - из MQTT: команда "sync_lead:<ms>";
         - из BLE:  {"type":"sync.set","lead_ms":150};
         - программно: sync_config_set_lead_ms(ms).
       Валидация: 50..1000 мс. Меньше 50 — NODE не успеет
       принять c: и включить каналы (LoRa ToA ~112 мс при
       SF8/BW250). Больше 1000 — неоправданная задержка
       реакции светофора на смену фазы.
   ============================================================ */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Периоды синхронизации (остаются константами) */
#define SYNC_MIN_INTERVAL_MS      250
#define TIME_PUSH_PERIOD_MS       10000
#define HOLD_RESYNC_INTERVAL_MS   500
#define LORA_SYNC_INTERVAL_MS     300
#define SYNC_MAX_DRIFT_MS         250
#define UTC_DELTA_MAX_MS          5000

/* === FIX LEAD-MS-RUNTIME ===
   Runtime-переменная вместо #define. Значение по умолчанию
   задаётся в sync_config.c (150 мс). */
#define SYNC_LEAD_MS_DEFAULT      150
#define SYNC_LEAD_MS_MIN          50
#define SYNC_LEAD_MS_MAX          1000

extern volatile uint32_t g_sync_lead_ms;

uint32_t sync_config_get_lead_ms(void);
/* Возвращает ESP_OK, если значение в [MIN..MAX] и применено.
   Иначе — ESP_ERR_INVALID_ARG, g_sync_lead_ms не меняется. */
int      sync_config_set_lead_ms(uint32_t ms);
/* === END FIX LEAD-MS-RUNTIME === */

#ifdef __cplusplus
}
#endif