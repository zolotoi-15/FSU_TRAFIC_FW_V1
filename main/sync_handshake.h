#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   sync_handshake — ускоренное восстановление LoRa-связи
   с явной фазой синхронизации и синхронным миганием ЖДИТЕ.

   === FIX SYNC-PONG-UTC ===
   В sync:pong NODE теперь возвращает НЕ локальный
   traffic_light_get_phase_ms(), а pm, пересчитанный по UTC
   через end_utc_ms из sync:lock. ROOT сравнивает по той же
   UTC-шкале. ToA/RTT/обработка полностью исключаются из drift.
   ============================================================ */

typedef enum {
    SYNC_STATE_IDLE        = 0,
    SYNC_STATE_PROBE       = 1,
    SYNC_STATE_TIME        = 2,
    SYNC_STATE_PHASE       = 3,
    SYNC_STATE_ALIGNED     = 4,
} sync_state_t;

/* Параметры handshake */
#define SYNC_PROBE_INTERVAL_MS       200
#define SYNC_PROBE_MAX_MS            20000
#define SYNC_TIME_TRIES              3
#define SYNC_PONG_TRIES              3
#define SYNC_LOCK_TRIES              5

/* === FIX SYNC-PONG-UTC ===
   С UTC-сравнением drift отражает только реальный дрейф
   (единицы мс). Оставляем строгий порог 50 мс. */
#define SYNC_ALIGN_DRIFT_MS          50
#define SYNC_ALIGN_STREAK            3
#define SYNC_ALIGN_TIMEOUT_MS        12000

/* Параметры синхронного мигания ЖДИТЕ */
#define SYNC_DISPLAY_BLINK_MS        250
#define SYNC_DISPLAY_RESYNC_MS       1000
#define SYNC_DISPLAY_CH              11
#define SYNC_DISPLAY_STREAK_MIN      3

/* ROOT API */
void          sync_handshake_root_start(void);
void          sync_handshake_root_tick(uint32_t now_ms);
void          sync_handshake_root_on_rx(const char *evt, size_t len);
sync_state_t  sync_handshake_root_state(void);
bool          sync_handshake_root_is_syncing(void);

/* NODE API */
void          sync_handshake_node_start(void);
void          sync_handshake_node_tick(uint32_t now_ms);
void          sync_handshake_node_on_rx(const char *cmd, size_t len);
sync_state_t  sync_handshake_node_state(void);
bool          sync_handshake_node_is_syncing(void);

#ifdef __cplusplus
}
#endif