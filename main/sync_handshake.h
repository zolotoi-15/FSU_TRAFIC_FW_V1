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

   ROOT:
     sync_handshake_root_start()      — при потере NODE
     sync_handshake_root_tick(now)    — вызывать каждые 20 мс
     sync_handshake_root_on_rx(...)   — при приёме EVT/STATUS от NODE
     sync_handshake_root_state()      — для phase_sync_task

   NODE:
     sync_handshake_node_start()      — при потере ROOT
     sync_handshake_node_tick(now)    — вызывать каждые 20 мс
     sync_handshake_node_on_rx(...)   — при приёме CMD от ROOT
     sync_handshake_node_state()      — для traffic_task_loop

   === FIX SYNC-BLINK-NODE ===
   Пока идёт синхронизация, обе стороны мигают ЖДИТЕ (ch11):
     - PROBE: локальный таймер (просто «я жив»);
     - TIME/PHASE/ALIGNED: абсолютная фаза от UTC
       (utc_ms / SYNC_DISPLAY_BLINK_MS) & 1 — обе стороны
       используют одну формулу → мигание синхронно до миллисекунды;
     - ROOT в ALIGNED раз в SYNC_DISPLAY_RESYNC_MS шлёт
       sync:display:<0|1>:<utc_ms_next_edge> для форса фазы NODE.

   === FIX SYNC-DISPLAY-VERIFY ===
   ROOT переходит ALIGNED → IDLE (и запускает конфиг) ТОЛЬКО
   когда NODE несколько STATUS подряд прислал "disp", совпадающий
   с текущим битом ROOT (root_current_disp_bit()).

   === FIX SYNC-LOCK-ANY-STATE ===
   NODE принимает sync:lock в любом состоянии, кроме ALIGNED
   (и IDLE → принудительно PROBE→PHASE). Раньше lock принимался
   только в PHASE. Но ROOT начинает слать lock сразу после time:,
   а time: может потеряться → NODE в TIME игнорировал lock →
   бесконечный цикл PROBE→TIME→(lock ignore)→TIMEOUT→PROBE.

   === FIX SYNC-BLINK-NODE ===
   После перехода NODE в PHASE через lock, мигание display
   продолжается по UTC — иначе при задержке sync:done NODE
   замирает с одним битом display.
   ============================================================ */

typedef enum {
    SYNC_STATE_IDLE        = 0,   /* нормальная работа (RUNNING) */
    SYNC_STATE_PROBE       = 1,   /* ROOT ищет NODE / NODE ищет ROOT */
    SYNC_STATE_TIME        = 2,   /* обмен time: / timeq */
    SYNC_STATE_PHASE       = 3,   /* обмен sync:lock / sync:pong */
    SYNC_STATE_ALIGNED     = 4,   /* ROOT подтвердил → ждём display sync */
} sync_state_t;

/* ============================================================
   Параметры handshake (можно тюнить)
   ============================================================ */
#define SYNC_PROBE_INTERVAL_MS       200   /* probe/ack период */
#define SYNC_PROBE_MAX_MS            20000 /* сколько ждать ответ до повтора */
#define SYNC_TIME_TRIES              3     /* сколько раз шлём time: */
#define SYNC_PONG_TRIES              3     /* сколько раз шлём sync:pong */
#define SYNC_LOCK_TRIES              5     /* сколько раз шлём sync:lock */
#define SYNC_ALIGN_DRIFT_MS          50    /* допустимый |Δpm| для aligned */
#define SYNC_ALIGN_STREAK            3     /* сколько подряд для done */
/* === FIX SYNC-BLINK-NODE: 8000 → 12000 ===
   8 с слишком мало: time×3(1.5с) + lock×5(2.5с) + 5 pong по RTT(2.5с)
   уже ~7 с, а на радиоканале с паузами запросто выходит за 8 с. */
#define SYNC_ALIGN_TIMEOUT_MS        12000
/* === END FIX SYNC-BLINK-NODE === */

/* ============================================================
   Параметры синхронного мигания ЖДИТЕ
   ============================================================ */
#define SYNC_DISPLAY_BLINK_MS        250   /* период мигания */
#define SYNC_DISPLAY_RESYNC_MS       1000  /* как часто ROOT шлёт sync:display */
#define SYNC_DISPLAY_CH              11    /* ch11 — display */

/* === FIX SYNC-DISPLAY-VERIFY ===
   ROOT переходит ALIGNED → IDLE (и запускает конфиг) только
   когда NODE мигает display синхронно с ROOT несколько STATUS
   подряд. При периоде STATUS 500 мс в фазе синхронизации
   SYNC_DISPLAY_STREAK_MIN = 3 даёт ~1.5 с подтверждённой
   синхронности. */
#define SYNC_DISPLAY_STREAK_MIN      3
/* === END FIX SYNC-DISPLAY-VERIFY === */

/* ============================================================
   ROOT
   ============================================================ */
void          sync_handshake_root_start(void);
void          sync_handshake_root_tick(uint32_t now_ms);
void          sync_handshake_root_on_rx(const char *evt, size_t len);
sync_state_t  sync_handshake_root_state(void);
bool          sync_handshake_root_is_syncing(void);

/* ============================================================
   NODE
   ============================================================ */
void          sync_handshake_node_start(void);
void          sync_handshake_node_tick(uint32_t now_ms);
void          sync_handshake_node_on_rx(const char *cmd, size_t len);
sync_state_t  sync_handshake_node_state(void);
bool          sync_handshake_node_is_syncing(void);

#ifdef __cplusplus
}
#endif