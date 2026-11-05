#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Запуск задач ROOT: phase_sync, watchdog, cfg_push, heartbeat. */
void root_wd_start_tasks(void);

/* === FIX UTC: асинхронная отправка sync: ===
   send_sync_to_node_async — только отправка, возвращает apply_utc
     (> 0 — успех; 0 — ошибка/пропуск). Не блокирует вызывающего.
   apply_sync_at_utc — ждёт apply_utc (до 2 с), применяет force_phase.
     Возвращает 1 при успехе, 0 при ошибке. */
uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase);
uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_utc);

/* Синхронная обёртка (fallback из watchdog / phase_sync 30s) */
uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms);
void     apply_sync_at(uint8_t phase, uint32_t phase_ms,
                       uint32_t duration_ms, uint32_t apply_ms);

extern volatile bool s_cfg_ack;
extern volatile bool s_cfg_pushing;
extern volatile bool s_cfg_applied_and_started;

/* === FIX #2: NODE запросил CFG → будим cfg_push_task === */
extern volatile bool s_cfg_req_pending;
/* === END FIX #2 === */

/* === FIX: отложенный resync (не блокирует lora_poll) === */
extern volatile bool s_force_resync_pending;

#ifdef __cplusplus
}
#endif