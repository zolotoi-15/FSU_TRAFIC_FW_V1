#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Запуск задач ROOT: phase_sync, watchdog, cfg_push. */
void root_wd_start_tasks(void);

/* === FIX: экспорт для node_proto.c (ROOT замечает перезагрузку NODE) === */
uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms);
void     apply_sync_at(uint8_t phase, uint32_t phase_ms,
                       uint32_t duration_ms, uint32_t apply_ms);

extern volatile bool s_cfg_ack;
extern volatile bool s_cfg_pushing;

/* === FIX P4: глобальный флаг «CFG применён + traffic:start отправлен» ===
   phase_sync_task НЕ шлёт sync:, пока этот флаг false.
   root_node_watchdog_task ставит его в true ПОСЛЕ traffic:start. */
extern volatile bool s_cfg_applied_and_started;

#ifdef __cplusplus
}
#endif