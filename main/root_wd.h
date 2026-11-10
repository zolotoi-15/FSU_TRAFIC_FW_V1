#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   root_wd.h — ROOT: phase-sync + heartbeat. CFG-push удалён.

   === SIMPLIFIED-NO-CFG ===
       ROOT шлёт NODE готовые маски каналов через
       'c:<phase>:<pm>:<on_hex>:<blink_hex>:<dur>'.
       Функции старой sync: архитектуры (send_sync_to_node_async,
       send_sync_to_node_async_ex, apply_sync_at_utc, apply_sync_at)
       оставлены как алиасы на send_channels_to_node — для
       совместимости с traffic_light.c (insert/safe_exit/apply_stage_dd
       вызывают send_sync_to_node_async_ex).
   ============================================================ */

void root_wd_start_tasks(void);
void root_apply_new_config_from_mqtt(const char *json, int len);

/* === FIX SYNC-COMPAT ===
   Алиасы на send_channels_to_node. traffic_light.c использует
   их в insert/safe_exit. В NO-CFG force=true обходит дедуп. */
uint32_t send_sync_to_node_async_ex(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms, bool force,
                                    uint8_t *out_next_phase);

uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase);

uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms);

uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_utc);

void     apply_sync_at(uint8_t phase, uint32_t phase_ms,
                       uint32_t duration_ms, uint32_t apply_ms);
/* === END FIX SYNC-COMPAT === */
uint32_t send_all_off_to_node(uint32_t duration_ms);

/* Stub-глобалы для совместимости с другими модулями.
   В NO-CFG s_cfg_ack всегда true — CFG не передаётся. */
extern volatile bool     s_cfg_ack;
extern volatile bool     s_cfg_pushing;
extern volatile bool     s_cfg_applied_and_started;
extern volatile bool     s_cfg_req_pending;
extern volatile bool     s_force_resync_pending;
extern volatile uint32_t s_cfg_last_ok_ms;

#ifdef __cplusplus
}
#endif