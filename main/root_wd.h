#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void root_wd_start_tasks(void);

uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase);
/* === FIX INSERT-NO-DOUBLE-SYNC: force=true обходит SYNC_MIN_INTERVAL_MS === */
uint32_t send_sync_to_node_async_ex(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms, bool force,
                                    uint8_t *out_next_phase);
/* === END FIX INSERT-NO-DOUBLE-SYNC === */
uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_utc);

uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms);
void     apply_sync_at(uint8_t phase, uint32_t phase_ms,
                       uint32_t duration_ms, uint32_t apply_ms);

extern volatile bool s_cfg_ack;
extern volatile bool s_cfg_pushing;
extern volatile bool s_cfg_applied_and_started;
extern volatile bool s_cfg_req_pending;
extern volatile bool s_force_resync_pending;

/* === FIX CFG-FROM-ROOT: время последнего успешного cfg:ok === */
extern volatile uint32_t s_cfg_last_ok_ms;
/* === END FIX CFG-FROM-ROOT === */
void root_apply_new_config_from_mqtt(const char *json, int len);

#ifdef __cplusplus
}
#endif