#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void root_wd_start_tasks(void);
void root_apply_new_config_from_mqtt(const char *json, int len);

/* === FIX SYNC-COMPAT === */
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

/* === FIX PED-DISPLAY === */
uint32_t send_channels_to_node(uint8_t phase, uint32_t phase_ms,
                               uint32_t duration_ms, bool force);
/* === END FIX PED-DISPLAY === */

/* === FIX PED-DISPLAY-CLEAR-BEFORE-RETURN-HOLD ===
   Принудительный c: без display (ch11). Вызывается из
   traffic_light.c в ветке return_to_hold после сброса
   s_insert_pending, чтобы NODE получил маску без ch11 и не
   ушёл в ALL_OFF. */
uint32_t send_channels_to_node_no_display(uint8_t phase, uint32_t phase_ms,
                                          uint32_t duration_ms, bool force);
/* === END FIX PED-DISPLAY-CLEAR-BEFORE-RETURN-HOLD === */

extern volatile bool     s_cfg_ack;
extern volatile bool     s_cfg_pushing;
extern volatile bool     s_cfg_applied_and_started;
extern volatile bool     s_cfg_req_pending;
extern volatile bool     s_force_resync_pending;
extern volatile uint32_t s_cfg_last_ok_ms;

#ifdef __cplusplus
}
#endif