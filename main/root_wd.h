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

/* === FIX MQTT-CFG-LIVE-UPDATE ===
   Обновление конфигурации через MQTT без перезагрузки.
   YELLOW_BLINK на ROOT+NODE → CFG на NODE → cfg:ok →
   STATUS RUNNING → нормальный режим. */
void root_apply_new_config_from_mqtt(const char *json, int len);
/* === END FIX MQTT-CFG-LIVE-UPDATE === */

#ifdef __cplusplus
}
#endif