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

/* === FIX PED-DISPLAY-CLEAR-BEFORE-RETURN-HOLD === */
uint32_t send_channels_to_node_no_display(uint8_t phase, uint32_t phase_ms,
                                          uint32_t duration_ms, bool force);
/* === END FIX PED-DISPLAY-CLEAR-BEFORE-RETURN-HOLD === */

/* === FIX RT-CHANNELS ===
   UTC-тайминг: c: содержит apply_utc_ms — момент, когда и ROOT, и
   NODE должны переключить каналы. ToA/джиттер LoRa не влияют на
   синхронность — оба смотрят на UTC.

   apply_utc_ms == 0 → fallback на старое поведение (сразу по
   приходу c:, + lead_ms на ROOT). Используется, когда время ещё
   не синхронизировано (нет SNTP/RTC/DS1307). */
uint32_t send_sync_to_node_async_utc(uint8_t phase, uint32_t phase_ms,
                                     uint32_t duration_ms,
                                     uint64_t apply_utc_ms,
                                     bool force, uint8_t *out_next_phase);

uint32_t send_channels_to_node_utc(uint8_t phase, uint32_t phase_ms,
                                   uint32_t duration_ms,
                                   uint64_t apply_utc_ms, bool force);

/* Хелпер: вычислить ближайший apply_utc_ms = now + lead, кратный 10 мс.
   Возвращает 0, если UTC ещё не валиден. */
uint64_t root_compute_apply_utc_ms(void);
/* === END FIX RT-CHANNELS === */

extern volatile bool     s_cfg_ack;
extern volatile bool     s_cfg_pushing;
extern volatile bool     s_cfg_applied_and_started;
extern volatile bool     s_cfg_req_pending;
extern volatile bool     s_force_resync_pending;
extern volatile uint32_t s_cfg_last_ok_ms;

#ifdef __cplusplus
}
#endif