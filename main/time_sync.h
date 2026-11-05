#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t time_sync_init(void);
bool time_sync_get(uint32_t *unix_sec, uint16_t *ms);
void time_sync_set(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us);
esp_err_t time_sync_start_sntp(void);
void time_sync_node_report(void);

/* === FIX UTC: обычный push с 30-секундным троттлингом === */
void time_sync_root_push(uint32_t rtt_us);

/* === FIX TIME-FIRST: force=true обходит троттлинг.
   Используется при первом sync: после старта NODE,
   чтобы NODE получил time: до первого sync:. === */
void time_sync_root_push_force(uint32_t rtt_us);

/* === FIX #3a: дельта ROOT/NODE (по последнему sync:) === */
void     time_sync_note_delta_ms(int32_t delta_ms);
int32_t  time_sync_get_delta_ms(void);
uint32_t time_sync_get_delta_age_ms(void);   /* для диагностики */
/* === END FIX #3a === */

#ifdef __cplusplus
}
#endif