#pragma once
#include "traffic_light.h"

#ifdef __cplusplus
extern "C" {
#endif

void fault_handler_register(void);

/* === FIX REQUEUE: phase_sync_task при ошибке send_sync возвращает
   pending-фазу через phase_change_requeue(), чтобы не потерять. === */
uint8_t phase_change_take_pending(uint32_t *dur);
void    phase_change_requeue(uint8_t phase, uint32_t dur_ms);

/* === FIX INSERT-NO-DOUBLE-SYNC: traffic_light.c insert гасит pending,
   чтобы phase_sync_task не отправил дубль sync: после apply_stage_dd. === */
void    phase_change_cancel_pending(void);
/* === END FIX INSERT-NO-DOUBLE-SYNC === */

#ifdef __cplusplus
}
#endif