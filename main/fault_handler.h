#pragma once
#include "traffic_light.h"

#ifdef __cplusplus
extern "C" {
#endif

void fault_handler_register(void);

/* === SIMPLIFIED-NO-CFG: stubs для совместимости с traffic_internal.h ===
   traffic_light.c по-прежнему вызывает phase_change_cancel_pending()
   в force_phase/next/insert — реализация в fault_handler.c как no-op. */
uint8_t phase_change_take_pending(uint32_t *dur);
void    phase_change_requeue(uint8_t phase, uint32_t dur_ms);
void    phase_change_cancel_pending(void);

#ifdef __cplusplus
}
#endif