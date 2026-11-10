#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   node_proto.h — NODE/ROOT протокол.

   === SIMPLIFIED-NO-CFG ===
       NODE больше не загружает конфигурацию. ROOT шлёт готовые
       маски каналов через 'c:<phase>:<pm>:<on_hex>:<blink_hex>:<dur>'.
       Все функции cfg:* удалены/стали stub'ами для совместимости.
   ============================================================ */

void node_proto_register_callbacks(void);
void node_proto_client_start(const char *host, uint16_t port);
void node_proto_start_tasks(void);
void node_proto_start_root_tasks(void);
void node_handle_local_cmd(const char *cmd);

/* === FIX FAST-STATUS === */
void node_send_status_now(void);
void node_send_status_for_ms(uint32_t ms);
/* === END FIX FAST-STATUS === */

/* === SIMPLIFIED-NO-CFG: stub'ы для совместимости ===
   Внешние модули могут проверять «получен ли CFG», но в новой
   архитектуре CFG не передаётся вовсе — NODE всегда «готов». */
bool node_cfg_received(void);
bool node_cfg_confirmed_by_root(void);
/* === END SIMPLIFIED-NO-CFG === */

#ifdef __cplusplus
}
#endif