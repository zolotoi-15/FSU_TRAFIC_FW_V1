#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void node_proto_register_callbacks(void);
void node_proto_client_start(const char *host, uint16_t port);
void node_proto_start_tasks(void);
void node_proto_start_root_tasks(void);
void node_handle_local_cmd(const char *cmd);

/* === FIX FAST-STATUS === */
void node_send_status_now(void);
void node_send_status_for_ms(uint32_t ms);
/* === END FIX FAST-STATUS === */

/* === FIX CFG-FROM-ROOT: диагностика === */
bool node_cfg_received(void);
bool node_cfg_confirmed_by_root(void);
/* === END FIX CFG-FROM-ROOT === */

#ifdef __cplusplus
}
#endif