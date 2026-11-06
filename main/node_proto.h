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

/* === FIX FAST-STATUS ===
   Форсировать немедленный STATUS (без ACK). */
void node_send_status_now(void);
/* Форсировать STATUS на N мс (без ACK), чтобы ROOT точно
   получил RUNNING после traffic:start. */
void node_send_status_for_ms(uint32_t ms);
/* === END FIX FAST-STATUS === */

#ifdef __cplusplus
}
#endif