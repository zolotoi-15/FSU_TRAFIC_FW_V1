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

#ifdef __cplusplus
}
#endif