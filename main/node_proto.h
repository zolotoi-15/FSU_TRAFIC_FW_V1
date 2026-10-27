#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Node: TCP-клиент к root, приём cmd:/cfg:, отправка status:/evt:. */
void node_proto_register_callbacks(void);
void node_proto_client_start(const char *host, uint16_t port);
void node_proto_start_tasks(void);