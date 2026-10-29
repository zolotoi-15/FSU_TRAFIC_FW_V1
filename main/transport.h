#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef void (*transport_rx_cb_t)(const char *data, size_t len);

esp_err_t transport_init(void);
bool      transport_is_lora(void);
bool      transport_is_tcp(void);

esp_err_t transport_send_cmd(const char *cmd, size_t len);
esp_err_t transport_send_cfg(const char *json, size_t len);
esp_err_t transport_send_status(const char *json, size_t len);
esp_err_t transport_send_evt(const char *evt, size_t len);

void transport_set_cmd_cb(transport_rx_cb_t cb);
void transport_set_cfg_cb(transport_rx_cb_t cb);
void transport_set_status_cb(transport_rx_cb_t cb);
void transport_set_evt_cb(transport_rx_cb_t cb);

/* Ручное переключение (например, по команде MQTT) */
void transport_force_lora(void);
void transport_force_tcp(void);