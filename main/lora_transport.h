// lora_transport.h
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef void (*lora_rx_cb_t)(const char *payload, size_t len);
typedef void (*lora_evt_cb_t)(const char *evt, size_t len);

esp_err_t lora_transport_init(void);
esp_err_t lora_transport_send_cmd(const char *cmd, size_t len);
esp_err_t lora_transport_send_cfg(const char *json, size_t len);
esp_err_t lora_transport_send_status(const char *json, size_t len);
esp_err_t lora_transport_send_evt(const char *evt, size_t len);
void      lora_transport_set_cmd_cb(lora_rx_cb_t cb);
void      lora_transport_set_cfg_cb(lora_rx_cb_t cb);
void      lora_transport_set_status_cb(lora_rx_cb_t cb);
void      lora_transport_set_evt_cb(lora_evt_cb_t cb);
bool      lora_transport_is_ready(void);