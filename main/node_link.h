#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Порт по умолчанию для root↔node TCP */
#ifndef NODE_LINK_PORT
#define NODE_LINK_PORT 3333
#endif

/* Колбэки */
typedef void (*node_json_cb_t)(const char *json, size_t len);
typedef void (*node_cfg_cb_t)(const char *json, size_t len);
typedef void (*node_cmd_cb_t)(const char *cmd, size_t len);
typedef void (*node_evt_cb_t)(const char *evt, size_t len);

void node_link_set_cb(node_json_cb_t cb);
void node_link_set_cfg_cb(node_cfg_cb_t cb);
void node_link_set_cmd_cb(node_cmd_cb_t cb);
void node_link_set_evt_cb(node_evt_cb_t cb);

/* ROOT: TCP-сервер */
esp_err_t node_link_server_start(uint16_t port);
bool      node_link_server_has_client(void);
esp_err_t node_link_server_send_config(const char *json, size_t len);
esp_err_t node_link_server_send_cmd(const char *cmd, size_t len);
esp_err_t node_link_server_send_cmd_nonblock(const char *cmd, size_t len);

/* NODE: TCP-клиент */
esp_err_t node_link_client_start(const char *host, uint16_t port);
esp_err_t node_link_client_send(const char *json, size_t len);
esp_err_t node_link_client_send_evt(const char *evt, size_t len);
bool      node_link_client_connected(void);

/* Общее */
void node_link_stop(void);

/* === ВАРИАНТ B: поэтапная инициализация === */
/* ROOT: ждать подключения node (семафор из server_task) */
bool node_link_wait_for_client(uint32_t timeout_ms);

/* ROOT: принудительно закрыть сокет с node (при смене канала AP) */
void node_link_force_reconnect(void);

#ifdef __cplusplus
}
#endif