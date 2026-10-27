#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_LINK_PORT 3333

/* ============================================================
   Callbacks
   ============================================================ */
typedef void (*node_json_cb_t)(const char *json, size_t len);
void node_link_set_cb(node_json_cb_t cb);

typedef void (*node_cfg_cb_t)(const char *json, size_t len);
void node_link_set_cfg_cb(node_cfg_cb_t cb);

typedef void (*node_cmd_cb_t)(const char *cmd, size_t len);
void node_link_set_cmd_cb(node_cmd_cb_t cb);

typedef void (*node_evt_cb_t)(const char *evt, size_t len);
void node_link_set_evt_cb(node_evt_cb_t cb);

/* ============================================================
   ROOT: TCP-сервер
   ============================================================ */
esp_err_t node_link_server_start(uint16_t port);
bool      node_link_server_has_client(void);

/* cfg: — блокирующий, с таймаутом 500 мс (обязателен). */
esp_err_t node_link_server_send_config(const char *json, size_t len);

/* cmd: — блокирующий, с таймаутом 500 мс.
   Используется для traffic:start/stop и т.п. (важные команды). */
esp_err_t node_link_server_send_cmd(const char *cmd, size_t len);

/* FIX: cmd: — NON-BLOCK. Если мьютекс занят — возвращает
   ESP_ERR_TIMEOUT СРАЗУ. Используется для sync:/fault:
   (потеря кадра не критична — следующий придёт через 100-250 мс). */
esp_err_t node_link_server_send_cmd_nonblock(const char *cmd, size_t len);

/* ============================================================
   NODE: TCP-клиент
   ============================================================ */
esp_err_t node_link_client_start(const char *host, uint16_t port);
esp_err_t node_link_client_send(const char *json, size_t len);
esp_err_t node_link_client_send_evt(const char *evt, size_t len);
bool      node_link_client_connected(void);

void node_link_stop(void);

#ifdef __cplusplus
}
#endif