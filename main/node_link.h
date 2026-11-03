/* ============================================================
   node_link.h — ЗАГЛУШКИ (TCP отключён, Wi-Fi нет).
   Все функции возвращают «нет соединения».
   Если где-то вызывается node_link_* — просто ничего не делает.
   ============================================================ */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Колбэки — не используются, но объявлены для совместимости */
typedef void (*node_json_cb_t)(const char *json, size_t len);
typedef void (*node_cfg_cb_t)(const char *json, size_t len);
typedef void (*node_cmd_cb_t)(const char *cmd, size_t len);
typedef void (*node_evt_cb_t)(const char *evt, size_t len);

static inline void node_link_set_cb(node_json_cb_t cb)     { (void)cb; }
static inline void node_link_set_cfg_cb(node_cfg_cb_t cb)  { (void)cb; }
static inline void node_link_set_cmd_cb(node_cmd_cb_t cb)  { (void)cb; }
static inline void node_link_set_evt_cb(node_evt_cb_t cb)  { (void)cb; }

/* ROOT: TCP-сервер — отключён */
static inline esp_err_t node_link_server_start(uint16_t port)
    { (void)port; return ESP_OK; }
static inline bool node_link_server_has_client(void) { return false; }
static inline esp_err_t node_link_server_send_config(const char *json, size_t len)
    { (void)json; (void)len; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t node_link_server_send_cmd(const char *cmd, size_t len)
    { (void)cmd; (void)len; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t node_link_server_send_cmd_nonblock(const char *cmd, size_t len)
    { (void)cmd; (void)len; return ESP_ERR_NOT_SUPPORTED; }

/* NODE: TCP-клиент — отключён */
static inline esp_err_t node_link_client_start(const char *host, uint16_t port)
    { (void)host; (void)port; return ESP_OK; }
static inline esp_err_t node_link_client_send(const char *json, size_t len)
    { (void)json; (void)len; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t node_link_client_send_evt(const char *evt, size_t len)
    { (void)evt; (void)len; return ESP_ERR_NOT_SUPPORTED; }
static inline bool node_link_client_connected(void) { return false; }

/* Общее */
static inline void node_link_stop(void) { }
static inline bool node_link_wait_for_client(uint32_t timeout_ms)
    { (void)timeout_ms; return false; }
static inline void node_link_force_reconnect(void) { }

#ifdef __cplusplus
}
#endif