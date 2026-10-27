#pragma once
#include "esp_err.h"
#include "mesh_protocol.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MESH_ROLE_ROOT = 0,   /* бывший master */
    MESH_ROLE_NODE = 1,   /* бывший slave  */
} mesh_role_t;

esp_err_t mesh_link_init(mesh_role_t role, uint8_t channel);

/* API для root */
esp_err_t mesh_master_send_traffic(uint8_t action, uint8_t mode);
esp_err_t mesh_master_send_relay(uint8_t ch, bool state);
esp_err_t mesh_master_send_config(const char *json, size_t len);

/* API для node */
esp_err_t mesh_slave_send_status(void);

/* ✅ JSON node → root → MQTT */
esp_err_t mesh_node_send_json(const char *json, size_t len);

typedef void (*mesh_json_cb_t)(const uint8_t *from_mac,
                               const char *json, size_t len);
void mesh_set_json_cb(mesh_json_cb_t cb);

/* Диагностика */
int8_t         mesh_get_last_rssi(void);
const uint8_t *mesh_get_peer_mac(void);
bool           mesh_peer_connected(void);

typedef void (*mesh_status_cb_t)(const mesh_status_t *st);
void mesh_set_status_cb(mesh_status_cb_t cb);

uint8_t mesh_get_current_channel(void);
uint8_t mesh_get_real_wifi_channel(void);

/* root подключён к роутеру → можно публиковать MQTT */
bool mesh_is_root_connected(void);

/* FIX: экспорт layer для публикации в MQTT (как в примере internal_communication) */
int mesh_get_layer(void);
const char *mesh_get_ip(void);

#ifdef __cplusplus
}
#endif