#pragma once

#include "wifi_mqtt.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "traffic_light.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   Общие extern-переменные (определены в wifi_mqtt.c).
   ============================================================ */
extern bool       s_mqtt_connected;
extern char       s_topic[160];
extern char       s_lwt_topic[192];
extern char       s_sub_topic[192];
extern char       s_json_topic[192];
extern char       s_layer_topic[192];
extern char       s_kp_topic[192];
extern char       s_node_json_topic[192];

extern TickType_t s_last_event_time;
extern int        s_publish_fail_count;

/* ============================================================
   Общие функции (из wifi_mqtt.c).
   ============================================================ */
void mqtt_restart_safe(void);   /* внутренний restart */
void mqtt_publish_now_safe(void);

/* ============================================================
   Внутренние функции модулей.
   ============================================================ */

/* mqtt_json.c */
int mqtt_build_json(char *json, size_t size);

/* mqtt_cmd.c */
void mqtt_handle_command(const char *data, int data_len);
void mqtt_handle_json_config(const char *data, int data_len);
void mqtt_json_accum_reset(void);
bool mqtt_json_accum_feed(const char *data, int data_len,
                          int offset, int total);
void mqtt_json_accum_process(void);

/* mqtt_fault.c */
void mqtt_fault_apply_remote(const char *fault_str, size_t len);
void mqtt_fault_publish_node(const char *fault_str, size_t len);
void mqtt_fault_publish_node_json(const char *json, size_t len);
void mqtt_fault_reset_tracking(void);
int  mqtt_fault_priority(traffic_fault_t f);
traffic_fault_t mqtt_fault_get_node(void);

/* mqtt_apply.c */
void mqtt_apply_config_runtime(void);
void mqtt_notify_phase(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif