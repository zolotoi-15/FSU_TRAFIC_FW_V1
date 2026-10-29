#pragma once

#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "traffic_light.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char     broker_url[128];
    uint16_t port;
    char     username[64];
    char     password[64];
    char     base_topic[64];
    char     lwt_msg[32];
} mqtt_config_t;

extern mqtt_config_t g_mqtt_config;

#define MQTT_PUBLISH_PERIOD_MS      1000
#define MQTT_STUCK_TIMEOUT_MS       (5 * 60 * 1000)
#define MQTT_PUBLISH_FAIL_THRESHOLD 3

#define MQTT_JSON_BUFFER_SIZE       4096   // было 8192
#define MQTT_JSON_INPUT_SIZE        16384  // было 32768 — 11 КБ JSON влезает

esp_err_t wifi_mqtt_init(void);
void      wifi_mqtt_stop(void);
void      wifi_mqtt_restart(void);

bool wifi_mqtt_is_connected(void);

esp_err_t wifi_mqtt_publish(const char *data);
void      wifi_mqtt_publish_now(void);

void wifi_mqtt_load_config(void);
void wifi_mqtt_save_config(void);

void mqtt_app_publish_wrapper(const char *msg);

/* ============================================================
   NODE JSON — основной топик <base>/Node
   ============================================================ */
void wifi_mqtt_publish_node_json(const char *json, size_t len);
void wifi_mqtt_publish_node_fault(const char *fault_str, size_t len);
void wifi_mqtt_apply_remote_fault(const char *fault_str, size_t len);
const char *wifi_mqtt_last_node_fault(void);

void wifi_mqtt_reset_node_fault_tracking(void);
void wifi_mqtt_set_fault_from_node(bool from_node);

/* FIX: COMM_LOST — root держит ALL_OFF из-за потери связи.
   Пока флаг ON, root игнорирует evt:fault:none / NODE.FAULT=NONE
   от node. */
void wifi_mqtt_set_root_comm_lost(bool lost);

/* FIX: MUTUAL_START — публичное применение конфига к runtime
   (пины, mode, START/STOP). Вызывается из app_main для root
   сразу после traffic_light_init, чтобы root применил пины
   и остался STOPPED, пока node не подключится. */
void wifi_mqtt_apply_config_runtime(void);

/* ============================================================
   Приоритет аварий (ALL_OFF > YELLOW_BLINK > NONE).
   ============================================================ */
int             wifi_mqtt_fault_priority_ex(traffic_fault_t f);
traffic_fault_t wifi_mqtt_get_node_fault(void);

void wifi_mqtt_notify_phase(uint8_t phase, uint32_t phase_ms,
                            uint32_t duration_ms);
void wifi_mqtt_set_phase_sync_enabled(bool en);

extern esp_mqtt_client_handle_t g_mqtt_client;

#ifdef __cplusplus
}
#endif