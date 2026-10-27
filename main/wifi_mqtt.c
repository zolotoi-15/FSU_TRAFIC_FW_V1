/* ============================================================
   wifi_mqtt.c — публичный API + MQTT-клиент.
   Модули:
     mqtt_json.c   — build_json
     mqtt_cmd.c    — /Set + /SetJson
     mqtt_fault.c  — зеркалирование fault
     mqtt_apply.c  — apply_config_runtime, notify_phase
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "wifi_ap.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "wifi_mqtt";

esp_mqtt_client_handle_t g_mqtt_client = NULL;
mqtt_config_t g_mqtt_config;

/* ============================================================
   Определения общих переменных (extern в mqtt_internal.h).
   ============================================================ */
bool       s_mqtt_connected = false;
char       s_topic[160];
char       s_lwt_topic[192];
char       s_sub_topic[192];
char       s_json_topic[192];
char       s_layer_topic[192];
char       s_kp_topic[192];
char       s_node_json_topic[192];

TickType_t s_last_event_time = 0;
int        s_publish_fail_count = 0;

static bool       s_restart_in_progress = false;
static SemaphoreHandle_t s_restart_mutex = NULL;

#define NVS_NS_MQTT "mqtt_cfg"

/* ============================================================ */
void wifi_mqtt_load_config(void)
{
    nvs_handle_t nvs;
    mqtt_config_t def = {
        .broker_url = "mqtt://m2.wqtt.ru",
        .port       = 13257,
        .username   = "u_GGENLB",
        .password   = "LTHNW22D",
        .base_topic = "FSU_Trafic",
        .lwt_msg    = "offline"
    };
    memcpy(&g_mqtt_config, &def, sizeof(def));

    if (nvs_open(NVS_NS_MQTT, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len;
        len = sizeof(g_mqtt_config.broker_url);
        if (nvs_get_str(nvs, "broker_url", g_mqtt_config.broker_url, &len) != ESP_OK)
            strcpy(g_mqtt_config.broker_url, def.broker_url);
        uint16_t port = 0;
        if (nvs_get_u16(nvs, "port", &port) == ESP_OK) g_mqtt_config.port = port;
        len = sizeof(g_mqtt_config.username);
        if (nvs_get_str(nvs, "username", g_mqtt_config.username, &len) != ESP_OK)
            strcpy(g_mqtt_config.username, def.username);
        len = sizeof(g_mqtt_config.password);
        if (nvs_get_str(nvs, "password", g_mqtt_config.password, &len) != ESP_OK)
            strcpy(g_mqtt_config.password, def.password);
        len = sizeof(g_mqtt_config.base_topic);
        if (nvs_get_str(nvs, "base_topic", g_mqtt_config.base_topic, &len) != ESP_OK)
            strcpy(g_mqtt_config.base_topic, def.base_topic);
        len = sizeof(g_mqtt_config.lwt_msg);
        if (nvs_get_str(nvs, "lwt_msg", g_mqtt_config.lwt_msg, &len) != ESP_OK)
            strcpy(g_mqtt_config.lwt_msg, def.lwt_msg);
        nvs_close(nvs);
    } else {
        wifi_mqtt_save_config();
    }
}

void wifi_mqtt_save_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_MQTT, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "broker_url", g_mqtt_config.broker_url);
        nvs_set_u16(nvs, "port", g_mqtt_config.port);
        nvs_set_str(nvs, "username", g_mqtt_config.username);
        nvs_set_str(nvs, "password", g_mqtt_config.password);
        nvs_set_str(nvs, "base_topic", g_mqtt_config.base_topic);
        nvs_set_str(nvs, "lwt_msg", g_mqtt_config.lwt_msg);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void build_topics(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_topic, sizeof(s_topic),
             "%s_%02X%02X%02X%02X%02X%02X",
             g_mqtt_config.base_topic,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_lwt_topic,  sizeof(s_lwt_topic),  "%s/LWT",     s_topic);
    snprintf(s_sub_topic,  sizeof(s_sub_topic),  "%s/Set",     s_topic);
    snprintf(s_json_topic, sizeof(s_json_topic), "%s/SetJson", s_topic);
    snprintf(s_layer_topic, sizeof(s_layer_topic), "%s/Layer", s_topic);
    snprintf(s_kp_topic,    sizeof(s_kp_topic),    "%s/key_pressed", s_topic);
    snprintf(s_node_json_topic, sizeof(s_node_json_topic), "%s/Node", s_topic);

    ESP_LOGI(TAG, "Topic:      %s", s_topic);
    ESP_LOGI(TAG, "Subscribe:  %s", s_sub_topic);
    ESP_LOGI(TAG, "JSON in:    %s", s_json_topic);
    ESP_LOGI(TAG, "Node JSON:  %s", s_node_json_topic);
}

/* ============================================================
   Публичные обёртки (API из wifi_mqtt.h).
   ============================================================ */
int wifi_mqtt_fault_priority_ex(traffic_fault_t f)
{
    return mqtt_fault_priority(f);
}

traffic_fault_t wifi_mqtt_get_node_fault(void)
{
    return mqtt_fault_get_node();
}

void wifi_mqtt_publish_node_json(const char *json, size_t len)
{
    mqtt_fault_publish_node_json(json, len);
}

void wifi_mqtt_publish_node_fault(const char *fault_str, size_t len)
{
    mqtt_fault_publish_node(fault_str, len);
}

void wifi_mqtt_apply_remote_fault(const char *fault_str, size_t len)
{
    mqtt_fault_apply_remote(fault_str, len);
}

void wifi_mqtt_reset_node_fault_tracking(void)
{
    mqtt_fault_reset_tracking();
}

void wifi_mqtt_apply_config_runtime(void)
{
    mqtt_apply_config_runtime();
}

void wifi_mqtt_notify_phase(uint8_t phase, uint32_t phase_ms,
                            uint32_t duration_ms)
{
    mqtt_notify_phase(phase, phase_ms, duration_ms);
}

void wifi_mqtt_set_phase_sync_enabled(bool en)
{
    ESP_LOGI(TAG, "phase-sync flag=%d (ignored, sync: always on)", en);
}

const char *wifi_mqtt_last_node_fault(void)
{
    /* возвращаем пустую строку — s_last_node_fault_str в mqtt_fault.c */
    return "";
}

/* ============================================================
   MQTT event handler.
   ============================================================ */
static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    s_last_event_time = xTaskGetTickCount();

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_mqtt_connected = true;
        s_publish_fail_count = 0;
        esp_mqtt_client_publish(g_mqtt_client, s_lwt_topic, "online", 0, 1, 1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_sub_topic,  1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_json_topic, 1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_kp_topic,   0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        mqtt_json_accum_reset();
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        s_mqtt_connected = false;
        break;

    case MQTT_EVENT_DATA:
    {
        bool is_json_topic = (event->topic_len > 0 &&
                              event->topic_len <= (int)strlen(s_json_topic) &&
                              strncmp(event->topic, s_json_topic,
                                      event->topic_len) == 0);

        if (is_json_topic) {
            int total = event->total_data_len > 0
                        ? event->total_data_len : event->data_len;
            bool done = mqtt_json_accum_feed(event->data, event->data_len,
                                             event->current_data_offset, total);
            if (done) mqtt_json_accum_process();
            break;
        }

        if (event->topic_len > 0 &&
            event->topic_len <= (int)strlen(s_kp_topic) &&
            strncmp(event->topic, s_kp_topic, event->topic_len) == 0) {
            ESP_LOGW(TAG, "Keypressed via MQTT: %.*s",
                     event->data_len, event->data);
            break;
        }

        if (event->topic_len > 0 &&
            event->topic_len <= (int)strlen(s_sub_topic) &&
            strncmp(event->topic, s_sub_topic, event->topic_len) == 0) {
            mqtt_handle_command(event->data, event->data_len);
        }
        break;
    }
    default: break;
    }
}

/* ============================================================
   Restart / publish / tasks.
   ============================================================ */
void wifi_mqtt_restart(void)
{
    if (!s_restart_mutex) s_restart_mutex = xSemaphoreCreateMutex();
    if (!s_restart_mutex) return;
    if (xSemaphoreTake(s_restart_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) return;
    if (s_restart_in_progress) { xSemaphoreGive(s_restart_mutex); return; }
    s_restart_in_progress = true;

    if (g_mqtt_client) {
        esp_mqtt_client_stop(g_mqtt_client);
        esp_mqtt_client_destroy(g_mqtt_client);
        g_mqtt_client = NULL;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    s_mqtt_connected = false;
    s_publish_fail_count = 0;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri  = g_mqtt_config.broker_url,
        .broker.address.port = g_mqtt_config.port,
        .credentials.username = g_mqtt_config.username,
        .credentials.authentication.password = g_mqtt_config.password,
        .session.last_will.topic = s_lwt_topic,
        .session.last_will.msg = g_mqtt_config.lwt_msg,
        .session.last_will.msg_len = strlen(g_mqtt_config.lwt_msg),
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .buffer.size     = 16384,
        .buffer.out_size = MQTT_JSON_BUFFER_SIZE,
        .network.timeout_ms = 20000,
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .session.keepalive = 60,
        .task.stack_size = 8192,
        .task.priority = 6,
    };
    g_mqtt_client = esp_mqtt_client_init(&cfg);
    if (!g_mqtt_client) {
        s_restart_in_progress = false;
        xSemaphoreGive(s_restart_mutex);
        return;
    }
    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_mqtt_client);
    s_last_event_time = xTaskGetTickCount();
    s_restart_in_progress = false;
    xSemaphoreGive(s_restart_mutex);
}

esp_err_t wifi_mqtt_publish(const char *data)
{
    if (!s_mqtt_connected || !g_mqtt_client) return ESP_FAIL;
    int id = esp_mqtt_client_publish(g_mqtt_client, s_topic, data, 0, 1, 0);
    return (id >= 0) ? ESP_OK : ESP_FAIL;
}

void mqtt_app_publish_wrapper(const char *msg)
{
    if (!msg) return;
    if (!s_mqtt_connected || !g_mqtt_client) {
        ESP_LOGW(TAG, "key_pressed drop '%s'", msg);
        return;
    }
    int id = esp_mqtt_client_publish(g_mqtt_client, s_kp_topic, msg, 0, 1, 0);
    ESP_LOGI(TAG, "key_pressed -> %s: '%s' (id=%d)", s_kp_topic, msg, id);
}

void wifi_mqtt_publish_now(void)
{
    if (!s_mqtt_connected || !g_mqtt_client) return;
    char *json = malloc(MQTT_JSON_BUFFER_SIZE);
    if (!json) return;
    mqtt_build_json(json, MQTT_JSON_BUFFER_SIZE);
    wifi_mqtt_publish(json);
    free(json);
}

static void mqtt_publish_task(void *arg)
{
    char *json = malloc(MQTT_JSON_BUFFER_SIZE);
    char layer_ip[128];
    if (!json) { vTaskDelete(NULL); return; }
    ESP_LOGI(TAG, "mqtt_publish_task started");
    while (1) {
        if (s_mqtt_connected) {
            wifi_ap_record_t ap;
            int rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
            snprintf(layer_ip, sizeof(layer_ip),
                     "role:%s IP:%s rssi:%d ap:%s clients:%d",
                     g_config.role == DEVICE_ROLE_SLAVE ? "node" : "root",
                     wifi_get_ip(), rssi,
                     wifi_ap_is_running() ? wifi_get_ap_ssid() : "-",
                     wifi_ap_client_count());
            esp_mqtt_client_publish(g_mqtt_client, s_layer_topic, layer_ip, 0, 1, 0);
            mqtt_build_json(json, MQTT_JSON_BUFFER_SIZE);
            if (wifi_mqtt_publish(json) == ESP_OK) {
                s_publish_fail_count = 0;
            } else {
                s_publish_fail_count++;
                if (s_publish_fail_count >= MQTT_PUBLISH_FAIL_THRESHOLD) {
                    s_publish_fail_count = 0;
                    wifi_mqtt_restart();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MQTT_PUBLISH_PERIOD_MS));
    }
}

static void mqtt_watchdog_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(30000));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        TickType_t now = xTaskGetTickCount();
        int32_t since = (int32_t)(now - s_last_event_time);
        if (since < 0) since = 0;
        if (s_mqtt_connected) {
            if (since > MQTT_STUCK_TIMEOUT_MS) {
                wifi_mqtt_restart();
                s_last_event_time = xTaskGetTickCount();
            }
        } else {
            wifi_mqtt_restart();
            s_last_event_time = xTaskGetTickCount();
        }
    }
}

esp_err_t wifi_mqtt_init(void)
{
    wifi_mqtt_load_config();
    build_topics();
    wifi_mqtt_restart();
    xTaskCreate(mqtt_publish_task, "mqtt_pub", 8192, NULL, 4, NULL);
    xTaskCreate(mqtt_watchdog_task, "mqtt_wd", 4096, NULL, 3, NULL);
    return ESP_OK;
}

void wifi_mqtt_stop(void)
{
    if (g_mqtt_client) {
        esp_mqtt_client_stop(g_mqtt_client);
        esp_mqtt_client_destroy(g_mqtt_client);
        g_mqtt_client = NULL;
    }
    s_mqtt_connected = false;
    mqtt_json_accum_reset();
}

bool wifi_mqtt_is_connected(void) { return s_mqtt_connected; }