#include "mqtt_internal.h"
#include "config_manager.h"
#include "wifi_ap.h"
#include "modem_ppp.h"
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
#include "time_sync.h" 

static const char *TAG = "wifi_mqtt";

esp_mqtt_client_handle_t g_mqtt_client = NULL;
mqtt_config_t g_mqtt_config;

bool       s_mqtt_connected = false;
char       s_topic[160];
char       s_lwt_topic[192];
char       s_sub_topic[192];
char       s_json_topic[192];
char       s_layer_topic[192];
char       s_kp_topic[192];
char       s_node_json_topic[192];
/* === FIX T_CHANNELS === */
char       s_node_channels_topic[192];
/* === END FIX T_CHANNELS === */

TickType_t s_last_event_time = 0;
int        s_publish_fail_count = 0;

static bool       s_restart_in_progress = false;
static SemaphoreHandle_t s_restart_mutex = NULL;

static bool s_mqtt_tasks_started = false;

#define MQTT_DOWN_RESTART_MS    (30 * 1000)
#define MQTT_FORCE_MODEM_MS     (5 * 60 * 1000)

static volatile uint32_t s_mqtt_last_connected_ms    = 0;
static volatile uint32_t s_mqtt_last_disconnected_ms = 0;
static volatile uint32_t s_mqtt_down_since_ms        = 0;

char       s_node_json_topic[192];
/* === FIX MQTT-EVENTS === */
char       s_events_topic[192];
/* === END FIX MQTT-EVENTS === */
char       s_node_channels_topic[192];

static portMUX_TYPE s_mqtt_state_mtx = portMUX_INITIALIZER_UNLOCKED;

static inline void mqtt_set_down_since(uint32_t ms)
{
    portENTER_CRITICAL(&s_mqtt_state_mtx);
    s_mqtt_down_since_ms = ms;
    portEXIT_CRITICAL(&s_mqtt_state_mtx);
}

static inline uint32_t mqtt_get_down_since(void)
{
    uint32_t v;
    portENTER_CRITICAL(&s_mqtt_state_mtx);
    v = s_mqtt_down_since_ms;
    portEXIT_CRITICAL(&s_mqtt_state_mtx);
    return v;
}

/* === FIX MQTT-EVENTS ===
   Публикует событие аварии/восстановления в <base>/Events.
   Формат: {"ts":"2026-11-10T12:34:56.789Z","type":"fault",
            "detail":"short to GND (SR=1, V_IN=0, I=0)","role":"root"} */
void wifi_mqtt_publish_event(const char *type, const char *detail,
                             const void *data, size_t data_len)
{
    if (!s_mqtt_connected || !g_mqtt_client) return;
    if (!type) return;

    uint32_t sec = 0; uint16_t ms = 0;
    char ts[40] = "1970-01-01T00:00:00.000Z";
    if (time_sync_get(&sec, &ms)) {
        time_t t = (time_t)sec;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        char date[24] = {0};
        strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &tm_utc);
        snprintf(ts, sizeof(ts), "%s.%03uZ", date, (unsigned)(ms % 1000));
    }

    char buf[512];
    int n;
    if (data && data_len > 0 && data_len < 256) {
        n = snprintf(buf, sizeof(buf),
                     "{\"ts\":\"%s\",\"type\":\"%s\",\"detail\":\"%s\","
                     "\"data\":\"%.*s\",\"role\":\"%s\"}",
                     ts, type, detail ? detail : "",
                     (int)data_len, (const char *)data,
                     g_config.role == DEVICE_ROLE_MASTER ? "root" : "node");
    } else {
        n = snprintf(buf, sizeof(buf),
                     "{\"ts\":\"%s\",\"type\":\"%s\",\"detail\":\"%s\","
                     "\"role\":\"%s\"}",
                     ts, type, detail ? detail : "",
                     g_config.role == DEVICE_ROLE_MASTER ? "root" : "node");
    }
    if (n > 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_events_topic, buf, n, 1, 1);
        ESP_LOGI(TAG, "EVENT → %s: %s", s_events_topic, buf);
    }
}
/* === END FIX MQTT-EVENTS === */

#define NVS_NS_MQTT "mqtt_cfg"

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

static void build_broker_uri(char *out, size_t out_len)
{
    const char *uri = g_mqtt_config.broker_url;
    uint16_t port = g_mqtt_config.port;

    if (!uri || !*uri) {
        snprintf(out, out_len, "mqtt://127.0.0.1:%u", port);
        return;
    }

    const char *scheme_end = strstr(uri, "://");
    const char *host_start = scheme_end ? scheme_end + 3 : uri;
    const char *host_end   = strchr(host_start, '/');
    if (!host_end) host_end = host_start + strlen(host_start);

    const char *last_colon = NULL;
    for (const char *p = host_start; p < host_end; p++) {
        if (*p == ':') last_colon = p;
    }

    if (last_colon) {
        snprintf(out, out_len, "%s", uri);
    } else {
        size_t host_len = (size_t)(host_end - uri);
        size_t rest_len = strlen(host_end);
        if (host_len + 1 + 5 + rest_len + 1 > out_len) {
            snprintf(out, out_len, "%s", uri);
            return;
        }
        memcpy(out, uri, host_len);
        out[host_len] = '\0';
        snprintf(out + host_len, out_len - host_len, ":%u%s", port, host_end);
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

    snprintf(s_lwt_topic,  sizeof(s_lwt_topic),  "%s/LWT",         s_topic);
    snprintf(s_sub_topic,  sizeof(s_sub_topic),  "%s/Set",         s_topic);
    snprintf(s_json_topic, sizeof(s_json_topic), "%s/SetJson",     s_topic);
    snprintf(s_layer_topic, sizeof(s_layer_topic), "%s/Layer",     s_topic);
    snprintf(s_kp_topic,    sizeof(s_kp_topic),    "%s/key_pressed", s_topic);
    snprintf(s_node_json_topic, sizeof(s_node_json_topic), "%s/Node", s_topic);
    /* === FIX T_CHANNELS === */
    snprintf(s_node_channels_topic, sizeof(s_node_channels_topic),
             "%s/Node/Channels", s_topic);
    /* === END FIX T_CHANNELS === */

    ESP_LOGI(TAG, "Topic:      %s", s_topic);
    ESP_LOGI(TAG, "Subscribe:  %s", s_sub_topic);
    ESP_LOGI(TAG, "JSON in:    %s", s_json_topic);
    ESP_LOGI(TAG, "Node JSON:  %s", s_node_json_topic);
    /* === FIX T_CHANNELS === */
    ESP_LOGI(TAG, "NodeChannels: %s", s_node_channels_topic);
    /* === END FIX T_CHANNELS === */
}

int wifi_mqtt_fault_priority_ex(traffic_fault_t f) { return mqtt_fault_priority(f); }
traffic_fault_t wifi_mqtt_get_node_fault(void) { return mqtt_fault_get_node(); }
void wifi_mqtt_publish_node_json(const char *json, size_t len) {
    mqtt_fault_publish_node_json(json, len);
}
void wifi_mqtt_publish_node_fault(const char *fault_str, size_t len) {
    mqtt_fault_publish_node(fault_str, len);
}
void wifi_mqtt_apply_remote_fault(const char *fault_str, size_t len) {
    mqtt_fault_apply_remote(fault_str, len);
}
void wifi_mqtt_reset_node_fault_tracking(void) { mqtt_fault_reset_tracking(); }
void wifi_mqtt_apply_config_runtime(void) { mqtt_apply_config_runtime(); }
void wifi_mqtt_notify_phase(uint8_t phase, uint32_t phase_ms,
                            uint32_t duration_ms) {
    mqtt_notify_phase(phase, phase_ms, duration_ms);
}
void wifi_mqtt_set_phase_sync_enabled(bool en) {
    ESP_LOGI(TAG, "phase-sync flag=%d (ignored, sync: always on)", en);
}
const char *wifi_mqtt_last_node_fault(void) { return ""; }

/* === FIX T_CHANNELS === */
void wifi_mqtt_publish_node_channels(const char *json, size_t len) {
    mqtt_fault_publish_node_channels(json, len);
}
/* === END FIX T_CHANNELS === */

uint32_t wifi_mqtt_get_down_ms(void)
{
    if (s_mqtt_connected) return 0;
    uint32_t down_since = mqtt_get_down_since();
    if (down_since == 0) return 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now < down_since) return 0;
    return now - down_since;
}

void wifi_mqtt_force_restart_modem(void)
{
    ESP_LOGE(TAG, "wifi_mqtt: force restart modem (PPP stop)");
    modem_ppp_stop();
}

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
        s_mqtt_last_connected_ms = (uint32_t)(esp_timer_get_time() / 1000);
        mqtt_set_down_since(0);
        esp_mqtt_client_publish(g_mqtt_client, s_lwt_topic, "online", 0, 1, 1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_sub_topic,  1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_json_topic, 1);
        esp_mqtt_client_subscribe(g_mqtt_client, s_kp_topic,   0);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT subscribed msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        s_mqtt_last_disconnected_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (mqtt_get_down_since() == 0) {
            mqtt_set_down_since(s_mqtt_last_disconnected_ms);
        }
        mqtt_json_accum_reset();
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        s_mqtt_connected = false;
        if (mqtt_get_down_since() == 0) {
            mqtt_set_down_since((uint32_t)(esp_timer_get_time() / 1000));
        }
        break;

    case MQTT_EVENT_DATA:
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA: topic='%.*s' data_len=%d "
                      "total_data_len=%d offset=%d",
                 event->topic_len, event->topic,
                 event->data_len, event->total_data_len,
                 event->current_data_offset);

        bool is_json_topic = (event->topic_len > 0 &&
                              event->topic_len <= (int)strlen(s_json_topic) &&
                              strncmp(event->topic, s_json_topic,
                                      event->topic_len) == 0);

        if (is_json_topic) {
            int total = event->total_data_len > 0
                        ? event->total_data_len : event->data_len;
            bool done = mqtt_json_accum_feed(event->data, event->data_len,
                                             event->current_data_offset, total);
            ESP_LOGI(TAG, "accum_feed: offset=%d len=%d total=%d done=%d",
                     event->current_data_offset, event->data_len,
                     total, (int)done);
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

void wifi_mqtt_restart(void)
{
    extern bool modem_ppp_is_connected(void);
    if (!modem_ppp_is_connected()) {
        ESP_LOGW(TAG, "wifi_mqtt_restart: no PPP IP → skip");
        return;
    }

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

    static char s_broker_uri[192];
    build_broker_uri(s_broker_uri, sizeof(s_broker_uri));
    ESP_LOGI(TAG, "MQTT connect → %s", s_broker_uri);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri  = s_broker_uri,
        .credentials.username = g_mqtt_config.username,
        .credentials.authentication.password = g_mqtt_config.password,
        .session.last_will.topic = s_lwt_topic,
        .session.last_will.msg = g_mqtt_config.lwt_msg,
        .session.last_will.msg_len = strlen(g_mqtt_config.lwt_msg),
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .buffer.size     = 8192,
        .buffer.out_size = 4096,
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
        extern const char *modem_ppp_get_ip(void);
        extern bool modem_ppp_is_connected(void);
        bool ppp_up = modem_ppp_is_connected();

        if (ppp_up && s_mqtt_connected) {
            const char *ip = modem_ppp_get_ip();
            snprintf(layer_ip, sizeof(layer_ip),
                     "role:%s IP:%s ppp:%d",
                     g_config.role == DEVICE_ROLE_SLAVE ? "node" : "root",
                     ip,
                     ppp_up ? 1 : 0);
            esp_mqtt_client_publish(g_mqtt_client, s_layer_topic,
                                    layer_ip, 0, 1, 0);
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

        TickType_t now_tick = xTaskGetTickCount();
        int32_t since = (int32_t)(now_tick - s_last_event_time);
        if (since < 0) since = 0;

        extern bool modem_ppp_is_connected(void);
        bool ppp_up = modem_ppp_is_connected();

        if (s_mqtt_connected) {
            if (since > MQTT_STUCK_TIMEOUT_MS) {
                ESP_LOGW(TAG, "MQTT stuck (no events %ld ms) → restart",
                         (long)since);
                if (ppp_up) {
                    wifi_mqtt_restart();
                } else {
                    ESP_LOGW(TAG, "MQTT stuck but PPP down → skip restart");
                }
                s_last_event_time = xTaskGetTickCount();
            }
        } else {
            uint32_t down_ms = wifi_mqtt_get_down_ms();

            if (!ppp_up) {
                if (down_ms > MQTT_FORCE_MODEM_MS) {
                    ESP_LOGE(TAG, "MQTT down %lu ms + PPP down → "
                                  "force restart modem",
                             (unsigned long)down_ms);
                    wifi_mqtt_force_restart_modem();
                    mqtt_set_down_since((uint32_t)(esp_timer_get_time() / 1000));
                    s_last_event_time = xTaskGetTickCount();
                } else {
                    ESP_LOGD(TAG, "MQTT down %lu ms, PPP down → "
                                  "wait modem_retry", (unsigned long)down_ms);
                }
                continue;
            }

            if (down_ms > MQTT_FORCE_MODEM_MS) {
                ESP_LOGE(TAG, "MQTT down %lu ms (> %d) → force restart modem",
                         (unsigned long)down_ms, MQTT_FORCE_MODEM_MS);
                wifi_mqtt_force_restart_modem();
                mqtt_set_down_since((uint32_t)(esp_timer_get_time() / 1000));
                s_last_event_time = xTaskGetTickCount();
            } else if (down_ms > MQTT_DOWN_RESTART_MS) {
                ESP_LOGW(TAG, "MQTT down %lu ms (> %d) → "
                              "wifi_mqtt_restart() (fast)",
                         (unsigned long)down_ms, MQTT_DOWN_RESTART_MS);
                wifi_mqtt_restart();
                s_last_event_time = xTaskGetTickCount();
            } else if (since > MQTT_STUCK_TIMEOUT_MS) {
                ESP_LOGW(TAG, "MQTT down %ld ms → wifi_mqtt_restart()",
                         (long)since);
                wifi_mqtt_restart();
                s_last_event_time = xTaskGetTickCount();
            }
        }
    }
}

esp_err_t wifi_mqtt_init(void)
{
    if (g_config.role != DEVICE_ROLE_MASTER) {
        ESP_LOGI(TAG, "wifi_mqtt_init: NODE — MQTT disabled");
        return ESP_OK;
    }

    extern bool modem_ppp_is_connected(void);
    if (!modem_ppp_is_connected()) {
        ESP_LOGW(TAG, "wifi_mqtt_init: no PPP IP yet → defer");
        return ESP_ERR_INVALID_STATE;
    }

    wifi_mqtt_load_config();
    build_topics();
    wifi_mqtt_restart();

    if (!s_mqtt_tasks_started) {
        s_mqtt_tasks_started = true;
        xTaskCreate(mqtt_publish_task, "mqtt_pub", 8192, NULL, 4, NULL);
        xTaskCreate(mqtt_watchdog_task, "mqtt_wd", 4096, NULL, 3, NULL);
        ESP_LOGI(TAG, "wifi_mqtt: tasks created (mqtt_pub + mqtt_wd)");
    } else {
        ESP_LOGI(TAG, "wifi_mqtt: tasks already running, skip");
    }

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

void wifi_mqtt_publish_comm_lost(uint32_t lost_ms)
{
    if (!s_mqtt_connected || !g_mqtt_client) {
        ESP_LOGW(TAG, "publish_comm_lost: MQTT not connected");
        return;
    }
    char msg[256];
    int n = snprintf(msg, sizeof(msg),
        "{\"error\":\"comm_lost\","
        "\"role\":\"%s\","
        "\"lost_ms\":%lu,"
        "\"channels\":\"ALL_OFF\","
        "\"action\":\"recovering\"}",
        g_config.role == DEVICE_ROLE_MASTER ? "root" : "node",
        (unsigned long)lost_ms);
    if (n > 0) {
        int id = esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                         msg, n, 1, 0);
        ESP_LOGW(TAG, "COMM_LOST published (id=%d): %s", id, msg);
    }
}

void wifi_mqtt_publish_comm_restored(void)
{
    if (!s_mqtt_connected || !g_mqtt_client) return;
    char msg[192];
    int n = snprintf(msg, sizeof(msg),
        "{\"status\":\"comm_restored\","
        "\"role\":\"%s\","
        "\"action\":\"start_with_red\"}",
        g_config.role == DEVICE_ROLE_MASTER ? "root" : "node");
    if (n > 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic, msg, n, 1, 0);
        ESP_LOGI(TAG, "COMM_RESTORED published: %s", msg);
    }
}

bool wifi_mqtt_is_connected(void) { return s_mqtt_connected; }