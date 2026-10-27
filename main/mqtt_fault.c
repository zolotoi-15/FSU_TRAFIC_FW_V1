/* ============================================================
   mqtt_fault.c — зеркалирование fault между root и node.
   apply_remote_fault, publish_node_fault, publish_node_json,
   reset_node_fault_tracking.
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "node_link.h"
#include "traffic_light.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "mqtt_fault";

/* ============================================================
   Локальное состояние.
   ============================================================ */
char s_node_fault_json[1024] = {0};
static char s_last_node_fault_str[256] = {0};
static char s_last_applied_node_fault_mode[16] = {0};

static volatile traffic_fault_t s_node_fault = TRAFFIC_FAULT_NONE;
static bool s_fault_is_from_node = false;

static char    *s_node_json     = NULL;
static size_t   s_node_json_len = 0;
static SemaphoreHandle_t s_node_mutex = NULL;

static volatile bool s_root_comm_lost_all_off = false;

extern volatile bool s_fault_from_remote_ex;

/* ============================================================ */
int mqtt_fault_priority(traffic_fault_t f)
{
    switch (f) {
    case TRAFFIC_FAULT_ALL_OFF:      return 2;
    case TRAFFIC_FAULT_YELLOW_BLINK: return 1;
    default:                         return 0;
    }
}

traffic_fault_t mqtt_fault_get_node(void)
{
    return s_node_fault;
}

void wifi_mqtt_set_root_comm_lost(bool lost)
{
    s_root_comm_lost_all_off = lost;
    if (lost) ESP_LOGW(TAG, "root comm-lost flag = ON (ALL_OFF locked)");
    else      ESP_LOGW(TAG, "root comm-lost flag = OFF");
}

void wifi_mqtt_set_fault_from_node(bool from_node)
{
    s_fault_is_from_node = from_node;
}

/* ============================================================ */
/* publish_node_fault                                            */
/* ============================================================ */
void mqtt_fault_publish_node(const char *fault_str, size_t len)
{
    if (!s_mqtt_connected || !g_mqtt_client) return;
    char *out = malloc(len + 64);
    if (!out) return;
    int n = snprintf(out, len + 64,
        "{\"node_fault\":true,\"report\":\"%.*s\"}", (int)len, fault_str);
    if (n > 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic, out, n, 1, 0);
        ESP_LOGW(TAG, "Published node fault: %.*s", (int)len, fault_str);
    }
    free(out);
}

/* ============================================================ */
/* apply_remote_fault                                            */
/* ============================================================ */
void mqtt_fault_apply_remote(const char *fault_str, size_t len)
{
    if (len < 6 || strncmp(fault_str, "fault:", 6) != 0) return;

    const char *p = fault_str + 6;
    char mode[16] = {0};
    const char *c1 = memchr(p, ':', len - 6);
    if (!c1) return;
    size_t mlen = c1 - p;
    if (mlen >= sizeof(mode)) mlen = sizeof(mode) - 1;
    memcpy(mode, p, mlen);
    mode[mlen] = '\0';

    traffic_fault_t nf = TRAFFIC_FAULT_NONE;
    if (strcmp(mode, "all_off") == 0)      nf = TRAFFIC_FAULT_ALL_OFF;
    else if (strcmp(mode, "yellow_blink") == 0) nf = TRAFFIC_FAULT_YELLOW_BLINK;

    if (s_root_comm_lost_all_off && nf == TRAFFIC_FAULT_NONE) {
        ESP_LOGI(TAG, "remote fault NONE ignored (comm-lost ALL_OFF locked)");
        mqtt_fault_publish_node(fault_str, len);
        return;
    }

    s_node_fault = nf;

    traffic_fault_t root_fault = traffic_light_get_fault();

    traffic_fault_t combined = root_fault;
    if (mqtt_fault_priority(nf) > mqtt_fault_priority(combined))
        combined = nf;

    if (combined != root_fault) {
        ESP_LOGW(TAG, "remote fault='%s' → combined=%s (apply on root)",
                 mode,
                 combined == TRAFFIC_FAULT_ALL_OFF      ? "ALL_OFF" :
                 combined == TRAFFIC_FAULT_YELLOW_BLINK ? "YELLOW_BLINK" : "NONE");
        s_fault_is_from_node = (nf != TRAFFIC_FAULT_NONE);
        bool saved_remote = s_fault_from_remote_ex;
        s_fault_from_remote_ex = true;
        traffic_light_set_fault(combined);
        s_fault_from_remote_ex = saved_remote;

        if (combined == TRAFFIC_FAULT_ALL_OFF) {
            snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                     "fault:all_off:node");
        } else if (combined == TRAFFIC_FAULT_YELLOW_BLINK) {
            snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                     "fault:yellow_blink:node");
        } else {
            s_last_node_fault_str[0] = '\0';
            s_node_fault_json[0] = '\0';
        }
    }

    if (nf == TRAFFIC_FAULT_NONE &&
        root_fault == TRAFFIC_FAULT_YELLOW_BLINK &&
        !s_root_comm_lost_all_off) {
        ESP_LOGI(TAG, "remote fault cleared → root fault NONE");
        bool saved_remote = s_fault_from_remote_ex;
        s_fault_from_remote_ex = true;
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        s_fault_from_remote_ex = saved_remote;
        s_last_node_fault_str[0] = '\0';
        s_node_fault_json[0] = '\0';
        s_fault_is_from_node = false;
    }

    mqtt_fault_publish_node(fault_str, len);
    wifi_mqtt_publish_now();
}

/* ============================================================ */
/* publish_node_json — JSON от node → MQTT + парсинг FAULT       */
/* ============================================================ */
void mqtt_fault_publish_node_json(const char *json, size_t len)
{
    if (!json || len == 0) return;

    if (len < 20 || json[0] != '{' || !strstr(json, "\"role\":\"node\"")) {
        ESP_LOGW(TAG, "node JSON rejected: len=%u first='%c'",
                 (unsigned)len, (len > 0) ? json[0] : '?');
        return;
    }

    if (s_mqtt_connected && g_mqtt_client) {
        int id = esp_mqtt_client_publish(g_mqtt_client, s_node_json_topic,
                                         json, (int)len, 1, 0);
        if (id < 0) ESP_LOGW(TAG, "publish node JSON failed");
    }

    if (!s_node_mutex) s_node_mutex = xSemaphoreCreateMutex();
    if (!s_node_mutex || xSemaphoreTake(s_node_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return;

    if (s_node_json) free(s_node_json);
    s_node_json = malloc(len + 1);
    if (!s_node_json) { xSemaphoreGive(s_node_mutex); return; }

    memcpy(s_node_json, json, len);
    s_node_json[len] = '\0';
    s_node_json_len = len;

    const char *f = strstr(json, "\"FAULT\":");
    if (!f) { xSemaphoreGive(s_node_mutex); return; }

    const char *p = f + strlen("\"FAULT\":");
    if (*p != '{') { xSemaphoreGive(s_node_mutex); return; }

    int depth = 0;
    const char *obj_start = p;
    const char *q = p;
    while (*q) {
        if (*q == '{') depth++;
        else if (*q == '}') { depth--; if (depth == 0) { q++; break; } }
        q++;
    }
    const char *obj_end = q;

    size_t flen = (size_t)(obj_end - obj_start);
    if (flen < sizeof(s_node_fault_json)) {
        memcpy(s_node_fault_json, obj_start, flen);
        s_node_fault_json[flen] = '\0';
    }

    const char *new_mode = "NONE";
    const char *scan_start = obj_start + 1;
    const char *scan_end   = obj_end - 1;
    const char *mp = NULL;
    for (const char *s = scan_start; s + 7 <= scan_end; s++) {
        if (strncmp(s, "\"mode\":", 7) == 0) { mp = s; break; }
    }
    if (mp) {
        const char *q1 = mp + 7;
        while (q1 < scan_end && (*q1 == ' ' || *q1 == '\t')) q1++;
        if (q1 < scan_end && *q1 == '"') {
            q1++;
            const char *q2 = q1;
            while (q2 < scan_end && *q2 != '"') q2++;
            if (q2 < scan_end) {
                size_t mlen2 = (size_t)(q2 - q1);
                if (mlen2 == 7 && strncmp(q1, "ALL_OFF", 7) == 0)
                    new_mode = "ALL_OFF";
                else if (mlen2 == 12 && strncmp(q1, "YELLOW_BLINK", 12) == 0)
                    new_mode = "YELLOW_BLINK";
                else if (mlen2 == 4 && strncmp(q1, "NONE", 4) == 0)
                    new_mode = "NONE";
                else
                    ESP_LOGW(TAG, "unknown FAULT.mode: '%.*s'",
                             (int)mlen2, q1);
            }
        }
    }

    traffic_fault_t nf = TRAFFIC_FAULT_NONE;
    if (strcmp(new_mode, "ALL_OFF") == 0)      nf = TRAFFIC_FAULT_ALL_OFF;
    else if (strcmp(new_mode, "YELLOW_BLINK") == 0) nf = TRAFFIC_FAULT_YELLOW_BLINK;

    s_node_fault = nf;

    strncpy(s_last_applied_node_fault_mode, new_mode,
            sizeof(s_last_applied_node_fault_mode) - 1);
    s_last_applied_node_fault_mode[sizeof(s_last_applied_node_fault_mode) - 1] = '\0';

    if (g_config.role == DEVICE_ROLE_MASTER) {
        traffic_fault_t root_fault = traffic_light_get_fault();

        traffic_fault_t combined = root_fault;
        if (mqtt_fault_priority(nf) > mqtt_fault_priority(combined))
            combined = nf;

        if (combined != root_fault) {
            ESP_LOGW(TAG, "NODE.FAULT=%s, root=%s → combined=%s (apply on root)",
                     new_mode, traffic_light_fault_str(),
                     combined == TRAFFIC_FAULT_ALL_OFF      ? "ALL_OFF" :
                     combined == TRAFFIC_FAULT_YELLOW_BLINK ? "YELLOW_BLINK" : "NONE");
            s_fault_is_from_node = (nf != TRAFFIC_FAULT_NONE);
            bool saved_remote = s_fault_from_remote_ex;
            s_fault_from_remote_ex = true;
            traffic_light_set_fault(combined);
            s_fault_from_remote_ex = saved_remote;

            if (combined == TRAFFIC_FAULT_ALL_OFF) {
                snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                         "fault:all_off:node");
            } else if (combined == TRAFFIC_FAULT_YELLOW_BLINK) {
                snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                         "fault:yellow_blink:node");
            } else {
                s_last_node_fault_str[0] = '\0';
                s_node_fault_json[0] = '\0';
            }
        }

        if (s_root_comm_lost_all_off && nf == TRAFFIC_FAULT_NONE) {
            ESP_LOGI(TAG, "NODE.FAULT=NONE ignored (comm-lost ALL_OFF locked)");
        } else if (nf == TRAFFIC_FAULT_NONE &&
                   root_fault == TRAFFIC_FAULT_YELLOW_BLINK) {
            ESP_LOGI(TAG, "NODE.FAULT=NONE → root fault NONE");
            bool saved_remote = s_fault_from_remote_ex;
            s_fault_from_remote_ex = true;
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            s_fault_from_remote_ex = saved_remote;
            s_last_node_fault_str[0] = '\0';
            s_node_fault_json[0] = '\0';
            s_fault_is_from_node = false;
        }
    }

    xSemaphoreGive(s_node_mutex);
}

/* ============================================================ */
void mqtt_fault_reset_tracking(void)
{
    strncpy(s_last_applied_node_fault_mode, "NONE",
            sizeof(s_last_applied_node_fault_mode) - 1);
    s_last_applied_node_fault_mode[sizeof(s_last_applied_node_fault_mode) - 1] = '\0';
    s_last_node_fault_str[0] = '\0';
    s_node_fault_json[0] = '\0';
    s_fault_is_from_node = false;
    s_node_fault = TRAFFIC_FAULT_NONE;
    ESP_LOGI(TAG, "reset node fault tracking → NONE");
}