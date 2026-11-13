/* ============================================================
   mqtt_fault.c — mirror fault между ROOT и NODE (через MQTT).

   === FIX FAULT-LOCAL-FORMAT ===
   NODE шлёт fault:local:<MODE> без маски (follow_root).
   ROOT→NODE: fault:yellow_blink:0x00A0:1,2:reason1,reason2.

   === FIX NODE-STATUS-ROOT-FORMAT ===
   STATUS от NODE в формате ROOT + "role":"node" в хвосте.

   === FIX LINK-LOSS-8S-NO-FALSE-ALLOFF ===
   s_root_comm_lost_all_off блокирует снятие ALL_OFF через
   mqtt_fault_apply_remote(NONE). Снимается только через
   wifi_mqtt_set_root_comm_lost(false) — вызывается в
   root_wd.c → root_node_watchdog_task в ветках
   wait_node_ack (node_confirmed || timeout) и
   has_node && !was_connected (success).

   Проблема: если ROOT в ALL_OFF из-за FIX LINK-LOSS-8S и
   параллельно NODE восстановился (своё ALL_OFF → RUNNING),
   NODE шлёт STATUS с FAULT.mode="NONE". mqtt_fault_apply_remote
   видит nf==NONE, s_root_comm_lost_all_off==true → IGNORE.
   ROOT остаётся в ALL_OFF, пока свой watchdog не выйдет через
   wait_node_ack → timeout (3 с). Это работает, но 3 с задержки.

   Дополнительно: если NODE шлёт FAULT.mode="ALL_OFF" (его
   собственный watchdog), nf==ALL_OFF — блокировка не срабатывает,
   ROOT применяет ALL_OFF у себя. Это правильно (симметрия).

   s_root_comm_lost_all_off оставляем как защиту от «ложного
   NONE» — NODE мог сам выставить ALL_OFF из-за root_lost, и его
   NONE в первые секунды после восстановления связи не должен
   снимать ALL_OFF на ROOT, пока ROOT сам не решит (через свой
   watchdog-таймер).
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "mqtt_fault";

char s_node_fault_json[1024] = {0};
static char s_last_node_fault_str[256] = {0};

static volatile traffic_fault_t s_node_fault = TRAFFIC_FAULT_NONE;
static bool s_fault_is_from_node = false;

static portMUX_TYPE s_node_fault_mtx = portMUX_INITIALIZER_UNLOCKED;

static inline traffic_fault_t mqtt_get_node_fault_internal(void) {
    traffic_fault_t v;
    portENTER_CRITICAL(&s_node_fault_mtx);
    v = s_node_fault;
    portEXIT_CRITICAL(&s_node_fault_mtx);
    return v;
}
static inline void mqtt_set_node_fault_internal(traffic_fault_t f) {
    portENTER_CRITICAL(&s_node_fault_mtx);
    s_node_fault = f;
    portEXIT_CRITICAL(&s_node_fault_mtx);
}

static volatile bool s_root_comm_lost_all_off = false;
extern volatile bool s_fault_from_remote_ex;

int mqtt_fault_priority(traffic_fault_t f) {
    switch (f) {
    case TRAFFIC_FAULT_ALL_OFF:      return 2;
    case TRAFFIC_FAULT_YELLOW_BLINK: return 1;
    default:                         return 0;
    }
}

traffic_fault_t mqtt_fault_get_node(void) { return mqtt_get_node_fault_internal(); }

void wifi_mqtt_set_root_comm_lost(bool lost) {
    s_root_comm_lost_all_off = lost;
    ESP_LOGW(TAG, "root comm-lost = %s", lost ? "ON (ALL_OFF locked)" : "OFF");
}
void wifi_mqtt_set_fault_from_node(bool from_node) { s_fault_is_from_node = from_node; }

static void apply_fault_from_remote(traffic_fault_t f) {
    bool saved = s_fault_from_remote_ex;
    s_fault_from_remote_ex = true;
    traffic_light_set_fault(f);
    s_fault_from_remote_ex = saved;
}

void mqtt_fault_publish_node(const char *fault_str, size_t len) {
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

/* ============================================================
   parse_fault_report
   === FIX FAULT-LOCAL-FORMAT ===
   ============================================================ */
static void parse_fault_report(const char *fault_str, size_t len,
                               traffic_fault_t *out_nf, char *out_mode, size_t mode_sz)
{
    *out_nf = TRAFFIC_FAULT_NONE;
    if (mode_sz > 0) out_mode[0] = '\0';
    snprintf(out_mode, mode_sz, "NONE");

    if (len < 6 || strncmp(fault_str, "fault:", 6) != 0) return;

    const char *p    = fault_str + 6;
    const char *end  = fault_str + len;
    const char *c1   = memchr(p, ':', end - p);

    const char *mode_start;
    if (c1) {
        size_t first_len = c1 - p;
        if (first_len == 5 && strncmp(p, "local", 5) == 0) {
            mode_start = c1 + 1;
        } else {
            mode_start = p;
        }
    } else {
        mode_start = p;
    }

    const char *mode_end = memchr(mode_start, ':', end - mode_start);
    if (!mode_end) mode_end = end;

    size_t mlen = mode_end - mode_start;
    if (mlen >= mode_sz) mlen = mode_sz - 1;
    memcpy(out_mode, mode_start, mlen);
    out_mode[mlen] = '\0';

    for (size_t i = 0; i < mlen; i++) {
        if (out_mode[i] >= 'A' && out_mode[i] <= 'Z')
            out_mode[i] = out_mode[i] - 'A' + 'a';
    }

    if (strcmp(out_mode, "all_off") == 0)           *out_nf = TRAFFIC_FAULT_ALL_OFF;
    else if (strcmp(out_mode, "yellow_blink") == 0) *out_nf = TRAFFIC_FAULT_YELLOW_BLINK;
    else if (strcmp(out_mode, "none") == 0)         *out_nf = TRAFFIC_FAULT_NONE;
}
/* === END FIX FAULT-LOCAL-FORMAT === */

void mqtt_fault_apply_remote(const char *fault_str, size_t len) {
    if (!fault_str || len < 6) return;

    char mode[24];
    traffic_fault_t nf = TRAFFIC_FAULT_NONE;
    parse_fault_report(fault_str, len, &nf, mode, sizeof(mode));

    /* === FIX LINK-LOSS-8S-NO-FALSE-ALLOFF ===
       Оставляем защиту: s_root_comm_lost_all_off блокирует
       только ложный NONE (NODE мог сам выставить ALL_OFF из-за
       root_lost и сразу прислать NONE — это не повод снимать
       ALL_OFF на ROOT, пока ROOT сам не выйдет через свой
       watchdog-таймер).

       Снятие — через wifi_mqtt_set_root_comm_lost(false) из
       root_wd.c → root_node_watchdog_task
       (wait_node_ack → node_confirmed || timeout, и
        has_node && !was_connected → success).

       ALL_OFF от NODE (nf==ALL_OFF) — не блокируется
       (симметрия, ROOT применяет ALL_OFF у себя). */
    if (s_root_comm_lost_all_off && nf == TRAFFIC_FAULT_NONE) {
        ESP_LOGI(TAG, "remote fault NONE ignored (comm-lost ALL_OFF locked, "
                      "will clear via ROOT watchdog)");
        mqtt_fault_publish_node(fault_str, len);
        return;
    }
    /* === END FIX LINK-LOSS-8S-NO-FALSE-ALLOFF === */

    mqtt_set_node_fault_internal(nf);
    traffic_fault_t root_fault = traffic_light_get_fault();
    traffic_fault_t combined = root_fault;
    if (mqtt_fault_priority(nf) > mqtt_fault_priority(combined))
        combined = nf;

    traffic_fault_t to_apply = TRAFFIC_FAULT_NONE;
    bool do_apply = false;

    if (combined != root_fault) {
        ESP_LOGW(TAG, "remote fault='%s' → combined=%s", mode,
                 combined == TRAFFIC_FAULT_ALL_OFF      ? "ALL_OFF" :
                 combined == TRAFFIC_FAULT_YELLOW_BLINK ? "YELLOW_BLINK" : "NONE");
        s_fault_is_from_node = (nf != TRAFFIC_FAULT_NONE);
        if (combined == TRAFFIC_FAULT_ALL_OFF)
            snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                     "fault:all_off:node");
        else if (combined == TRAFFIC_FAULT_YELLOW_BLINK)
            snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                     "fault:yellow_blink:node");
        else
            s_last_node_fault_str[0] = '\0';
        to_apply = combined;
        do_apply = true;
    }

    if (nf == TRAFFIC_FAULT_NONE &&
        root_fault != TRAFFIC_FAULT_NONE &&
        !s_root_comm_lost_all_off) {
        ESP_LOGI(TAG, "remote fault cleared (root was %s) → root NONE",
                 traffic_light_fault_str());
        s_last_node_fault_str[0] = '\0';
        s_fault_is_from_node = false;
        to_apply = TRAFFIC_FAULT_NONE;
        do_apply = true;
    }

    if (do_apply) apply_fault_from_remote(to_apply);
    mqtt_fault_publish_node(fault_str, len);
    wifi_mqtt_publish_now();
}

/* ============================================================
   parse_node_json_fault
   ============================================================ */
static bool parse_node_json_fault(const char *json, traffic_fault_t *out_nf,
                                  char *out_mode, size_t mode_sz) {
    *out_nf = TRAFFIC_FAULT_NONE;
    snprintf(out_mode, mode_sz, "NONE");

    const char *f = strstr(json, "\"FAULT\":");
    if (!f) return false;
    const char *p = f + 8;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '{') return false;

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

    const char *mp = NULL;
    for (const char *s = obj_start + 1; s + 7 <= obj_end - 1; s++) {
        if (strncmp(s, "\"mode\":", 7) == 0) { mp = s; break; }
    }
    if (!mp) return true;
    const char *q1 = mp + 7;
    while (q1 < obj_end - 1 && (*q1 == ' ' || *q1 == '\t')) q1++;
    if (*q1 != '"') return true;
    q1++;
    const char *q2 = q1;
    while (q2 < obj_end - 1 && *q2 != '"') q2++;
    if (q2 >= obj_end - 1) return true;
    size_t mlen = (size_t)(q2 - q1);
    if (mlen >= mode_sz) mlen = mode_sz - 1;
    memcpy(out_mode, q1, mlen);
    out_mode[mlen] = '\0';

    if (strcmp(out_mode, "ALL_OFF") == 0)           *out_nf = TRAFFIC_FAULT_ALL_OFF;
    else if (strcmp(out_mode, "YELLOW_BLINK") == 0) *out_nf = TRAFFIC_FAULT_YELLOW_BLINK;
    return true;
}

/* ============================================================
   mqtt_fault_publish_node_json
   === FIX NODE-STATUS-ROOT-FORMAT ===
   ============================================================ */
void mqtt_fault_publish_node_json(const char *json, size_t len) {
    if (!json || len == 0) return;

    if (len < 20 || json[0] != '{' || !strstr(json, "\"role\":\"node\"")) {
        ESP_LOGW(TAG, "node JSON rejected: len=%u first='%c'",
                 (unsigned)len, json[0]);
        return;
    }

    if (s_mqtt_connected && g_mqtt_client) {
        int id = esp_mqtt_client_publish(g_mqtt_client, s_node_json_topic,
                                         json, (int)len, 1, 0);
        if (id < 0) ESP_LOGW(TAG, "publish node JSON to %s failed", s_node_json_topic);
        else        ESP_LOGI(TAG, "published node JSON → %s (%u bytes)",
                             s_node_json_topic, (unsigned)len);
    } else {
        ESP_LOGW(TAG, "publish node JSON: MQTT not connected");
    }

    char new_mode[16];
    traffic_fault_t nf = TRAFFIC_FAULT_NONE;
    if (!parse_node_json_fault(json, &nf, new_mode, sizeof(new_mode))) {
        return;
    }

    mqtt_set_node_fault_internal(nf);
    traffic_fault_t to_apply = TRAFFIC_FAULT_NONE;
    bool do_apply = false;

    if (g_config.role == DEVICE_ROLE_MASTER) {
        traffic_fault_t root_fault = traffic_light_get_fault();
        traffic_fault_t combined = root_fault;
        if (mqtt_fault_priority(nf) > mqtt_fault_priority(combined))
            combined = nf;

        if (combined != root_fault) {
            ESP_LOGW(TAG, "NODE.FAULT=%s, root=%s → combined=%s (apply)",
                     new_mode, traffic_light_fault_str(),
                     combined == TRAFFIC_FAULT_ALL_OFF      ? "ALL_OFF" :
                     combined == TRAFFIC_FAULT_YELLOW_BLINK ? "YELLOW_BLINK" : "NONE");
            s_fault_is_from_node = (nf != TRAFFIC_FAULT_NONE);
            if (combined == TRAFFIC_FAULT_ALL_OFF)
                snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                         "fault:all_off:node");
            else if (combined == TRAFFIC_FAULT_YELLOW_BLINK)
                snprintf(s_last_node_fault_str, sizeof(s_last_node_fault_str),
                         "fault:yellow_blink:node");
            else
                s_last_node_fault_str[0] = '\0';
            to_apply = combined;
            do_apply = true;
        }

        if (s_root_comm_lost_all_off && nf == TRAFFIC_FAULT_NONE) {
            ESP_LOGI(TAG, "NODE.FAULT=NONE ignored (comm-lost ALL_OFF locked)");
        } else if (nf == TRAFFIC_FAULT_NONE &&
                   root_fault != TRAFFIC_FAULT_NONE) {
            ESP_LOGI(TAG, "NODE.FAULT=NONE (root was %s) → root NONE",
                     traffic_light_fault_str());
            s_last_node_fault_str[0] = '\0';
            s_fault_is_from_node = false;
            to_apply = TRAFFIC_FAULT_NONE;
            do_apply = true;
        }
    }

    if (do_apply) apply_fault_from_remote(to_apply);
}

/* === FIX T_CHANNELS === */
void mqtt_fault_publish_node_channels(const char *json, size_t len)
{
    if (!json || len == 0) return;
    if (!s_mqtt_connected || !g_mqtt_client) {
        ESP_LOGW(TAG, "publish node channels: MQTT not connected");
        return;
    }
    int id = esp_mqtt_client_publish(g_mqtt_client, s_node_channels_topic,
                                     json, (int)len, 1, 1);
    if (id < 0) {
        ESP_LOGW(TAG, "publish node channels failed");
    } else {
        ESP_LOGI(TAG, "published node channels → %s (%u bytes)",
                 s_node_channels_topic, (unsigned)len);
    }
}
/* === END FIX T_CHANNELS === */

void mqtt_fault_reset_tracking(void) {
    s_last_node_fault_str[0] = '\0';
    s_node_fault_json[0] = '\0';
    s_fault_is_from_node = false;
    mqtt_set_node_fault_internal(TRAFFIC_FAULT_NONE);
    ESP_LOGI(TAG, "reset node fault tracking → NONE");
}