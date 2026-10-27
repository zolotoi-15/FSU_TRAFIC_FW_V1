/* ============================================================
   mqtt_cmd.c — обработка MQTT-команд (/Set) и JSON-конфига (/SetJson).
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "wifi_ap.h"
#include "node_link.h"
#include "rs485.h"
#include "can_bus.h"
#include "shift_register.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "mqtt_cmd";

/* ============================================================
   JSON accumulator для /SetJson (многочастные сообщения).
   ============================================================ */
static char *s_json_accum        = NULL;
static int   s_json_accum_len    = 0;
static int   s_json_accum_total  = 0;
static bool  s_json_accum_active = false;

void mqtt_json_accum_reset(void)
{
    if (s_json_accum) { free(s_json_accum); s_json_accum = NULL; }
    s_json_accum_len = 0;
    s_json_accum_total = 0;
    s_json_accum_active = false;
}

/* Возвращает true, если сообщение полностью собрано. */
bool mqtt_json_accum_feed(const char *data, int data_len,
                          int offset, int total)
{
    if (offset == 0) {
        mqtt_json_accum_reset();
        int t = total > 0 ? total : data_len;
        if (t <= 0 || t > MQTT_JSON_INPUT_SIZE) return false;
        s_json_accum = calloc(1, t + 1);
        if (!s_json_accum) return false;
        s_json_accum_total  = t;
        s_json_accum_len    = 0;
        s_json_accum_active = true;
    }

    if (!s_json_accum_active) return false;

    if (offset < 0 || data_len <= 0 || offset + data_len > s_json_accum_total) {
        mqtt_json_accum_reset();
        return false;
    }

    memcpy(s_json_accum + offset, data, data_len);
    s_json_accum_len = offset + data_len;
    return (s_json_accum_len >= s_json_accum_total);
}

void mqtt_json_accum_process(void)
{
    if (!s_json_accum) return;
    s_json_accum[s_json_accum_total] = '\0';
    mqtt_handle_json_config(s_json_accum, s_json_accum_total);
    mqtt_json_accum_reset();
}

/* ============================================================
   handle_json_config — /SetJson
   FIX: config_load_json ВСЕГДА сохраняет в NVS (config_save).
   Затем root экспортирует и шлёт cfg: node.
   ============================================================ */
void mqtt_handle_json_config(const char *data, int data_len)
{
    if (data_len <= 0) { ESP_LOGW(TAG, "/SetJson: empty"); return; }
    if (data_len > MQTT_JSON_INPUT_SIZE) {
        ESP_LOGE(TAG, "/SetJson: too big");
        return;
    }

    /* FIX: дебаунс — если тот же размер приходит повторно < 2 с,
       игнорируем. Защита от MQTT-retain/reconnect-шторма. */
    static uint32_t s_last_json_ms  = 0;
    static int      s_last_json_len = 0;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_last_json_len == data_len &&
        s_last_json_ms != 0 &&
        (now_ms - s_last_json_ms) < 2000) {
        ESP_LOGW(TAG, "/SetJson: duplicate (%d bytes, %lu ms) — skip",
                 data_len, (unsigned long)(now_ms - s_last_json_ms));
        return;
    }
    s_last_json_ms  = now_ms;
    s_last_json_len = data_len;

    ESP_LOGI(TAG, "/SetJson: parsing %d bytes...", data_len);

    /* FIX: остановить traffic перед config_load_json, иначе
       traffic_task_loop читает полу-обновлённый stages[] во время
       парсинга и объявляет ложный stuck → YELLOW_BLINK. */
    bool was_running = (traffic_light_get_state() == TRAFFIC_STATE_RUNNING);
    traffic_light_stop();

    /* FIX: config_load_json ВСЕГДА сохраняет в NVS (config_save внутри).
       Это гарантирует, что новый конфиг (например min_recall) сохранён
       в NVS root и переживёт перезагрузку. */
    bool ok = config_load_json(data, data_len);
    if (ok) {
        ESP_LOGI(TAG, "/SetJson: config saved to NVS "
                      "(version=%.2f, ped_mode=%u)",
                 g_config.version,
                 g_config.traffic.pedestrian.request_mode);

        mqtt_apply_config_runtime();

        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"config\":\"loaded\",\"nvs\":\"ok\"}", 0, 1, 0);

        /* FIX: root → cfg: → node. config_export_json берёт NVS root,
           шлёт node, node делает config_load_json → NVS node. */
        if (g_config.role == DEVICE_ROLE_MASTER) {
            if (!node_link_server_has_client()) {
                ESP_LOGI(TAG, "node not connected → skip config forward");
            } else {
                extern bool config_export_json(char *b, size_t *l);
                char *buf = malloc(16384);
                size_t blen = 16384;
                if (buf && config_export_json(buf, &blen)) {
                    esp_err_t err = node_link_server_send_config(buf, blen);
                    if (err == ESP_OK) ESP_LOGI(TAG, "config forwarded: %u bytes",
                                                (unsigned)blen);
                    else ESP_LOGW(TAG, "config forward failed: %s",
                                  esp_err_to_name(err));
                }
                free(buf);
            }
        }
    } else {
        ESP_LOGE(TAG, "/SetJson: parse FAILED");
        if (was_running) {
            esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
            if (mr != ESP_OK) ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));
            traffic_light_reset();
            traffic_light_start();
        }
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"error\":\"json_parse\"}", 0, 1, 0);
    }
}

/* ============================================================
   handle_command — /Set
   ============================================================ */
void mqtt_handle_command(const char *data, int data_len)
{
    char cmd[256];
    int len = data_len < (int)sizeof(cmd) - 1 ? data_len : (int)sizeof(cmd) - 1;
    memcpy(cmd, data, len); cmd[len] = '\0';
    ESP_LOGI(TAG, "CMD: %s", cmd);

    /* Проксирование в node (кроме локальных команд root). */
    if (g_config.role == DEVICE_ROLE_MASTER &&
        strncmp(cmd, "nvs:erase",     9) != 0 &&
        strncmp(cmd, "role:",         5) != 0 &&
        strncmp(cmd, "config:reload", 13) != 0 &&
        strncmp(cmd, "sync:",         5) != 0 &&
        strncmp(cmd, "fault:",        6) != 0) {
        node_link_server_send_cmd(cmd, strlen(cmd));
    }

    int relay = -1, state = -1;
    bool changed = false;

    if (strncmp(cmd, "key_pressed:", 12) == 0) {
        ESP_LOGW(TAG, "Keypressed from: %s", cmd + 12);
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "net:info") == 0) {
        wifi_ap_record_t ap;
        int   rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
        uint8_t ch = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&ch, &sec);
        ESP_LOGI(TAG, "NET info: sta=%d ip=%s rssi=%d ch=%u ap_clients=%d",
                 wifi_sta_connected(), wifi_get_ip(), rssi, ch,
                 wifi_ap_client_count());
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "wifi:reconnect") == 0) {
        ESP_LOGW(TAG, "wifi:reconnect");
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_wifi_connect();
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "role:node") == 0) {
        ESP_LOGW(TAG, "role:node → reboot");
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"role\":\"node\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        config_save_role(DEVICE_ROLE_SLAVE);
        esp_restart();
    }
    if (strcmp(cmd, "role:root") == 0) {
        ESP_LOGW(TAG, "role:root → reboot");
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"role\":\"root\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        config_save_role(DEVICE_ROLE_MASTER);
        esp_restart();
    }

    if (sscanf(cmd, "relay:%d:%d", &relay, &state) == 2 ||
        sscanf(cmd, "sr:%d:%d",    &relay, &state) == 2) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
            if (state) sr_output_on(relay); else sr_output_off(relay);
            changed = true;
        }
        goto publish;
    }
    if (strcmp(cmd, "all:on") == 0)  { sr_all_on();  changed = true; goto publish; }
    if (strcmp(cmd, "all:off") == 0) { sr_all_off(); changed = true; goto publish; }

    if (sscanf(cmd, "toggle:%d", &relay) == 1) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
            sr_output_toggle(relay);
            changed = true;
        }
        goto publish;
    }

    if (strcmp(cmd, "traffic:start")  == 0) { traffic_light_start();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:stop")   == 0) { traffic_light_stop();   changed = true; goto publish; }
    if (strcmp(cmd, "traffic:pause")  == 0) { traffic_light_pause();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:resume") == 0) { traffic_light_resume(); changed = true; goto publish; }
    if (strcmp(cmd, "traffic:reset")  == 0) { traffic_light_reset();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:next")   == 0) { traffic_light_next_phase(); changed = true; goto publish; }

    {
        int mode = 0;
        if (sscanf(cmd, "traffic:mode:%d", &mode) == 1) {
            if (mode >= 1 && mode <= 3) {
                traffic_light_set_mode((traffic_mode_t)mode);
                changed = true;
            }
            goto publish;
        }
    }

    {
        int tch = -1;
        if (sscanf(cmd, "test:%d", &tch) == 1) {
            if (tch >= 0 && tch < SR_OUTPUT_COUNT)
                traffic_light_test_channel((uint8_t)tch);
            goto publish;
        }
    }

    if (strcmp(cmd, "pedestrian:request") == 0) {
        ESP_LOGI(TAG, "pedestrian:request (P7) mode=%u "
                      "(0=stage, 1=insert, 2=min_recall)",
                 g_config.traffic.pedestrian.request_mode);
        traffic_light_request_pedestrian();
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "rs485:stats") == 0) {
        rs485_stats_t st;
        rs485_get_stats(&st);
        ESP_LOGI(TAG, "RS485: rx_b=%lu rx_f=%lu ovf=%lu tx_b=%lu tx_err=%lu",
                 (unsigned long)st.rx_bytes, (unsigned long)st.rx_frames,
                 (unsigned long)st.rx_overflows, (unsigned long)st.tx_bytes,
                 (unsigned long)st.tx_errors);
        changed = true;
        goto publish;
    }
    if (strcmp(cmd, "rs485:reset_stats") == 0) {
        rs485_reset_stats();
        ESP_LOGI(TAG, "RS485 stats reset");
        changed = true;
        goto publish;
    }
    {
        int baud = 0;
        if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
            if (rs485_set_baudrate((uint32_t)baud) == ESP_OK)
                ESP_LOGI(TAG, "RS485 baud → %d", baud);
            changed = true;
            goto publish;
        }
        if (strncmp(cmd, "rs485:send:", 11) == 0) {
            const char *p = cmd + 11;
            size_t hex_len = strlen(p);
            if (hex_len > 0 && (hex_len % 2) == 0 && hex_len <= 512) {
                uint8_t buf[256];
                size_t n = hex_len / 2;
                bool ok = true;
                for (size_t i = 0; i < n; i++) {
                    int hi = -1, lo = -1;
                    char c1 = p[i * 2], c2 = p[i * 2 + 1];
                    if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                    else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                    else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                    if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                    else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                    else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                    if (hi < 0 || lo < 0) { ok = false; break; }
                    buf[i] = (uint8_t)((hi << 4) | lo);
                }
                if (ok) {
                    esp_err_t err = rs485_send(buf, n);
                    ESP_LOGI(TAG, "RS485 send %u bytes → %s",
                             (unsigned)n, esp_err_to_name(err));
                }
            }
            changed = true;
            goto publish;
        }
    }

    if (strcmp(cmd, "can:stats") == 0) {
        can_bus_stats_t cs;
        can_bus_get_stats(&cs);
        ESP_LOGI(TAG, "CAN: rx=%lu tx=%lu tx_err=%lu bus_off=%lu rec=%lu err_p=%lu state=%lu",
                 (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                 (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
                 (unsigned long)cs.recovery_count,
                 (unsigned long)cs.error_passive_count,
                 (unsigned long)cs.last_state);
        changed = true;
        goto publish;
    }
    if (strcmp(cmd, "can:reset_stats") == 0) {
        can_bus_reset_stats();
        ESP_LOGI(TAG, "CAN stats reset");
        changed = true;
        goto publish;
    }
    {
        unsigned int id = 0;
        char hex_data[32] = {0};
        int ext_flag = 0;
        int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d",
                             &id, hex_data, &ext_flag);
        if (matched >= 2 || (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
            uint8_t data[8] = {0};
            uint8_t dlc = 0;
            size_t hl = strlen(hex_data);
            if (hl > 0 && (hl % 2) == 0 && hl <= 16) {
                dlc = hl / 2;
                bool ok = true;
                for (size_t i = 0; i < dlc; i++) {
                    int hi = -1, lo = -1;
                    char c1 = hex_data[i * 2], c2 = hex_data[i * 2 + 1];
                    if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                    else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                    else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                    if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                    else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                    else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                    if (hi < 0 || lo < 0) { ok = false; break; }
                    data[i] = (uint8_t)((hi << 4) | lo);
                }
                if (ok) {
                    esp_err_t err = can_bus_send(id, data, dlc, ext_flag != 0);
                    ESP_LOGI(TAG, "CAN send id=0x%X dlc=%u ext=%d → %s",
                             id, dlc, ext_flag, esp_err_to_name(err));
                }
            } else {
                esp_err_t err = can_bus_send(id, NULL, 0, ext_flag != 0);
                ESP_LOGI(TAG, "CAN send id=0x%X (no data) → %s",
                         id, esp_err_to_name(err));
            }
            changed = true;
            goto publish;
        }
    }
    if (strcmp(cmd, "can:standby") == 0) {
        can_bus_set_standby(true);
        ESP_LOGI(TAG, "CAN → standby");
        changed = true;
        goto publish;
    }
    if (strcmp(cmd, "can:normal") == 0) {
        can_bus_set_standby(false);
        ESP_LOGI(TAG, "CAN → normal");
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "rtt") == 0) {
        extern int32_t  blink_get_rtt_samples(void);
        extern uint32_t blink_get_rtt_us(void);
        extern uint32_t blink_get_rtt_delay_us(void);
        ESP_LOGI(TAG, "RTT: us=%lu delay_us=%lu samples=%ld (%.3f ms)",
                 (unsigned long)blink_get_rtt_us(),
                 (unsigned long)blink_get_rtt_delay_us(),
                 (long)blink_get_rtt_samples(),
                 blink_get_rtt_delay_us() / 1000.0);
        changed = true;
        goto publish;
    }
    if (strcmp(cmd, "rtt:reset") == 0) {
        extern void blink_reset_rtt(void);
        blink_reset_rtt();
        ESP_LOGI(TAG, "RTT reset");
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "config:reload") == 0) {
        /* FIX: config:reload перечитывает NVS (не firmware config.json).
           NVS приоритет: firmware config.json грузится только при
           пустом NVS (первый старт / erase-flash).

           Если нужен firmware config.json — nvs:erase + reboot. */
        ESP_LOGI(TAG, "config:reload → reading NVS");
        traffic_light_stop();
        config_load();
        ESP_LOGI(TAG, "config:reload: version=%.2f, ped_mode=%u",
                 g_config.version,
                 g_config.traffic.pedestrian.request_mode);
        mqtt_apply_config_runtime();
        if (g_config.role == DEVICE_ROLE_MASTER && node_link_server_has_client()) {
            extern bool config_export_json(char *b, size_t *l);
            char *buf = malloc(16384);
            size_t blen = 16384;
            if (buf && config_export_json(buf, &blen)) {
                node_link_server_send_config(buf, blen);
            }
            free(buf);
        }
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "nvs:erase") == 0) {
        /* FIX: nvs:erase + reboot → NVS пуст → firmware config.json
           загрузится при следующем старте. */
        ESP_LOGW(TAG, "nvs:erase → reboot (next boot loads embedded config.json)");
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"nvs\":\"erasing\"}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        nvs_flash_erase();
        esp_restart();
    }

    ESP_LOGW(TAG, "Unknown: %s", cmd);
    return;

publish:
    if (changed) wifi_mqtt_publish_now();
}