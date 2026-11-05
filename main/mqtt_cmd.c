#include "mqtt_internal.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "wifi_ap.h"
#include "transport.h"
#include "rs485.h"
#include "can_bus.h"
#include "shift_register.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "modem_ppp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "mqtt_cmd";
#define NVS_NS_PENDING   "pending_cfg"
#define NVS_KEY_LEN      "len"
#define NVS_KEY_CHUNK    "chunk_%d"
#define PENDING_CHUNK_SZ 4096

static char *s_json_accum        = NULL;
static int   s_json_accum_len    = 0;
static int   s_json_accum_total  = 0;
static bool  s_json_accum_active = false;

void mqtt_json_accum_reset(void) {
    if (s_json_accum) { free(s_json_accum); s_json_accum = NULL; }
    s_json_accum_len = 0;
    s_json_accum_total = 0;
    s_json_accum_active = false;
}

bool mqtt_json_accum_feed(const char *data, int data_len, int offset, int total) {
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

static bool looks_like_config(const char *json, int len) {
    if (!json || len < 2) return false;
    if (json[0] != '{') return false;
    if (strstr(json, "\"meta\"") || strstr(json, "\"traffic\"") ||
        strstr(json, "\"shift_register\"") || strstr(json, "\"mqtt\""))
        return true;
    return false;
}

static esp_err_t save_pending_cfg_to_nvs(const char *data, int data_len) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(nvs, NVS_KEY_LEN, (uint32_t)data_len);
    if (err != ESP_OK) { nvs_close(nvs); return err; }

    char key[16];
    int n_chunks = 0;
    for (int off = 0; off < data_len; off += PENDING_CHUNK_SZ) {
        int chunk = (data_len - off > PENDING_CHUNK_SZ) ? PENDING_CHUNK_SZ : (data_len - off);
        snprintf(key, sizeof(key), NVS_KEY_CHUNK, off / PENDING_CHUNK_SZ);
        err = nvs_set_blob(nvs, key, data + off, chunk);
        if (err != ESP_OK) { nvs_close(nvs); return err; }
        n_chunks++;
    }
    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) ESP_LOGI(TAG, "save_pending_cfg OK (%d bytes, %d chunks)", data_len, n_chunks);
    return err;
}

void mqtt_json_accum_process(void) {
    if (!s_json_accum || s_json_accum_total <= 0) {
        mqtt_json_accum_reset();
        return;
    }
    if (!looks_like_config(s_json_accum, s_json_accum_total)) {
        ESP_LOGW(TAG, "accum_process: not a config (%d bytes), skip", s_json_accum_total);
        mqtt_json_accum_reset();
        return;
    }
    esp_err_t err = save_pending_cfg_to_nvs(s_json_accum, s_json_accum_total);
    mqtt_json_accum_reset();
    if (err != ESP_OK) {
        if (s_mqtt_connected && g_mqtt_client)
            esp_mqtt_client_publish(g_mqtt_client, s_topic, "{\"error\":\"nvs_save\"}", 0, 1, 0);
        return;
    }
    if (s_mqtt_connected && g_mqtt_client) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"config\":\"pending\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "rebooting to apply config");
    esp_restart();
}

void mqtt_handle_json_config(const char *data, int data_len) {
    if (data_len <= 0 || data_len > MQTT_JSON_INPUT_SIZE) return;
    if (!looks_like_config(data, data_len)) {
        ESP_LOGW(TAG, "/SetJson: not a config (%d bytes)", data_len);
        return;
    }
    static uint32_t s_last_ms = 0;
    static int      s_last_len = 0;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_last_len == data_len && s_last_ms != 0 && (now_ms - s_last_ms) < 2000) {
        ESP_LOGW(TAG, "/SetJson: duplicate, skip");
        return;
    }
    s_last_ms = now_ms; s_last_len = data_len;

    esp_err_t err = save_pending_cfg_to_nvs(data, data_len);
    if (err != ESP_OK) {
        if (s_mqtt_connected && g_mqtt_client)
            esp_mqtt_client_publish(g_mqtt_client, s_topic, "{\"error\":\"nvs_save\"}", 0, 1, 0);
        return;
    }
    if (s_mqtt_connected && g_mqtt_client) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"config\":\"pending\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "/SetJson: rebooting to apply config");
    esp_restart();
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static bool parse_hex(const char *p, uint8_t *buf, size_t max, size_t *out_n) {
    size_t hl = strlen(p);
    if (hl == 0 || (hl % 2) != 0 || hl / 2 > max) return false;
    size_t n = hl / 2;
    for (size_t i = 0; i < n; i++) {
        int hi = hex_nibble(p[i*2]);
        int lo = hex_nibble(p[i*2+1]);
        if (hi < 0 || lo < 0) return false;
        buf[i] = (uint8_t)((hi << 4) | lo);
    }
    *out_n = n;
    return true;
}

void mqtt_handle_command(const char *data, int data_len) {
    char cmd[256];
    int len = data_len < (int)sizeof(cmd) - 1 ? data_len : (int)sizeof(cmd) - 1;
    memcpy(cmd, data, len); cmd[len] = '\0';
    ESP_LOGI(TAG, "CMD: %s", cmd);

    if (g_config.role == DEVICE_ROLE_MASTER &&
        strncmp(cmd, "nvs:erase", 9) != 0 &&
        strncmp(cmd, "role:", 5) != 0 &&
        strncmp(cmd, "config:reload", 13) != 0 &&
        strncmp(cmd, "sync:", 5) != 0 &&
        strncmp(cmd, "fault:", 6) != 0) {
        transport_send_cmd(cmd, strlen(cmd));
    }

    int relay = -1, state = -1;
    bool changed = false;

    if (strncmp(cmd, "key_pressed:", 12) == 0) { changed = true; goto publish; }

    /* === FIX: без esp_wifi_* (Wi-Fi не инициализирован) === */
    if (strcmp(cmd, "net:info") == 0) {
        ESP_LOGI(TAG, "NET: ppp=%d ip=%s transport=%s",
                 modem_ppp_is_connected() ? 1 : 0,
                 modem_ppp_get_ip(),
                 transport_is_lora() ? "LoRa" : "TCP");
        changed = true; goto publish;
    }
    /* === END FIX === */

    if (strcmp(cmd, "transport:lora") == 0) { transport_force_lora(); changed = true; goto publish; }
    if (strcmp(cmd, "transport:tcp")  == 0) { transport_force_tcp();  changed = true; goto publish; }

    if (strcmp(cmd, "role:node") == 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic, "{\"role\":\"node\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        config_save_role(DEVICE_ROLE_SLAVE);
        esp_restart();
    }
    if (strcmp(cmd, "role:root") == 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic, "{\"role\":\"root\",\"reboot\":true}", 0, 1, 0);
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
    if (strcmp(cmd, "all:on")  == 0) { sr_all_on();  changed = true; goto publish; }
    if (strcmp(cmd, "all:off") == 0) { sr_all_off(); changed = true; goto publish; }
    if (sscanf(cmd, "toggle:%d", &relay) == 1) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) { sr_output_toggle(relay); changed = true; }
        goto publish;
    }
    if (strcmp(cmd, "traffic:start")  == 0) { traffic_light_start();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:stop")   == 0) { traffic_light_stop();   changed = true; goto publish; }
    if (strcmp(cmd, "traffic:pause")  == 0) { traffic_light_pause();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:resume") == 0) { traffic_light_resume(); changed = true; goto publish; }
    if (strcmp(cmd, "traffic:reset")  == 0) { traffic_light_reset();  changed = true; goto publish; }
    if (strcmp(cmd, "traffic:next")   == 0) { traffic_light_next_phase(); changed = true; goto publish; }
    { int mode = 0;
      if (sscanf(cmd, "traffic:mode:%d", &mode) == 1) {
          if (mode >= 1 && mode <= 3) {
              if (traffic_light_set_mode((traffic_mode_t)mode) == ESP_OK) {
                  changed = true;
              } else {
                  /* === FIX: сообщаем об ошибке === */
                  if (s_mqtt_connected && g_mqtt_client)
                      esp_mqtt_client_publish(g_mqtt_client, s_topic,
                          "{\"error\":\"mode_while_running\"}", 0, 1, 0);
              }
          }
          goto publish;
      }
    }
    { int tch = -1;
      if (sscanf(cmd, "test:%d", &tch) == 1) {
          if (tch >= 0 && tch < SR_OUTPUT_COUNT) traffic_light_test_channel((uint8_t)tch);
          goto publish;
      }
    }
    if (strcmp(cmd, "pedestrian:request") == 0) {
        traffic_light_request_pedestrian(); changed = true; goto publish;
    }
    if (strcmp(cmd, "rs485:stats") == 0) {
        rs485_stats_t st; rs485_get_stats(&st);
        ESP_LOGI(TAG, "RS485: rx_b=%lu rx_f=%lu ovf=%lu tx_b=%lu tx_err=%lu",
                 (unsigned long)st.rx_bytes, (unsigned long)st.rx_frames,
                 (unsigned long)st.rx_overflows, (unsigned long)st.tx_bytes,
                 (unsigned long)st.tx_errors);
        changed = true; goto publish;
    }
    if (strcmp(cmd, "rs485:reset_stats") == 0) { rs485_reset_stats(); changed = true; goto publish; }
    { int baud = 0;
      if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
          rs485_set_baudrate((uint32_t)baud); changed = true; goto publish;
      }
      if (strncmp(cmd, "rs485:send:", 11) == 0) {
          uint8_t buf[256]; size_t n = 0;
          if (parse_hex(cmd + 11, buf, sizeof(buf), &n)) rs485_send(buf, n);
          changed = true; goto publish;
      }
    }
    if (strcmp(cmd, "can:stats") == 0) {
        can_bus_stats_t cs; can_bus_get_stats(&cs);
        ESP_LOGI(TAG, "CAN: rx=%lu tx=%lu tx_err=%lu bus_off=%lu rec=%lu err_p=%lu state=%lu",
                 (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                 (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
                 (unsigned long)cs.recovery_count,
                 (unsigned long)cs.error_passive_count,
                 (unsigned long)cs.last_state);
        changed = true; goto publish;
    }
    if (strcmp(cmd, "can:reset_stats") == 0) { can_bus_reset_stats(); changed = true; goto publish; }
    { unsigned int id = 0; char hex_data[32] = {0}; int ext_flag = 0;
      int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d", &id, hex_data, &ext_flag);
      if (matched >= 2 || (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
          uint8_t data[8] = {0}; size_t dlc = 0;
          parse_hex(hex_data, data, sizeof(data), &dlc);
          can_bus_send(id, data, (uint8_t)dlc, ext_flag != 0);
          changed = true; goto publish;
      }
    }
    if (strcmp(cmd, "can:standby") == 0) { can_bus_set_standby(true);  changed = true; goto publish; }
    if (strcmp(cmd, "can:normal")  == 0) { can_bus_set_standby(false); changed = true; goto publish; }

    if (strcmp(cmd, "rtt") == 0) {
        extern int32_t  blink_get_rtt_samples(void);
        extern uint32_t blink_get_rtt_us(void);
        extern uint32_t blink_get_rtt_delay_us(void);
        ESP_LOGI(TAG, "RTT: us=%lu delay_us=%lu samples=%ld",
                 (unsigned long)blink_get_rtt_us(),
                 (unsigned long)blink_get_rtt_delay_us(),
                 (long)blink_get_rtt_samples());
        changed = true; goto publish;
    }
    if (strcmp(cmd, "rtt:reset") == 0) {
        extern void blink_reset_rtt(void);
        blink_reset_rtt(); changed = true; goto publish;
    }
    if (strcmp(cmd, "config:reload") == 0) {
        ESP_LOGI(TAG, "config:reload → reading NVS");
        traffic_light_stop();
        config_load();
        mqtt_apply_config_runtime();
        if (g_config.role == DEVICE_ROLE_MASTER) {
            extern bool config_export_json(char *b, size_t *l);
            char *buf = malloc(16384);
            size_t blen = 16384;
            if (buf) {
                if (config_export_json(buf, &blen)) transport_send_cfg(buf, blen);
                else ESP_LOGW(TAG, "config_export_json failed");
                free(buf);
            }
        }
        changed = true; goto publish;
    }
    if (strcmp(cmd, "nvs:erase") == 0) {
        ESP_LOGW(TAG, "nvs:erase → reboot (with WDT reset before erase)");
        if (s_mqtt_connected && g_mqtt_client) {
            esp_mqtt_client_publish(g_mqtt_client, s_topic,
                "{\"nvs\":\"erasing\"}", 0, 1, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        /* === FIX: nvs_flash_erase может блокировать 1-2 с — WDT reset === */
        esp_task_wdt_reset();
        nvs_flash_erase();
        esp_restart();
    }
    ESP_LOGW(TAG, "Unknown: %s", cmd);
    return;

publish:
    if (changed) wifi_mqtt_publish_now();
}