/* ============================================================
   ble_config.c — парсинг JSON-команд из BLE.
   ============================================================ */

#include "ble_config.h"
#include "ble_transport.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "lora_nvs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLE_CFG";

static void ble_send_resp(const char *type, const char *rid,
                          const char *detail, cJSON *data)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "type", type ? type : "resp");
    if (rid) cJSON_AddStringToObject(root, "rid", rid);
    if (detail) cJSON_AddStringToObject(root, "detail", detail);
    if (data) cJSON_AddItemToObject(root, "data", data);
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!out) return;

    /* === FIX BLE-NAME-CLASH === */
    fsu_ble_notify_response(out, strlen(out));
    /* === END FIX BLE-NAME-CLASH === */
   if (fsu_ble_is_connected()) {
        fsu_ble_notify_response(out, strlen(out));
    } else {
        ESP_LOGW(TAG, "resp dropped (not connected): %s", out);
    }
    free(out);
}

void ble_config_handle_cmd(const char *json, size_t len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) { ble_send_resp("error", NULL, "invalid_json", NULL); return; }

    const char *type = cJSON_GetStringValue(cJSON_GetObjectItem(root, "type"));
    const char *rid  = cJSON_GetStringValue(cJSON_GetObjectItem(root, "rid"));
    if (!type) {
        ble_send_resp("error", rid, "no_type", NULL);
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type, "config.get") == 0) {
        char *cfg = malloc(8192);
        if (!cfg) { ble_send_resp("error", rid, "no_mem", NULL); cJSON_Delete(root); return; }
        size_t clen = 8192;
        if (config_export_json(cfg, &clen)) {
            cJSON *j = cJSON_ParseWithLength(cfg, clen);
            ble_send_resp("config.get.ok", rid, NULL, j);
        } else {
            ble_send_resp("error", rid, "config_export_fail", NULL);
        }
        free(cfg);
    }
    else if (strcmp(type, "config.set") == 0) {
        cJSON *data = cJSON_GetObjectItem(root, "data");
        if (!data) { ble_send_resp("error", rid, "no_data", NULL); cJSON_Delete(root); return; }
        char *str = cJSON_PrintUnformatted(data);
        if (str) {
            if (config_load_json(str, strlen(str))) {
                ble_send_resp("config.set.ok", rid, NULL, NULL);
                ESP_LOGI(TAG, "config.set applied via BLE");
            } else {
                ble_send_resp("error", rid, "config_parse_fail", NULL);
            }
            free(str);
        }
    }
    else if (strcmp(type, "lora.get") == 0) {
        lora_cfg_nvs_t cfg;
        if (lora_nvs_load(&cfg)) {
            cJSON *j = cJSON_CreateObject();
            cJSON_AddNumberToObject(j, "freq_hz", cfg.freq_hz);
            cJSON_AddNumberToObject(j, "sf", cfg.sf);
            cJSON_AddNumberToObject(j, "bw_khz", cfg.bw_khz);
            cJSON_AddNumberToObject(j, "cr", cfg.cr);
            cJSON_AddNumberToObject(j, "tx_power_dbm", cfg.tx_power_dbm);
            ble_send_resp("lora.get.ok", rid, NULL, j);
        } else {
            ble_send_resp("error", rid, "no_nvs_cfg", NULL);
        }
    }
    else if (strcmp(type, "lora.set") == 0) {
        uint32_t freq = (uint32_t)cJSON_GetNumberValue(
                            cJSON_GetObjectItem(root, "freq_hz"));
        uint8_t sf    = (uint8_t)cJSON_GetNumberValue(
                            cJSON_GetObjectItem(root, "sf"));
        uint16_t bw   = (uint16_t)cJSON_GetNumberValue(
                            cJSON_GetObjectItem(root, "bw_khz"));
        uint8_t cr    = (uint8_t)cJSON_GetNumberValue(
                            cJSON_GetObjectItem(root, "cr"));
        int8_t  pwr   = (int8_t)cJSON_GetNumberValue(
                            cJSON_GetObjectItem(root, "tx_power_dbm"));
        bool relay    = cJSON_IsTrue(cJSON_GetObjectItem(root, "relay"));

        if (freq < 150000000 || freq > 960000000 ||
            sf < 7 || sf > 12 ||
            (bw != 125 && bw != 250 && bw != 500) ||
            cr < 5 || cr > 8 ||
            pwr < -9 || pwr > 22) {
            ble_send_resp("error", rid, "invalid_param", NULL);
            cJSON_Delete(root);
            return;
        }

        ESP_LOGW(TAG, "lora.set: freq=%lu sf=%u bw=%u cr=%u pwr=%d relay=%d",
                 (unsigned long)freq, sf, bw, cr, pwr, (int)relay);

        lora_cfg_nvs_t cfg = {
            .freq_hz = freq, .sf = sf, .bw_khz = bw,
            .cr = cr, .tx_power_dbm = pwr,
        };
        lora_nvs_save(&cfg);

        if (relay && g_config.role == DEVICE_ROLE_MASTER) {
            char cmd[96];
            int n = snprintf(cmd, sizeof(cmd),
                             "lora.cfg:%lu:%u:%u:%u:%d",
                             (unsigned long)freq, sf, bw, cr, pwr);
            if (n > 0) {
                transport_send_cmd_noack(cmd, n);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        ble_send_resp("lora.set.ok", rid, "reboot_in_2s", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    else if (strcmp(type, "traffic.start") == 0) {
        traffic_light_start();
        ble_send_resp("traffic.start.ok", rid, NULL, NULL);
    }
    else if (strcmp(type, "traffic.stop") == 0) {
        traffic_light_stop();
        ble_send_resp("traffic.stop.ok", rid, NULL, NULL);
    }
    else if (strcmp(type, "reboot") == 0) {
        ble_send_resp("reboot.ok", rid, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
    else {
        ble_send_resp("error", rid, "unknown_type", NULL);
    }
    cJSON_Delete(root);
}