/* ============================================================
   ble_config.c — парсинг JSON-команд из BLE.

   === FIX BLE-RESP-DOUBLE-SEND ===
       Раньше fsu_ble_notify_response вызывался ДВА раза:
       безусловно и повторно внутри if (fsu_ble_is_connected()).
       Телефон получал дублированный ответ. Оставлен только
       один вызов — внутри if.

   === FIX BLE-LORA-SET-TYPECHECK ===
       Явная проверка cJSON_IsNumber для каждого поля lora.set.

   === FIX LEAD-MS-RUNTIME ===
       sync.get / sync.set — runtime-настройка задержки ROOT
       между отправкой c: и локальным apply_stage_dd
       (g_sync_lead_ms, валидация [50..1000] мс).
   ============================================================ */

#include "ble_config.h"
#include "ble_transport.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "lora_nvs.h"
#include "sync_config.h"
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

    /* === FIX BLE-RESP-DOUBLE-SEND ===
       Один вызов внутри if. Если не подключён — просто лог. */
    if (fsu_ble_is_connected()) {
        fsu_ble_notify_response(out, strlen(out));
    } else {
        ESP_LOGW(TAG, "resp dropped (not connected): %s", out);
    }
    /* === END FIX BLE-RESP-DOUBLE-SEND === */
    free(out);
}

/* === FIX BLE-LORA-SET-TYPECHECK === */
static bool json_get_u32_checked(const cJSON *root, const char *key,
                                 uint32_t *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (uint32_t)item->valuedouble;
    return true;
}
static bool json_get_u8_checked(const cJSON *root, const char *key,
                                uint8_t *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (uint8_t)item->valueint;
    return true;
}
static bool json_get_u16_checked(const cJSON *root, const char *key,
                                 uint16_t *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (uint16_t)item->valueint;
    return true;
}
static bool json_get_i8_checked(const cJSON *root, const char *key,
                                int8_t *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (int8_t)item->valueint;
    return true;
}
/* === END FIX BLE-LORA-SET-TYPECHECK === */

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
                ESP_LOGI(TAG, "config.set applied via BLE "
                              "(runtime: config:reload or reboot)");
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
        /* === FIX BLE-LORA-SET-TYPECHECK === */
        uint32_t freq = 0;
        uint8_t  sf = 0, cr = 0;
        uint16_t bw = 0;
        int8_t   pwr = 0;
        if (!json_get_u32_checked(root, "freq_hz", &freq) ||
            !json_get_u8_checked(root, "sf", &sf) ||
            !json_get_u16_checked(root, "bw_khz", &bw) ||
            !json_get_u8_checked(root, "cr", &cr) ||
            !json_get_i8_checked(root, "tx_power_dbm", &pwr)) {
            ble_send_resp("error", rid, "missing_or_bad_field", NULL);
            cJSON_Delete(root);
            return;
        }
        bool relay = cJSON_IsTrue(cJSON_GetObjectItem(root, "relay"));
        /* === END FIX BLE-LORA-SET-TYPECHECK === */

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
    /* === FIX LEAD-MS-RUNTIME === */
    else if (strcmp(type, "sync.get") == 0) {
        cJSON *j = cJSON_CreateObject();
        cJSON_AddNumberToObject(j, "lead_ms", sync_config_get_lead_ms());
        cJSON_AddNumberToObject(j, "min", SYNC_LEAD_MS_MIN);
        cJSON_AddNumberToObject(j, "max", SYNC_LEAD_MS_MAX);
        ble_send_resp("sync.get.ok", rid, NULL, j);
    }
    else if (strcmp(type, "sync.set") == 0) {
        uint32_t ms = 0;
        if (!json_get_u32_checked(root, "lead_ms", &ms)) {
            ble_send_resp("error", rid, "missing_or_bad_field", NULL);
        } else if (sync_config_set_lead_ms(ms) != ESP_OK) {
            cJSON *j = cJSON_CreateObject();
            cJSON_AddNumberToObject(j, "min", SYNC_LEAD_MS_MIN);
            cJSON_AddNumberToObject(j, "max", SYNC_LEAD_MS_MAX);
            ble_send_resp("error", rid, "out_of_range", j);
        } else {
            cJSON *j = cJSON_CreateObject();
            cJSON_AddNumberToObject(j, "lead_ms", sync_config_get_lead_ms());
            ble_send_resp("sync.set.ok", rid, NULL, j);
        }
    }
    /* === END FIX LEAD-MS-RUNTIME === */
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