#include "traffic_nvs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "traffic_nvs";

#define NVS_NS_TRAFFIC       "traffic_cfg"
#define NVS_KEY_MODE         "mode"
#define NVS_KEY_ENABLED      "enabled"
#define NVS_KEY_FULL         "full"

/* ---- legacy: mode + enabled ---- */
esp_err_t traffic_nvs_save(traffic_mode_t mode, bool enabled)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_TRAFFIC, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(nvs, NVS_KEY_MODE, (uint8_t)mode);
    if (err == ESP_OK) err = nvs_set_u8(nvs, NVS_KEY_ENABLED, enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(nvs);

    nvs_close(nvs);
    ESP_LOGI(TAG, "Saved: mode=%d, enabled=%d (%s)",
             (int)mode, (int)enabled, esp_err_to_name(err));
    return err;
}

void traffic_nvs_load(traffic_mode_t *mode, bool *enabled)
{
    if (!mode || !enabled) return;
    *mode = TRAFFIC_MODE_SINGLE;
    *enabled = false;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_TRAFFIC, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "No saved config, defaults");
        return;
    }
    uint8_t m = 0, e = 0;
    if (nvs_get_u8(nvs, NVS_KEY_MODE, &m) == ESP_OK) {
        if (m == TRAFFIC_MODE_SINGLE || m == TRAFFIC_MODE_CROSS || m == TRAFFIC_MODE_CROSS_PED)
            *mode = (traffic_mode_t)m;
    }
    if (nvs_get_u8(nvs, NVS_KEY_ENABLED, &e) == ESP_OK) *enabled = (e != 0);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Loaded: mode=%d, enabled=%d", (int)*mode, (int)*enabled);
}

esp_err_t traffic_nvs_reset(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_TRAFFIC, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    nvs_erase_key(nvs, NVS_KEY_MODE);
    nvs_erase_key(nvs, NVS_KEY_ENABLED);
    nvs_erase_key(nvs, NVS_KEY_FULL);
    err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

/* ---- DD: full traffic_config_t ---- */
esp_err_t traffic_nvs_save_full(const traffic_config_t *tc)
{
    if (!tc) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_TRAFFIC, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(nvs, NVS_KEY_FULL, tc, sizeof(*tc));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Save full: stages=%u seq=%u trans=%u (%s)",
             tc->stage_count, tc->sequence_count, tc->transition_count,
             esp_err_to_name(err));
    return err;
}

esp_err_t traffic_nvs_load_full(traffic_config_t *tc)
{
    if (!tc) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_TRAFFIC, NVS_READONLY, &nvs);
    if (err != ESP_OK) return err;
    size_t sz = sizeof(*tc);
    err = nvs_get_blob(nvs, NVS_KEY_FULL, tc, &sz);
    nvs_close(nvs);
    if (err == ESP_OK && sz == sizeof(*tc)) {
        ESP_LOGI(TAG, "Load full OK (stages=%u seq=%u)",
                 tc->stage_count, tc->sequence_count);
    } else {
        ESP_LOGW(TAG, "Load full failed (%s, sz=%u/%u)",
                 esp_err_to_name(err), (unsigned)sz, (unsigned)sizeof(*tc));
    }
    return err;
}