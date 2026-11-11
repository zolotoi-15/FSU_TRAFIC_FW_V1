/* lora_nvs.c */
#include "lora_nvs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "lora_nvs";
#define NS "lora_cfg"

bool lora_nvs_load(lora_cfg_nvs_t *out)
{
    if (!out) return false;
    nvs_handle_t nvs;
    if (nvs_open(NS, NVS_READONLY, &nvs) != ESP_OK) return false;
    bool ok = true;
    if (nvs_get_u32(nvs, "freq", &out->freq_hz) != ESP_OK) ok = false;
    if (nvs_get_u8(nvs, "sf", &out->sf) != ESP_OK) ok = false;
    if (nvs_get_u16(nvs, "bw", &out->bw_khz) != ESP_OK) ok = false;
    if (nvs_get_u8(nvs, "cr", &out->cr) != ESP_OK) ok = false;
    if (nvs_get_i8(nvs, "pwr", &out->tx_power_dbm) != ESP_OK) ok = false;
    nvs_close(nvs);
    if (ok) {
        ESP_LOGI(TAG, "Loaded: freq=%lu sf=%u bw=%u cr=%u pwr=%d",
                 (unsigned long)out->freq_hz, out->sf, out->bw_khz,
                 out->cr, out->tx_power_dbm);
    }
    return ok;
}

void lora_nvs_save(const lora_cfg_nvs_t *cfg)
{
    if (!cfg) return;
    nvs_handle_t nvs;
    if (nvs_open(NS, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u32(nvs, "freq", cfg->freq_hz);
    nvs_set_u8(nvs, "sf", cfg->sf);
    nvs_set_u16(nvs, "bw", cfg->bw_khz);
    nvs_set_u8(nvs, "cr", cfg->cr);
    nvs_set_i8(nvs, "pwr", cfg->tx_power_dbm);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Saved");
}

void lora_nvs_erase(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}