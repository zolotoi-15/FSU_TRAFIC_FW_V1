/* ============================================================
   sync_config.c — runtime-настройки синхронизации ROOT/NODE.

   === FIX LEAD-MS-RUNTIME ===
   g_sync_lead_ms — задержка между отправкой c: на NODE и
   локальным apply_stage_dd на ROOT. По умолчанию 150 мс.
   Меняется из MQTT/BLE/программно через sync_config_set_lead_ms().
   ============================================================ */

#include "sync_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "sync_cfg";

/* 150 мс — минимум для LoRa ToA (112 мс SF8/BW250, preamble=12)
   + ~40 мс на обработку c: в NODE. */
volatile uint32_t g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;

uint32_t sync_config_get_lead_ms(void)
{
    uint32_t v = g_sync_lead_ms;
    if (v < SYNC_LEAD_MS_MIN) v = SYNC_LEAD_MS_MIN;
    if (v > SYNC_LEAD_MS_MAX) v = SYNC_LEAD_MS_MAX;
    return v;
}

int sync_config_set_lead_ms(uint32_t ms)
{
    if (ms < SYNC_LEAD_MS_MIN || ms > SYNC_LEAD_MS_MAX) {
        ESP_LOGW(TAG, "set_lead_ms(%lu) out of range [%u..%u] — rejected",
                 (unsigned long)ms,
                 (unsigned)SYNC_LEAD_MS_MIN, (unsigned)SYNC_LEAD_MS_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t old = g_sync_lead_ms;
    g_sync_lead_ms = ms;
    if (old != ms) {
        ESP_LOGI(TAG, "lead_ms: %lu → %lu", (unsigned long)old,
                 (unsigned long)ms);
    }
    return ESP_OK;
}