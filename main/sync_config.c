/* ============================================================
   sync_config.c — runtime-настройки синхронизации ROOT/NODE.

   === FIX LEAD-MS-RUNTIME ===
   g_sync_lead_ms — задержка между отправкой c: на NODE и
   локальным apply_stage_dd на ROOT. По умолчанию 150 мс.
   Меняется из MQTT/BLE/программно через sync_config_set_lead_ms().

   === FIX LEAD-MS-NVS ===
   При старте sync_config_init() читает lead_ms из NVS
   (namespace "sync_cfg", ключ "lead_ms"). Если пусто/битые
   данные — SYNC_LEAD_MS_DEFAULT (150). При изменении через
   sync_config_set_lead_ms() значение пишется в NVS.
   ============================================================ */

#include "sync_config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "sync_cfg";

#define NVS_NS_SYNC     "sync_cfg"
#define NVS_KEY_LEAD_MS "lead_ms"

/* 150 мс — минимум для LoRa ToA (112 мс SF8/BW250, preamble=12)
   + ~40 мс на обработку c: в NODE. */
volatile uint32_t g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;

/* ============================================================
   NVS helpers
   ============================================================ */
static esp_err_t nvs_read_lead_ms(uint32_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_SYNC, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        /* ESP_ERR_NVS_NOT_FOUND — namespace пуст, это норма. */
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "nvs_open(RO): %s", esp_err_to_name(err));
        }
        return err;
    }

    uint32_t v = 0;
    err = nvs_get_u32(nvs, NVS_KEY_LEAD_MS, &v);
    nvs_close(nvs);

    if (err == ESP_OK) {
        *out = v;
    }
    return err;
}

static esp_err_t nvs_write_lead_ms(uint32_t v)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_SYNC, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(RW): %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u32(nvs, NVS_KEY_LEAD_MS, v);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "NVS lead_ms saved: %lu", (unsigned long)v);
    } else {
        ESP_LOGW(TAG, "NVS lead_ms write: %s", esp_err_to_name(err));
    }
    return err;
}

/* ============================================================
   Public API
   ============================================================ */
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
        ESP_LOGI(TAG, "lead_ms: %lu → %lu",
                 (unsigned long)old, (unsigned long)ms);
    }

    /* === FIX LEAD-MS-NVS ===
       Пишем в NVS при каждом изменении. Если NVS не готов
       (редкий случай — nvs_flash_init ещё не вызван) — просто
       логируем, значение применяется в RAM. */
    esp_err_t err = nvs_write_lead_ms(ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "lead_ms applied in RAM, but NVS save failed (%s) "
                      "— after reboot will use default",
                 esp_err_to_name(err));
    }
    /* === END FIX LEAD-MS-NVS === */

    return ESP_OK;
}

esp_err_t sync_config_init(void)
{
    uint32_t v = 0;
    esp_err_t err = nvs_read_lead_ms(&v);

    if (err == ESP_OK) {
        if (v >= SYNC_LEAD_MS_MIN && v <= SYNC_LEAD_MS_MAX) {
            g_sync_lead_ms = v;
            ESP_LOGI(TAG, "lead_ms from NVS: %lu ms",
                     (unsigned long)v);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "NVS lead_ms=%lu out of range [%u..%u] → default %u",
                 (unsigned long)v,
                 (unsigned)SYNC_LEAD_MS_MIN, (unsigned)SYNC_LEAD_MS_MAX,
                 (unsigned)SYNC_LEAD_MS_DEFAULT);
        g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;
        /* Перезаписываем битое значение в NVS корректным дефолтом. */
        nvs_write_lead_ms(SYNC_LEAD_MS_DEFAULT);
        return ESP_OK;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND ||
        err == ESP_ERR_NVS_NOT_INITIALIZED) {
        ESP_LOGI(TAG, "no lead_ms in NVS → default %u ms",
                 (unsigned)SYNC_LEAD_MS_DEFAULT);
        g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "NVS read lead_ms failed (%s) → default %u ms",
             esp_err_to_name(err), (unsigned)SYNC_LEAD_MS_DEFAULT);
    g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;
    return ESP_FAIL;
}

esp_err_t sync_config_nvs_erase(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_SYNC, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(RW) for erase: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_key(nvs, NVS_KEY_LEAD_MS);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* Ключа и не было — не ошибка. */
        err = ESP_OK;
    }
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        g_sync_lead_ms = SYNC_LEAD_MS_DEFAULT;
        ESP_LOGI(TAG, "NVS lead_ms erased → default %u ms",
                 (unsigned)SYNC_LEAD_MS_DEFAULT);
    } else {
        ESP_LOGW(TAG, "NVS erase lead_ms failed: %s", esp_err_to_name(err));
    }
    return err;
}