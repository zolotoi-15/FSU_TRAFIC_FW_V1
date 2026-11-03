/* ============================================================
   blink_example_main.c — PRODUCTION app_main().
   ROOT: PPP → MQTT → LoRa (единственный канал к NODE).
   NODE: LoRa → телеметрия → ROOT → MQTT.

   === NO WIFI ===
   Wi-Fi (STA и AP) полностью отключён. Обмен ROOT↔NODE
   только через LoRa. PPPoS (модем) — не Wi-Fi, оставлен.
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"             /* нужен для wifi_ap_record_t в mqtt_json.c */
#include "esp_event.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"

#include "board_hw.h"
#include "mux.h"
#include "mux_data.h"
#include "mux_task.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "traffic_nvs.h"
#include "config_manager.h"
#include "wifi_ap.h"              /* нужен для wifi_get_ip()/wifi_sta_connected() — заглушки */
#include "wifi_mqtt.h"
#include "node_link.h"            /* нужен для node_link_server_has_client() — заглушка */
#include "node_proto.h"
#include "transport.h"
#include "lora.h"
#include "time_sync.h"
#include "ds1307.h"
#include "i2c_bus.h"
#include "connectivity.h"
#include "fault_handler.h"
#include "buttons.h"
#include "root_wd.h"
#include "app_shared.h"

extern const uint8_t config_json_start[] asm("_binary_config_json_start");
extern const uint8_t config_json_end[]   asm("_binary_config_json_end");

static const char *TAG = "board";

#define NVS_NS_PENDING   "pending_cfg"
#define NVS_KEY_LEN      "len"
#define NVS_KEY_CHUNK    "chunk_%d"
#define PENDING_CHUNK_SZ 4096

ds1307_handle_t rtc_handle = NULL;

volatile uint32_t s_last_sync_ms     = 0;
volatile bool     s_comm_lost        = false;
volatile uint32_t s_last_pong_ms     = 0;
volatile uint32_t s_rtt_us           = 0;
volatile uint32_t s_rtt_delay_us     = 0;
volatile int32_t  s_rtt_samples      = 0;

volatile uint32_t s_last_node_msg_ms    = 0;
volatile uint32_t s_last_fault_clear_ms = 0;

int32_t  blink_get_rtt_samples(void)  { return s_rtt_samples; }
uint32_t blink_get_rtt_us(void)       { return s_rtt_us; }
uint32_t blink_get_rtt_delay_us(void) { return s_rtt_delay_us; }

void blink_reset_rtt(void)
{
    s_rtt_us = 0;
    s_rtt_delay_us = 0;
    s_rtt_samples = 0;
    s_last_pong_ms = 0;
    traffic_light_set_sync_delay_us(0);
}

static void on_got_ip(void *arg, esp_event_base_t base,
                      int32_t id, void *data)
{
    static bool sntp_started = false;
    if (sntp_started) return;
    if (g_config.role != DEVICE_ROLE_MASTER) return;

    ESP_LOGI(TAG, "GOT_IP (%s) → starting SNTP + MQTT",
             base == IP_EVENT ? "IP" : "other");
    time_sync_start_sntp();
    sntp_started = true;

    if (!wifi_mqtt_is_connected()) {
        wifi_mqtt_init();
    }
}

static bool nvs_app_cfg_exists(void)
{
    nvs_handle_t nvs;
    if (nvs_open("app_cfg", NVS_READONLY, &nvs) != ESP_OK) return false;

    size_t sz = sizeof(app_config_t);
    app_config_t *tmp = malloc(sizeof(app_config_t));
    bool ok = false;
    if (tmp) {
        if (nvs_get_blob(nvs, "cfg_blob", tmp, &sz) == ESP_OK &&
            sz == sizeof(app_config_t)) {
            ok = true;
        }
        free(tmp);
    }
    nvs_close(nvs);
    return ok;
}

static bool apply_pending_cfg_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_PENDING, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "pending_cfg: not found (normal at first boot)");
        return false;
    }

    uint32_t len = 0;
    err = nvs_get_u32(nvs, NVS_KEY_LEN, &len);
    if (err != ESP_OK || len == 0 || len > 32768) {
        ESP_LOGW(TAG, "pending_cfg: no len or bad size (%u), skip",
                 (unsigned)len);
        nvs_close(nvs);
        return false;
    }

    if (len < 1000) {
        ESP_LOGW(TAG, "pending_cfg: too short (%u bytes < 1000), erase",
                 (unsigned)len);
        if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        return false;
    }

    ESP_LOGI(TAG, "pending_cfg: found %u bytes, heap free=%u, largest=%u",
             (unsigned)len,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    char *buf = malloc(len + 1);
    if (!buf) { nvs_close(nvs); return false; }

    char key[16];
    size_t got = 0;
    for (int off = 0; off < (int)len; off += PENDING_CHUNK_SZ) {
        int chunk = (len - off > PENDING_CHUNK_SZ)
                    ? PENDING_CHUNK_SZ : (int)(len - off);
        snprintf(key, sizeof(key), NVS_KEY_CHUNK, off / PENDING_CHUNK_SZ);
        size_t sz = chunk;
        if (nvs_get_blob(nvs, key, buf + off, &sz) != ESP_OK) {
            free(buf); nvs_close(nvs); return false;
        }
        got += sz;
    }
    buf[len] = '\0';

    bool ok = config_load_json(buf, len);
    free(buf);
    nvs_close(nvs);

    if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "pending_cfg: %s (version=%.2f, ped_mode=%u)",
             ok ? "applied OK" : "parse FAILED",
             g_config.version, g_config.traffic.pedestrian.request_mode);
    return ok;
}

static void load_embedded_config_if_needed(void)
{
    size_t len = (size_t)(config_json_end - config_json_start);
    if (len == 0) return;

    if (nvs_app_cfg_exists()) {
        ESP_LOGI(TAG, "NVS config found → use NVS (skip embedded config.json)");
        return;
    }

    ESP_LOGI(TAG, "NVS empty → loading embedded config.json (%u bytes)",
             (unsigned)len);
    config_load_json((const char *)config_json_start, len);
}

static void log_heap(const char *stage)
{
    ESP_LOGI(TAG, "HEAP[%s]: free=%u, min=%u, largest=%u",
             stage,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
}

static void rtc_init(void)
{
    i2c_bus_init();

    esp_err_t probe = i2c_master_probe(g_i2c_bus, DS1307_ADDRESS, 50);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "DS1307 not found on I2C bus (0x%02X) — RTC disabled "
                      "(time will come from SNTP)",
                 DS1307_ADDRESS);
        rtc_handle = NULL;
        return;
    }

    ds1307_config_t rtc_cfg = {
        .ds1307_device = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = DS1307_ADDRESS,
            .scl_speed_hz    = 100000,
        },
        .century = 21,
    };

    esp_err_t err = ds1307_init(g_i2c_bus, &rtc_cfg, &rtc_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ds1307_init failed (%s) — RTC disabled",
                 esp_err_to_name(err));
        rtc_handle = NULL;
    } else {
        ESP_LOGI(TAG, "DS1307 ready (addr=0x%02X, century=%d)",
                 DS1307_ADDRESS, rtc_cfg.century);
    }
}

static void cfg_push_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_push_task: waiting for LoRa...");

    uint32_t waited_ms = 0;
    while (!transport_is_lora() && waited_ms < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited_ms += 500;
    }
    if (!transport_is_lora()) {
        ESP_LOGE(TAG, "cfg_push: LoRa not ready → abort");
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    extern bool config_export_json(char *b, size_t *l);
    char *buf = malloc(16384);
    size_t blen = 16384;
    if (!buf || !config_export_json(buf, &blen)) {
        ESP_LOGE(TAG, "cfg_push: export failed");
        free(buf);
        vTaskDelete(NULL);
        return;
    }

    for (int attempt = 0; attempt < 5; attempt++) {
        ESP_LOGI(TAG, "cfg_push → LoRa attempt %d/5 (%u bytes)",
                 attempt + 1, (unsigned)blen);
        esp_err_t err = transport_send_cfg(buf, blen);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "cfg_push: OK (attempt %d)", attempt + 1);
            free(buf);
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGW(TAG, "cfg_push: failed (attempt %d): %s",
                 attempt + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    ESP_LOGE(TAG, "cfg_push: all attempts failed — NODE not responding");
    free(buf);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Board bring-up (ROOT/NODE) — NO WIFI, LoRa only");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    log_heap("app_main start");

    bool pending_applied = apply_pending_cfg_from_nvs();
    if (pending_applied) {
        log_heap("after pending_cfg applied");
    }
    if (!pending_applied) {
        load_embedded_config_if_needed();
    }
    config_init();

    bool is_root = (g_config.role == DEVICE_ROLE_MASTER);
    ESP_LOGI(TAG, "role = %s", is_root ? "ROOT" : "NODE");

    board_hw_init();
    sr_init();
    sr_all_off();
    mux_init();
    mux_data_init();

    rtc_init();
    time_sync_init();

    traffic_light_init();
    fault_handler_register();
    wifi_mqtt_apply_config_runtime();

    buttons_start_tasks();

    connectivity_register_rs485_can();

    transport_init();
    node_proto_register_callbacks();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_got_ip, NULL);

    if (is_root) {
        /* ─────────────── ROOT ─────────────── */
        ESP_LOGI(TAG, "=== ROOT init (no Wi-Fi AP) ===");

        connectivity_start_tasks();
        root_wd_start_tasks();
        mux_app_task_start();

        // === FIX: Запуск задачи heartbeat для поддержания и восстановления связи LoRa ===
        node_proto_start_root_tasks();
        // ================================================================================

        // xTaskCreate(cfg_push_task, "cfg_push", 4096, NULL, 3, NULL); // Оставлено закомментированным, как в оригинале

        ESP_LOGI(TAG, "ROOT ready: LoRa only, MQTT via PPP");

    } else {
        /* ─────────────── NODE ─────────────── */
        ESP_LOGI(TAG, "=== NODE init (no Wi-Fi STA) ===");

        node_proto_start_tasks();
        mux_app_task_start();

        ESP_LOGI(TAG, "NODE ready: LoRa only");
    }
}