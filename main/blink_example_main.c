/* ============================================================
   blink_example_main.c — ТОЛЬКО app_main().
   ТЕСТОВЫЙ РЕЖИМ: только LoRa.
   ROOT шлёт "PING" раз в 2 секунды, NODE принимает через callback
   (с polling через lora_poll()).
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "board_hw.h"
#include "mux.h"
#include "mux_data.h"
#include "mux_task.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "traffic_nvs.h"
#include "config_manager.h"
#include "wifi_ap.h"
#include "wifi_mqtt.h"
#include "node_link.h"
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
static const char *TAG_LORA_TEST = "LORA_TEST";

#define AP_PASSWORD          "fsu12345678"
#define NODE_LINK_PORT_FIX   3333

/* NVS namespace для pending-конфига (см. mqtt_cmd.c) */
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

static void on_sta_got_ip(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    static bool sntp_started = false;
    if (sntp_started) return;
    if (g_config.role != DEVICE_ROLE_MASTER) return;

    ESP_LOGI(TAG, "STA GOT_IP → starting SNTP");
    time_sync_start_sntp();
    sntp_started = true;

    if (!wifi_mqtt_is_connected()) {
        ESP_LOGI(TAG, "STA GOT_IP → wifi_mqtt_init()");
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

    ESP_LOGI(TAG, "pending_cfg: found %u bytes, "
                  "heap free=%u, largest=%u",
             (unsigned)len,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    char *buf = malloc(len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "pending_cfg: malloc %u failed", (unsigned)(len + 1));
        nvs_close(nvs);
        return false;
    }

    char key[16];
    size_t got = 0;
    for (int off = 0; off < (int)len; off += PENDING_CHUNK_SZ) {
        int chunk = (len - off > PENDING_CHUNK_SZ)
                    ? PENDING_CHUNK_SZ : (int)(len - off);
        snprintf(key, sizeof(key), NVS_KEY_CHUNK, off / PENDING_CHUNK_SZ);
        size_t sz = chunk;
        err = nvs_get_blob(nvs, key, buf + off, &sz);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pending_cfg: get %s failed: %s",
                     key, esp_err_to_name(err));
            free(buf);
            nvs_close(nvs);
            return false;
        }
        got += sz;
    }
    buf[len] = '\0';

    ESP_LOGI(TAG, "pending_cfg: loaded %u/%u bytes, parsing...",
             (unsigned)got, (unsigned)len);

    bool ok = config_load_json(buf, len);
    free(buf);
    nvs_close(nvs);

    if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    if (ok) {
        ESP_LOGI(TAG, "pending_cfg: applied OK "
                      "(version=%.2f, ped_mode=%u)",
                 g_config.version,
                 g_config.traffic.pedestrian.request_mode);
    } else {
        ESP_LOGE(TAG, "pending_cfg: parse FAILED");
    }
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

/* ============================================================
   ТЕСТОВЫЙ CALLBACK LoRa RX
   ============================================================ */
static void lora_test_rx_cb(const uint8_t *data, size_t len)
{
    ESP_LOGI(TAG_LORA_TEST, "RX (%u bytes): %.*s",
             (unsigned)len, (int)len, (const char *)data);
}

/* ============================================================
   ТЕСТОВЫЙ ЦИКЛ LoRa
   ============================================================ */
static void lora_test_root(void)
{
    ESP_LOGI(TAG_LORA_TEST, "ROOT: starting LoRa TX test (PING every 2s)");
    while (1) {
        uint8_t test[] = "PING";
        if (lora_send_packet(test, 4) == ESP_OK) {
            ESP_LOGI(TAG_LORA_TEST, "PING sent OK");
        } else {
            ESP_LOGE(TAG_LORA_TEST, "PING send FAILED");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void lora_test_node(void)
{
    ESP_LOGI(TAG_LORA_TEST, "NODE: starting LoRa RX test (polling)");
    lora_set_rx_callback(lora_test_rx_cb);
    lora_receive();  // войти в режим приёма

    while (1) {
        lora_poll();  // FIX: опрос IRQ
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Board bring-up (root/node) — LoRa TEST MODE");

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

    /* Минимальная инициализация железа */
    board_hw_init();
    sr_init();
    sr_all_off();
    mux_init();
    mux_data_init();

    log_heap("before lora_init");

    /* ── ТЕСТ: ТОЛЬКО LoRa ── */
    esp_err_t lora_ret = lora_init();
    if (lora_ret != ESP_OK) {
        ESP_LOGE(TAG_LORA_TEST, "lora_init failed: %s",
                 esp_err_to_name(lora_ret));
    } else {
        ESP_LOGI(TAG_LORA_TEST, "lora_init OK");
    }

    log_heap("after lora_init");

    /* Запускаем тестовый цикл */
    if (is_root) {
        lora_test_root();
    } else {
        lora_test_node();
    }
}