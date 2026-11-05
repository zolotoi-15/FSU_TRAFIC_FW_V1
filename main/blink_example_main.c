/* ============================================================
   blink_example_main.c — PRODUCTION app_main().

   === FIX: apply_pending_cfg_from_nvs НЕ стирает pending при
   ошибке парсинга.
   === LINK LOSS: 5 пропущенных пакетов подряд → связь потеряна.
   === FIX C: LINK_*_EXPECT_MS с запасом над реальными периодами
       (NODE_STATUS_PERIOD_MS / HEARTBEAT_INTERVAL_MS = 10 с),
       иначе ложные тревоги из-за границы периода.
   === FIX time-log: периодический лог дельты времени NODE vs ROOT
       (раз в 30 с).
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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
#include "wifi_ap.h"
#include "wifi_mqtt.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
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

/* === FIX C: LINK LOSS: период ожидания БОЛЬШЕ реального периода === */
#define LINK_ROOT_EXPECT_MS   15000
#define LINK_NODE_EXPECT_MS   15000
/* === END FIX C === */

ds1307_handle_t rtc_handle = NULL;

volatile uint32_t s_last_sync_ms     = 0;
volatile bool     s_comm_lost        = false;
volatile uint32_t s_last_pong_ms     = 0;
volatile uint32_t s_rtt_us           = 0;
volatile uint32_t s_rtt_delay_us     = 0;
volatile int32_t  s_rtt_samples      = 0;

volatile uint32_t s_last_node_msg_ms    = 0;
volatile uint32_t s_last_fault_clear_ms = 0;

/* === LINK LOSS: счётчики пропусков === */
volatile uint8_t s_root_missed_status = 0;
volatile uint8_t s_node_missed_ping   = 0;
/* === END LINK LOSS === */

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

/* ============================================================
   === LINK LOSS: helpers ===
   ============================================================ */
void link_monitor_note_status_rx(void) { s_root_missed_status = 0; }
void link_monitor_note_ping_rx(void)   { s_node_missed_ping   = 0; }

bool link_monitor_root_lost(void) { return s_root_missed_status >= LINK_LOST_MISSED_PKTS; }
bool link_monitor_node_lost(void) { return s_node_missed_ping   >= LINK_LOST_MISSED_PKTS; }

static void link_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "link_monitor_task started (%u missed pkts = link lost, "
                  "root_exp=%u ms, node_exp=%u ms, time-delta-log 30s)",
             LINK_LOST_MISSED_PKTS,
             (unsigned)LINK_ROOT_EXPECT_MS,
             (unsigned)LINK_NODE_EXPECT_MS);

    uint32_t last_root_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t last_node_ms = last_root_ms;
    uint32_t last_delta_log_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (g_config.role == DEVICE_ROLE_MASTER) {
            if (now - last_root_ms >= LINK_ROOT_EXPECT_MS) {
                last_root_ms = now;
                if (s_root_missed_status < LINK_LOST_MISSED_PKTS + 1)
                    s_root_missed_status++;
                if (s_root_missed_status == LINK_LOST_MISSED_PKTS) {
                    ESP_LOGE(TAG, "ROOT: %u missed STATUS → link LOST",
                             LINK_LOST_MISSED_PKTS);
                }
            }
        } else {
            if (now - last_node_ms >= LINK_NODE_EXPECT_MS) {
                last_node_ms = now;
                if (s_node_missed_ping < LINK_LOST_MISSED_PKTS + 1)
                    s_node_missed_ping++;
                if (s_node_missed_ping == LINK_LOST_MISSED_PKTS) {
                    ESP_LOGE(TAG, "NODE: %u missed ping/CMD → link LOST",
                             LINK_LOST_MISSED_PKTS);
                }
            }
        }

        /* === FIX time-log: NODE раз в 30 с логирует дельту времени === */
        if (g_config.role == DEVICE_ROLE_SLAVE &&
            (now - last_delta_log_ms >= 30000)) {
            last_delta_log_ms = now;
            uint32_t sec = 0; uint16_t ms = 0;
            bool valid = time_sync_get(&sec, &ms);
            uint32_t age = time_sync_get_delta_age_ms();
            ESP_LOGI(TAG, "NODE time: valid=%d %lu.%03u UTC delta=%ld ms "
                          "(age=%lu ms)",
                     valid ? 1 : 0, (unsigned long)sec, ms,
                     (long)time_sync_get_delta_ms(),
                     (unsigned long)age);
        }
        /* === END FIX time-log === */
    }
}
/* === END LINK LOSS === */

static void on_got_ip(void *arg, esp_event_base_t base,
                      int32_t id, void *data)
{
    static bool sntp_started = false;
    if (sntp_started) return;
    if (g_config.role != DEVICE_ROLE_MASTER) return;

    ESP_LOGI(TAG, "GOT_IP → starting SNTP + MQTT");
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
        ESP_LOGW(TAG, "pending_cfg: no len or bad size (%u), erase",
                 (unsigned)len);
        nvs_close(nvs);
        if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        return false;
    }

    if (len < 1000) {
        ESP_LOGW(TAG, "pending_cfg: too short (%u bytes < 1000), erase",
                 (unsigned)len);
        nvs_close(nvs);
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
    nvs_close(nvs);

    bool ok = config_load_json(buf, len);
    free(buf);

    if (ok) {
        if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        ESP_LOGI(TAG, "pending_cfg: applied OK (version=%.2f, ped_mode=%u)",
                 g_config.version, g_config.traffic.pedestrian.request_mode);
    } else {
        ESP_LOGE(TAG, "pending_cfg: parse FAILED, KEEP pending for next boot");
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

static void rtc_init(void)
{
    i2c_bus_init();

    esp_err_t probe = i2c_master_probe(g_i2c_bus, DS1307_ADDRESS, 50);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "DS1307 not found on I2C bus (0x%02X) — RTC disabled",
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

    /* === LINK LOSS + time-delta-log === */
    xTaskCreate(link_monitor_task, "link_mon", 3072, NULL, 3, NULL);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_got_ip, NULL);

    if (is_root) {
        ESP_LOGI(TAG, "=== ROOT init (no Wi-Fi AP) ===");
        root_wd_start_tasks();
        connectivity_start_tasks();
        mux_app_task_start();
        node_proto_start_root_tasks();
        ESP_LOGI(TAG, "ROOT ready: LoRa CFG → traffic → PPP → MQTT");
    } else {
        ESP_LOGI(TAG, "=== NODE init (no Wi-Fi STA) ===");
        node_proto_start_tasks();
        mux_app_task_start();
        ESP_LOGI(TAG, "NODE ready: LoRa only, waiting for CFG");
    }
}