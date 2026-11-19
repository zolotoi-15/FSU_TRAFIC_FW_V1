/* ============================================================
   blink_example_main.c — PRODUCTION app_main().
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#include "sync_handshake.h"
#include "app_shared.h"
#include "ble_transport.h"
#include "sync_config.h"
#include "modem_ppp.h"
#include "can_bus.h"

extern const uint8_t config_json_start[] asm("_binary_config_json_start");
extern const uint8_t config_json_end[]   asm("_binary_config_json_end");

static const char *TAG = "board";
#define USE_BLE 0

#define NVS_NS_PENDING   "pending_cfg"
#define NVS_KEY_LEN      "len"
#define NVS_KEY_CHUNK    "chunk_%d"
#define PENDING_CHUNK_SZ 4096

#define LINK_ROOT_EXPECT_MS   3000
#define LINK_NODE_EXPECT_MS   2000

#define CFG_LOAD_TASK_STACK   24576

ds1307_handle_t rtc_handle = NULL;

volatile uint32_t s_last_sync_ms     = 0;
volatile bool     s_comm_lost        = false;
volatile uint32_t s_last_pong_ms     = 0;
volatile uint32_t s_rtt_us           = 0;
volatile uint32_t s_rtt_delay_us     = 0;
volatile int32_t  s_rtt_samples      = 0;

volatile uint32_t s_last_node_msg_ms    = 0;
volatile uint32_t s_last_fault_clear_ms = 0;

volatile uint8_t s_root_missed_status = 0;
volatile uint8_t s_node_missed_ping   = 0;

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
   LINK LOSS helpers
   ============================================================ */
void link_monitor_note_status_rx(void) { s_root_missed_status = 0; }
void link_monitor_note_ping_rx(void)   { s_node_missed_ping   = 0; }

bool link_monitor_root_lost(void) { return s_root_missed_status >= LINK_LOST_MISSED_PKTS; }
bool link_monitor_node_lost(void) { return s_node_missed_ping   >= LINK_LOST_MISSED_PKTS; }

static void link_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "link_monitor_task started (%u missed pkts = link lost, "
                  "root_exp=%u ms, node_exp=%u ms)",
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

                bool got_status = false;
                if (s_last_node_msg_ms != 0) {
                    uint32_t node_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (node_tick >= s_last_node_msg_ms &&
                        (node_tick - s_last_node_msg_ms) <
                        (uint32_t)LINK_ROOT_EXPECT_MS) {
                        got_status = true;
                    }
                }
                if (!got_status) {
                    if (s_root_missed_status < LINK_LOST_MISSED_PKTS + 1)
                        s_root_missed_status++;
                    if (s_root_missed_status == LINK_LOST_MISSED_PKTS) {
                        ESP_LOGE(TAG, "ROOT: %u missed STATUS → link LOST "
                                      "→ sync_handshake",
                                 LINK_LOST_MISSED_PKTS);
                        sync_handshake_root_start();
                    }
                } else {
                    s_root_missed_status = 0;
                }
            }
        } else {
            if (now - last_node_ms >= LINK_NODE_EXPECT_MS) {
                last_node_ms = now;

                bool got_ping = false;
                if (s_last_sync_ms != 0) {
                    uint32_t node_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (node_tick >= s_last_sync_ms &&
                        (node_tick - s_last_sync_ms) <
                        (uint32_t)LINK_NODE_EXPECT_MS) {
                        got_ping = true;
                    }
                }
                if (!got_ping) {
                    if (s_node_missed_ping < LINK_LOST_MISSED_PKTS + 1)
                        s_node_missed_ping++;
                    if (s_node_missed_ping == LINK_LOST_MISSED_PKTS) {
                        ESP_LOGE(TAG, "NODE: %u missed ping/CMD → link LOST "
                                      "→ sync_handshake",
                                 LINK_LOST_MISSED_PKTS);
                        sync_handshake_node_start();
                    }
                } else {
                    s_node_missed_ping = 0;
                }
            }
        }

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
    }
}

/* ============================================================
   on_got_ip — SNTP + MQTT стартуют при получении IP (PPP или STA)
   ============================================================ */
static void on_got_ip(void *arg, esp_event_base_t base,
                      int32_t id, void *data)
{
    static bool sntp_started = false;
    if (g_config.role != DEVICE_ROLE_MASTER) return;

    if (id == IP_EVENT_STA_GOT_IP) {
        esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta_netif) {
            esp_netif_set_default_netif(sta_netif);
            ESP_LOGI(TAG, "Default netif → WiFi STA");
        }
    } else if (id == IP_EVENT_PPP_GOT_IP && !wifi_sta_has_ip()) {
        esp_netif_t *ppp_netif = modem_ppp_get_netif();
        if (ppp_netif) {
            esp_netif_set_default_netif(ppp_netif);
            ESP_LOGI(TAG, "Default netif → PPPoS");
        }
    }

    if (!sntp_started) {
        time_sync_start_sntp();
        sntp_started = true;
    }

    if (!wifi_mqtt_is_connected()) {
        for (int i = 0; i < 5; i++) {
            esp_err_t err = wifi_mqtt_init();
            if (err == ESP_OK) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ============================================================
   Config load helpers (без изменений)
   ============================================================ */
static bool nvs_app_cfg_exists(void)
{
    nvs_handle_t nvs;
    if (nvs_open("app_cfg", NVS_READONLY, &nvs) != ESP_OK) return false;

    static app_config_t tmp;
    size_t sz = sizeof(tmp);
    bool ok = false;
    if (nvs_get_blob(nvs, "cfg_blob", &tmp, &sz) == ESP_OK &&
        sz == sizeof(tmp)) {
        ok = true;
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
            nvs_erase_all(nvs); nvs_commit(nvs); nvs_close(nvs);
        }
        return false;
    }
    if (len < 1000) {
        ESP_LOGW(TAG, "pending_cfg: too short (%u bytes < 1000), erase",
                 (unsigned)len);
        nvs_close(nvs);
        if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs); nvs_commit(nvs); nvs_close(nvs);
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
    for (int off = 0; off < (int)len; off += PENDING_CHUNK_SZ) {
        int chunk = (len - off > PENDING_CHUNK_SZ)
                    ? PENDING_CHUNK_SZ : (int)(len - off);
        snprintf(key, sizeof(key), NVS_KEY_CHUNK, off / PENDING_CHUNK_SZ);
        size_t sz = chunk;
        if (nvs_get_blob(nvs, key, buf + off, &sz) != ESP_OK) {
            free(buf); nvs_close(nvs); return false;
        }
    }
    buf[len] = '\0';
    nvs_close(nvs);

    bool ok = config_load_json(buf, len);
    free(buf);

    if (ok) {
        if (nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs); nvs_commit(nvs); nvs_close(nvs);
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
    if (len == 0) {
        ESP_LOGE(TAG, "load_embedded_config: embedded config.json is EMPTY");
        return;
    }
    if (nvs_app_cfg_exists()) {
        ESP_LOGI(TAG, "NVS config found → use NVS (skip embedded config.json)");
        return;
    }

    ESP_LOGI(TAG, "NVS empty → loading embedded config.json (%u bytes)",
             (unsigned)len);
    bool ok = config_load_json((const char *)config_json_start, len);
    if (!ok) {
        ESP_LOGE(TAG, "load_embedded_config: config_load_json FAILED "
                      "— falling back to defaults (WILL CONFLICT!)");
    } else {
        ESP_LOGI(TAG, "load_embedded_config: OK (version=%.2f, stages=%u, seq=%u)",
                 g_config.version,
                 g_config.traffic.stage_count,
                 g_config.traffic.sequence_count);
    }
}

typedef struct {
    bool              pending_applied;
    SemaphoreHandle_t done;
} cfg_load_ctx_t;

static void cfg_load_task(void *arg)
{
    cfg_load_ctx_t *ctx = (cfg_load_ctx_t *)arg;

    ESP_LOGI(TAG, "cfg_load_task: stack=%u, heap free=%u, largest=%u",
             (unsigned)CFG_LOAD_TASK_STACK,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    ctx->pending_applied = apply_pending_cfg_from_nvs();
    if (ctx->pending_applied) {
        ESP_LOGI(TAG, "cfg_load_task: pending_cfg applied");
    } else {
        load_embedded_config_if_needed();
    }

    xSemaphoreGive(ctx->done);
    vTaskDelete(NULL);
}

static bool run_cfg_load(void)
{
    cfg_load_ctx_t ctx = { .pending_applied = false, .done = NULL };
    ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) {
        ESP_LOGE(TAG, "run_cfg_load: sem alloc failed");
        return false;
    }

    TaskHandle_t th = NULL;
    BaseType_t rc = xTaskCreate(cfg_load_task, "cfg_load",
                                CFG_LOAD_TASK_STACK, &ctx, 5, &th);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "run_cfg_load: xTaskCreate failed (stack=%u)",
                 (unsigned)CFG_LOAD_TASK_STACK);
        vSemaphoreDelete(ctx.done);
        return false;
    }

    xSemaphoreTake(ctx.done, portMAX_DELAY);
    vSemaphoreDelete(ctx.done);
    return ctx.pending_applied;
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
        ESP_LOGW(TAG, "DS1307 not found on I2C bus (0x%02X) — "
                      "using RTC-ESP32S3 + SNTP/LoRa only",
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
        ESP_LOGW(TAG, "ds1307_init failed (%s) — using RTC-ESP32S3 only",
                 esp_err_to_name(err));
        rtc_handle = NULL;
    } else {
        ESP_LOGI(TAG, "DS1307 ready (addr=0x%02X, century=%d)",
                 DS1307_ADDRESS, rtc_cfg.century);
    }
}

static void ensure_valid_cfg(void)
{
    if (g_config.traffic.sequence_count > 0 &&
        g_config.traffic.stage_count > 0) {
        ESP_LOGI(TAG, "ensure_valid_cfg: OK (stages=%u seq=%u ped_mode=%u)",
                 g_config.traffic.stage_count,
                 g_config.traffic.sequence_count,
                 g_config.traffic.pedestrian.request_mode);
        return;
    }
    size_t len = (size_t)(config_json_end - config_json_start);
    if (len == 0) {
        ESP_LOGE(TAG, "ensure_valid_cfg: embedded config.json is EMPTY");
        return;
    }
    ESP_LOGW(TAG, "ensure_valid_cfg: g_config has no sequence/stages → "
                  "force-load embedded config.json (%u bytes)", (unsigned)len);
    bool ok = config_load_json((const char *)config_json_start, len);
    if (!ok) {
        ESP_LOGE(TAG, "ensure_valid_cfg: force-load FAILED");
    } else {
        ESP_LOGI(TAG, "ensure_valid_cfg: force-load OK "
                      "(stages=%u seq=%u ped_mode=%u)",
                 g_config.traffic.stage_count,
                 g_config.traffic.sequence_count,
                 g_config.traffic.pedestrian.request_mode);
    }
}

/* ============================================================
   Хелпер: сгенерировать AP SSID (FSU_ROOT_XXXX / FSU_NODE_XXXX)
   ============================================================ */
static void build_ap_ssid(char *out, size_t out_len, bool is_root)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, out_len, "FSU_%s_%02X%02X",
             is_root ? "ROOT" : "NODE", mac[4], mac[5]);
}

/* ============================================================
   WiFi bringup: AP всегда; STA если есть creds (APSTA).
   ============================================================ */
static void wifi_bringup(bool is_root)
{
    char ap_ssid[33];
    build_ap_ssid(ap_ssid, sizeof(ap_ssid), is_root);

    bool has_wifi_creds = (strlen(g_config.wifi_ssid) > 0);

    if (has_wifi_creds) {
        ESP_LOGI(TAG, "WiFi bringup: APSTA (STA='%s' + AP='%s')",
                 g_config.wifi_ssid, ap_ssid);
        esp_err_t err = wifi_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "wifi_start failed (%s) → fallback to AP-only",
                     esp_err_to_name(err));
            esp_err_t ap_err = wifi_ap_start_only(ap_ssid, "fsu12345678");
            if (ap_err != ESP_OK) {
                ESP_LOGE(TAG, "wifi_ap_start_only: %s",
                         esp_err_to_name(ap_err));
            }
        }
    } else {
        ESP_LOGI(TAG, "WiFi bringup: AP-only ('%s') — no STA creds",
                 ap_ssid);
        esp_err_t ap_err = wifi_ap_start_only(ap_ssid, "fsu12345678");
        if (ap_err != ESP_OK) {
            ESP_LOGE(TAG, "wifi_ap_start_only: %s", esp_err_to_name(ap_err));
        }
    }
}

/* ============================================================
   app_main
   ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "Board bring-up (ROOT/NODE) — LoRa + AP config + STA/PPP");

    /* --- NVS --- */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* --- Event loop + netif СРАЗУ (нужны wifi_init/ppp/wifi_mqtt) --- */
    esp_err_t e = esp_netif_init();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(e));
    }
    e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(e));
    }

    /* --- sync_config (lead_ms) ДО traffic_light_init --- */
    sync_config_init();

    log_heap("app_main start");

    /* --- config: pending → NVS → embedded --- */
    bool pending_applied = run_cfg_load();
    if (pending_applied) {
        log_heap("after pending_cfg applied");
    }
    config_init();
    ensure_valid_cfg();

    bool is_root = (g_config.role == DEVICE_ROLE_MASTER);
    ESP_LOGI(TAG, "role = %s, wifi_ssid='%s'",
             is_root ? "ROOT" : "NODE", g_config.wifi_ssid);

    /* --- Peripherals --- */
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

    /* --- WiFi: AP всегда + STA (если creds) --- */
   // wifi_init();
  //  wifi_bringup(is_root);

    /* --- BLE (для настройки через телефон) --- */
    /* BLE disabled — config via WiFi AP */
//#if USE_BLE
  /*  esp_err_t ble_err = fsu_ble_init();
    if (ble_err == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "BLE disabled (CONFIG_BT_ENABLED=n) — config via WiFi AP 192.168.4.1");
} else if (ble_err != ESP_OK) {
    ESP_LOGW(TAG, "fsu_ble_init: %s", esp_err_to_name(ble_err));
} else {
    ESP_LOGI(TAG, "BLE transport initialized");
}*/
/*#else
    ESP_LOGW(TAG, "BLE disabled (CONFIG_BT_ENABLED=n) — config via WiFi AP 192.168.4.1");
#endif*/

 esp_err_t can_err= can_bus_init(500000);
 if (can_err != ESP_OK) {
        ESP_LOGW(TAG, "CAN init failed: %s", esp_err_to_name(can_err));
    }
can_err= can_bus_start();
if (can_err != ESP_OK) {
        ESP_LOGW(TAG, "CAN start failed: %s", esp_err_to_name(can_err));
    }

    /* --- RS485/CAN transport + протокол --- */
    connectivity_register_rs485_can();
    transport_init();
    node_proto_register_callbacks();

    blink_reset_rtt();

    xTaskCreate(link_monitor_task, "link_mon", 3072, NULL, 3, NULL);

    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_got_ip, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL);

    /* --- ROOT/NODE специфичные задачи (ОДИН раз) --- */
    if (is_root) {
        ESP_LOGI(TAG, "=== ROOT init: root_wd + connectivity + mux + node_proto ===");
        root_wd_start_tasks();
        connectivity_start_tasks();
        mux_app_task_start();
        node_proto_start_root_tasks();
        ESP_LOGI(TAG, "ROOT ready: LoRa CFG → traffic → PPP/MQTT + WiFi/BLE");
    } else {
        ESP_LOGI(TAG, "=== NODE init: node_proto + mux ===");
        node_proto_start_tasks();
        mux_app_task_start();
        ESP_LOGI(TAG, "NODE ready: LoRa only, waiting CFG via BLE/WiFi/MQTT");
    }
}