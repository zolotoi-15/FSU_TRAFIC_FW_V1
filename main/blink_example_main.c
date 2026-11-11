/* ============================================================
   blink_example_main.c — PRODUCTION app_main().

   === FIX LINK-LOSS-SMART ===
       link_monitor_task: счётчик пропусков инкрементится ТОЛЬКО
       если в окне LINK_*_EXPECT_MS реально не было пакета.
       Раньше счётчик рос слепо по таймеру и ложно срабатывал
       (has_node=0 при живом NODE).

   === FIX SYNC-DELAY-ZERO ===
       blink_reset_rtt() вызывается при старте — s_sync_delay_us
       = 0. ROOT не шлёт delay:<us>, NODE игнорирует (см.
       node_proto.c). Без этого NODE мог ждать до 2 с перед
       apply_mask → отставание от ROOT.

   === FIX BLE-TRANSPORT ===
       ble_transport_init() вызывается после transport_init().
       GATT-сервер для конфигурации с телефона (config.get/set,
       lora.get/set, traffic.start/stop, reboot).
       BLE-имя: ROOT → "FSU_ROOT", NODE → "FSU_NODE_XXXX".
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
#include "app_shared.h"
#include "ble_transport.h"

extern const uint8_t config_json_start[] asm("_binary_config_json_start");
extern const uint8_t config_json_end[]   asm("_binary_config_json_end");

static const char *TAG = "board";

#define NVS_NS_PENDING   "pending_cfg"
#define NVS_KEY_LEN      "len"
#define NVS_KEY_CHUNK    "chunk_%d"
#define PENDING_CHUNK_SZ 4096

#define LINK_ROOT_EXPECT_MS   10000
#define LINK_NODE_EXPECT_MS   10000

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
    traffic_light_set_sync_delay_us(0);   /* === FIX SYNC-DELAY-ZERO === */
}

/* ============================================================
   === LINK LOSS: helpers ===
   ============================================================ */
void link_monitor_note_status_rx(void) { s_root_missed_status = 0; }
void link_monitor_note_ping_rx(void)   { s_node_missed_ping   = 0; }

bool link_monitor_root_lost(void) { return s_root_missed_status >= LINK_LOST_MISSED_PKTS; }
bool link_monitor_node_lost(void) { return s_node_missed_ping   >= LINK_LOST_MISSED_PKTS; }

/* ============================================================
   link_monitor_task
   === FIX LINK-LOSS-SMART ===
   ============================================================ */
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

                /* === FIX LINK-LOSS-SMART:
                   Проверяем, был ли STATUS реально за окно.
                   s_last_node_msg_ms обновляется в transport.c
                   на КАЖДЫЙ RX от NODE (STATUS/EVT/T_CHANNELS). === */
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
                        ESP_LOGE(TAG, "ROOT: %u missed STATUS → link LOST",
                                 LINK_LOST_MISSED_PKTS);
                    }
                } else {
                    /* Был пакет — сбрасываем счётчик (на случай
                       гонки с link_monitor_note_status_rx). */
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
                        ESP_LOGE(TAG, "NODE: %u missed ping/CMD → link LOST",
                                 LINK_LOST_MISSED_PKTS);
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
        esp_err_t err = wifi_mqtt_init();
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "wifi_mqtt_init: deferred (no IP yet)");
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "wifi_mqtt_init: %s", esp_err_to_name(err));
        }
    }
}

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
        ESP_LOGE(TAG, "ensure_valid_cfg: embedded config.json is EMPTY — "
                      "cannot recover");
        return;
    }

    ESP_LOGW(TAG, "ensure_valid_cfg: g_config has no sequence/stages → "
                  "force-load embedded config.json (%u bytes)", (unsigned)len);
    bool ok = config_load_json((const char *)config_json_start, len);
    if (!ok) {
        ESP_LOGE(TAG, "ensure_valid_cfg: force-load FAILED — "
                      "device will use defaults (CONFLICTING stages!)");
    } else {
        ESP_LOGI(TAG, "ensure_valid_cfg: force-load OK "
                      "(stages=%u seq=%u ped_mode=%u)",
                 g_config.traffic.stage_count,
                 g_config.traffic.sequence_count,
                 g_config.traffic.pedestrian.request_mode);
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

    bool pending_applied = run_cfg_load();
    if (pending_applied) {
        log_heap("after pending_cfg applied");
    }

    config_init();

    ensure_valid_cfg();

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

    /* === FIX BLE-TRANSPORT ===
       GATT-сервер для конфигурации с телефона. Имя:
       ROOT → "FSU_ROOT", NODE → "FSU_NODE_XXXX".
       Не блокирует LoRa — работает параллельно. */
    esp_err_t ble_err = fsu_ble_init();          /* === FIX BLE-NAME-CLASH === */
if (ble_err != ESP_OK) {
    ESP_LOGW(TAG, "fsu_ble_init: %s", esp_err_to_name(ble_err));
} else {
    ESP_LOGI(TAG, "BLE transport initialized");
}
    /* === END FIX BLE-TRANSPORT === */

    /* === FIX SYNC-DELAY-ZERO ===
       s_sync_delay_us = 0. NODE не ждёт перед apply_mask. */
    blink_reset_rtt();
    /* === END FIX SYNC-DELAY-ZERO === */

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
        ESP_LOGI(TAG, "ROOT ready: LoRa CFG → traffic → PPP → MQTT + BLE");
    } else {
        ESP_LOGI(TAG, "=== NODE init (no Wi-Fi STA) ===");
        node_proto_start_tasks();
        mux_app_task_start();
        ESP_LOGI(TAG, "NODE ready: LoRa only, waiting for CFG + BLE config");
    }
}