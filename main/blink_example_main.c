/* ============================================================
   blink_example_main.c — ТОЛЬКО app_main().
   === ROOT: старт только AP (ch=6). STA включится, если PPPoS fail. ===
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
#include "time_sync.h"
#include "ds1307.h"
#include "i2c_bus.h"
#include "connectivity.h"
#include "fault_handler.h"
#include "buttons.h"
#include "node_proto.h"
#include "root_wd.h"
#include "app_shared.h"

extern const uint8_t config_json_start[] asm("_binary_config_json_start");
extern const uint8_t config_json_end[]   asm("_binary_config_json_end");

static const char *TAG = "board";

#define AP_PASSWORD          "fsu12345678"
#define NODE_LINK_PORT_FIX   3333

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

void app_main(void)
{
    ESP_LOGI(TAG, "Board bring-up (root/node)");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    load_embedded_config_if_needed();
    config_init();

    bool is_root = (g_config.role == DEVICE_ROLE_MASTER);
    ESP_LOGI(TAG, "role = %s", is_root ? "ROOT" : "NODE");

    board_hw_init();

    ds1307_config_t ds1307_cfg = {
        .ds1307_device.scl_speed_hz = 100000,
        .ds1307_device.device_address = DS1307_ADDRESS,
    };
    if (ds1307_init(g_i2c_bus, &ds1307_cfg, &rtc_handle) != ESP_OK) {
        rtc_handle = NULL;
    } else if (i2c_master_probe(g_i2c_bus, DS1307_ADDRESS, 50) != ESP_OK) {
        rtc_handle = NULL;
    }
    time_sync_init();

    if (sr_init() == ESP_OK) {
        sr_all_off();
        connectivity_register_rs485_can();
    }

    mux_init();
    mux_data_init();

    esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    if (mr != ESP_OK) ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));

    traffic_light_init();

    ESP_LOGI(TAG, "Traffic state after init: %s (all outputs OFF)",
             traffic_light_state_str());

    fault_handler_register();
    if (!is_root) traffic_light_set_follow_root(true);

    if (is_root) {
        wifi_mqtt_apply_config_runtime();
    }

    buttons_start_tasks();
    mux_app_task_start();

    wifi_init();

    if (is_root) {
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                   &on_sta_got_ip, NULL);

        // === FIX: root стартует ТОЛЬКО AP (STA включится, если PPPoS fail) ===
        ESP_LOGI(TAG, "ROOT: starting AP-only mode (ch=6, beacon awake)");
        esp_err_t err = wifi_ap_start_only("FSU_Traffic_AP", AP_PASSWORD);
        if (err != ESP_OK) ESP_LOGE(TAG, "wifi_ap_start_only: %s",
                                    esp_err_to_name(err));

        node_proto_register_callbacks();
        node_link_server_start(NODE_LINK_PORT_FIX);

        root_wd_start_tasks();
    } else {
        ESP_LOGI(TAG, "NODE: scanning for FSU_*_AP ...");
        g_config.wifi_ssid[0] = '\0';
        g_config.wifi_password[0] = '\0';
        wifi_start();

        char    ap_ssid[33] = {0};
        int8_t  ap_rssi = -127;
        uint8_t ap_ch   = 0;
        if (wifi_scan_for_fsu_ap(ap_ssid, sizeof(ap_ssid),
                                 &ap_rssi, &ap_ch, "FSU_") == ESP_OK) {
            ESP_LOGI(TAG, "NODE: connecting to '%s' (rssi=%d ch=%u)",
                     ap_ssid, ap_rssi, ap_ch);

            esp_wifi_set_channel(ap_ch, WIFI_SECOND_CHAN_NONE);

            wifi_config_t sta_cfg = {0};
            strncpy((char*)sta_cfg.sta.ssid, ap_ssid,
                    sizeof(sta_cfg.sta.ssid) - 1);
            strncpy((char*)sta_cfg.sta.password, AP_PASSWORD,
                    sizeof(sta_cfg.sta.password) - 1);
            sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            sta_cfg.sta.pmf_cfg.capable  = true;
            sta_cfg.sta.pmf_cfg.required = false;
            esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "NODE: no FSU_*_AP found, will retry");
        }

        node_proto_register_callbacks();
        node_proto_client_start("192.168.4.1", NODE_LINK_PORT_FIX);

        ESP_LOGI(TAG, "NODE: waiting for root (traffic STOPPED until cfg from root)");

        node_proto_start_tasks();
    }

    connectivity_start_tasks();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}