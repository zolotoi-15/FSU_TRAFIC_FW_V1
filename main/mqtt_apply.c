/* ============================================================
   mqtt_apply.c — применение конфига к runtime.

   === ПРАВИЛО ===
   ROOT: ALL_OFF при старте. START — только из root_node_watchdog_task
         ПОСЛЕ cfg:ok от NODE.
   NODE: ALL_OFF при старте. START — только по traffic:start от ROOT.
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "wifi_ap.h"
#include "node_link.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "mqtt_apply";

#define PHASE_SYNC_MIN_INTERVAL_MS  250

static volatile uint8_t  s_last_sync_phase = 0xFF;
static volatile uint32_t s_last_sync_ms    = 0;

void mqtt_apply_config_runtime(void)
{
    traffic_pins_single_t ps = {
        .pin_red    = g_config.traffic.pins.single.red,
        .pin_yellow = g_config.traffic.pins.single.yellow,
        .pin_green  = g_config.traffic.pins.single.green,
    };
    traffic_light_set_pins_single(&ps);
    traffic_pins_cross_t pc = {
        .a_north_red    = g_config.traffic.pins.cross.a_north.red,
        .a_north_yellow = g_config.traffic.pins.cross.a_north.yellow,
        .a_north_green  = g_config.traffic.pins.cross.a_north.green,
        .a_south_red    = g_config.traffic.pins.cross.a_south.red,
        .a_south_yellow = g_config.traffic.pins.cross.a_south.yellow,
        .a_south_green  = g_config.traffic.pins.cross.a_south.green,
        .b_west_red     = g_config.traffic.pins.cross.b_west.red,
        .b_west_yellow  = g_config.traffic.pins.cross.b_west.yellow,
        .b_west_green   = g_config.traffic.pins.cross.b_west.green,
        .b_east_red     = g_config.traffic.pins.cross.b_east.red,
        .b_east_yellow  = g_config.traffic.pins.cross.b_east.yellow,
        .b_east_green   = g_config.traffic.pins.cross.b_east.green,
    };
    traffic_light_set_pins_cross(&pc);
    traffic_pins_cross_ped_t pp = {
        .axis_a_red     = g_config.traffic.pins.cross_ped.axis_a.red,
        .axis_a_yellow  = g_config.traffic.pins.cross_ped.axis_a.yellow,
        .axis_a_green   = g_config.traffic.pins.cross_ped.axis_a.green,
        .axis_a2_red    = g_config.traffic.pins.cross_ped.axis_a2.red,
        .axis_a2_yellow = g_config.traffic.pins.cross_ped.axis_a2.yellow,
        .axis_a2_green  = g_config.traffic.pins.cross_ped.axis_a2.green,
        .axis_a3_red    = g_config.traffic.pins.cross_ped.axis_a3.red,
        .axis_a3_yellow = g_config.traffic.pins.cross_ped.axis_a3.yellow,
        .axis_a3_green  = g_config.traffic.pins.cross_ped.axis_a3.green,
        .ped_red        = g_config.traffic.pins.cross_ped.ped_red,
        .ped_green      = g_config.traffic.pins.cross_ped.ped_green,
    };
    traffic_light_set_pins_cross_ped(&pp);

    traffic_light_stop();
    esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    if (mr != ESP_OK) ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));
    traffic_light_reset();

    if (!g_config.traffic.enabled || !g_config.traffic.auto_start) {
        ESP_LOGI(TAG, "auto_start disabled → traffic STOPPED");
        return;
    }

    /* === ПРАВИЛО: ROOT и NODE при старте — ALL_OFF.
       START делает root_node_watchdog_task (ROOT) после cfg:ok,
       и NODE — по traffic:start от ROOT. === */
    if (g_config.role == DEVICE_ROLE_MASTER) {
        ESP_LOGI(TAG, "ROOT: ALL_OFF at start (wait for cfg:ok from NODE)");
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        return;
    } else {
        ESP_LOGI(TAG, "NODE: ALL_OFF at start (wait for CFG + traffic:start from ROOT)");
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        return;
    }
}

void mqtt_notify_phase(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!node_link_server_has_client()) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t min_interval = PHASE_SYNC_MIN_INTERVAL_MS;

    if (phase == s_last_sync_phase &&
        (now_ms - s_last_sync_ms) < min_interval) {
        return;
    }
    s_last_sync_phase = phase;
    s_last_sync_ms    = now_ms;

    char cmd[128];
    int n;
    if (traffic_light_get_fault() == TRAFFIC_FAULT_YELLOW_BLINK) {
        int blink_on = traffic_light_get_fault_blink_state() ? 1 : 0;
        n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%lu:%lu:%d",
                     phase,
                     (unsigned long)phase_ms,
                     (unsigned long)duration_ms,
                     (unsigned long)now_ms,
                     blink_on);
    } else {
        n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%lu:%lu",
                     phase,
                     (unsigned long)phase_ms,
                     (unsigned long)duration_ms,
                     (unsigned long)now_ms);
    }
    if (n > 0) {
        esp_err_t err = node_link_server_send_cmd_nonblock(cmd, n);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "sync send failed: %s", esp_err_to_name(err));
        }
    }
}