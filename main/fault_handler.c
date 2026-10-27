#include "fault_handler.h"
#include "app_shared.h"
#include "traffic_light.h"
#include "wifi_mqtt.h"
#include "node_link.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fault_handler";

extern volatile uint32_t s_last_node_msg_ms;
extern volatile uint32_t s_last_fault_clear_ms;

#define SYNC_REMOTE_FLAG() do { s_fault_from_remote = s_fault_from_remote_ex; } while (0)
#define SET_REMOTE_FLAG(v) do { s_fault_from_remote = (v); s_fault_from_remote_ex = (v); } while (0)

static volatile bool s_fault_from_remote = false;

static void on_fault_changed(traffic_fault_t fault)
{
    SYNC_REMOTE_FLAG();
    if (s_fault_from_remote) {
        SET_REMOTE_FLAG(false);
        ESP_LOGI(TAG, "fault changed (from remote) → no echo");
        if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
            s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        return;
    }

    if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
        traffic_fault_t node_fault = wifi_mqtt_get_node_fault();
        if (node_fault != TRAFFIC_FAULT_NONE) {
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t since  = now_ms - s_last_node_msg_ms;
            if (s_last_node_msg_ms != 0 && since < 5000) {
                ESP_LOGI(TAG, "root fault NONE, but node still in fault (%d, %lu ms ago) → keep",
                         (int)node_fault, (unsigned long)since);
                return;
            }
            ESP_LOGW(TAG, "root fault NONE, node_fault=%d stale (%lu ms) → clear anyway",
                     (int)node_fault, (unsigned long)since);
        }
    }

    if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
        s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    char buf[256];
    size_t n = traffic_light_build_fault_report(buf, sizeof(buf));
    if (n == 0) {
        ESP_LOGW(TAG, "fault changed but report is empty");
        return;
    }

    ESP_LOGW(TAG, "fault changed → '%s'", buf);

    if (g_config.role == DEVICE_ROLE_MASTER) {
        if (node_link_server_has_client()) {
            esp_err_t err = node_link_server_send_cmd_nonblock(buf, n);
            if (err == ESP_OK) {
                ESP_LOGW(TAG, "root fault → node: '%s'", buf);
            } else if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "root fault → node FAILED: %s", esp_err_to_name(err));
            }
        }
        wifi_mqtt_publish_node_fault(buf, n);
    } else {
        if (node_link_client_connected()) {
            esp_err_t err = node_link_client_send_evt(buf, n);
            if (err == ESP_OK) {
                ESP_LOGW(TAG, "node fault → root: '%s'", buf);
            } else {
                ESP_LOGE(TAG, "node fault → root: send failed '%s' (%s)",
                         buf, esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "node fault → root: NOT CONNECTED, drop '%s'", buf);
        }
    }
}

static void on_phase_change(uint8_t phase, uint32_t dur_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    wifi_mqtt_notify_phase(phase, 0, dur_ms);
}

void fault_handler_register(void)
{
    traffic_light_set_fault_notify(on_fault_changed);
    traffic_light_set_phase_change_cb(on_phase_change);
}