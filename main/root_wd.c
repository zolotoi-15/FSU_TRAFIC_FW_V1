#include "root_wd.h"
#include "app_shared.h"
#include "node_link.h"
#include "transport.h"            /* transport_is_lora() */
#include "traffic_light.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "wifi_ap.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "root_wd";

#define ROOT_AP_CHECK_MS      2000
#define NODE_COMM_LOST_MS     15000   /* потеря AP-клиента > 15 с → ALL_OFF */
#define PING_INTERVAL_MS      5000    /* TCP ping: раз в 5 с (LoRa не шлёт) */
#define RTT_PING_PERIOD_MS    2000
#define RTT_STALE_MS          10000

/* Для LoRa sync: реже, чтобы не занимать эфир */
#define LORA_SYNC_INTERVAL_MS 2000    /* sync через LoRa: раз в 2 с */
#define TCP_SYNC_INTERVAL_MS  500     /* sync через TCP: раз в 500 мс */

extern volatile uint32_t s_last_fault_clear_ms;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* ============================================================ */
/* root_ap_client_watchdog_task                                  */
/* ============================================================ */
static void root_ap_client_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_ap_client_watchdog_task started (lost > %d ms → ALL_OFF)",
             NODE_COMM_LOST_MS);
    bool     was_running = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(ROOT_AP_CHECK_MS));
            continue;
        }
        int n = wifi_ap_client_count();
        if (n < 0) { vTaskDelay(pdMS_TO_TICKS(ROOT_AP_CHECK_MS)); continue; }
        bool has_node = (n > 0);
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (has_node && !was_running) {
            ESP_LOGI(TAG, "ROOT: node connected (%d AP client) → START", n);
            if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
            }
            wifi_mqtt_reset_node_fault_tracking();
            wifi_mqtt_set_root_comm_lost(false);
            in_comm_lost_all_off = false;
            lost_since_ms = 0;

            if (node_link_server_has_client())
                node_link_server_send_cmd("fault:none", strlen("fault:none"));

            if (g_config.traffic.enabled && g_config.traffic.auto_start &&
                traffic_light_get_state() == TRAFFIC_STATE_STOPPED) {
                traffic_light_start();
                if (traffic_light_get_phase_count() > 0) {
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    wifi_mqtt_notify_phase(0, 0, dur);
                }
                if (node_link_server_has_client())
                    node_link_server_send_cmd("traffic:start",
                                              strlen("traffic:start"));
            }
            time_sync_root_push(0);
            was_running = true;
        } else if (!has_node && was_running) {
            ESP_LOGW(TAG, "ROOT: node disconnected → STOP");
            traffic_light_stop();
            if (node_link_server_has_client())
                node_link_server_send_cmd("traffic:stop",
                                          strlen("traffic:stop"));
            was_running = false;
            lost_since_ms = now;
        } else if (!has_node && !was_running && lost_since_ms != 0) {
            if (!in_comm_lost_all_off &&
                (now - lost_since_ms) >= (uint32_t)NODE_COMM_LOST_MS) {
                if (traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {
                    ESP_LOGE(TAG, "ROOT: node lost > %d ms → ALL_OFF",
                             NODE_COMM_LOST_MS);
                    SET_REMOTE_FLAG(true);
                    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                    SET_REMOTE_FLAG(false);
                    wifi_mqtt_set_root_comm_lost(true);
                    in_comm_lost_all_off = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(ROOT_AP_CHECK_MS));
    }
}

/* ============================================================ */
/* phase_sync_task — RTT ping/pong + sync                       */
/*                                                              */
/* Через TCP: ping: раз в 5 с, sync: раз в 500 мс.              */
/* Через LoRa: ping: НЕ шлём (выбивает радио из приёма),        */
/*             sync: раз в 2 с — достаточно для traffic.        */
/* ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (RTT ping/pong)");
    uint8_t  last_phase  = 0xFF;
    uint32_t last_ms     = 0;
    uint32_t last_ping_ms = 0;
    uint32_t last_rtt_ping_ms = 0;
    bool     was_client   = false;

    while (1) {
        if (g_config.role == DEVICE_ROLE_MASTER) {
            traffic_state_t st = traffic_light_get_state();
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            bool has_client = node_link_server_has_client();
            bool in_all_off = (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF);
            bool is_lora = transport_is_lora();

            if (has_client && !was_client) {
                ESP_LOGI(TAG, "phase_sync: node connected → immediate sync");
                if (st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                    uint8_t  phase = traffic_light_get_phase();
                    uint32_t dur   = traffic_light_get_phase_duration_ms();
                    uint32_t pm    = traffic_light_get_phase_ms();
                    if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) pm = 0;
                    wifi_mqtt_notify_phase(phase, pm, dur);
                    last_phase = phase;
                    last_ms    = now;
                }
                traffic_light_set_sync_delay_us(0);
                time_sync_root_push(0);
            }
            was_client = has_client;

            /* RTT ping — только через TCP, не через LoRa */
            if (has_client && !is_lora &&
                (now - last_rtt_ping_ms >= RTT_PING_PERIOD_MS)) {
                last_rtt_ping_ms = now;
                char cmd[64];
                int n = snprintf(cmd, sizeof(cmd), "ping:%lu",
                                 (unsigned long)esp_timer_get_time());
                if (n > 0) node_link_server_send_cmd(cmd, n);
            }
            if (s_last_pong_ms > 0 && (now - s_last_pong_ms) > RTT_STALE_MS) {
                blink_reset_rtt();
            }

            /* TCP ping: раз в PING_INTERVAL_MS, только когда st != RUNNING */
            if (has_client && !is_lora && st != TRAFFIC_STATE_RUNNING && !in_all_off &&
                (now - last_ping_ms >= PING_INTERVAL_MS)) {
                last_ping_ms = now;
                node_link_server_send_cmd("ping", 4);
            }

            if (has_client && !is_lora) {
                time_sync_root_push(blink_get_rtt_us());
            }

            /* sync: шлём только если st == RUNNING && !in_all_off */
            if (st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                uint8_t  phase = traffic_light_get_phase();
                uint32_t dur   = traffic_light_get_phase_duration_ms();
                uint32_t pm;
                if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
                    pm = 0;
                } else {
                    pm = traffic_light_get_phase_ms();
                    uint32_t since_clear = now - s_last_fault_clear_ms;
                    if (dur > 0 && pm > dur + TRAFFIC_PHASE_OVERDUE_MS &&
                        since_clear > 1000) {
                        ESP_LOGE(TAG, "ROOT: phase stuck → FAULT YELLOW_BLINK");
                        SET_REMOTE_FLAG(true);
                        traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
                        SET_REMOTE_FLAG(false);
                        last_phase = 0xFF;
                        vTaskDelay(pdMS_TO_TICKS(100));
                        continue;
                    }
                }
                traffic_fault_t f = traffic_light_get_fault();
                uint32_t interval = (f == TRAFFIC_FAULT_YELLOW_BLINK) ? 100 : 500;

                /* Для LoRa — реже (2 с), чтобы радио успевало принимать */
                if (is_lora) {
                    interval = LORA_SYNC_INTERVAL_MS;
                } else {
                    interval = TCP_SYNC_INTERVAL_MS;
                }

                bool need = (phase != last_phase) || (now - last_ms >= interval);
                if (need) {
                    wifi_mqtt_notify_phase(phase, pm, dur);
                    last_phase = phase;
                    last_ms    = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void root_wd_start_tasks(void)
{
    xTaskCreate(phase_sync_task, "phase_sync", 4096, NULL, 4, NULL);
    xTaskCreate(root_ap_client_watchdog_task, "root_ap_wd", 3072, NULL, 3, NULL);
}