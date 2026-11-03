/* ============================================================
   root_wd.c — ROOT: watchdog NODE по LoRa + phase sync.
   === FIX: симметричное восстановление ===
   ROOT замечает перезагрузку NODE (STATUS STOPPED после RUNNING)
   и повторно шлёт traffic:start + sync: + CFG.
   ROOT не двигает фазы сам (follow_root=true), применяет через
   apply_ms — синхронно с NODE.
   cfg_push_task — постоянный цикл (не vTaskDelete), повтор CFG
   при s_cfg_ack == false.
   ============================================================ */

#include "root_wd.h"
#include "app_shared.h"
#include "transport.h"
#include "traffic_light.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "root_wd";

#define ROOT_NODE_CHECK_MS      2000
#define NODE_COMM_LOST_MS       30000
#define LORA_SYNC_INTERVAL_MS   1000
#define CFG_PUSH_RX_WAIT_MS     3000
#define SYNC_APPLY_AHEAD_MS     500
#define CFG_RETRY_PERIOD_MS     10000

extern volatile uint32_t s_last_fault_clear_ms;
extern volatile uint32_t s_last_node_msg_ms;

volatile bool s_cfg_ack = false;
volatile bool s_cfg_pushing = false;

static volatile bool s_follow_root_set = false;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* ============================================================
   send_sync_to_node — шлёт sync: с apply_ms.
   === FIX: не static (вызывается из node_proto.c) ===
   ============================================================ */
uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t apply_ms = now_ms + SYNC_APPLY_AHEAD_MS;

    char cmd[96];
    int n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%lu:%lu",
                     phase,
                     (unsigned long)phase_ms,
                     (unsigned long)duration_ms,
                     (unsigned long)apply_ms);
    if (n > 0) {
        esp_err_t err = transport_send_cmd(cmd, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send_sync: %s failed: %s",
                     cmd, esp_err_to_name(err));
            return 0;
        }
    }
    return apply_ms;
}

/* ============================================================
   apply_sync_at — ждёт apply_ms и применяет traffic_light_force_phase.
   === FIX: не static ===
   ============================================================ */
void apply_sync_at(uint8_t phase, uint32_t phase_ms,
                   uint32_t duration_ms, uint32_t apply_ms)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t wait_ms = (int32_t)(apply_ms - now_ms);

    if (wait_ms > 0 && wait_ms <= (int32_t)(SYNC_APPLY_AHEAD_MS + 200)) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    traffic_light_force_phase(phase, phase_ms, duration_ms);
}

/* ============================================================
   root_node_watchdog_task — ALL_OFF, если NODE не было > 30 с.
   ============================================================ */
static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started (lost > %d ms → ALL_OFF)",
             NODE_COMM_LOST_MS);

    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
            continue;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        bool has_node = false;
        if (transport_is_lora() && s_last_node_msg_ms != 0) {
            has_node = (now - s_last_node_msg_ms) < NODE_COMM_LOST_MS;
        }

        if (has_node && !was_connected) {
            ESP_LOGI(TAG, "ROOT: NODE on LoRa → START");
            if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
            }
            wifi_mqtt_reset_node_fault_tracking();
            wifi_mqtt_set_root_comm_lost(false);
            in_comm_lost_all_off = false;
            lost_since_ms = 0;

            transport_send_cmd("traffic:start", strlen("traffic:start"));

            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                uint8_t phase = traffic_light_get_phase();
                uint32_t pm = traffic_light_get_phase_ms();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                uint32_t apply_ms = send_sync_to_node(phase, pm, dur);
                if (apply_ms != 0) {
                    apply_sync_at(phase, pm, dur, apply_ms);
                }
            }

            time_sync_root_push(0);
            was_connected = true;

        } else if (!has_node && was_connected) {
            ESP_LOGW(TAG, "ROOT: NODE lost on LoRa → STOP");
            traffic_light_stop();
            transport_send_cmd("traffic:stop", strlen("traffic:stop"));
            was_connected = false;
            lost_since_ms = now;
        }

        if (!has_node && !in_comm_lost_all_off && !was_connected) {
            uint32_t since = (lost_since_ms != 0)
                             ? (now - lost_since_ms)
                             : (now - boot_ms);
            if (since >= (uint32_t)NODE_COMM_LOST_MS) {
                if (traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {
                    ESP_LOGE(TAG, "ROOT: NODE absent > %d ms → ALL_OFF",
                             NODE_COMM_LOST_MS);
                    SET_REMOTE_FLAG(true);
                    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                    SET_REMOTE_FLAG(false);
                    wifi_mqtt_set_root_comm_lost(true);
                    in_comm_lost_all_off = true;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
    }
}

/* ============================================================
   phase_sync_task — sync фазы через LoRa с apply_ms.
   ROOT не двигает фазы сам (follow_root=true).
   ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (LoRa sync with apply_ms)");

    if (!s_follow_root_set) {
        traffic_light_set_follow_root(true);
        s_follow_root_set = true;
    }

    uint8_t  last_phase  = 0xFF;
    uint32_t last_ms     = 0;

    while (1) {
        if (g_config.role == DEVICE_ROLE_MASTER) {
            if (s_cfg_pushing) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            traffic_state_t st = traffic_light_get_state();
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            bool in_all_off = (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF);
            bool has_node = transport_is_lora() && s_last_node_msg_ms != 0 &&
                            (now - s_last_node_msg_ms) < NODE_COMM_LOST_MS;

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off) {
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

                bool need = (phase != last_phase) ||
                            (now - last_ms >= LORA_SYNC_INTERVAL_MS);
                if (need) {
                    uint32_t apply_ms = send_sync_to_node(phase, pm, dur);
                    if (apply_ms != 0) {
                        apply_sync_at(phase, pm, dur, apply_ms);
                    }
                    last_phase = phase;
                    last_ms    = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   === FIX: cfg_push_task — постоянный цикл (не vTaskDelete) ===
   Раз в CFG_RETRY_PERIOD_MS (10 с) проверяет: если s_cfg_ack == false
   (NODE не подтвердил CFG или перезагрузился) — повторяет CFG.
   ============================================================ */
static void cfg_push_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_push_task started (continuous retry)");

    /* Первый запуск — ждём NODE */
    uint32_t waited = 0;
    while (!transport_is_lora() && waited < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    if (!transport_is_lora()) {
        ESP_LOGE(TAG, "cfg_push: LoRa not ready");
    }

    waited = 0;
    while (s_last_node_msg_ms == 0 && waited < 60000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CFG_RETRY_PERIOD_MS));

        if (g_config.role != DEVICE_ROLE_MASTER) continue;
        if (!transport_is_lora()) continue;
        if (s_last_node_msg_ms == 0) continue;
        if (s_cfg_pushing) continue;
        if (s_cfg_ack) continue;   /* CFG подтверждён — не шлём */

        ESP_LOGI(TAG, "cfg_push: retry (s_cfg_ack=false)");

        extern bool config_export_json(char *b, size_t *l);
        char *buf = malloc(16384);
        size_t blen = 16384;
        if (!buf || !config_export_json(buf, &blen)) {
            free(buf);
            continue;
        }

        s_cfg_pushing = true;

        /* подождать 3 с, чтобы NODE вернулся в RX */
        vTaskDelay(pdMS_TO_TICKS(CFG_PUSH_RX_WAIT_MS));

        for (int attempt = 0; attempt < 10; attempt++) {
            if (s_cfg_ack) break;

            transport_send_cmd("cfg:reset", strlen("cfg:reset"));
            vTaskDelay(pdMS_TO_TICKS(200));

            ESP_LOGI(TAG, "cfg_push → LoRa attempt %d/10 (%u bytes)",
                     attempt + 1, (unsigned)blen);
            esp_err_t err = transport_send_cfg(buf, blen);
            if (err == ESP_OK) {
                for (int w = 0; w < 50; w++) {
                    if (s_cfg_ack) break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (s_cfg_ack) {
                    ESP_LOGI(TAG, "cfg_push: cfg:ok received");
                    break;
                }
            } else {
                ESP_LOGW(TAG, "cfg_push: failed (attempt %d): %s",
                         attempt + 1, esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        s_cfg_pushing = false;
        free(buf);

        if (s_cfg_ack) {
            ESP_LOGI(TAG, "cfg_push: success — wait %d ms before next check",
                     CFG_RETRY_PERIOD_MS);
        } else {
            ESP_LOGE(TAG, "cfg_push: NODE did not confirm cfg after 10 attempts");
        }
    }
}

void root_wd_start_tasks(void)
{
    xTaskCreate(phase_sync_task,          "phase_sync",   4096, NULL, 4, NULL);
    xTaskCreate(root_node_watchdog_task,  "root_node_wd", 8192, NULL, 3, NULL);
    xTaskCreate(cfg_push_task,            "cfg_push",     8192, NULL, 3, NULL);
}