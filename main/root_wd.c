/* ============================================================
   root_wd.c — ROOT: CFG на NODE → cfg:ok → START ROOT+NODE.

   === ПОРЯДОК ===
   1. ROOT в ALL_OFF (mqtt_apply_config_runtime).
   2. cfg_push_task шлёт CFG на NODE, пока не получит cfg:ok.
   3. После cfg:ok root_node_watchdog_task:
      a. отправляет traffic:start на NODE (ПЕРВЫМ)
      b. запускает traffic_light_start() на ROOT
      c. отправляет sync: с pm=0
      d. ставит s_cfg_applied_and_started = true
   4. phase_sync_task начинает слать sync: только после
      s_cfg_applied_and_started.

   === FIX ===
   - traffic:start ПЕРВЫМ, traffic_light_start() ВТОРЫМ.
   - ping не шлётся, пока !s_cfg_ack (не мешаем CFG).
   - cfg_push_task: фиксированная задержка 5 с вместо ожидания STATUS.
   - traffic:start повторяется не более 3 раз.
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

#define ROOT_NODE_CHECK_MS          2000
#define NODE_COMM_LOST_MS           60000
#define LORA_SYNC_INTERVAL_MS       1000
#define CFG_PUSH_RX_WAIT_MS         3000
#define SYNC_APPLY_AHEAD_MS         500
#define CFG_RETRY_PERIOD_MS         10000
#define NO_CFG_ALL_OFF_MS           (60 * 1000)
#define TRAFFIC_START_MAX_RETRIES   3
/* === FIX: фиксированная задержка перед первым CFG === */
#define CFG_INITIAL_DELAY_MS        5000
/* === END FIX === */

extern volatile uint32_t s_last_fault_clear_ms;
extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool s_node_running;

volatile bool s_cfg_ack = false;
volatile bool s_cfg_pushing = false;

/* === Глобальный флаг: CFG применён + traffic:start отправлен === */
volatile bool s_cfg_applied_and_started = false;

static volatile bool s_follow_root_set = false;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* === Мгновенный sync: apply_ms = 0 === */
uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;
    if (s_cfg_pushing) return 0;

    char cmd[96];
    int n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%lu:0",
                     phase,
                     (unsigned long)phase_ms,
                     (unsigned long)duration_ms);
    if (n > 0) {
        esp_err_t err = transport_send_cmd(cmd, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send_sync: %s failed: %s",
                     cmd, esp_err_to_name(err));
            return 0;
        }
    }
    return 1;
}

void apply_sync_at(uint8_t phase, uint32_t phase_ms,
                   uint32_t duration_ms, uint32_t apply_ms)
{
    (void)apply_ms;
    traffic_light_force_phase(phase, phase_ms, duration_ms);
}

/* ============================================================
   root_node_watchdog_task — ROOT: ждёт cfg:ok,
   затем шлёт traffic:start на NODE ПЕРВЫМ,
   запускает traffic_light_start() на ROOT,
   шлёт sync:, ставит s_cfg_applied_and_started.
   ============================================================ */
static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started");

    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);

    uint32_t node_seen_ms = 0;
    bool     no_cfg_all_off = false;

    uint32_t last_start_ms = 0;
    uint32_t start_retry_count = 0;

    if (!s_cfg_ack) {
        ESP_LOGW(TAG, "root_wd: no cfg:ok at start → ALL_OFF");
        traffic_light_stop();
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        wifi_mqtt_set_root_comm_lost(true);
    }

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
        if (s_cfg_pushing) {
            has_node = true;
        }

        if (has_node && node_seen_ms == 0) {
            node_seen_ms = now;
        }
        if (s_cfg_ack) {
            no_cfg_all_off = false;
        }

        /* === ЗАПУСК: cfg:ok + NODE на связи → один раз === */
        if (s_cfg_ack && has_node && !s_cfg_applied_and_started) {
            ESP_LOGW(TAG, "cfg:ok + NODE present → START на ROOT и NODE");

            /* 1. Снять ALL_OFF на ROOT (без START) */
            if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
            }

            /* 2. traffic:start на NODE — ПЕРВЫМ */
            transport_send_cmd("traffic:start", strlen("traffic:start"));

            /* 3. Запустить traffic на ROOT */
            traffic_light_set_follow_root(true);
            traffic_light_start();

            /* 4. sync: с pm=0 */
            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                uint8_t phase = traffic_light_get_phase();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                send_sync_to_node(phase, 0, dur);
                traffic_light_force_phase(phase, 0, dur);
            }

            time_sync_root_push(0);

            /* 5. Разрешаем phase_sync_task слать sync: */
            s_cfg_applied_and_started = true;

            was_connected = true;
            lost_since_ms = 0;
            in_comm_lost_all_off = false;
            last_start_ms = now;
            start_retry_count = 0;
            wifi_mqtt_reset_node_fault_tracking();
            wifi_mqtt_set_root_comm_lost(false);

            ESP_LOGI(TAG, "ROOT: cfg applied + traffic:start + sync: sent");
        }

        /* === Повтор traffic:start, пока NODE не RUNNING (не более 3 раз) === */
        if (s_cfg_applied_and_started && has_node && !s_node_running) {
            if (start_retry_count < TRAFFIC_START_MAX_RETRIES &&
                (now - last_start_ms) >= 2000) {
                ESP_LOGI(TAG, "ROOT: NODE not RUNNING → re-send traffic:start (%u/%u)",
                         start_retry_count + 1, TRAFFIC_START_MAX_RETRIES);
                transport_send_cmd("traffic:start", strlen("traffic:start"));
                last_start_ms = now;
                start_retry_count++;
            } else if (start_retry_count >= TRAFFIC_START_MAX_RETRIES) {
                ESP_LOGW(TAG, "ROOT: traffic:start retries exhausted (%u), "
                              "waiting for STATUS RUNNING",
                         TRAFFIC_START_MAX_RETRIES);
                start_retry_count = TRAFFIC_START_MAX_RETRIES + 1;
            }
        }

        if (has_node && !was_connected && !s_cfg_applied_and_started) {
            ESP_LOGI(TAG, "ROOT: NODE on LoRa, waiting for cfg:ok → ALL_OFF");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            was_connected = true;
        }

        if (!has_node && was_connected) {
            ESP_LOGW(TAG, "ROOT: NODE lost on LoRa → STOP + ALL_OFF");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            transport_send_cmd("traffic:stop", strlen("traffic:stop"));
            was_connected = false;
            lost_since_ms = now;
            start_retry_count = 0;
        }

        if (has_node && !s_cfg_ack && node_seen_ms != 0 && !no_cfg_all_off &&
            (now - node_seen_ms) >= NO_CFG_ALL_OFF_MS) {
            ESP_LOGE(TAG, "ROOT: NODE present but cfg not ack'd > %d ms → ALL_OFF",
                     NO_CFG_ALL_OFF_MS);
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            wifi_mqtt_set_root_comm_lost(true);
            no_cfg_all_off = true;
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
                s_cfg_applied_and_started = false;
                start_retry_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
    }
}

/* ============================================================
   phase_sync_task — ROOT шлёт sync: при смене фазы,
   но только если s_cfg_applied_and_started == true.
   ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (instant sync)");

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
            if (!s_cfg_ack) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            /* не шлём sync:, пока root_wd не отправил traffic:start */
            if (!s_cfg_applied_and_started) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            traffic_state_t st = traffic_light_get_state();
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            bool in_all_off = (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF);
            bool has_node = transport_is_lora() && s_last_node_msg_ms != 0 &&
                            (now - s_last_node_msg_ms) < NODE_COMM_LOST_MS;
            if (s_cfg_pushing) has_node = true;

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                uint8_t  phase = traffic_light_get_phase();
                uint32_t dur   = traffic_light_get_phase_duration_ms();
                uint32_t pm    = traffic_light_get_phase_ms();

                bool phase_changed = (phase != last_phase);
                bool fallback      = (now - last_ms >= LORA_SYNC_INTERVAL_MS);

                if (phase_changed || fallback) {
                    uint32_t pm_send = pm;
                    if (dur > 0 && pm_send >= dur) pm_send = 0;

                    uint32_t r = send_sync_to_node(phase, pm_send, dur);
                    if (r != 0) {
                        traffic_light_force_phase(phase, pm_send, dur);
                    }
                    last_phase = phase;
                    last_ms    = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
   cfg_push_task — ROOT шлёт CFG на NODE, пока не получит cfg:ok.
   === FIX: фиксированная задержка 5 с вместо ожидания STATUS ===
   ============================================================ */
static void cfg_push_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_push_task started (continuous retry)");

    uint32_t waited = 0;
    while (!transport_is_lora() && waited < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    if (!transport_is_lora()) {
        ESP_LOGE(TAG, "cfg_push: LoRa not ready");
    }

    /* === FIX: фиксированная задержка 5 с — NODE успевает поднять LoRa === */
    ESP_LOGI(TAG, "cfg_push: initial delay %d ms before first CFG",
             CFG_INITIAL_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(CFG_INITIAL_DELAY_MS));
    /* === END FIX === */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CFG_RETRY_PERIOD_MS));

        if (g_config.role != DEVICE_ROLE_MASTER) continue;
        if (!transport_is_lora()) continue;
        if (s_cfg_pushing) continue;
        if (s_cfg_ack) continue;

        ESP_LOGI(TAG, "cfg_push: retry (s_cfg_ack=false)");

        extern bool config_export_json(char *b, size_t *l);
        char *buf = malloc(8192);
        size_t blen = 8192;
        if (!buf || !config_export_json(buf, &blen)) {
            free(buf);
            continue;
        }

        s_cfg_pushing = true;

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
            vTaskDelay(pdMS_TO_TICKS(10000));
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