/* ============================================================
   root_wd.c — ROOT: phase-sync + heartbeat.

   === FIX SET-REMOTE-FLAG ===
       Макрос SET_REMOTE_FLAG определён локально в этом файле
       (в исходной версии он был утерян при рефакторинге).
       Используется везде, где ROOT сам меняет fault, чтобы
       fault_handler не сделал echo.

   === FIX NO-PRE-DELAY ===
       Убран vTaskDelay(SYNC_PRE_DELAY_MS) перед c:. Раньше pm
       читался ДО sleep 50 мс, отправлялся ПОСЛЕ — NODE получал
       устаревший pm.

   === FIX PM-AT-SEND ===
       pm вычисляется внутри send_channels_to_node_impl в момент
       snprintf (hint=UINT32_MAX), а не в phase_sync_task.

   === FIX C-IMMEDIATE-AFTER-START ===
       Убран vTaskDelay(300) между traffic:start и первым c:.
       Если NODE ещё не RUNNING — буферизация в node_proto.c
       сохранит c: и применит после traffic:start.

   === FIX C-REFRESH-300 ===
       LORA_SYNC_INTERVAL_MS 500 → 300: меньше дрейф между c:.

   === FIX SYNC-HANDSHAKE ===
       При старте и потере NODE — sync_handshake_root_start().
       phase_sync_task в самом начале вызывает
       sync_handshake_root_tick(); пока is_syncing() — c: не шлём.
   ============================================================ */

#include "root_wd.h"
#include "app_shared.h"
#include "sync_config.h"
#include "sync_handshake.h"
#include "transport.h"
#include "traffic_light.h"
#include "traffic_internal.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "root_wd";

#define ROOT_NODE_CHECK_MS          200
#define NODE_COMM_LOST_MS           4000
#define HEARTBEAT_INTERVAL_MS       2000
#define SYNC_AFTER_START_DELAY_MS   2000

volatile bool     s_cfg_ack                 = true;
volatile bool     s_cfg_pushing             = false;
volatile bool     s_cfg_applied_and_started = true;
volatile bool     s_cfg_req_pending         = false;
volatile bool     s_force_resync_pending    = false;
volatile uint32_t s_cfg_last_ok_ms          = 0;

extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool     s_node_running;

extern uint32_t blink_get_rtt_us(void);

extern volatile bool s_hold_active;
extern volatile bool s_insert_pending;

/* ============================================================
   === SET_REMOTE_FLAG ===
   Пометка "мы сами меняем fault", чтобы fault_handler не сделал
   echo. Переменная объявлена в app_shared.h, определена в
   node_proto.c (volatile bool s_fault_from_remote_ex).
   ============================================================ */
#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)
/* === END SET_REMOTE_FLAG === */

/* ============================================================
   send_channels_to_node_impl
   === FIX PM-AT-SEND ===
   phase_ms_hint == UINT32_MAX → pm берётся СЕЙЧАС из
   traffic_light_get_phase_ms(), в момент snprintf, а не в
   вызывающем коде. Иначе между чтением pm и отправкой c:
   накапливается задержка (vTaskDelay + LoRa ToA).
   ============================================================ */
static uint32_t send_channels_to_node_impl(uint8_t phase,
                                           uint32_t phase_ms_hint,
                                           uint32_t duration_ms, bool force,
                                           bool allow_display)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;
    if (g_config.traffic.sequence_count == 0) return 0;
    if (phase >= g_config.traffic.sequence_count) return 0;

    static uint8_t  s_last_phase = 0xFF;
    static uint32_t s_last_ms    = 0;
    static uint16_t s_last_on    = 0;
    static uint16_t s_last_blink = 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    uint16_t on_mask = 0, blink_mask = 0;
    traffic_light_get_stage_masks(phase, &on_mask, &blink_mask);

    if (allow_display) {
        uint8_t disp_ch = g_config.traffic.pins.cross_ped.ped_display;
        if (disp_ch < 16) {
            bool need_display = (s_ped_request || s_insert_pending) &&
                                !s_ped_display_done &&
                                !stage_is_ped_green_or_blink(phase);
            if (need_display) {
                on_mask |= (1u << disp_ch);
            }
        }
    }

    if (!force && phase == s_last_phase &&
        on_mask == s_last_on && blink_mask == s_last_blink &&
        (now_ms - s_last_ms) < (LORA_SYNC_INTERVAL_MS - 100)) {
        return 0;
    }

    s_last_phase = phase;
    s_last_ms    = now_ms;
    s_last_on    = on_mask;
    s_last_blink = blink_mask;

    if (duration_ms == 0) {
        duration_ms = traffic_light_stage_duration_ms(phase);
    }

    /* === FIX PM-AT-SEND === */
    uint32_t pm_use;
    if (phase_ms_hint == UINT32_MAX) {
        pm_use = traffic_light_get_phase_ms();
    } else {
        pm_use = phase_ms_hint;
    }
    if (duration_ms > 0 && pm_use >= duration_ms) pm_use = 0;
    /* === END FIX PM-AT-SEND === */

    /* === FIX END-UTC-CLAMP === */
    uint64_t now_utc    = time_sync_get_utc_ms();
    uint64_t end_utc_ms = 0;
    if (now_utc != 0) {
        uint32_t remain = (pm_use < duration_ms)
                          ? (duration_ms - pm_use)
                          : duration_ms;
        end_utc_ms = now_utc + (uint64_t)remain;
    }
    /* === END FIX END-UTC-CLAMP === */

    char cmd[128];
    int n = snprintf(cmd, sizeof(cmd), "c:%u:%lu:%04X:%04X:%lu:%llu",
                     phase,
                     (unsigned long)pm_use,
                     (unsigned)on_mask,
                     (unsigned)blink_mask,
                     (unsigned long)duration_ms,
                     (unsigned long long)end_utc_ms);
    if (n <= 0 || n >= (int)sizeof(cmd)) return 0;

    esp_err_t err = transport_send_cmd_noack(cmd, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_channels: '%s' failed: %s",
                 cmd, esp_err_to_name(err));
        return 0;
    }
    ESP_LOGI(TAG, "-> NODE %s", cmd);
    return 1;
}

uint32_t send_channels_to_node(uint8_t phase, uint32_t phase_ms,
                               uint32_t duration_ms, bool force)
{
    return send_channels_to_node_impl(phase, phase_ms, duration_ms,
                                      force, true);
}

uint32_t send_channels_to_node_no_display(uint8_t phase, uint32_t phase_ms,
                                          uint32_t duration_ms, bool force)
{
    return send_channels_to_node_impl(phase, phase_ms, duration_ms,
                                      force, false);
}

uint32_t send_all_off_to_node(uint32_t duration_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;

    uint64_t now_utc = time_sync_get_utc_ms();
    uint64_t end_utc_ms = (now_utc != 0)
                          ? (now_utc + (uint64_t)duration_ms) : 0;

    char cmd[80];
    int n = snprintf(cmd, sizeof(cmd), "c:0:0:0000:0000:%lu:%llu",
                     (unsigned long)duration_ms,
                     (unsigned long long)end_utc_ms);
    if (n <= 0) return 0;

    esp_err_t err = transport_send_cmd_noack(cmd, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_all_off: '%s' failed: %s",
                 cmd, esp_err_to_name(err));
        return 0;
    }
    ESP_LOGW(TAG, "-> NODE %s (ALL_OFF)", cmd);
    return 1;
}

uint32_t send_sync_to_node_async_ex(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms, bool force,
                                    uint8_t *out_next_phase)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;

    uint32_t r = send_channels_to_node(phase, phase_ms, duration_ms, force);

    if (out_next_phase && r > 0 && g_config.traffic.sequence_count > 0) {
        *out_next_phase = (phase + 1) % g_config.traffic.sequence_count;
    }
    return r;
}

uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase)
{
    return send_sync_to_node_async_ex(phase, phase_ms, duration_ms,
                                      false, out_next_phase);
}

uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms)
{
    uint32_t r = send_channels_to_node(phase, phase_ms, duration_ms, false);
    if (r > 0) {
        traffic_light_force_phase(phase, phase_ms, duration_ms);
    }
    return r;
}

uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_utc)
{
    (void)apply_utc;
    traffic_light_force_phase(phase, phase_ms, duration_ms);
    return 1;
}

void apply_sync_at(uint8_t phase, uint32_t phase_ms,
                   uint32_t duration_ms, uint32_t apply_ms)
{
    (void)apply_ms;
    traffic_light_force_phase(phase, phase_ms, duration_ms);
}

/* ============================================================
   root_apply_new_config_from_mqtt
   ============================================================ */
void root_apply_new_config_from_mqtt(const char *json, int len)
{
    if (g_config.role != DEVICE_ROLE_MASTER) {
        ESP_LOGW(TAG, "root_apply_new_config: not ROOT, skip");
        return;
    }
    if (!json || len <= 0) {
        ESP_LOGW(TAG, "root_apply_new_config: empty input");
        return;
    }

    ESP_LOGW(TAG, "=== MQTT config update: %d bytes ===", len);

    traffic_light_stop();
    SET_REMOTE_FLAG(true);
    traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
    SET_REMOTE_FLAG(false);

    {
        char fault_buf[256];
        size_t fn = traffic_light_build_fault_report(fault_buf, sizeof(fault_buf));
        if (fn > 0) transport_send_cmd_noack(fault_buf, fn);
    }

    bool ok = config_load_json(json, len);
    if (!ok) {
        ESP_LOGE(TAG, "root_apply_new_config: config_load_json FAILED");
        return;
    }
    ESP_LOGI(TAG, "ROOT: new config applied (version=%.2f, "
                  "seq=%u, stages=%u, hold=%d/%u, ped_display=%u)",
             g_config.version,
             g_config.traffic.sequence_count,
             g_config.traffic.stage_count,
             g_config.traffic.hold.enabled,
             g_config.traffic.hold.stage_count,
             g_config.traffic.pins.cross_ped.ped_display);

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
        .ped_display    = g_config.traffic.pins.cross_ped.ped_display,
    };
    traffic_light_set_pins_cross_ped(&pp);

    esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    if (mr != ESP_OK) {
        ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));
    }
    traffic_light_reset();

    if (g_config.traffic.enabled && g_config.traffic.auto_start) {
        ESP_LOGW(TAG, "ROOT: auto-start after config update (c: via phase_sync)");

        uint32_t utc_sec; uint16_t utc_ms;
        if (time_sync_get(&utc_sec, &utc_ms)) {
            time_sync_root_push_force(blink_get_rtt_us());
        }

        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
        traffic_light_set_follow_root(false);
        traffic_light_start_with_red();
    } else {
        ESP_LOGW(TAG, "ROOT: auto_start disabled - stay in YELLOW_BLINK");
    }
}

/* ============================================================
   root_node_watchdog_task
   ============================================================ */
static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started (comm_lost=%d ms, "
                  "SYNC-HANDSHAKE)", NODE_COMM_LOST_MS);

    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t last_start_ms = 0;

    traffic_light_stop();
    SET_REMOTE_FLAG(true);
    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
    SET_REMOTE_FLAG(false);
    wifi_mqtt_set_root_comm_lost(true);
    traffic_light_set_state_recovering();

    sync_handshake_root_start();

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
            continue;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        bool has_node = (transport_is_lora() &&
                         s_last_node_msg_ms != 0 &&
                         !link_monitor_root_lost());

        if (has_node && !was_connected) {
            ESP_LOGI(TAG, "ROOT: NODE on LoRa -> RECOVER");

            traffic_state_t rst = traffic_light_get_state();
            if (rst == TRAFFIC_STATE_RECOVERING ||
                rst == TRAFFIC_STATE_STOPPED) {

                uint32_t utc_sec; uint16_t utc_ms;
                if (!time_sync_get(&utc_sec, &utc_ms)) {
                    ESP_LOGW(TAG, "ROOT: time_sync not valid → wait");
                    vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
                    continue;
                }
                time_sync_root_push_force(blink_get_rtt_us());

                ESP_LOGW(TAG, "ROOT: START-WITH-RED (recover)");
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                traffic_light_set_follow_root(false);
                traffic_light_start_with_red();

                /* === FIX C-IMMEDIATE-AFTER-START ===
                   traffic:start и c: — подряд, без vTaskDelay(300).
                   Если NODE ещё не RUNNING — буферизация в
                   node_proto.c сохранит c: и применит после
                   traffic:start. */
                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                send_channels_to_node(0, UINT32_MAX,
                    traffic_light_stage_duration_ms(0), true);
                /* === END FIX C-IMMEDIATE-AFTER-START === */

                wifi_mqtt_publish_comm_restored();
            }

            was_connected = true;
            lost_since_ms = 0;
            in_comm_lost_all_off = false;
            last_start_ms = 0;
            wifi_mqtt_set_root_comm_lost(false);
        }

        if (has_node && s_node_running &&
            traffic_light_get_state() == TRAFFIC_STATE_STOPPED &&
            traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {

            ESP_LOGW(TAG, "ROOT: NODE RUNNING but ROOT STOPPED → "
                          "START ROOT + force c:");

            uint32_t utc_sec; uint16_t utc_ms;
            if (time_sync_get(&utc_sec, &utc_ms)) {
                time_sync_root_push_force(blink_get_rtt_us());
            }

            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
            traffic_light_set_follow_root(false);
            traffic_light_start_with_red();

            was_connected = true;
            lost_since_ms = 0;
            in_comm_lost_all_off = false;
            last_start_ms = 0;
            wifi_mqtt_set_root_comm_lost(false);

            vTaskDelay(pdMS_TO_TICKS(SYNC_AFTER_START_DELAY_MS));
            send_channels_to_node(0, UINT32_MAX,
                traffic_light_stage_duration_ms(0), true);
        }

        if (has_node && was_connected && !s_node_running &&
            traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
            if (last_start_ms == 0 || (now - last_start_ms) >= 5000) {
                ESP_LOGI(TAG, "ROOT: NODE not RUNNING -> re-send traffic:start");
                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                last_start_ms = now;
            }
        }

        if (traffic_light_get_fault() == TRAFFIC_FAULT_YELLOW_BLINK &&
            !s_node_running) {
            if (last_start_ms == 0 ||
                (now - last_start_ms) >= 5000) {
                ESP_LOGW(TAG, "ROOT: YELLOW_BLINK + NODE not RUNNING "
                              "-> traffic:start");
                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                last_start_ms = now;
            }
        }

        if (!has_node && was_connected) {
            ESP_LOGW(TAG, "ROOT: NODE lost -> sync_handshake");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            traffic_light_set_state_recovering();
            transport_send_cmd_noack("traffic:stop", strlen("traffic:stop"));
            sync_handshake_root_start();
            was_connected = false;
            lost_since_ms = now;
        }

        if (!has_node && !in_comm_lost_all_off && !was_connected) {
            uint32_t since = (lost_since_ms != 0)
                             ? (now - lost_since_ms)
                             : (now - boot_ms);
            if (since >= (uint32_t)NODE_COMM_LOST_MS) {
                if (traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {
                    ESP_LOGE(TAG, "ROOT: NODE absent > %d ms -> ALL_OFF",
                             NODE_COMM_LOST_MS);
                    SET_REMOTE_FLAG(true);
                    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                    SET_REMOTE_FLAG(false);
                    traffic_light_set_state_recovering();
                    wifi_mqtt_set_root_comm_lost(true);
                    in_comm_lost_all_off = true;

                    wifi_mqtt_publish_comm_lost(NODE_COMM_LOST_MS);

                    sync_handshake_root_start();
                }
            }
        }

        if (has_node && in_comm_lost_all_off) {
            in_comm_lost_all_off = false;
        }

        vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
    }
}

/* ============================================================
   phase_sync_task
   === FIX NO-PRE-DELAY + C-IMMEDIATE-ON-PHASE-CHANGE ===
   При смене фазы c: уходит немедленно. pm — hint UINT32_MAX,
   вычисляется внутри send_channels_to_node_impl.
   ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (refresh=%d ms, "
                  "TIME-PUSH=%d ms, C-ALWAYS-IN-RUNNING, NO-PRE-DELAY, "
                  "PM-AT-SEND, SYNC-HANDSHAKE)",
             LORA_SYNC_INTERVAL_MS, TIME_PUSH_PERIOD_MS);

    uint8_t  last_phase = 0xFF;
    uint32_t last_ms    = 0;
    uint32_t last_time_push_ms = 0;

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!transport_is_lora()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        sync_handshake_root_tick(now);
        if (sync_handshake_root_is_syncing()) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        traffic_state_t st = traffic_light_get_state();
        bool in_fault = (traffic_light_get_fault() != TRAFFIC_FAULT_NONE);

        if (st == TRAFFIC_STATE_RUNNING && !in_fault) {
            if (last_time_push_ms == 0 ||
                (now - last_time_push_ms) >= TIME_PUSH_PERIOD_MS) {
                last_time_push_ms = now;
                time_sync_root_push(blink_get_rtt_us());
            }
        }

        if (st == TRAFFIC_STATE_RUNNING && !in_fault) {
            uint8_t  phase = traffic_light_get_phase();
            uint32_t dur   = traffic_light_get_phase_duration_ms();

            if (phase != last_phase) {
                if (send_channels_to_node(phase, UINT32_MAX, dur, false)) {
                    last_phase = phase;
                    last_ms = now;
                    ESP_LOGI(TAG, "c: phase changed -> ph=%u", phase);
                }
            } else if ((now - last_ms) >= LORA_SYNC_INTERVAL_MS) {
                if (send_channels_to_node(phase, UINT32_MAX, dur, false)) {
                    last_ms = now;
                    ESP_LOGD(TAG, "c: refresh ph=%u", phase);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
   root_heartbeat_task
   ============================================================ */
static void root_heartbeat_task(void *arg)
{
    ESP_LOGI(TAG, "root_heartbeat_task started (periodic ping every %d ms)",
             HEARTBEAT_INTERVAL_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
        if (g_config.role != DEVICE_ROLE_MASTER) continue;
        if (!transport_is_lora()) continue;
        if (sync_handshake_root_is_syncing()) continue;
        esp_err_t err = transport_send_cmd_noack("ping", 4);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: heartbeat ping failed: %s",
                     esp_err_to_name(err));
        }
    }
}

/* ============================================================
   root_wd_start_tasks
   ============================================================ */
void root_wd_start_tasks(void)
{
    ESP_LOGI(TAG, "Starting ROOT watchdog tasks (SYNC-HANDSHAKE, "
                  "FAST-LINK-MONITOR: comm_lost=%d ms, hb=%d ms)",
             NODE_COMM_LOST_MS, HEARTBEAT_INTERVAL_MS);

    xTaskCreate(phase_sync_task,          "phase_sync",    4096, NULL, 4, NULL);
    xTaskCreate(root_node_watchdog_task,  "root_node_wd",  8192, NULL, 3, NULL);
    xTaskCreate(root_heartbeat_task,      "root_hb",       3072, NULL, 3, NULL);
}