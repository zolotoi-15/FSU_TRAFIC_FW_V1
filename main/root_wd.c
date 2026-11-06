#include "root_wd.h"
#include "app_shared.h"
#include "transport.h"
#include "traffic_light.h"
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
#define TRAFFIC_START_RETRY_MS      5000

#define NODE_COMM_LOST_MS           20000
#define LORA_SYNC_INTERVAL_MS       30000
#define CFG_PUSH_RX_WAIT_MS         3000
#define CFG_RETRY_PERIOD_MS         5000
#define NO_CFG_ALL_OFF_MS           (60 * 1000)

#define CFG_INITIAL_DELAY_MS        500

#define CFG_MAX_ATTEMPTS            10
#define CFG_RETRY_COOLDOWN_MS       2000
#define CFG_RESET_ACK_DELAY_MS      500
#define HEARTBEAT_INTERVAL_MS       10000

#define CFG_ACK_STALE_MS            60000

#define HOLD_RESYNC_INTERVAL_MS     5000

#define CFG_LORA_BUF_SIZE           2048

/* === FIX-SYNC-LEAD: 300 → 500 ms. NODE должен успеть принять
   sync: до перехода ROOT на следующую фазу. При LoRa ToA ~100-150 мс
   + FRAG_GAP_MS=20 — 500 мс даёт запас. === */
#define SYNC_LEAD_MS                500
/* === END FIX-SYNC-LEAD === */
#define SYNC_TOA_EST_MS             112

#define TRAFFIC_START_MAX_RETRIES   3
#define FRESH_STATUS_MS             5000

#define TIME_PUSH_PERIOD_MS         30000
#define SYNC_PRE_DELAY_MS           50
#define TIME_PUSH_BEFORE_START_MS   300

#define FAIL_TX_SYNC_MAX            10
#define FAIL_NODE_LOST_MS           5000
#define FAIL_ROOT_REBOOT_DELAY_MS   500

#define SYNC_MIN_INTERVAL_MS        250

static volatile uint8_t s_fail_tx_sync = 0;

extern volatile uint32_t s_last_fault_clear_ms;
extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool s_node_running;

extern uint32_t blink_get_rtt_us(void);

extern volatile bool s_hold_active;
extern volatile bool s_insert_pending;

volatile bool s_cfg_ack = false;
volatile bool s_cfg_pushing = false;
volatile bool s_cfg_applied_and_started = false;
volatile bool s_cfg_req_pending = false;
volatile bool s_force_resync_pending = false;

volatile uint32_t s_cfg_last_ok_ms = 0;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

extern uint8_t phase_change_take_pending(uint32_t *dur);
extern void    phase_change_requeue(uint8_t phase, uint32_t dur_ms);

static volatile uint8_t  s_last_sent_sync_phase = 0xFF;
static volatile uint32_t s_last_sent_sync_ms    = 0;

static inline bool cfg_ack_is_stale(void)
{
    if (!s_cfg_ack) return false;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t node_now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (s_last_node_msg_ms != 0) {
        uint32_t since_status = node_now - s_last_node_msg_ms;
        if (since_status < CFG_ACK_STALE_MS) {
            return false;
        }
        ESP_LOGW(TAG, "cfg_ack_is_stale: NODE silent %lu ms",
                 (unsigned long)since_status);
        return true;
    }

    if (s_cfg_last_ok_ms == 0) return true;
    if (now < s_cfg_last_ok_ms) return true;
    return (now - s_cfg_last_ok_ms) > CFG_ACK_STALE_MS;
}

uint32_t send_sync_to_node_async_ex(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms, bool force,
                                    uint8_t *out_next_phase)
{
    (void)duration_ms;
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;
    if (s_cfg_pushing) return 0;

    if (g_config.traffic.sequence_count == 0) {
        ESP_LOGW(TAG, "send_sync: sequence_count=0, skip");
        return 0;
    }
    if (phase >= g_config.traffic.sequence_count) {
        ESP_LOGW(TAG, "send_sync: phase=%u >= seq_count=%u, skip",
                 phase, g_config.traffic.sequence_count);
        return 0;
    }

    uint32_t now_check = (uint32_t)(esp_timer_get_time() / 1000);
    if (!force && phase == s_last_sent_sync_phase &&
        (now_check - s_last_sent_sync_ms) < SYNC_MIN_INTERVAL_MS) {
        ESP_LOGD(TAG, "send_sync: same phase=%u, %lu ms ago — skip",
                 phase, (unsigned long)(now_check - s_last_sent_sync_ms));
        return 0;
    }
    s_last_sent_sync_phase = phase;
    s_last_sent_sync_ms    = now_check;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t apply_ms = now_ms + SYNC_LEAD_MS;

    uint32_t pm_target = phase_ms + SYNC_LEAD_MS;

    uint32_t dur = traffic_light_stage_duration_ms(phase);
    if (dur > 0 && pm_target >= dur) {
        ESP_LOGW(TAG, "send_sync: pm_target=%lu >= dur=%lu → set 0",
                 (unsigned long)pm_target, (unsigned long)dur);
        pm_target = 0;
    }

    uint8_t next_phase = (phase + 1) % g_config.traffic.sequence_count;

    uint16_t on_mask = 0;
    uint16_t blink_mask = 0;
    traffic_light_get_stage_masks(phase, &on_mask, &blink_mask);

    char cmd[80];
    int n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%u:0x%04X:0x%04X",
                     phase,
                     (unsigned long)pm_target,
                     next_phase,
                     (unsigned)on_mask,
                     (unsigned)blink_mask);
    if (n <= 0) return 0;

    esp_err_t err = transport_send_cmd_noack(cmd, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_sync: %s failed: %s", cmd, esp_err_to_name(err));
        return 0;
    }

    if (out_next_phase) *out_next_phase = next_phase;

    ESP_LOGI(TAG, "sync: ph=%u pm_target=%lu apply_ms=%lu next=%u "
                  "on=0x%04X blink=0x%04X%s",
             phase, (unsigned long)pm_target,
             (unsigned long)apply_ms, next_phase,
             (unsigned)on_mask, (unsigned)blink_mask,
             force ? " (force)" : "");

    return apply_ms;
}

uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase)
{
    return send_sync_to_node_async_ex(phase, phase_ms, duration_ms,
                                      false, out_next_phase);
}

uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_ms)
{
    if (apply_ms == 0) return 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t wait_ms = (int32_t)(apply_ms - now_ms);
    if (wait_ms > 2000) wait_ms = 2000;
    if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));

    now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t lag = (int32_t)(now_ms - apply_ms);
    if (lag < 0) lag = 0;
    if (lag > 1000) lag = 1000;

    uint32_t pm_target = phase_ms + SYNC_LEAD_MS;
    uint32_t pm_apply = pm_target + (uint32_t)lag;
    if (duration_ms > 0 && pm_apply >= duration_ms) {
        ESP_LOGW(TAG, "apply_sync: pm_apply=%lu >= dur=%lu → set 0",
                 (unsigned long)pm_apply, (unsigned long)duration_ms);
        pm_apply = 0;
    }

    traffic_light_force_phase(phase, pm_apply, duration_ms);

    ESP_LOGI(TAG, "sync applied (ROOT): ph=%u pm=%lu lag=%ld dur=%lu",
             phase, (unsigned long)pm_apply, (long)lag,
             (unsigned long)duration_ms);
    return 1;
}

uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms)
{
    uint32_t apply_ms = send_sync_to_node_async(phase, phase_ms,
                                                duration_ms, NULL);
    if (apply_ms == 0) return 0;
    return apply_sync_at_utc(phase, phase_ms, duration_ms, apply_ms);
}

void apply_sync_at(uint8_t phase, uint32_t phase_ms,
                   uint32_t duration_ms, uint32_t apply_ms)
{
    (void)apply_ms;
    traffic_light_force_phase(phase, phase_ms, duration_ms);
}

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
    ESP_LOGW(TAG, "ROOT: YELLOW_BLINK (config update)");

    {
        char fault_buf[256];
        size_t fn = traffic_light_build_fault_report(fault_buf, sizeof(fault_buf));
        if (fn > 0) {
            transport_send_cmd_noack(fault_buf, fn);
        }
    }

    bool ok = config_load_json(json, len);
    if (!ok) {
        ESP_LOGE(TAG, "root_apply_new_config: config_load_json FAILED, "
                      "ROOT+NODE остаются в YELLOW_BLINK");
        return;
    }
    ESP_LOGI(TAG, "ROOT: new config applied locally (version=%.2f, "
                  "seq=%u, stages=%u, hold=%d/%u)",
             g_config.version,
             g_config.traffic.sequence_count,
             g_config.traffic.stage_count,
             g_config.traffic.hold.enabled,
             g_config.traffic.hold.stage_count);

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

    esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    if (mr != ESP_OK) {
        ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));
    }
    traffic_light_reset();

    s_cfg_ack = false;
    s_cfg_applied_and_started = false;
    s_cfg_pushing = false;
    s_cfg_last_ok_ms = 0;

    s_cfg_req_pending = true;

    ESP_LOGI(TAG, "ROOT: config update queued, waiting for NODE cfg:ok "
                  "(YELLOW_BLINK until then)");
}

static void on_lora_ack(uint8_t seq, uint8_t type)
{
    (void)seq; (void)type;
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (s_cfg_ack || s_cfg_pushing) return;

    if (!s_cfg_req_pending) {
        ESP_LOGI(TAG, "ROOT: ACK from NODE → trigger CFG push");
    }
    s_cfg_req_pending = true;
}

static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC, NO-WAIT, WAIT-NODE-RUNNING, FAST-REACT, "
                  "FAIL-SAFE-ROOT, CFG-REQ-FALLBACK, CONNECT-ANY-ORDER, "
                  "HOLD/INSERT, CFG-FROM-ROOT, CFG-ACK-STALE, HOLD-RESYNC, "
                  "FIX-INSERT-FIRST-PHASE, FIX-SYNC-LEAD)",
             SYNC_LEAD_MS);

    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t node_seen_ms = 0;
    bool     no_cfg_all_off = false;
    uint32_t last_start_ms = 0;
    uint32_t start_retry_count = 0;
    bool     node_running_confirmed = false;

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

        if (cfg_ack_is_stale()) {
            ESP_LOGW(TAG, "ROOT: s_cfg_ack stale → reset, request CFG");
            s_cfg_ack = false;
            s_cfg_applied_and_started = false;
            s_cfg_req_pending = true;
        }

        bool has_node = false;
        if (transport_is_lora() && s_last_node_msg_ms != 0 && !s_cfg_pushing) {
            has_node = !link_monitor_root_lost();
        }

        if (has_node && node_seen_ms == 0) {
            node_seen_ms = now;
        }
        if (s_cfg_ack) no_cfg_all_off = false;

        if (s_node_running && has_node && !s_cfg_pushing) {
            if (!node_running_confirmed) {
                ESP_LOGI(TAG, "ROOT: NODE RUNNING confirmed");
            }
            node_running_confirmed = true;
        }
        if (!has_node || (!s_node_running && !s_cfg_pushing)) {
            if (node_running_confirmed && !s_cfg_pushing) {
                ESP_LOGW(TAG, "ROOT: NODE RUNNING lost (has_node=%d, running=%d)",
                         (int)has_node, (int)s_node_running);
            }
            node_running_confirmed = false;
        }

        if (s_cfg_applied_and_started && !has_node && !s_cfg_pushing &&
            lost_since_ms != 0 &&
            (now - lost_since_ms) >= FAIL_NODE_LOST_MS) {
            if (traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {
                ESP_LOGE(TAG, "ROOT: NODE absent > %d ms → ALL_OFF (fail-safe)",
                         FAIL_NODE_LOST_MS);
                traffic_light_stop();
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
                s_cfg_applied_and_started = false;
                start_retry_count = 0;
            }
        }

        if (s_cfg_ack && !cfg_ack_is_stale() && has_node && !s_cfg_pushing &&
            !s_cfg_applied_and_started && !node_running_confirmed) {
            if (start_retry_count < TRAFFIC_START_MAX_RETRIES &&
                (last_start_ms == 0 ||
                 (now - last_start_ms) >= TRAFFIC_START_RETRY_MS)) {
                ESP_LOGI(TAG, "ROOT: cfg:ok fresh → traffic:start (%u/%u), "
                              "wait RUNNING from NODE",
                         start_retry_count + 1, TRAFFIC_START_MAX_RETRIES);
                transport_send_cmd_noack("traffic:start", strlen("traffic:start"));
                last_start_ms = now;
                start_retry_count++;
            } else if (start_retry_count >= TRAFFIC_START_MAX_RETRIES) {
                ESP_LOGW(TAG, "ROOT: traffic:start retries exhausted (%u), "
                              "NODE not RUNNING",
                         TRAFFIC_START_MAX_RETRIES);
                start_retry_count = TRAFFIC_START_MAX_RETRIES + 1;
            }
        }

        if (s_cfg_ack && !cfg_ack_is_stale() && has_node && !s_cfg_pushing &&
            node_running_confirmed && !s_cfg_applied_and_started) {
            ESP_LOGW(TAG, "cfg:ok fresh + NODE RUNNING → START ROOT + first sync:");

            if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
                ESP_LOGI(TAG, "ROOT: clearing fault (%s) — NODE cfg:ok + RUNNING",
                         traffic_light_fault_str());
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
            }

            uint32_t utc_sec = 0; uint16_t utc_ms = 0;
            bool time_ok = time_sync_get(&utc_sec, &utc_ms);
            if (time_ok) {
                time_sync_root_push_force(blink_get_rtt_us());
                vTaskDelay(pdMS_TO_TICKS(TIME_PUSH_BEFORE_START_MS));
                ESP_LOGI(TAG, "ROOT: time: pushed (rtt=%lu us) → starting traffic",
                         (unsigned long)blink_get_rtt_us());
            } else {
                ESP_LOGW(TAG, "ROOT: no valid time yet → traffic:start "
                              "(sync: deferred)");
            }

            traffic_light_set_follow_root(false);
            traffic_light_start();

            s_cfg_applied_and_started = true;
            was_connected = true;
            lost_since_ms = 0;
            in_comm_lost_all_off = false;
            last_start_ms = now;
            start_retry_count = 0;
            wifi_mqtt_reset_node_fault_tracking();
            wifi_mqtt_set_root_comm_lost(false);

            if (time_ok &&
                traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                uint8_t phase = traffic_light_get_phase();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                send_sync_to_node(phase, 0, dur);
                ESP_LOGI(TAG, "ROOT: first sync: ph=%u dur=%lu sent",
                         phase, (unsigned long)dur);
            }

            ESP_LOGI(TAG, "ROOT: START + first sync:%s",
                     time_ok ? "" : " (time invalid, deferred)");
        }

        if (s_cfg_applied_and_started && has_node && !s_node_running) {
            bool fresh_status = (s_last_node_msg_ms != 0) &&
                                ((now - s_last_node_msg_ms) < FRESH_STATUS_MS);
            if (!fresh_status) {
                vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
                continue;
            }
            if (start_retry_count < TRAFFIC_START_MAX_RETRIES &&
                (now - last_start_ms) >= TRAFFIC_START_RETRY_MS) {
                ESP_LOGI(TAG, "ROOT: NODE not RUNNING → re-send traffic:start "
                              "(%u/%u) — БЕЗ sync: (NO-DOUBLE-SYNC)",
                         start_retry_count + 1, TRAFFIC_START_MAX_RETRIES);
                transport_send_cmd_noack("traffic:start", strlen("traffic:start"));
                last_start_ms = now;
                start_retry_count++;
            } else if (start_retry_count >= TRAFFIC_START_MAX_RETRIES) {
                ESP_LOGW(TAG, "ROOT: traffic:start retries exhausted (%u)",
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
            ESP_LOGW(TAG, "ROOT: NODE lost → STOP + ALL_OFF");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            transport_send_cmd_noack("traffic:stop", strlen("traffic:stop"));
            was_connected = false;
            lost_since_ms = now;
            s_cfg_applied_and_started = false;
            s_cfg_ack = false;
            start_retry_count = 0;
            node_running_confirmed = false;
        }

        if (has_node && !s_cfg_ack && node_seen_ms != 0 && !no_cfg_all_off &&
            (now - node_seen_ms) >= NO_CFG_ALL_OFF_MS) {
            ESP_LOGE(TAG, "ROOT: NODE present but cfg not ack'd > %d ms → ALL_OFF",
                     NO_CFG_ALL_OFF_MS);
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
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

static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC, NO-WAIT, FAIL-SAFE-ROOT, HOLD/INSERT, "
                  "CFG-ACK-STALE, HOLD-RESYNC, FIX-INSERT-FIRST-PHASE, "
                  "FIX-SYNC-LEAD)",
             SYNC_LEAD_MS);

    uint32_t last_ms = 0;
    uint32_t last_time_push_ms = 0;
    bool     time_pushed_since_start = false;

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (g_config.role == DEVICE_ROLE_MASTER) {
            if (s_cfg_pushing || !s_cfg_ack || cfg_ack_is_stale()) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            traffic_state_t st = traffic_light_get_state();
            bool in_all_off = (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF);
            bool has_node = transport_is_lora() && s_last_node_msg_ms != 0 &&
                            !link_monitor_root_lost();
            if (s_cfg_pushing) has_node = true;

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off &&
                s_cfg_applied_and_started && !time_pushed_since_start) {
                uint32_t ts = 0; uint16_t tm = 0;
                if (time_sync_get(&ts, &tm)) {
                    time_pushed_since_start = true;
                    last_time_push_ms = now;
                    time_sync_root_push_force(blink_get_rtt_us());
                    ESP_LOGI(TAG, "phase_sync: first time: pushed (rtt=%lu us) → "
                                  "sync:current phase now",
                             (unsigned long)blink_get_rtt_us());
                    uint8_t phase = traffic_light_get_phase();
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    uint32_t pm  = traffic_light_get_phase_ms();
                    uint32_t apply_ms = send_sync_to_node_async(
                        phase, pm, dur, NULL);
                    if (apply_ms != 0) {
                        s_fail_tx_sync = 0;
                        apply_sync_at_utc(phase, pm, dur, apply_ms);
                    } else {
                        s_fail_tx_sync++;
                        if (s_fail_tx_sync >= FAIL_TX_SYNC_MAX) {
                            ESP_LOGE(TAG, "ROOT: sync TX failed %u times → "
                                          "ALL_OFF + reboot", s_fail_tx_sync);
                            traffic_light_stop();
                            SET_REMOTE_FLAG(true);
                            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                            SET_REMOTE_FLAG(false);
                            vTaskDelay(pdMS_TO_TICKS(FAIL_ROOT_REBOOT_DELAY_MS));
                            esp_restart();
                        }
                    }
                }
            }

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off &&
                s_cfg_applied_and_started && time_pushed_since_start) {
                if (last_time_push_ms == 0 ||
                    (now - last_time_push_ms) >= TIME_PUSH_PERIOD_MS) {
                    last_time_push_ms = now;
                    time_sync_root_push(blink_get_rtt_us());
                }
            }

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off &&
                s_cfg_applied_and_started) {
                uint32_t dur_pending = 0;
                uint8_t ph_pending = phase_change_take_pending(&dur_pending);
                if (ph_pending != 0xFF) {
                    vTaskDelay(pdMS_TO_TICKS(SYNC_PRE_DELAY_MS));

                    uint32_t apply_ms = send_sync_to_node_async(
                        ph_pending, 0, dur_pending, NULL);
                    if (apply_ms == 0) {
                        ESP_LOGW(TAG, "phase_change → sync:%u TX failed, "
                                      "re-queue", ph_pending);
                        phase_change_requeue(ph_pending, dur_pending);
                        s_fail_tx_sync++;
                        if (s_fail_tx_sync >= FAIL_TX_SYNC_MAX) {
                            ESP_LOGE(TAG, "ROOT: sync TX failed %u times → "
                                          "ALL_OFF + reboot", s_fail_tx_sync);
                            traffic_light_stop();
                            SET_REMOTE_FLAG(true);
                            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                            SET_REMOTE_FLAG(false);
                            vTaskDelay(pdMS_TO_TICKS(FAIL_ROOT_REBOOT_DELAY_MS));
                            esp_restart();
                        }
                    } else {
                        s_fail_tx_sync = 0;
                        apply_sync_at_utc(ph_pending, 0, dur_pending, apply_ms);
                        ESP_LOGI(TAG, "phase_change → sync:%u:lead:%lu (r=1)",
                                 ph_pending, (unsigned long)dur_pending);
                    }
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }

            if (s_force_resync_pending) {
                s_force_resync_pending = false;
                if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                    uint8_t phase = traffic_light_get_phase();
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    uint32_t pm  = traffic_light_get_phase_ms();
                    ESP_LOGW(TAG, "force resync: phase=%u pm=%lu dur=%lu",
                             phase, (unsigned long)pm, (unsigned long)dur);
                    send_sync_to_node(phase, pm, dur);
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }

            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                if (s_hold_active && !s_insert_pending) {
                    /* === FIX-INSERT-FIRST-PHASE: hold resync — 5 сек. === */
                    if (last_ms == 0 ||
                        (now - last_ms >= HOLD_RESYNC_INTERVAL_MS)) {
                        uint8_t  phase = traffic_light_get_phase();
                        uint32_t dur   = traffic_light_get_phase_duration_ms();
                        uint32_t pm    = traffic_light_get_phase_ms();
                        send_sync_to_node(phase, pm, dur);
                        last_ms = now;
                        ESP_LOGI(TAG, "hold resync: ph=%u pm=%lu dur=%lu",
                                 phase, (unsigned long)pm, (unsigned long)dur);
                    }
                } else if (s_insert_pending) {
                    last_ms = now;
                } else if (s_cfg_applied_and_started && last_ms != 0 &&
                           (now - last_ms >= LORA_SYNC_INTERVAL_MS)) {
                    uint8_t  phase = traffic_light_get_phase();
                    uint32_t dur   = traffic_light_get_phase_duration_ms();
                    uint32_t pm    = traffic_light_get_phase_ms();
                    vTaskDelay(pdMS_TO_TICKS(SYNC_PRE_DELAY_MS));
                    send_sync_to_node(phase, pm, dur);
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }
        } else {
            time_pushed_since_start = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void cfg_push_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_push_task started (short JSON for LoRa, "
                  "CFG-REQ-FALLBACK, CONNECT-ANY-ORDER, CFG-FROM-ROOT, "
                  "CFG-ACK-STALE)");

    uint32_t waited = 0;
    while (!transport_is_lora() && waited < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    if (!transport_is_lora()) {
        ESP_LOGE(TAG, "cfg_push: LoRa not ready");
    }

    ESP_LOGI(TAG, "cfg_push: wait for cfg:req from NODE or first push");
    vTaskDelay(pdMS_TO_TICKS(CFG_INITIAL_DELAY_MS));

    uint32_t last_push_ms = 0;

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!transport_is_lora()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (s_cfg_pushing) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (s_cfg_ack && cfg_ack_is_stale()) {
            ESP_LOGW(TAG, "cfg_push: s_cfg_ack stale → reset, push CFG");
            s_cfg_ack = false;
            s_cfg_applied_and_started = false;
            s_cfg_req_pending = true;
        }
        if (s_cfg_ack) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        bool by_req = s_cfg_req_pending;
        bool first_push = (last_push_ms == 0);
        bool by_timeout = (!first_push &&
                           (now_ms - last_push_ms) >= CFG_RETRY_PERIOD_MS);

        if (!by_req && !first_push && !by_timeout) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (first_push) {
            ESP_LOGI(TAG, "cfg_push: first push (CONNECT-ANY-ORDER)");
        } else if (by_req) {
            ESP_LOGI(TAG, "cfg_push: cfg:req from NODE → start push");
        } else {
            ESP_LOGW(TAG, "cfg_push: no cfg:req from NODE for %lu ms → "
                          "push by timeout (CFG-REQ-FALLBACK)",
                     (unsigned long)(now_ms - last_push_ms));
        }

        s_cfg_req_pending = false;
        last_push_ms = now_ms;

        extern bool config_export_lora_json(char *b, size_t *l);
        char *buf = malloc(CFG_LORA_BUF_SIZE);
        size_t blen = CFG_LORA_BUF_SIZE;
        if (!buf || !config_export_lora_json(buf, &blen)) {
            ESP_LOGE(TAG, "cfg_push: short JSON export failed");
            if (buf) free(buf);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "cfg_push: short JSON ready (%u bytes)", (unsigned)blen);

        s_cfg_pushing = true;
        vTaskDelay(pdMS_TO_TICKS(CFG_PUSH_RX_WAIT_MS));

        for (int attempt = 0; attempt < CFG_MAX_ATTEMPTS; attempt++) {
            if (s_cfg_ack) break;

            transport_send_cmd("cfg:reset", strlen("cfg:reset"));
            vTaskDelay(pdMS_TO_TICKS(CFG_RESET_ACK_DELAY_MS));

            ESP_LOGI(TAG, "cfg_push → LoRa attempt %d/%d (%u bytes)",
                     attempt + 1, CFG_MAX_ATTEMPTS, (unsigned)blen);

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
                ESP_LOGW(TAG, "cfg_push: TX OK but no cfg:ok within 5s");
            } else {
                ESP_LOGW(TAG, "cfg_push: failed (attempt %d): %s",
                         attempt + 1, esp_err_to_name(err));
            }

            if (attempt < CFG_MAX_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(CFG_RETRY_COOLDOWN_MS));
            }
        }

        s_cfg_pushing = false;
        free(buf);

        if (s_cfg_ack) {
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            ESP_LOGE(TAG, "cfg_push: NODE did not confirm cfg after %d attempts, "
                          "retry in %d ms",
                     CFG_MAX_ATTEMPTS, CFG_RETRY_PERIOD_MS);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

static void root_heartbeat_task(void *arg)
{
    ESP_LOGI(TAG, "root_heartbeat_task started (periodic ping every %d ms, "
                  "NO-WAIT)", HEARTBEAT_INTERVAL_MS);
    vTaskDelay(pdMS_TO_TICKS(CFG_INITIAL_DELAY_MS));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
        if (g_config.role != DEVICE_ROLE_MASTER) continue;
        if (!transport_is_lora()) continue;
        if (s_cfg_pushing) continue;
        esp_err_t err = transport_send_cmd_noack("ping", 4);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: heartbeat ping failed: %s", esp_err_to_name(err));
        }
    }
}

void root_wd_start_tasks(void)
{
    ESP_LOGI(TAG, "Starting ROOT watchdog tasks (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC, NO-WAIT, WAIT-NODE-RUNNING, FAST-REACT, "
                  "FAIL-SAFE-ROOT, CFG-REQ-FALLBACK, CONNECT-ANY-ORDER, "
                  "HOLD/INSERT, CFG-FROM-ROOT, CFG-ACK-STALE, HOLD-RESYNC, "
                  "FIX-INSERT-FIRST-PHASE, FIX-SYNC-LEAD)...",
             SYNC_LEAD_MS);

    transport_set_ack_cb(on_lora_ack);

    xTaskCreate(phase_sync_task,          "phase_sync",    4096, NULL, 4, NULL);
    xTaskCreate(root_node_watchdog_task,  "root_node_wd",  8192, NULL, 3, NULL);
    xTaskCreate(cfg_push_task,            "cfg_push",      8192, NULL, 3, NULL);
    xTaskCreate(root_heartbeat_task,      "root_hb",       3072, NULL, 3, NULL);
}