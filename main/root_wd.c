/* ============================================================
   root_wd.c — ROOT: phase-sync + heartbeat + node_watchdog.

   === FIX ROOT-WAIT-NODE-ACK ===
       ROOT после обнаружения NODE НЕ вызывает
       traffic_light_start_with_red() сразу. Отправляет
       traffic:start + c:0:...:red, ждёт STATUS с RUNNING
       (WAIT_NODE_ACK_TIMEOUT_MS=3000), затем локально start.

   === FIX WAIT-TIME-SYNC-TIMEOUT ===
       Не ждём time_sync вечно. SNTP может быть недоступен
       за APN. Через WAIT_TIME_SYNC_MAX_MS (10000 мс) стартуем
       БЕЗ UTC — NODE получит c: с end_utc=0, будет работать
       по локальному pm. Time дошлётся позже (когда/если SNTP
       ответит) — phase_sync_task делает time_sync_root_push
       раз в TIME_PUSH_PERIOD_MS, как только s_valid станет
       true.

   === FIX FAULT-HAS-TOP-PRIORITY ===
       phase_sync_task НЕ шлёт c: при активной аварии
       (traffic_light_fault_has_priority() == true). NODE
       в YELLOW_BLINK/ALL_OFF управляется fault:*.

   === FIX NO-DOUBLE-C-ON-PHASE-CHANGE ===
       phase_sync_task больше НЕ отправляет c: при смене
       фазы. Первичный c: с pm=0 ушёл из
       root_apply_stage_sync_first() в traffic_light.c.

   === FIX PM-AT-SEND ===
       pm вычисляется внутри send_channels_to_node_impl в
       момент snprintf (hint=UINT32_MAX).

   === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
       Перед traffic:start в ветке ROOT-WAIT-NODE-ACK
       (восстановление связи: was_connected=false → true)
       отправляем явный fault:none. NODE мог сам уйти в
       ALL_OFF из-за потери ROOT (node_link_watchdog_task)
       и блокирует traffic:start/c: через
       traffic_light_fault_has_priority() без fault:none.

   === FIX NO-FAULT-NONE-ON-RESEND ===
       Убран fault:none из ветки повторного traffic:start
       (!s_node_running). Раньше ROOT слал fault:none каждые
       5 с, когда NODE не RUNNING. Это снимало YELLOW_BLINK
       на NODE, если NODE в аварии (short to GND), а ROOT
       ещё не успел применить её у себя: STATUS от NODE
       приходит раз в 1 с (при fault) или 5 с (норма), а
       traffic:start шлётся раз в 5 с — ROOT «сбрасывал»
       аварию на NODE каждые 5 с.

       fault:none оставлен ТОЛЬКО в ветке восстановления
       связи (!was_connected) и в sync_handshake.c, где
       NODE действительно мог уйти в standby по
       s_root_lost_flag.
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
#include "app_shared.h"

static const char *TAG = "root_wd";

#define ROOT_NODE_CHECK_MS          200
#define NODE_COMM_LOST_MS           4000
#define HEARTBEAT_INTERVAL_MS       2000
#define SYNC_AFTER_START_DELAY_MS   2000

/* === FIX ROOT-WAIT-NODE-ACK === */
#define WAIT_NODE_ACK_TIMEOUT_MS   3000
/* === END FIX ROOT-WAIT-NODE-ACK === */

/* === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
   Пауза между fault:none и traffic:start — чтобы LoRa успел
   доставить первый пакет (ToA 10 байт ≈ 40-50 мс SF8/BW250).
   transport_send_cmd_noack блокирующий (возвращается после
   физической передачи), так что 80 мс — это запас поверх
   уже доставленного пакета. */
#define FAULT_NONE_TO_START_GAP_MS  80
/* === END FIX ROOT-RETURN-SENDS-FAULT-NONE === */

/* === FIX WAIT-TIME-SYNC-TIMEOUT ===
   Не ждём time_sync вечно. SNTP может быть недоступен за
   APN оператора. Через WAIT_TIME_SYNC_MAX_MS стартуем без
   UTC — NODE получит c: с end_utc=0, будет работать по
   локальному pm, а time: дошлётся позже (когда/если SNTP
   ответит). */
#define WAIT_TIME_SYNC_MAX_MS       10000
/* === END FIX WAIT-TIME-SYNC-TIMEOUT === */

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
   SET_REMOTE_FLAG
   ============================================================ */
#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* ============================================================
   send_channels_to_node_impl
   === FIX PM-AT-SEND ===
   === FIX FAULT-HAS-TOP-PRIORITY ===
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

    /* === FIX FAULT-HAS-TOP-PRIORITY ===
       При активной аварии НЕ шлём c:. NODE управляется
       fault:* (см. root_push_fault_to_node). Иначе c: с
       маской не-аварийного режима перезапишет fault. */
    if (traffic_light_fault_has_priority()) {
        ESP_LOGW(TAG, "send_channels(%u): fault active (%s), skip c:",
                 phase, traffic_light_fault_str());
        return 0;
    }
    /* === END FIX === */

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

    uint32_t pm_use;
    if (phase_ms_hint == UINT32_MAX) {
        pm_use = traffic_light_get_phase_ms();
    } else {
        pm_use = phase_ms_hint;
    }
    if (duration_ms > 0 && pm_use >= duration_ms) pm_use = 0;

    uint64_t now_utc    = time_sync_get_utc_ms();
    uint64_t end_utc_ms = 0;
    if (now_utc != 0) {
        uint32_t remain = (pm_use < duration_ms)
                          ? (duration_ms - pm_use)
                          : duration_ms;
        end_utc_ms = now_utc + (uint64_t)remain;
    }

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
   === FIX FAULT-MASK-ON-CONFIG-LOAD ===
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
        /* === FIX FAULT-MASK-ON-CONFIG-LOAD ===
           Явно применяем YELLOW_BLINK, чтобы apply_fault_mode
           построил маску по уже применённому g_config.pins. */
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
        SET_REMOTE_FLAG(false);
        /* === END FIX === */
    }
}

/* ============================================================
   root_node_watchdog_task
   === FIX WAIT-TIME-SYNC-TIMEOUT ===
   === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
   === FIX NO-FAULT-NONE-ON-RESEND ===
   ============================================================ */
static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started (comm_lost=%d ms, "
                  "SYNC-HANDSHAKE, ROOT-WAIT-NODE-ACK timeout=%d ms, "
                  "WAIT-TIME-SYNC-MAX=%d ms, FAULT-NONE-GAP=%d ms, "
                  "NO-FAULT-NONE-ON-RESEND)",
             NODE_COMM_LOST_MS, WAIT_NODE_ACK_TIMEOUT_MS,
             WAIT_TIME_SYNC_MAX_MS, FAULT_NONE_TO_START_GAP_MS);

    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t last_start_ms = 0;

    bool     wait_node_ack    = false;
    uint32_t wait_node_ack_ms = 0;

    /* === FIX WAIT-TIME-SYNC-TIMEOUT === */
    uint32_t wait_time_sync_since = 0;
    /* === END FIX === */

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

        /* === FIX ROOT-WAIT-NODE-ACK ===
           Ожидание подтверждения от NODE. ROOT в ALL_OFF,
           каналы выключены. */
        if (wait_node_ack) {
            bool node_confirmed = s_node_running;
            bool timeout = (now - wait_node_ack_ms) >= WAIT_NODE_ACK_TIMEOUT_MS;

            if (node_confirmed || timeout) {
                if (timeout && !node_confirmed) {
                    ESP_LOGW(TAG, "ROOT: wait NODE ack timeout (%u ms) — "
                                  "force local start_with_red",
                             WAIT_NODE_ACK_TIMEOUT_MS);
                } else {
                    ESP_LOGI(TAG, "ROOT: NODE ack RUNNING → local "
                                  "start_with_red (channels ON)");
                }

                uint32_t utc_sec; uint16_t utc_ms;
                if (time_sync_get(&utc_sec, &utc_ms)) {
                    time_sync_root_push_force(blink_get_rtt_us());
                }
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                traffic_light_set_follow_root(false);
                traffic_light_start_with_red();

                wait_node_ack = false;
                was_connected = true;
                lost_since_ms = 0;
                in_comm_lost_all_off = false;
                last_start_ms = 0;
                wait_time_sync_since = 0;
                wifi_mqtt_set_root_comm_lost(false);
                wifi_mqtt_publish_comm_restored();
            }
            vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
            continue;
        }
        /* === END FIX ROOT-WAIT-NODE-ACK === */

        if (has_node && !was_connected) {
            ESP_LOGI(TAG, "ROOT: NODE on LoRa -> RECOVER "
                          "(wait NODE ack before local red)");

            traffic_state_t rst = traffic_light_get_state();
            if (rst == TRAFFIC_STATE_RECOVERING ||
                rst == TRAFFIC_STATE_STOPPED) {

                uint32_t utc_sec; uint16_t utc_ms;
                bool time_ok = time_sync_get(&utc_sec, &utc_ms);

                /* === FIX WAIT-TIME-SYNC-TIMEOUT ===
                   SNTP может не ответить за APN. Через
                   WAIT_TIME_SYNC_MAX_MS стартуем без UTC:
                   NODE получит c: с end_utc=0, будет работать по
                   локальному pm. Time дошлётся позже (phase_sync
                   task раз в TIME_PUSH_PERIOD_MS). */
                if (!time_ok) {
                    if (wait_time_sync_since == 0) {
                        wait_time_sync_since = now;
                    }
                    uint32_t waited = now - wait_time_sync_since;
                    if (waited < WAIT_TIME_SYNC_MAX_MS) {
                        ESP_LOGW(TAG, "ROOT: time_sync not valid → wait "
                                      "(%lu/%d ms, sntp_pending=%d)",
                                 (unsigned long)waited,
                                 WAIT_TIME_SYNC_MAX_MS,
                                 time_sync_is_sntp_pending() ? 1 : 0);
                        vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
                        continue;
                    }
                    ESP_LOGE(TAG, "ROOT: time_sync still invalid after "
                                  "%lu ms → start WITHOUT UTC "
                                  "(sntp_pending=%d)",
                             (unsigned long)waited,
                             time_sync_is_sntp_pending() ? 1 : 0);
                } else {
                    wait_time_sync_since = 0;
                    time_sync_root_push_force(blink_get_rtt_us());
                }
                /* === END FIX WAIT-TIME-SYNC-TIMEOUT === */

                /* === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
                   Восстановление связи (was_connected=false → true).
                   NODE мог сам уйти в ALL_OFF из-за потери ROOT
                   (node_link_watchdog_task → set_fault(ALL_OFF) +
                   s_root_lost_flag=true). NODE блокирует
                   traffic:start/c: через fault_has_priority() и
                   ждёт fault:none. Без явного fault:none здесь —
                   deadlock: ROOT шлёт traffic:start, NODE
                   игнорирует, ROOT по timeout форсирует локальный
                   старт, но NODE остаётся в ALL_OFF.

                   Шлём fault:none ПЕРВЫМ, с паузой на доставку
                   LoRa (ToA для 10 байт ≈ 40-50 мс при SF8/BW250,
                   transport_send_cmd_noack блокирующий, так что
                   80 мс — запас). */
                ESP_LOGW(TAG, "ROOT: send fault:none + traffic:start + "
                              "c:0:...:red to NODE (ROOT stays ALL_OFF "
                              "until NODE confirms)");
                transport_send_cmd_noack("fault:none", strlen("fault:none"));
                vTaskDelay(pdMS_TO_TICKS(FAULT_NONE_TO_START_GAP_MS));

                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                send_channels_to_node(0, UINT32_MAX,
                    traffic_light_stage_duration_ms(0), true);
                /* === END FIX ROOT-RETURN-SENDS-FAULT-NONE === */

                wait_node_ack = true;
                wait_node_ack_ms = now;
            } else {
                was_connected = true;
                lost_since_ms = 0;
                in_comm_lost_all_off = false;
                last_start_ms = 0;
                wait_time_sync_since = 0;
                wifi_mqtt_set_root_comm_lost(false);
                wifi_mqtt_publish_comm_restored();
            }
            vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
            continue;
        }

        if (!wait_node_ack && has_node && s_node_running &&
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

        /* === FIX NO-FAULT-NONE-ON-RESEND ===
           Повторный traffic:start когда NODE не RUNNING.

           Раньше здесь слали fault:none КАЖДЫЕ 5 с. Это снимало
           YELLOW_BLINK на NODE, если NODE в аварии (short to
           GND) — потому что:
             - NODE шлёт fault:local:YELLOW_BLINK (EVT, no-ack),
             - ROOT применяет её у себя только после STATUS от
               NODE с FAULT.mode == "YELLOW_BLINK" (раз в 1 с
               при fault),
             - но traffic:start с fault:none уходит каждые 5 с
               (эта ветка),
             - NODE получает fault:none → снимает YELLOW_BLINK →
               «сбрасывает» аварию, хотя short to GND продолжается.

           Убираем fault:none отсюда. Если NODE в standby по
           s_root_lost_flag — он получит fault:none из ветки
           !was_connected (восстановление связи). */
        if (!wait_node_ack && has_node && was_connected && !s_node_running &&
            traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
            if (last_start_ms == 0 || (now - last_start_ms) >= 5000) {
                ESP_LOGI(TAG, "ROOT: NODE not RUNNING -> re-send traffic:start "
                              "(no fault:none)");
                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                last_start_ms = now;
            }
        }
        /* === END FIX NO-FAULT-NONE-ON-RESEND === */

        /* === FIX NO-FAULT-NONE-ON-RESEND ===
           Аналогично для ветки YELLOW_BLINK + NODE not RUNNING.
           Раньше слали fault:none — это могло сбить YELLOW_BLINK
           на NODE, если NODE уже получил fault:yellow_blink от
           ROOT (а ROOT сейчас в другом fault — YELLOW_BLINK).
           Убираем fault:none, оставляем только traffic:start.
           Если NODE в standby — fault:none придёт из
           !was_connected или из sync_handshake. */
        if (!wait_node_ack &&
            traffic_light_get_fault() == TRAFFIC_FAULT_YELLOW_BLINK &&
            !s_node_running) {
            if (last_start_ms == 0 ||
                (now - last_start_ms) >= 5000) {
                ESP_LOGW(TAG, "ROOT: YELLOW_BLINK + NODE not RUNNING "
                              "-> traffic:start (no fault:none)");
                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                last_start_ms = now;
            }
        }
        /* === END FIX NO-FAULT-NONE-ON-RESEND === */

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
            wait_time_sync_since = 0;
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
   === FIX FAULT-HAS-TOP-PRIORITY ===
   === FIX NO-DOUBLE-C-ON-PHASE-CHANGE ===
   ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (refresh=%d ms, "
                  "TIME-PUSH=%d ms, FAULT-PRIORITY, PM-AT-SEND)",
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
        bool in_fault = traffic_light_fault_has_priority();

        if (st == TRAFFIC_STATE_RUNNING && !in_fault) {
            if (last_time_push_ms == 0 ||
                (now - last_time_push_ms) >= TIME_PUSH_PERIOD_MS) {
                last_time_push_ms = now;
                time_sync_root_push(blink_get_rtt_us());
            }
        }

        /* === FIX FAULT-HAS-TOP-PRIORITY ===
           c: отправляем ТОЛЬКО в нормальном режиме (RUNNING + no fault).
           При fault c: уже отправлен из traffic_light_set_fault. */
        if (st == TRAFFIC_STATE_RUNNING && !in_fault) {
            uint8_t  phase = traffic_light_get_phase();
            uint32_t dur   = traffic_light_get_phase_duration_ms();

            if (phase != last_phase) {
                last_phase = phase;
                last_ms = now;
                ESP_LOGD(TAG, "phase changed → c: already sent by "
                              "root_apply_stage_sync_first (ph=%u)", phase);
            } else if ((now - last_ms) >= LORA_SYNC_INTERVAL_MS) {
                if (send_channels_to_node(phase, UINT32_MAX, dur, false)) {
                    last_ms = now;
                    ESP_LOGD(TAG, "c: refresh ph=%u", phase);
                }
            }
        } else {
            last_phase = 0xFF;
            last_ms = 0;
        }
        /* === END FIX FAULT-HAS-TOP-PRIORITY === */

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
                  "FAST-LINK-MONITOR: comm_lost=%d ms, hb=%d ms, "
                  "ROOT-WAIT-NODE-ACK timeout=%d ms, "
                  "WAIT-TIME-SYNC-MAX=%d ms, "
                  "FAULT-NONE-GAP=%d ms, NO-FAULT-NONE-ON-RESEND)",
             NODE_COMM_LOST_MS, HEARTBEAT_INTERVAL_MS,
             WAIT_NODE_ACK_TIMEOUT_MS, WAIT_TIME_SYNC_MAX_MS,
             FAULT_NONE_TO_START_GAP_MS);

    xTaskCreate(phase_sync_task,          "phase_sync",    4096, NULL, 4, NULL);
    xTaskCreate(root_node_watchdog_task,  "root_node_wd",  8192, NULL, 3, NULL);
    xTaskCreate(root_heartbeat_task,      "root_hb",       3072, NULL, 3, NULL);
}