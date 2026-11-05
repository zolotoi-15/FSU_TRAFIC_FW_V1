/* ============================================================
   node_proto.c — NODE: CFG → STOPPED → traffic:start → RUNNING.

   === FIX SYNC-PM: sync:ph:pm_target:next:on_mask:blink_mask
       pm_target — pm НА МОМЕНТ применения (0 = начало фазы,
                   >0 = resync).
       on_mask   — маска каналов, которые включить.
       blink_mask — маска каналов, которые мигать.

   === FIX SYNC-MASK: NODE применяет маску НАПРЯМУЮ через
       traffic_light_apply_mask() — не заглядывая в свой
       stage_dd. ROOT — единственный источник правды.

   === FIX BACKWARD-COMPAT: NODE принимает и старый 3-полевой
       sync:ph:pm:next (ROOT на старой прошивке). В этом случае
       маски берутся из своего stage_dd через публичный
       traffic_light_get_stage_masks() (БЕЗ traffic_internal.h).

   === FIX P7-sync: NODE ПРИМЕНЯЕТ pedestrian:request от ROOT
       локально.

   === FIX time-delta: log/note delta при приёме time:.
   ============================================================ */

#include "node_proto.h"
#include "app_shared.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "wifi_ap.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "shift_register.h"
#include "rs485.h"
#include "can_bus.h"
#include "mux_data.h"
#include "root_wd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "node_proto";

#define NODE_COMM_LOST_MS     20000
#define CFG_OK_PERIODIC_MS    5000
#define NODE_STATUS_PERIOD_MS 10000
#define NODE_STATUS_FAULT_MS  1000

#define CFG_REQ_PERIOD_MS     5000
static volatile uint32_t s_last_cfg_req_ms = 0;

static volatile bool s_node_cfg_confirmed = false;
static volatile uint32_t s_last_cfg_req_wd_ms = 0;
#define WD_CFG_REQ_PERIOD_MS  5000

/* === FIX SYNC-PM: SYNC_LEAD_MS = запас на ToA (112) + ack_tx.
   NODE ждёт SYNC_LEAD_MS после приёма sync:, затем применяет
   маску через apply_mask(). ROOT делает то же самое. */
#define SYNC_LEAD_MS          300
/* === END FIX SYNC-PM === */

volatile bool s_fault_from_remote_ex = false;
volatile bool s_node_running = false;

static volatile uint32_t s_node_delay_us = 0;
static uint32_t s_last_node_pub_ms = 0;

static uint32_t s_last_cfg_crc = 0;
static size_t   s_last_cfg_len = 0;
static volatile bool s_cfg_received = false;

static volatile bool s_send_cfg_ok = false;

extern volatile bool s_cfg_ack;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

bool node_cfg_received(void) { return s_cfg_received; }

/* ============================================================
   on_cfg_from_root
   ============================================================ */
static void on_cfg_from_root(const char *json, size_t len)
{
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)json, len);
    if (s_cfg_received && crc == s_last_cfg_crc && len == s_last_cfg_len) {
        ESP_LOGI(TAG, "cfg identical (crc=0x%08X, %u bytes) — skip",
                 crc, (unsigned)len);
        traffic_light_set_follow_root(true);
        s_send_cfg_ok = true;
        transport_clear_cfg_expected();
        return;
    }

    bool is_short = (len > 0 && json[0] == '{' &&
                     strstr(json, "\"t\":") != NULL &&
                     strstr(json, "\"meta\":") == NULL);

    ESP_LOGI(TAG, "cfg from root: %u bytes (crc=0x%08X), format=%s",
             (unsigned)len, crc, is_short ? "SHORT" : "FULL");

    traffic_light_stop();

    bool ok;
    if (is_short) {
        ok = config_load_lora_json(json, len);
    } else {
        ok = config_load_json(json, len);
    }

    if (!ok) {
        ESP_LOGE(TAG, "cfg parse FAILED, NODE stays ALL_OFF");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        transport_clear_cfg_expected();
        return;
    }

    ESP_LOGI(TAG, "cfg applied: version=%.2f ped_mode=%u",
             g_config.version, g_config.traffic.pedestrian.request_mode);

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

    traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    traffic_light_reset();
    traffic_light_set_follow_root(true);

    if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
    }

    ESP_LOGI(TAG, "cfg applied, NODE stays STOPPED until traffic:start from ROOT");

    s_last_cfg_crc = crc;
    s_last_cfg_len = len;
    s_cfg_received = true;
    s_send_cfg_ok = true;
    transport_clear_cfg_expected();
}

/* ============================================================
   on_cmd_from_root
   === FIX SYNC-PM + SYNC-MASK + BACKWARD-COMPAT:
       NODE принимает:
         1) sync:ph:pm_target:next:on_mask:blink_mask  (новый)
         2) sync:ph:pm_target:next                     (старый)
       Для нового формата NODE применяет маску НАПРЯМУЮ через
       apply_mask(). Для старого — маски берутся из своего
       stage_dd через ПУБЛИЧНЫЙ traffic_light_get_stage_masks()
       (БЕЗ traffic_internal.h — иначе find_stage не объявлен).
   ============================================================ */
static void on_cmd_from_root(const char *cmd, size_t len)
{
    link_monitor_note_ping_rx();

    char buf[256];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n); buf[n] = '\0';

    if (transport_is_cfg_rx_in_progress() &&
        strcmp(buf, "cfg:reset") != 0) {
        ESP_LOGD(TAG, "CMD '%s' ignored (CFG rx in progress)", buf);
        return;
    }

    if (strcmp(buf, "cfg:reset") == 0) {
        transport_reset_cfg_accum();
        if (s_cfg_received) {
            ESP_LOGW(TAG, "cfg:reset → reset s_cfg_received (wait new CFG)");
            s_cfg_received = false;
        }
        ESP_LOGI(TAG, "cfg:reset — accum_cfg cleared");
        return;
    }

    if (strncmp(buf, "fault:none", 10) == 0) {
        ESP_LOGW(TAG, "fault:none from ROOT → clear local fault");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
        return;
    }

    /* === FIX time-delta === */
    if (strncmp(buf, "time:", 5) == 0) {
        unsigned long sec = 0; unsigned ms = 0; unsigned long rtt = 0;
        if (sscanf(buf + 5, "%lu:%u:%lu", &sec, &ms, &rtt) >= 2) {
            uint32_t local_sec = 0; uint16_t local_ms = 0;
            if (time_sync_get(&local_sec, &local_ms)) {
                int32_t delta = (int32_t)((int64_t)sec * 1000 + ms
                                        - ((int64_t)local_sec * 1000 + local_ms));
                time_sync_note_delta_ms(delta);
                ESP_LOGI(TAG, "time: root=%lu.%03lu local=%lu.%03u delta=%ld ms",
                         sec, ms, (unsigned long)local_sec, local_ms, (long)delta);
            } else {
                ESP_LOGI(TAG, "time: first set root=%lu.%03lu", sec, ms);
            }
            time_sync_set((uint32_t)sec, (uint16_t)ms, (uint32_t)rtt);
        }
        return;
    }
    /* === END FIX time-delta === */

    if (strcmp(buf, "timeq") == 0) {
        uint32_t t_sec = 0; uint16_t t_ms = 0;
        if (time_sync_get(&t_sec, &t_ms)) {
            char evt[64];
            int en = snprintf(evt, sizeof(evt), "time:%lu:%u:%lu",
                              (unsigned long)t_sec, (unsigned)t_ms,
                              (unsigned long)(esp_timer_get_time() / 1000));
            if (en > 0) transport_send_evt(evt, en);
        }
        return;
    }

    if (strcmp(buf, "ping") == 0) {
        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);
        return;
    }

    if (strncmp(buf, "ping:", 5) == 0) {
        unsigned int ts = 0;
        if (sscanf(buf + 5, "%u", &ts) == 1) {
            uint32_t t_recv = (uint32_t)esp_timer_get_time();
            char pong[64];
            int pn = snprintf(pong, sizeof(pong), "pong:%u:%lu:%lu",
                              ts, (unsigned long)t_recv, (unsigned long)t_recv);
            if (pn > 0) transport_send_evt(pong, pn);
        }
        return;
    }
    if (strncmp(buf, "delay:", 6) == 0) {
        unsigned int d = 0;
        if (sscanf(buf + 6, "%u", &d) == 1) {
            s_node_delay_us = d;
            traffic_light_set_sync_delay_us(d);
        }
        return;
    }

    /* === FIX P7-sync === */
    if (strcmp(buf, "pedestrian:request") == 0) {
        ESP_LOGW(TAG, "pedestrian:request from ROOT → apply locally");
        traffic_light_request_pedestrian();
        return;
    }
    /* === END FIX P7-sync === */

    /* === FIX SYNC-PM + SYNC-MASK + BACKWARD-COMPAT ===
       NODE принимает и 5-полевой (новый), и 3-полевой (старый).
       Маски для старого — через ПУБЛИЧНЫЙ
       traffic_light_get_stage_masks() (БЕЗ traffic_internal.h). === */
    if (strncmp(buf, "sync:", 5) == 0) {
        unsigned ph = 0;
        unsigned long pm_target = 0;
        unsigned next_ph = 0;
        uint16_t on_mask = 0;
        uint16_t blink_mask = 0;

        /* Новый формат: sync:ph:pm:next:on_mask:blink_mask.
           %hx — hex в short (16 бит), совместим с uint16_t*. */
        int matched = sscanf(buf + 5, "%u:%lu:%u:0x%hx:0x%hx",
                             &ph, &pm_target, &next_ph,
                             &on_mask, &blink_mask);

        bool has_mask = (matched == 5);

        if (!has_mask) {
            /* Старый формат: sync:ph:pm:next. ROOT outdated.
               Маски — из своего stage_dd через публичный геттер. */
            matched = sscanf(buf + 5, "%u:%lu:%u",
                             &ph, &pm_target, &next_ph);
            if (matched < 3) {
                ESP_LOGW(TAG, "sync: bad format: '%s'", buf);
                return;
            }
            if (ph < g_config.traffic.sequence_count) {
                traffic_light_get_stage_masks((uint8_t)ph,
                                              &on_mask, &blink_mask);
            }
            ESP_LOGW(TAG, "sync: LEGACY 3-field format (ROOT outdated?) — "
                          "on=0x%04X blink=0x%04X (from local stage_dd)",
                     on_mask, blink_mask);
        }

        if (!s_cfg_received) {
            ESP_LOGW(TAG, "sync ignored — no CFG yet (ALL_OFF)");
            return;
        }
        if (traffic_light_get_state() != TRAFFIC_STATE_RUNNING) {
            ESP_LOGW(TAG, "sync ignored — NODE not RUNNING");
            return;
        }

        uint32_t t0_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_last_sync_ms = t0_ms;
        if (s_comm_lost) s_comm_lost = false;
        s_node_cfg_confirmed = true;

        if (next_ph >= g_config.traffic.sequence_count) {
            next_ph = (ph + 1) % g_config.traffic.sequence_count;
        }
        uint32_t dur = traffic_light_stage_duration_ms(next_ph);

        uint32_t apply_ms = t0_ms + SYNC_LEAD_MS;

        ESP_LOGI(TAG, "sync: ph=%u pm_target=%lu apply_ms=%lu next=%u "
                      "on=0x%04X blink=0x%04X%s",
                 ph, pm_target, (unsigned long)apply_ms, next_ph,
                 on_mask, blink_mask, has_mask ? "" : " [legacy]");

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int32_t wait_ms = (int32_t)(apply_ms - now_ms);
        if (wait_ms > 2000) {
            ESP_LOGW(TAG, "sync: wait_ms=%ld > 2000 → clamp", (long)wait_ms);
            wait_ms = 2000;
        }
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }

        now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int32_t lag = (int32_t)(now_ms - apply_ms);
        if (lag < 0) lag = 0;

        /* pm_apply = pm_target + lag
           pm_target=0 → pm_apply=lag — начало фазы.
           pm_target>0 → pm_apply=pm_target+lag — resync. */
        uint32_t pm_apply = (uint32_t)pm_target + (uint32_t)lag;
        if (dur > 0 && pm_apply >= dur) pm_apply = dur - 1;

        /* NODE применяет маску НАПРЯМУЮ через apply_mask() */
        traffic_light_apply_mask(on_mask, blink_mask,
                                 (uint8_t)ph, pm_apply, dur);
        ESP_LOGI(TAG, "sync applied (NODE): ph=%u pm=%lu lag=%ld dur=%lu "
                      "on=0x%04X blink=0x%04X",
                 ph, (unsigned long)pm_apply, (long)lag,
                 (unsigned long)dur, on_mask, blink_mask);
        return;
    }
    /* === END FIX SYNC-PM + SYNC-MASK + BACKWARD-COMPAT === */

    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        extern void node_handle_local_cmd(const char *cmd);
        node_handle_local_cmd(buf);
        return;
    }

    int relay = -1, state = -1, mode = 0;
    if (strcmp(buf, "traffic:start") == 0) {
        if (!s_cfg_received) {
            ESP_LOGW(TAG, "traffic:start ignored — no CFG yet → ALL_OFF");
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            return;
        }
        if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
            ESP_LOGI(TAG, "traffic:start ignored — already RUNNING");
            s_node_cfg_confirmed = true;
            return;
        }
        ESP_LOGW(TAG, "traffic:start from root → clear ALL_OFF + START");
        if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }
        traffic_light_set_follow_root(true);
        s_node_cfg_confirmed = true;
        traffic_light_start();
        return;
    }
    if (strcmp(buf, "traffic:stop")   == 0) { traffic_light_stop();   return; }
    if (strcmp(buf, "traffic:pause")  == 0) { traffic_light_pause();  return; }
    if (strcmp(buf, "traffic:resume") == 0) { traffic_light_resume(); return; }
    if (strcmp(buf, "traffic:reset")  == 0) { traffic_light_reset();  return; }
    if (strcmp(buf, "traffic:next")   == 0) { traffic_light_next_phase(); return; }
    if (sscanf(buf, "traffic:mode:%d", &mode) == 1) {
        if (mode >= 1 && mode <= 3) traffic_light_set_mode((traffic_mode_t)mode);
        return;
    }
    if (sscanf(buf, "relay:%d:%d", &relay, &state) == 2 ||
        sscanf(buf, "sr:%d:%d",    &relay, &state) == 2) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
            if (state) sr_output_on(relay); else sr_output_off(relay);
        }
        return;
    }
    if (strcmp(buf, "all:on")  == 0) { sr_all_on();  return; }
    if (strcmp(buf, "all:off") == 0) { sr_all_off(); return; }
}

void node_handle_local_cmd(const char *cmd)
{
    int baud = 0;
    if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
        rs485_set_baudrate((uint32_t)baud); return;
    }
    if (strncmp(cmd, "rs485:send:", 11) == 0) {
        const char *p = cmd + 11;
        size_t hl = strlen(p);
        if (hl > 0 && (hl % 2) == 0 && hl <= 512) {
            uint8_t buf[256]; size_t n = hl / 2;
            for (size_t i = 0; i < n; i++) {
                int hi = -1, lo = -1;
                char c1 = p[i*2], c2 = p[i*2+1];
                if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                if (hi < 0 || lo < 0) return;
                buf[i] = (uint8_t)((hi << 4) | lo);
            }
            rs485_send(buf, n);
        }
        return;
    }
    unsigned int id = 0; char hex_data[32] = {0}; int ext_flag = 0;
    int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d",
                         &id, hex_data, &ext_flag);
    if (matched >= 2 || (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
        uint8_t data[8] = {0}; uint8_t dlc = 0;
        size_t hl = strlen(hex_data);
        if (hl > 0 && (hl % 2) == 0 && hl <= 16) {
            dlc = hl / 2;
            for (size_t i = 0; i < dlc; i++) {
                int hi = -1, lo = -1;
                char c1 = hex_data[i*2], c2 = hex_data[i*2+1];
                if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                if (hi < 0 || lo < 0) return;
                data[i] = (uint8_t)((hi << 4) | lo);
            }
        }
        can_bus_send(id, data, dlc, ext_flag != 0);
        return;
    }
    if (strcmp(cmd, "can:standby") == 0) { can_bus_set_standby(true);  return; }
    if (strcmp(cmd, "can:normal")  == 0) { can_bus_set_standby(false); return; }
}

/* ============================================================
   on_node_status_from_transport (ROOT)
   ============================================================ */
static void on_node_status_from_transport(const char *json, size_t len)
{
    link_monitor_note_status_rx();

    if (g_config.role == DEVICE_ROLE_MASTER) {
        wifi_mqtt_set_root_comm_lost(false);
    }

    extern volatile uint32_t s_last_node_msg_ms;
    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t now = s_last_node_msg_ms;

    bool should_publish = (s_last_node_pub_ms == 0 ||
                           (now - s_last_node_pub_ms) >= 2000);
    if (should_publish) {
        s_last_node_pub_ms = now;
        wifi_mqtt_publish_node_json(json, len);
    }

    if (g_config.role == DEVICE_ROLE_MASTER) {
        static bool s_node_was_running = false;
        bool node_running = (strstr(json, "\"state\":\"RUNNING\"") != NULL);
        bool node_all_off = (strstr(json, "\"ALL_OFF\"") != NULL);
        s_node_running = node_running;

        if (node_all_off) {
            extern volatile bool s_cfg_ack;
            extern volatile bool s_cfg_applied_and_started;
            if (s_cfg_ack || s_cfg_applied_and_started) {
                ESP_LOGW(TAG, "NODE ALL_OFF → reset cfg tracking");
                s_cfg_ack = false;
                s_cfg_applied_and_started = false;
            }
        }

        if (!node_running && s_node_was_running) {
            ESP_LOGW(TAG, "NODE stopped (reboot?) → re-send traffic:start + resync");
            if (s_cfg_ack) {
                transport_send_cmd("traffic:start", strlen("traffic:start"));
                extern volatile bool s_force_resync_pending;
                s_force_resync_pending = true;
            } else {
                ESP_LOGW(TAG, "NODE stopped but cfg not ack'd → ALL_OFF");
                traffic_light_stop();
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
            }
        }
        s_node_was_running = node_running;
    }
}

/* ============================================================
   on_evt_from_node (ROOT)
   ============================================================ */
static void on_evt_from_node(const char *evt, size_t len)
{
    if (len >= 6 && strncmp(evt, "cfg:ok", 6) == 0) {
        s_cfg_ack = true;
        ESP_LOGI(TAG, "NODE confirmed cfg OK (len=%u)", (unsigned)len);
        return;
    }

    if (len == 7 && strncmp(evt, "cfg:req", 7) == 0) {
        extern volatile bool s_cfg_req_pending;
        s_cfg_req_pending = true;
        ESP_LOGI(TAG, "ROOT: cfg:req from NODE → s_cfg_req_pending=true");
        return;
    }

    if (len == 16 && strncmp(evt, "fault:recover_ok", 16) == 0) {
        if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
            ESP_LOGW(TAG, "NODE recover_ok but ROOT still in fault → ignore");
            return;
        }
        ESP_LOGW(TAG, "NODE recover_ok, ROOT ok → send fault:none");
        transport_send_cmd("fault:none", 10);
        return;
    }

    if (len > 5 && strncmp(evt, "pong:", 5) == 0) return;
    if (len > 5 && strncmp(evt, "time:", 5) == 0) return;
    if (len > 6 && strncmp(evt, "fault:", 6) == 0) {
        SET_REMOTE_FLAG(true);
        wifi_mqtt_apply_remote_fault(evt, len);
        SET_REMOTE_FLAG(false);

        if (len >= 14 && strncmp(evt, "fault:all_off:", 14) == 0) {
            if (!s_node_running) {
                extern volatile bool s_cfg_ack;
                extern volatile bool s_cfg_applied_and_started;
                if (s_cfg_ack || s_cfg_applied_and_started) {
                    ESP_LOGW(TAG, "NODE fault:all_off (not running) → "
                                  "reset cfg tracking, re-send CFG");
                    s_cfg_ack = false;
                    s_cfg_applied_and_started = false;
                }
            }
        }
        return;
    }
    if (len == 18 && strncmp(evt, "pedestrian:request", 18) == 0) {
        if (!traffic_light_ped_button_enabled()) return;
        if (s_cfg_pushing) {
            ESP_LOGW(TAG, "pedestrian:request from NODE ignored (cfg_push active)");
            return;
        }
        ESP_LOGW(TAG, "pedestrian:request from NODE → apply locally on ROOT");
        traffic_light_request_pedestrian();
        return;
    }
}

static void cfg_ok_tx_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_ok_tx_task started");
    uint32_t last_cfg_ok_ms = 0;
    bool     cfg_ok_sent_once = false;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (s_send_cfg_ok) {
            s_send_cfg_ok = false;
            esp_err_t err = transport_send_evt("cfg:ok", strlen("cfg:ok"));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "cfg:ok sent");
                cfg_ok_sent_once = true;
                last_cfg_ok_ms = now;
            } else {
                ESP_LOGW(TAG, "cfg:ok send failed: %s", esp_err_to_name(err));
                s_send_cfg_ok = true;
            }
        }

        if (cfg_ok_sent_once && s_cfg_received &&
            s_last_sync_ms == 0 &&
            (now - last_cfg_ok_ms) >= CFG_OK_PERIODIC_MS) {
            esp_err_t err = transport_send_evt("cfg:ok", strlen("cfg:ok"));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "cfg:ok sent (periodic, no sync yet)");
                last_cfg_ok_ms = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void node_send_task(void *arg)
{
    char *json = malloc(512);
    if (!json) { vTaskDelete(NULL); return; }
    ESP_LOGI(TAG, "node_send_task started (transport=%s)",
             transport_is_lora() ? "LoRa" : "TCP");

    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        if (transport_is_cfg_rx_in_progress()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!transport_is_tx_idle()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!s_cfg_received && transport_is_lora()) {
            uint32_t now_req = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (s_last_cfg_req_ms == 0 ||
                (now_req - s_last_cfg_req_ms) >= CFG_REQ_PERIOD_MS) {
                s_last_cfg_req_ms = now_req;
                esp_err_t rq = transport_send_evt("cfg:req", 7);
                ESP_LOGI(TAG, "cfg:req → ROOT (%s)",
                         rq == ESP_OK ? "OK" : esp_err_to_name(rq));
            }
        }

        uint16_t sr = sr_get_state();
        traffic_fault_t cur_fault = traffic_light_get_fault();
        int off = snprintf(json, 512,
            "{\"role\":\"node\",\"SR\":{\"state\":%u},"
            "\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
            "\"phase_ms\":%lu,\"duration_ms\":%lu},"
            "\"FAULT\":{\"mode\":\"%s\",\"count\":%u}}",
            sr,
            traffic_light_state_str(),
            traffic_light_get_mode(),
            traffic_light_get_phase(),
            (unsigned long)traffic_light_get_phase_ms(),
            (unsigned long)traffic_light_get_phase_duration_ms(),
            traffic_light_fault_str(),
            traffic_light_get_fault_count());

        transport_send_status(json, off);

        uint32_t period = (cur_fault != TRAFFIC_FAULT_NONE)
                          ? NODE_STATUS_FAULT_MS
                          : NODE_STATUS_PERIOD_MS;
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

/* ============================================================
   node_link_watchdog_task
   ============================================================ */
static void node_link_watchdog_task(void *arg)
{
    bool     was_connected = false;
    uint32_t disconnected_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);

    uint32_t first_sync_ms = 0;
    bool     no_cfg_all_off = false;

    bool nvs_cfg_exists = (g_config.traffic.sequence_count > 0 &&
                           g_config.traffic.stage_count > 0);

    if (nvs_cfg_exists) {
        ESP_LOGW(TAG, "node_wd: NVS CFG found (seq=%u) → "
                      "stay STOPPED, wait traffic:start",
                 g_config.traffic.sequence_count);

        {
            extern bool config_export_lora_json(char *b, size_t *l);
            char *buf = malloc(2048);
            size_t blen = 2048;
            if (buf) {
                if (config_export_lora_json(buf, &blen)) {
                    s_last_cfg_crc = esp_rom_crc32_le(0,
                        (const uint8_t *)buf, blen);
                    s_last_cfg_len = blen;
                    ESP_LOGI(TAG, "node_wd: NVS CFG crc=0x%08X len=%u",
                             s_last_cfg_crc, (unsigned)s_last_cfg_len);
                } else {
                    ESP_LOGW(TAG, "node_wd: export_lora_json failed, crc=0");
                }
                free(buf);
            }
        }

        s_cfg_received = true;
        traffic_light_reset();
        traffic_light_set_follow_root(true);
    } else {
        ESP_LOGW(TAG, "node_wd: no CFG in NVS → ALL_OFF + request CFG from ROOT");
        s_cfg_received = false;
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
    }

    while (!transport_is_lora()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if ((uint32_t)(esp_timer_get_time() / 1000) - boot_ms > 10000) {
            ESP_LOGE(TAG, "node_wd: LoRa not ready after 10s → ALL_OFF");
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            return;
        }
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (g_config.role == DEVICE_ROLE_MASTER) continue;

        bool connected = transport_is_lora();
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (connected && !was_connected && first_sync_ms == 0) {
            first_sync_ms = now;
            ESP_LOGI(TAG, "node_wd: first LoRa connect → start no-CFG watchdog");
        }

        if (s_cfg_received) no_cfg_all_off = false;

        if (connected && !was_connected) {
            was_connected = true;
            disconnected_since_ms = 0;
            s_last_sync_ms = 0;
            s_comm_lost = false;

            if (in_comm_lost_all_off) {
                ESP_LOGW(TAG, "LoRa restored → clear ALL_OFF, "
                              "wait traffic:start from ROOT");
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                in_comm_lost_all_off = false;
            }
        } else if (!connected && was_connected) {
            was_connected = false;
            disconnected_since_ms = now;
            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING)
                traffic_light_stop();
        }

        if (connected && !s_node_cfg_confirmed && transport_is_lora() &&
            !transport_is_cfg_rx_in_progress()) {
            uint32_t now_req = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (s_last_cfg_req_wd_ms == 0 ||
                (now_req - s_last_cfg_req_wd_ms) >= WD_CFG_REQ_PERIOD_MS) {
                s_last_cfg_req_wd_ms = now_req;
                esp_err_t rq = transport_send_evt("cfg:req", 7);
                ESP_LOGI(TAG, "node_wd: cfg:req → ROOT (%s), confirmed=%d",
                         rq == ESP_OK ? "OK" : esp_err_to_name(rq),
                         s_node_cfg_confirmed);
            }
        }

        if (!connected && disconnected_since_ms != 0 &&
            (now - disconnected_since_ms) >= NODE_COMM_LOST_MS &&
            !in_comm_lost_all_off) {
            ESP_LOGE(TAG, "LoRa disconnected > %d ms → ALL_OFF",
                     NODE_COMM_LOST_MS);
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            in_comm_lost_all_off = true;
        }

        if (connected) {
            bool lost = (s_last_sync_ms != 0) && link_monitor_node_lost();
            if (lost && !s_comm_lost) {
                s_comm_lost = true;
                traffic_light_stop();
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
                in_comm_lost_all_off = true;
                ESP_LOGE(TAG, "NODE: %u missed pkts → link LOST → ALL_OFF",
                         LINK_LOST_MISSED_PKTS);
            } else if (!lost && s_comm_lost) {
                s_comm_lost = false;
                in_comm_lost_all_off = false;
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                ESP_LOGW(TAG, "NODE: link restored → "
                              "wait traffic:start/sync: from ROOT");
            }

            if (!s_cfg_received && first_sync_ms != 0 && !no_cfg_all_off &&
                !transport_is_cfg_rx_in_progress() &&
                (now - first_sync_ms) >= NODE_COMM_LOST_MS) {
                ESP_LOGE(TAG, "no CFG from root > %d ms → ALL_OFF",
                         NODE_COMM_LOST_MS);
                traffic_light_stop();
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
                no_cfg_all_off = true;
            }
        }
    }
}

void node_proto_register_callbacks(void)
{
    if (g_config.role == DEVICE_ROLE_SLAVE) {
        transport_set_cfg_cb(on_cfg_from_root);
        transport_set_cmd_cb(on_cmd_from_root);
    } else {
        transport_set_evt_cb(on_evt_from_node);
        transport_set_status_cb(on_node_status_from_transport);
    }
}

void node_proto_client_start(const char *host, uint16_t port)
{
    (void)host; (void)port;
    transport_init();
}

void node_proto_start_tasks(void)
{
    xTaskCreate(node_send_task, "node_send", 4096, NULL, 4, NULL);
    xTaskCreate(cfg_ok_tx_task, "cfg_ok_tx", 4096, NULL, 4, NULL);
    xTaskCreate(node_link_watchdog_task, "node_wd", 4096, NULL, 3, NULL);
}

void node_proto_start_root_tasks(void)
{
    ESP_LOGI(TAG, "node_proto_start_root_tasks: no-op "
                  "(heartbeat lives in root_wd.c)");
}