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
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "node_proto";

#define NODE_COMM_LOST_MS     20000
#define CFG_OK_PERIODIC_MS    5000
#define CFG_REQ_PERIODIC_MS   5000
#define NODE_STATUS_PERIOD_MS 10000
#define NODE_STATUS_FAULT_MS  1000
#define NODE_STATUS_STOPPED_MS 2000
#define STATUS_FORCE_MS       5000

/* === FIX CFG-ACK-HEARTBEAT: NODE шлёт cfg:ok периодически,
   чтобы ROOT не считал s_cfg_ack протухшим. === */
#define CFG_OK_HEARTBEAT_MS   15000
/* === END FIX CFG-ACK-HEARTBEAT === */

#define CFG_REQ_PERIOD_MS     5000
static volatile uint32_t s_last_cfg_req_ms = 0;

static volatile bool s_node_cfg_confirmed = false;
static volatile uint32_t s_last_cfg_req_wd_ms = 0;
#define WD_CFG_REQ_PERIOD_MS  5000
#define WD_CFG_REQ_FAST_MS    2000
#define WD_CFG_REQ_FAST_WINDOW_MS 30000

#define SYNC_LEAD_MS          300

static volatile bool     s_status_now = false;
static volatile uint32_t s_status_force_until_ms = 0;

#define FAIL_CFG_PARSE_MAX      3
#define FAIL_SYNC_FORMAT_MAX    5
#define FAIL_APPLY_MASK_MAX     3
#define FAIL_SYNC_IGNORED_MAX   10
#define FAIL_NO_SYNC_MS         10000
#define FAIL_NO_ROOT_DATA_MS    60000
#define FAIL_REBOOT_DELAY_MS    500

#define CFG_FROM_ROOT_TIMEOUT_MS  30000

static volatile uint8_t  s_fail_cfg_parse    = 0;
static volatile uint8_t  s_fail_sync_format  = 0;
static volatile uint8_t  s_fail_apply_mask   = 0;
static volatile uint8_t  s_fail_sync_ignored = 0;

static volatile uint32_t s_traffic_started_ms = 0;
static volatile uint32_t s_last_root_data_ms  = 0;

volatile bool s_fault_from_remote_ex = false;
volatile bool s_node_running = false;

static volatile uint32_t s_node_delay_us = 0;
static uint32_t s_last_node_pub_ms = 0;

static uint32_t s_last_cfg_crc = 0;
static size_t   s_last_cfg_len = 0;
static volatile bool s_cfg_received = false;
static volatile bool s_cfg_confirmed_by_root = false;
static volatile uint32_t s_cfg_from_root_started_ms = 0;

static volatile bool s_send_cfg_ok = false;

extern volatile bool s_cfg_ack;

extern volatile bool     s_insert_pending;
extern volatile uint8_t  s_insert_count;
extern volatile uint8_t  s_insert_idx;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

bool node_cfg_received(void) { return s_cfg_received; }
bool node_cfg_confirmed_by_root(void) { return s_cfg_confirmed_by_root; }

void node_send_status_now(void) { s_status_now = true; }

void node_send_status_for_ms(uint32_t ms)
{
    s_status_now = true;
    s_status_force_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + ms;
}

static void node_fail_safe_reboot(const char *reason)
{
    ESP_LOGE(TAG, "=== FAIL-SAFE REBOOT: %s ===", reason);
    traffic_light_stop();
    SET_REMOTE_FLAG(true);
    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
    SET_REMOTE_FLAG(false);
    vTaskDelay(pdMS_TO_TICKS(FAIL_REBOOT_DELAY_MS));
    esp_restart();
}

static inline void node_note_root_data(void)
{
    s_last_root_data_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

static void on_cfg_from_root(const char *json, size_t len)
{
    node_note_root_data();

    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)json, len);
    if (s_cfg_received && crc == s_last_cfg_crc && len == s_last_cfg_len) {
        ESP_LOGI(TAG, "cfg identical (crc=0x%08X, %u bytes) — skip",
                 crc, (unsigned)len);
        s_fail_cfg_parse = 0;
        traffic_light_set_follow_root(true);
        s_send_cfg_ok = true;
        s_node_cfg_confirmed = true;
        s_cfg_confirmed_by_root = true;
        s_cfg_from_root_started_ms = 0;
        transport_clear_cfg_expected();

        if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
            ESP_LOGI(TAG, "cfg identical → fault cleared");
        }
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
        s_fail_cfg_parse++;
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        transport_clear_cfg_expected();

        if (s_fail_cfg_parse >= FAIL_CFG_PARSE_MAX) {
            node_fail_safe_reboot("cfg parse FAILED x3");
        }
        return;
    }

    s_fail_cfg_parse = 0;

    ESP_LOGI(TAG, "cfg applied: version=%.2f ped_mode=%u hold=%d/%u",
             g_config.version, g_config.traffic.pedestrian.request_mode,
             g_config.traffic.hold.enabled, g_config.traffic.hold.stage_count);

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

    if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
        ESP_LOGI(TAG, "cfg applied → fault cleared (NODE stays STOPPED)");
    }

    ESP_LOGI(TAG, "cfg applied, NODE stays STOPPED until traffic:start from ROOT");

    s_last_cfg_crc = crc;
    s_last_cfg_len = len;
    s_cfg_received = true;
    s_cfg_confirmed_by_root = true;
    s_cfg_from_root_started_ms = 0;
    s_send_cfg_ok = true;
    s_node_cfg_confirmed = true;
    transport_clear_cfg_expected();
}

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
            s_cfg_confirmed_by_root = false;
            s_cfg_from_root_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
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

    if (strncmp(buf, "fault:yellow_blink", 18) == 0) {
        ESP_LOGW(TAG, "fault:yellow_blink from ROOT → apply YELLOW_BLINK");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
        SET_REMOTE_FLAG(false);
        return;
    }

    if (strncmp(buf, "time:", 5) == 0) {
        node_note_root_data();
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
        node_note_root_data();
        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (!s_node_cfg_confirmed && s_cfg_received &&
            !transport_is_cfg_rx_in_progress()) {
            esp_err_t err = transport_send_evt("cfg:req", 7);
            ESP_LOGI(TAG, "ping from ROOT → cfg:req (%s), confirmed=0",
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
        }
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

    if (strcmp(buf, "pedestrian:request") == 0) {
        ESP_LOGW(TAG, "pedestrian:request from ROOT → apply locally");
        traffic_light_request_pedestrian();
        return;
    }

    if (strncmp(buf, "sync:", 5) == 0) {
        node_note_root_data();

        unsigned ph = 0;
        unsigned long pm_target = 0;
        unsigned next_ph = 0;
        uint16_t on_mask = 0;
        uint16_t blink_mask = 0;

        int matched = sscanf(buf + 5, "%u:%lu:%u:0x%hx:0x%hx",
                             &ph, &pm_target, &next_ph,
                             &on_mask, &blink_mask);
        bool has_mask = (matched == 5);

        if (!has_mask) {
            matched = sscanf(buf + 5, "%u:%lu:%u",
                             &ph, &pm_target, &next_ph);
            if (matched < 3) {
                ESP_LOGW(TAG, "sync: bad format: '%s'", buf);
                s_fail_sync_format++;
                if (s_fail_sync_format >= FAIL_SYNC_FORMAT_MAX) {
                    node_fail_safe_reboot("sync bad format x5");
                }
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
        s_fail_sync_format = 0;

        if (!s_cfg_received) {
            ESP_LOGW(TAG, "sync ignored — no CFG yet (ALL_OFF)");
            s_fail_sync_ignored++;
            if (s_fail_sync_ignored >= FAIL_SYNC_IGNORED_MAX) {
                node_fail_safe_reboot("sync ignored (no CFG) x10");
            }
            return;
        }
        if (traffic_light_get_state() != TRAFFIC_STATE_RUNNING) {
            ESP_LOGW(TAG, "sync ignored — NODE not RUNNING");
            s_fail_sync_ignored++;
            if (s_fail_sync_ignored >= FAIL_SYNC_IGNORED_MAX) {
                node_fail_safe_reboot("sync ignored (not RUNNING) x10");
            }
            return;
        }
        s_fail_sync_ignored = 0;

        if (on_mask == 0 && blink_mask == 0) {
            ESP_LOGW(TAG, "sync: empty mask (on=0 blink=0) — ignore");
            s_fail_apply_mask++;
            if (s_fail_apply_mask >= FAIL_APPLY_MASK_MAX) {
                node_fail_safe_reboot("empty mask x3");
            }
            return;
        }
        s_fail_apply_mask = 0;

        s_traffic_started_ms = 0;

        s_status_force_until_ms = 0;
        s_status_now = false;

        uint32_t t0_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_last_sync_ms = t0_ms;
        if (s_comm_lost) s_comm_lost = false;
        s_node_cfg_confirmed = true;

        if (next_ph >= g_config.traffic.sequence_count) {
            next_ph = (ph + 1) % g_config.traffic.sequence_count;
        }

        uint32_t dur = traffic_light_stage_duration_ms(ph);

        uint32_t pm_apply = (uint32_t)pm_target;
        if (dur > 0 && pm_apply >= dur) {
            ESP_LOGW(TAG, "sync: pm_target=%lu >= dur=%lu → set 0",
                     (unsigned long)pm_apply, (unsigned long)dur);
            pm_apply = 0;
        }

        ESP_LOGI(TAG, "sync: ph=%u pm_target=%lu next=%u dur=%lu "
                      "on=0x%04X blink=0x%04X%s",
                 ph, pm_target, next_ph, (unsigned long)dur,
                 on_mask, blink_mask,
                 has_mask ? "" : " [legacy]");

        traffic_light_apply_mask(on_mask, blink_mask,
                                 (uint8_t)ph, pm_apply, dur);
        ESP_LOGI(TAG, "sync applied (NODE): ph=%u pm=%lu dur=%lu "
                      "on=0x%04X blink=0x%04X",
                 ph, (unsigned long)pm_apply,
                 (unsigned long)dur, on_mask, blink_mask);
        return;
    }

    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        extern void node_handle_local_cmd(const char *cmd);
        node_handle_local_cmd(buf);
        return;
    }

    int relay = -1, state = -1, mode = 0;
    if (strcmp(buf, "traffic:start") == 0) {
        node_note_root_data();
        if (!s_cfg_received || !s_cfg_confirmed_by_root) {
            ESP_LOGW(TAG, "traffic:start ignored — no CFG from ROOT yet → ALL_OFF");
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            node_send_status_for_ms(2000);
            return;
        }
        if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
            ESP_LOGI(TAG, "traffic:start ignored — already RUNNING");
            s_node_cfg_confirmed = true;
            node_send_status_for_ms(STATUS_FORCE_MS);
            return;
        }
        ESP_LOGW(TAG, "traffic:start from root → clear ALL_OFF + START");

        traffic_light_set_follow_root(true);

        s_insert_pending = false;
        s_insert_count   = 0;
        s_insert_idx     = 0;

        if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }

        s_node_cfg_confirmed = true;
        traffic_light_start();

        s_traffic_started_ms = (uint32_t)(esp_timer_get_time() / 1000);

        node_send_status_for_ms(STATUS_FORCE_MS);
        ESP_LOGI(TAG, "traffic:start applied → STATUS forced %d ms (RUNNING)",
                 STATUS_FORCE_MS);
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
        /* === FIX PHASE-MISMATCH: если NODE на другой фазе, чем ROOT —
           форсируем resync НЕМЕДЛЕННО, не полагаясь на phase_sync_task.
           Критично после insert и return_to_hold: ROOT вернулся в
           hold-фазу, а NODE остался на последней insert-фазе
           (PED blink). phase_sync_task в hold сам resync шлёт редко,
           поэтому без этого NODE застревает навсегда. === */
        if (!s_cfg_pushing &&
            traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
            int node_phase = -1;
            const char *pp = strstr(json, "\"phase\":");
            if (pp) {
                node_phase = atoi(pp + 8);
            }
            if (node_phase >= 0 &&
                node_phase != (int)traffic_light_get_phase()) {
                ESP_LOGW(TAG, "NODE phase=%d != ROOT phase=%u → force resync NOW",
                         node_phase, traffic_light_get_phase());

                extern volatile bool s_force_resync_pending;
                s_force_resync_pending = true;

                /* FIX: шлём sync сразу, не полагаясь на phase_sync_task.
                   force=false — send_sync_to_node_async_ex сам обойдёт
                   SYNC_MIN_INTERVAL_MS при смене фазы (s_last_sent_sync_phase). */
                extern uint32_t send_sync_to_node_async_ex(uint8_t phase,
                                                           uint32_t phase_ms,
                                                           uint32_t duration_ms,
                                                           bool force,
                                                           uint8_t *out_next_phase);
                uint8_t phase = traffic_light_get_phase();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                uint32_t pm  = traffic_light_get_phase_ms();
                send_sync_to_node_async_ex(phase, pm, dur, false, NULL);
            }
        }
        /* === END FIX PHASE-MISMATCH === */

        static bool s_node_was_running = false;
        bool node_running = (strstr(json, "\"state\":\"RUNNING\"") != NULL);
        bool node_all_off = (strstr(json, "\"ALL_OFF\"") != NULL);
        s_node_running = node_running;

        if (node_all_off) {
            extern volatile bool s_cfg_ack;
            extern volatile bool s_cfg_applied_and_started;
            if (s_cfg_ack || s_cfg_applied_and_started) {
                ESP_LOGW(TAG, "NODE ALL_OFF → reset cfg tracking, re-request CFG");
                s_cfg_ack = false;
                s_cfg_applied_and_started = false;
                extern volatile bool s_cfg_req_pending;
                s_cfg_req_pending = true;
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

static void on_evt_from_node(const char *evt, size_t len)
{
    if (len >= 6 && strncmp(evt, "cfg:ok", 6) == 0) {
        s_cfg_ack = true;
        extern volatile uint32_t s_cfg_last_ok_ms;
        s_cfg_last_ok_ms = (uint32_t)(esp_timer_get_time() / 1000);
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
                    extern volatile bool s_cfg_req_pending;
                    s_cfg_req_pending = true;
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
    ESP_LOGI(TAG, "cfg_ok_tx_task started (CFG-ACK-HEARTBEAT %d ms)",
             CFG_OK_HEARTBEAT_MS);
    uint32_t last_cfg_ok_ms = 0;
    uint32_t last_cfg_req_ms = 0;
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

        /* === FIX CFG-ACK-HEARTBEAT: периодический cfg:ok как heartbeat.
           ROOT сбрасывает s_cfg_ack по CFG_ACK_STALE_MS (60 сек), если
           не получает cfg:ok. Раньше cfg:ok слался только один раз
           после CFG, и ROOT запускал циклическую перезагрузку CFG.
           Теперь NODE шлёт cfg:ok каждые 15 сек — ROOT всегда знает,
           что CFG актуален. === */
        if (cfg_ok_sent_once && s_cfg_received &&
            (now - last_cfg_ok_ms) >= CFG_OK_HEARTBEAT_MS) {
            esp_err_t err = transport_send_evt("cfg:ok", strlen("cfg:ok"));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "cfg:ok sent (periodic heartbeat)");
                last_cfg_ok_ms = now;
            }
        }
        /* === END FIX CFG-ACK-HEARTBEAT === */

        if (!s_node_cfg_confirmed && s_cfg_received &&
            transport_is_lora() && !transport_is_cfg_rx_in_progress() &&
            (now - last_cfg_req_ms) >= CFG_REQ_PERIODIC_MS) {
            last_cfg_req_ms = now;
            esp_err_t err = transport_send_evt("cfg:req", 7);
            ESP_LOGW(TAG, "cfg:req (periodic, ROOT not confirmed) → %s",
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
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

    if (!s_cfg_confirmed_by_root) {
        node_send_status_for_ms(3000);
    }

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
        const char *state_str = traffic_light_state_str();
        const char *fault_str = traffic_light_fault_str();

        if (!s_cfg_confirmed_by_root) {
            state_str = "STOPPED";
            fault_str = "ALL_OFF";
        }

        int off = snprintf(json, 512,
            "{\"role\":\"node\",\"SR\":{\"state\":%u},"
            "\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
            "\"phase_ms\":%lu,\"duration_ms\":%lu},"
            "\"FAULT\":{\"mode\":\"%s\",\"count\":%u}}",
            sr,
            state_str,
            traffic_light_get_mode(),
            traffic_light_get_phase(),
            (unsigned long)traffic_light_get_phase_ms(),
            (unsigned long)traffic_light_get_phase_duration_ms(),
            fault_str,
            traffic_light_get_fault_count());

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool force = s_status_now ||
                     (s_status_force_until_ms != 0 &&
                      now_ms < s_status_force_until_ms);
        if (s_status_now) s_status_now = false;
        if (s_status_force_until_ms != 0 && now_ms >= s_status_force_until_ms) {
            s_status_force_until_ms = 0;
        }

        if (force) {
            esp_err_t err = transport_send_status_noack(json, off);
            ESP_LOGI(TAG, "STATUS forced (no-ack) → ROOT: %s",
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
        } else {
            transport_send_status(json, off);
        }

        uint32_t period;
        if (force) {
            period = 200;
        } else if (cur_fault != TRAFFIC_FAULT_NONE) {
            period = NODE_STATUS_FAULT_MS;
        } else if (traffic_light_get_state() != TRAFFIC_STATE_RUNNING) {
            period = NODE_STATUS_STOPPED_MS;
        } else {
            period = NODE_STATUS_PERIOD_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

static void node_link_watchdog_task(void *arg)
{
    ESP_LOGW(TAG, "node_wd: task started (role=%s)",
             g_config.role == DEVICE_ROLE_MASTER ? "MASTER" : "SLAVE");

    bool     was_connected = false;
    uint32_t disconnected_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t wd_start_ms = 0;

    uint32_t first_sync_ms = 0;
    bool     no_cfg_all_off = false;

    bool nvs_cfg_exists = (g_config.traffic.sequence_count > 0 &&
                           g_config.traffic.stage_count > 0);

    if (nvs_cfg_exists) {
        ESP_LOGW(TAG, "node_wd: NVS CFG found (seq=%u) → "
                      "stay STOPPED, wait CFG from ROOT",
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

        traffic_light_reset();
        traffic_light_set_follow_root(true);
        s_cfg_received = false;
        s_cfg_confirmed_by_root = false;
        s_cfg_from_root_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_node_cfg_confirmed = false;

        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);

        ESP_LOGW(TAG, "node_wd: NVS CFG → request CFG from ROOT (cfg:req)");
    } else {
        ESP_LOGW(TAG, "node_wd: no CFG in NVS → ALL_OFF + request CFG from ROOT");
        s_cfg_received = false;
        s_cfg_confirmed_by_root = false;
        s_cfg_from_root_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
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
    ESP_LOGW(TAG, "node_wd: LoRa ready, entering main loop");
    wd_start_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (g_config.role == DEVICE_ROLE_MASTER) continue;

        bool connected = transport_is_lora();
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (nvs_cfg_exists && !s_cfg_confirmed_by_root &&
            s_cfg_from_root_started_ms != 0 &&
            (now - s_cfg_from_root_started_ms) >= CFG_FROM_ROOT_TIMEOUT_MS) {
            node_fail_safe_reboot("CFG from ROOT not received in 30 s");
        }

        if (connected) {
            if (s_traffic_started_ms != 0 &&
                traffic_light_get_state() == TRAFFIC_STATE_RUNNING &&
                (now - s_traffic_started_ms) >= FAIL_NO_SYNC_MS &&
                s_last_sync_ms == 0) {
                node_fail_safe_reboot("traffic:start without sync: > 10 s");
            }

            if (s_cfg_received && s_last_root_data_ms != 0 &&
                s_traffic_started_ms != 0 &&
                (now - s_last_root_data_ms) >= FAIL_NO_ROOT_DATA_MS) {
                node_fail_safe_reboot("no root data > 60 s");
            }
        }

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
            s_traffic_started_ms = 0;
            s_last_root_data_ms  = 0;
        }

        if (connected && !s_node_cfg_confirmed && transport_is_lora() &&
            !transport_is_cfg_rx_in_progress()) {
            uint32_t now_req = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t req_period = WD_CFG_REQ_PERIOD_MS;
            if (wd_start_ms != 0 && (now - wd_start_ms) < WD_CFG_REQ_FAST_WINDOW_MS) {
                req_period = WD_CFG_REQ_FAST_MS;
            }
            if (s_last_cfg_req_wd_ms == 0 ||
                (now_req - s_last_cfg_req_wd_ms) >= req_period) {
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