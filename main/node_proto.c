#include "node_proto.h"
#include "app_shared.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "node_link.h"
#include "wifi_ap.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "shift_register.h"
#include "rs485.h"
#include "can_bus.h"
#include "mux_data.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "node_proto";

#define MQTT_JSON_BUFFER_SIZE 8192
#define AP_PASSWORD           "fsu12345678"
#define NODE_COMM_LOST_MS     30000
#define NODE_COMM_CHECK_MS    500
#define RTT_MAX_VALID_US      200000

#define AP_RETRY_MIN_INTERVAL_US  (5000000LL)   // 5 с
#define AP_RETRY_POLL_MS          2000          // 2 с

// === ВАРИАНТ B: 120 с (было 60) — запас на смену канала AP ===
#define NODE_AP_REBOOT_MS   (120 * 1000)

// === НОВОЕ: лимит попыток STA-подключения ===
#define NODE_AP_MAX_ATTEMPTS  5

// === ВАРИАНТ B: флаг форсированного переподключения Wi-Fi ===
static volatile bool s_force_wifi_reconnect = false;

volatile bool s_fault_from_remote_ex = false;

static volatile uint32_t s_node_delay_us = 0;

static uint32_t s_last_node_pub_ms = 0;

/* FIX: CRC идентичного конфига. */
static uint32_t s_last_cfg_crc = 0;
static size_t   s_last_cfg_len = 0;
static bool     s_cfg_applied  = false;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* ============================================================
   FIX: on_cfg_from_root — конфигурация от root.
   ============================================================ */
static void on_cfg_from_root(const char *json, size_t len)
{
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)json, len);
    if (s_cfg_applied && crc == s_last_cfg_crc && len == s_last_cfg_len) {
        ESP_LOGI(TAG, "cfg from root: identical (crc=0x%08X, %u bytes) — "
                      "skip restart, traffic stays %s",
                 crc, (unsigned)len, traffic_light_state_str());
        traffic_light_set_follow_root(true);
        return;
    }

    ESP_LOGI(TAG, "cfg from root: %u bytes (crc=0x%08X, changed)",
             (unsigned)len, crc);

    traffic_light_stop();

    if (!config_load_json(json, len)) {
        ESP_LOGE(TAG, "cfg from root: parse FAILED, traffic stays STOPPED");
        return;
    }

    ESP_LOGI(TAG, "cfg from root: applied to NVS (version=%.2f, ped_mode=%u)",
             g_config.version,
             g_config.traffic.pedestrian.request_mode);

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
    if (mr != ESP_OK) ESP_LOGW(TAG, "node set_mode: %s", esp_err_to_name(mr));
    traffic_light_reset();

    if (g_config.traffic.enabled && g_config.traffic.auto_start) {
        traffic_light_start();
        ESP_LOGI(TAG, "node: cfg applied, traffic STARTED (mode=%u, seq=%u)",
                 g_config.traffic.mode, g_config.traffic.sequence_count);
    } else {
        ESP_LOGI(TAG, "node: cfg applied, traffic STOPPED (auto_start=0)");
    }

    traffic_light_set_follow_root(true);

    s_last_cfg_crc = crc;
    s_last_cfg_len = len;
    s_cfg_applied  = true;

    ESP_LOGI(TAG, "PED_BUTTON: enabled=%d request_mode=%u",
             traffic_light_ped_button_enabled(),
             g_config.traffic.pedestrian.request_mode);
}

/* ============================================================
   on_cmd_from_root
   ============================================================ */
static void on_cmd_from_root(const char *cmd, size_t len)
{
    char buf[256];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n); buf[n] = '\0';

    if (strncmp(buf, "time:", 5) == 0) {
        unsigned long sec = 0; unsigned ms = 0; unsigned long rtt = 0;
        if (sscanf(buf + 5, "%lu:%u:%lu", &sec, &ms, &rtt) >= 2) {
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
            if (en > 0 && node_link_client_connected())
                node_link_client_send_evt(evt, en);
        }
        return;
    }

    if (strncmp(buf, "fault:", 6) == 0) {
        ESP_LOGW(TAG, "cmd fault from root: '%s'", buf);
        const char *p = buf + 6;
        char mode[16] = {0};
        const char *c1 = strchr(p, ':');
        if (!c1) return;
        size_t mlen = c1 - p;
        if (mlen >= sizeof(mode)) mlen = sizeof(mode) - 1;
        memcpy(mode, p, mlen);
        mode[mlen] = '\0';

        if (strcmp(mode, "none") == 0) {
            if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                if (!s_comm_lost) traffic_light_start();
            }
            return;
        }

        traffic_fault_t nf = TRAFFIC_FAULT_NONE;
        if (strcmp(mode, "all_off") == 0) nf = TRAFFIC_FAULT_ALL_OFF;
        else if (strcmp(mode, "yellow_blink") == 0) nf = TRAFFIC_FAULT_YELLOW_BLINK;

        if (nf == TRAFFIC_FAULT_YELLOW_BLINK) {
            uint32_t last_ok = traffic_light_last_recovery_ok_ms();
            uint32_t now_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (last_ok != 0 && (now_ms - last_ok) < 5000) return;
        }

        traffic_fault_t current = traffic_light_get_fault();
        int nf_pri  = wifi_mqtt_fault_priority_ex(nf);
        int cur_pri = wifi_mqtt_fault_priority_ex(current);

        if (current == TRAFFIC_FAULT_ALL_OFF && nf_pri < cur_pri) return;

        if (nf_pri >= cur_pri) {
            if (current != nf) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(nf);
                SET_REMOTE_FLAG(false);
            }
        } else {
            char rep[128];
            size_t rn = traffic_light_build_fault_report(rep, sizeof(rep));
            if (rn > 0 && node_link_client_connected())
                node_link_client_send_evt(rep, rn);
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
            if (pn > 0 && node_link_client_connected())
                node_link_client_send_evt(pong, pn);
        }
        return;
    }
    if (strncmp(buf, "delay:", 6) == 0) {
        unsigned int d = 0;
        if (sscanf(buf + 6, "%u", &d) == 1) {
            s_node_delay_us = d;
            traffic_light_set_sync_delay_us(d);
            ESP_LOGI(TAG, "node: sync_delay = %u us (%.3f ms)", d, d / 1000.0);
        }
        return;
    }

    ESP_LOGI(TAG, "cmd from root: '%s'", buf);

    if (strcmp(buf, "pedestrian:request") == 0) {
        if (!traffic_light_ped_button_enabled()) {
            ESP_LOGW(TAG, "cmd pedestrian:request ignored (button disabled)");
            return;
        }
        traffic_light_request_pedestrian();
        return;
    }

    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        extern void node_handle_local_cmd(const char *cmd);
        node_handle_local_cmd(buf);
        return;
    }

    if (strncmp(buf, "sync:", 5) == 0) {
        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);

        unsigned ph = 0, pm = 0, dur = 0, ts = 0;
        int blink_on = -1;
        int matched = sscanf(buf + 5, "%u:%u:%u:%u:%d",
                             &ph, &pm, &dur, &ts, &blink_on);
        if (matched >= 3) {
            if (s_comm_lost) {
                s_comm_lost = false;
                ESP_LOGI(TAG, "first sync: after comm_lost → clearing flag");
            }
            if (blink_on >= 0 &&
                traffic_light_get_fault() == TRAFFIC_FAULT_YELLOW_BLINK) {
                traffic_light_apply_fault_blink(blink_on != 0);
            }
            if (dur > 0 && pm > dur + TRAFFIC_PHASE_OVERDUE_MS && pm != 0) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
                SET_REMOTE_FLAG(false);
                return;
            }
            uint32_t delay_us = s_node_delay_us;
uint32_t corrected_pm = pm + delay_us / 1000;
if (dur > 0 && corrected_pm >= dur) {
    // === FIX: не skip, а clamp к dur-1, чтобы force_phase сбросил phase_ms ===
    ESP_LOGD(TAG, "sync: pm_corr=%lu >= dur=%lu → clamp to %lu",
             (unsigned long)corrected_pm, (unsigned long)dur,
             (unsigned long)(dur - 1));
    corrected_pm = dur - 1;
}
traffic_light_force_phase((uint8_t)ph, corrected_pm, dur);
            return;
        }
    }

    int relay = -1, state = -1, mode = 0;
    if (strcmp(buf, "traffic:start")  == 0) { traffic_light_start();  return; }
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

/* ============================================================
   node_handle_local_cmd
   ============================================================ */
void node_handle_local_cmd(const char *cmd)
{
    int baud = 0;
    if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
        rs485_set_baudrate((uint32_t)baud);
        return;
    }
    if (strncmp(cmd, "rs485:send:", 11) == 0) {
        const char *p = cmd + 11;
        size_t hl = strlen(p);
        if (hl > 0 && (hl % 2) == 0 && hl <= 512) {
            uint8_t buf[256];
            size_t n = hl / 2;
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
    unsigned int id = 0;
    char hex_data[32] = {0};
    int ext_flag = 0;
    int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d",
                         &id, hex_data, &ext_flag);
    if (matched >= 2 || (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
        uint8_t data[8] = {0};
        uint8_t dlc = 0;
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
   FIX: on_node_json_from_tcp (root side) — status: от node.
   ============================================================ */
static void on_node_json_from_tcp(const char *json, size_t len)
{
    extern volatile uint32_t s_last_node_msg_ms;
    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    uint32_t now = s_last_node_msg_ms;
    if (s_last_node_pub_ms != 0 && (now - s_last_node_pub_ms) < 2000) return;
    s_last_node_pub_ms = now;

    ESP_LOGI(TAG, "node JSON from TCP: %u bytes → MQTT", (unsigned)len);
    wifi_mqtt_publish_node_json(json, len);
}

/* ============================================================
   FIX: on_evt_from_node (root side) — evt: от node.
   ============================================================ */
static void on_evt_from_node(const char *evt, size_t len)
{
    extern volatile uint32_t s_last_node_msg_ms;
    extern volatile uint32_t s_last_pong_ms;
    extern volatile uint32_t s_rtt_us;
    extern volatile uint32_t s_rtt_delay_us;
    extern volatile int32_t  s_rtt_samples;

    ESP_LOGW(TAG, "evt from node: %.*s", (int)len, evt);
    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (len > 5 && strncmp(evt, "pong:", 5) == 0) {
        uint32_t ts = 0, t_recv = 0, t_send = 0;
        if (sscanf(evt + 5, "%" SCNu32 ":%" SCNu32 ":%" SCNu32,
                   &ts, &t_recv, &t_send) >= 1) {
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            if (now_us >= ts) {
                uint32_t rtt = now_us - ts;
                if (rtt > 0 && rtt < RTT_MAX_VALID_US) {
                    uint32_t delay = rtt / 2;
                    if (s_rtt_delay_us == 0) s_rtt_delay_us = delay;
                    else s_rtt_delay_us = (s_rtt_delay_us * 7 + delay * 3) / 10;
                    s_rtt_us = rtt;
                    s_rtt_samples++;
                    s_last_pong_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    traffic_light_set_sync_delay_us(s_rtt_delay_us);
                    ESP_LOGI(TAG, "pong: rtt=%lu us → delay=%lu us (%.3f ms)",
                             (unsigned long)rtt, (unsigned long)s_rtt_delay_us,
                             s_rtt_delay_us / 1000.0);
                    if (node_link_server_has_client()) {
                        char dcmd[48];
                        int dn = snprintf(dcmd, sizeof(dcmd), "delay:%lu",
                                          (unsigned long)s_rtt_delay_us);
                        if (dn > 0) node_link_server_send_cmd(dcmd, dn);
                    }
                }
            }
        }
        return;
    }
    if (len > 5 && strncmp(evt, "time:", 5) == 0) {
        unsigned long n_sec = 0; unsigned n_ms = 0; unsigned long n_up = 0;
        if (sscanf(evt + 5, "%lu:%u:%lu", &n_sec, &n_ms, &n_up) >= 2) {
            uint32_t r_sec = 0; uint16_t r_ms = 0;
            if (time_sync_get(&r_sec, &r_ms)) {
                int64_t diff_ms = ((int64_t)n_sec - (int64_t)r_sec) * 1000
                                + ((int64_t)n_ms - (int64_t)r_ms);
                if (diff_ms < 0) diff_ms = -diff_ms;
                if (diff_ms > 2000) {
                    ESP_LOGW(TAG, "node clock drift %lld ms",
                             (long long)diff_ms);
                }
            }
        }
        return;
    }
    if (len > 6 && strncmp(evt, "fault:", 6) == 0) {
        SET_REMOTE_FLAG(true);
        wifi_mqtt_apply_remote_fault(evt, len);
        SET_REMOTE_FLAG(false);
        return;
    }
    if (len == 18 && strncmp(evt, "pedestrian:request", 18) == 0) {
        if (!traffic_light_ped_button_enabled()) return;
        traffic_light_request_pedestrian();
        if (node_link_server_has_client()) {
            node_link_server_send_cmd("pedestrian:request",
                                      strlen("pedestrian:request"));
        }
        return;
    }
}

/* ============================================================
   FIX: node_send_task (node side) — status: → root.
   ============================================================ */
static void node_send_task(void *arg)
{
    char *json = malloc(MQTT_JSON_BUFFER_SIZE);
    if (!json) { vTaskDelete(NULL); return; }
    ESP_LOGI(TAG, "node_send_task started");
    while (1) {
        if (node_link_client_connected()) {
            uint16_t sr = sr_get_state();
            const char *fault_mode = traffic_light_fault_str();
            uint8_t fault_cnt = traffic_light_get_fault_count();

            int off = snprintf(json, MQTT_JSON_BUFFER_SIZE,
                "{\"role\":\"node\",\"SR\":{\"state\":%u},"
                "\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
                "\"phase_ms\":%lu,\"duration_ms\":%lu},"
                "\"FAULT\":{\"mode\":\"%s\",\"count\":%u,\"channels\":[",
                sr,
                traffic_light_state_str(),
                traffic_light_get_mode(),
                traffic_light_get_phase(),
                (unsigned long)traffic_light_get_phase_ms(),
                (unsigned long)traffic_light_get_phase_duration_ms(),
                fault_mode, fault_cnt);
            for (uint8_t i = 0; i < fault_cnt && i < TRAFFIC_FAULT_MAX; i++) {
                traffic_fault_info_t info;
                traffic_light_get_fault_info(i, &info);
                off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                                "%s%u", (i == 0) ? "" : ",", info.channel);
            }
            off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                            "],\"reasons\":[");
            for (uint8_t i = 0; i < fault_cnt && i < TRAFFIC_FAULT_MAX; i++) {
                traffic_fault_info_t info;
                traffic_light_get_fault_info(i, &info);
                char safe[sizeof(info.text) * 2];
                size_t j = 0;
                for (size_t k = 0; info.text[k] && j + 2 < sizeof(safe); k++) {
                    char c = info.text[k];
                    if (c == '"' || c == '\\') safe[j++] = '\\';
                    safe[j++] = c;
                }
                safe[j] = '\0';
                off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                                "%s\"%s\"", (i == 0) ? "" : ",", safe);
            }
            off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off, "]},");

            rs485_stats_t rs;
            rs485_get_stats(&rs);
            can_bus_stats_t cs;
            can_bus_get_stats(&cs);

            off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                "\"RS485\":{\"rx_b\":%lu,\"rx_f\":%lu,\"ovf\":%lu,\"tx_b\":%lu,\"tx_err\":%lu},"
                "\"CAN\":{\"rx\":%lu,\"tx\":%lu,\"tx_err\":%lu,"
                "\"bus_off\":%lu,\"rec\":%lu,\"err_p\":%lu,\"state\":%lu},",
                (unsigned long)rs.rx_bytes, (unsigned long)rs.rx_frames,
                (unsigned long)rs.rx_overflows, (unsigned long)rs.tx_bytes,
                (unsigned long)rs.tx_errors,
                (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
                (unsigned long)cs.recovery_count,
                (unsigned long)cs.error_passive_count,
                (unsigned long)cs.last_state);

            uint32_t t_sec = 0; uint16_t t_ms = 0;
            if (time_sync_get(&t_sec, &t_ms)) {
                off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                    "\"TIME\":{\"unix\":%lu,\"ms\":%u},",
                    (unsigned long)t_sec, (unsigned)t_ms);
            } else {
                off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                    "\"TIME\":null,");
            }
            off += snprintf(json + off, MQTT_JSON_BUFFER_SIZE - off,
                "\"NET\":{\"ip\":\"%s\",\"sta\":%d},\"LOST\":%d}",
                wifi_get_ip(),
                wifi_sta_connected() ? 1 : 0,
                s_comm_lost ? 1 : 0);

            node_link_client_send(json, off);
        }
        time_sync_node_report();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ============================================================
   node_ap_retry_task
   === ВАРИАНТ B FIX: убран esp_wifi_restore() ===
   === НОВОЕ: лимит попыток + vTaskDelay перед первым сканом ===
   ============================================================ */
static void node_ap_retry_task(void *arg)
{
    ESP_LOGI(TAG, "node_ap_retry_task started (max %d attempts, "
                  "then stop until reboot)",
             NODE_AP_MAX_ATTEMPTS);

    // === НОВОЕ: дать Wi-Fi время на инициализацию перед первым сканом ===
    vTaskDelay(pdMS_TO_TICKS(1000));

    int64_t  s_last_connect_us   = 0;
    int64_t  s_first_attempt_us  = 0;
    int      s_attempts          = 0;
    bool     s_stopped           = false;

    while (1) {
        if (s_force_wifi_reconnect) {
            ESP_LOGW(TAG, "force Wi-Fi reconnect (TCP lost, AP channel may have changed)");
            s_force_wifi_reconnect = false;
            esp_wifi_disconnect();
            s_first_attempt_us = 0;
            s_last_connect_us  = 0;
            s_attempts         = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (!wifi_sta_connected()) {
            if (!wifi_sta_connecting()) {
                int64_t now = esp_timer_get_time();
                if (s_first_attempt_us == 0) s_first_attempt_us = now;

                if ((now - s_first_attempt_us) > (NODE_AP_REBOOT_MS * 1000LL)) {
                    ESP_LOGE(TAG, "NODE: not connected to root AP for %d s → REBOOT",
                             NODE_AP_REBOOT_MS / 1000);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                }

                if (s_last_connect_us == 0 ||
                    (now - s_last_connect_us) > AP_RETRY_MIN_INTERVAL_US) {

                    if (s_attempts >= NODE_AP_MAX_ATTEMPTS && !s_stopped) {
                        ESP_LOGE(TAG, "NODE: %d STA connect attempts failed → "
                                      "STOP scanning until reboot",
                                 s_attempts);
                        s_stopped = true;
                        esp_wifi_disconnect();
                        vTaskDelete(NULL);
                    }

                    char   ap_ssid[33] = {0};
                    int8_t ap_rssi = -127;
                    if (wifi_scan_for_fsu_ap(ap_ssid, sizeof(ap_ssid),
                                             &ap_rssi, "FSU_") == ESP_OK) {
                        s_attempts++;
                        ESP_LOGI(TAG, "retry: found '%s' rssi=%d, connecting "
                                      "(attempt %d/%d)",
                                 ap_ssid, ap_rssi,
                                 s_attempts, NODE_AP_MAX_ATTEMPTS);
                        wifi_config_t sta_cfg = {0};
                        strncpy((char*)sta_cfg.sta.ssid, ap_ssid,
                                sizeof(sta_cfg.sta.ssid) - 1);
                        strncpy((char*)sta_cfg.sta.password, AP_PASSWORD,
                                sizeof(sta_cfg.sta.password) - 1);
                        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
                        sta_cfg.sta.pmf_cfg.capable  = false;
                        sta_cfg.sta.pmf_cfg.required = false;
                        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
                        esp_wifi_connect();
                        s_last_connect_us = now;
                    } else {
                        s_attempts++;
                        ESP_LOGW(TAG, "retry: no FSU_*_AP found "
                                      "(attempt %d/%d, root may still be "
                                      "switching channel or off)",
                                 s_attempts, NODE_AP_MAX_ATTEMPTS);
                        s_last_connect_us = 0;
                    }
                }
            }
        } else {
            s_last_connect_us  = 0;
            s_first_attempt_us = 0;
            s_attempts         = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(AP_RETRY_POLL_MS));
    }
}


/* ============================================================
   FIX: node_link_watchdog_task — потеря TCP.
   === ВАРИАНТ B: при разрыве TCP устанавливаем s_force_wifi_reconnect ===
   ============================================================ */
static void node_link_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "node_link_watchdog_task started (lost > %d ms → ALL_OFF)",
             NODE_COMM_LOST_MS);
    bool was_connected = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(NODE_COMM_CHECK_MS));
        if (g_config.role == DEVICE_ROLE_MASTER) continue;

        bool connected = node_link_client_connected();

        if (connected && !was_connected) {
            ESP_LOGI(TAG, "NODE: root TCP connected (reset sync/lost, waiting for cfg)");
            was_connected = true;
            s_last_sync_ms = 0;
            s_comm_lost = false;
        } else if (!connected && was_connected) {
            ESP_LOGW(TAG, "NODE: root TCP disconnected → stop + force Wi-Fi reconnect");
            was_connected = false;
            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING)
                traffic_light_stop();

            s_force_wifi_reconnect = true;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        bool lost = connected &&
                    (s_last_sync_ms != 0) &&
                    (now - s_last_sync_ms > NODE_COMM_LOST_MS);

        if (lost && !s_comm_lost) {
            ESP_LOGE(TAG, "NODE: comm lost > %d ms → ALL_OFF", NODE_COMM_LOST_MS);
            s_comm_lost = true;
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
        } else if (!lost && s_comm_lost) {
            ESP_LOGW(TAG, "NODE: comm restored → resume");
            s_comm_lost = false;
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
            if (traffic_light_get_fault() == TRAFFIC_FAULT_NONE)
                traffic_light_start();
        }
    }
}

/* ============================================================
   Публичное API.
   ============================================================ */
void node_proto_register_callbacks(void)
{
    if (g_config.role == DEVICE_ROLE_SLAVE) {
        node_link_set_cfg_cb(on_cfg_from_root);
        node_link_set_cmd_cb(on_cmd_from_root);
    } else {
        node_link_set_evt_cb(on_evt_from_node);
        node_link_set_cb(on_node_json_from_tcp);
    }
}

void node_proto_client_start(const char *host, uint16_t port)
{
    node_link_client_start(host, port);
}

void node_proto_start_tasks(void)
{
    xTaskCreate(node_send_task, "node_send", 4096, NULL, 4, NULL);
    xTaskCreate(node_ap_retry_task, "node_ap_retry", 4096, NULL, 3, NULL);
    xTaskCreate(node_link_watchdog_task, "node_link_wd", 3072, NULL, 3, NULL);
}