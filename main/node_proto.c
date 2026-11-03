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
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "node_proto";

#define NODE_COMM_LOST_MS     30000
#define RTT_MAX_VALID_US      200000
#define CFG_OK_PERIODIC_MS    5000
#define SYNC_APPLY_MAX_WAIT_MS 1000

volatile bool s_fault_from_remote_ex = false;

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
   on_cfg_from_root — применить CFG.
   ============================================================ */
static void on_cfg_from_root(const char *json, size_t len)
{
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)json, len);
    if (s_cfg_received && crc == s_last_cfg_crc && len == s_last_cfg_len) {
        ESP_LOGI(TAG, "cfg identical (crc=0x%08X, %u bytes) — skip",
                 crc, (unsigned)len);
        traffic_light_set_follow_root(true);
        s_send_cfg_ok = true;
        return;
    }

    ESP_LOGI(TAG, "cfg from root: %u bytes (crc=0x%08X)", (unsigned)len, crc);
    traffic_light_stop();

    if (!config_load_json(json, len)) {
        ESP_LOGE(TAG, "cfg parse FAILED, traffic stays STOPPED");
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

    if (g_config.traffic.enabled && g_config.traffic.auto_start) {
        traffic_light_start();
    }

    s_last_cfg_crc = crc;
    s_last_cfg_len = len;
    s_cfg_received = true;

    s_send_cfg_ok = true;
    ESP_LOGI(TAG, "cfg applied (cfg:ok will be sent by cfg_ok_tx_task)");
}

/* ============================================================
   on_cmd_from_root — обработать команду от ROOT.
   ============================================================ */
static void on_cmd_from_root(const char *cmd, size_t len)
{
    char buf[256];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n); buf[n] = '\0';

    if (strcmp(buf, "cfg:reset") == 0) {
        transport_reset_cfg_accum();
        ESP_LOGI(TAG, "cfg:reset — accum_cfg cleared");
        return;
    }

    if (strncmp(buf, "time:", 5) == 0) {
        unsigned long sec = 0; unsigned ms = 0; unsigned long rtt = 0;
        if (sscanf(buf + 5, "%lu:%u:%lu", &sec, &ms, &rtt) >= 2)
            time_sync_set((uint32_t)sec, (uint16_t)ms, (uint32_t)rtt);
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
    
    // === FIX: Обработка ping обновляет таймер последней связи, предотвращая ALL_OFF ===
    if (strcmp(buf, "ping") == 0) {
        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);
        return;
    }
    // ================================================================================

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
        if (traffic_light_ped_button_enabled())
            traffic_light_request_pedestrian();
        return;
    }

    if (strncmp(buf, "sync:", 5) == 0) {
        unsigned ph = 0, pm = 0, dur = 0, ts = 0, apply_ms = 0;
        int blink_on = -1;

        int matched = sscanf(buf + 5, "%u:%u:%u:%u:%u",
                             &ph, &pm, &dur, &ts, &apply_ms);

        if (matched == 3) {
            apply_ms = 0;
        } else if (matched < 4) {
            return;
        }

        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (s_comm_lost) s_comm_lost = false;

        uint32_t delay_us = s_node_delay_us;
        uint32_t corr = pm + delay_us / 1000;
        if (dur > 0 && corr >= dur) corr = dur - 1;

        if (apply_ms != 0) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            int32_t wait_ms = (int32_t)(apply_ms - now_ms);
            if (wait_ms > 0 && wait_ms <= SYNC_APPLY_MAX_WAIT_MS) {
                ESP_LOGI(TAG, "sync: wait %ld ms for apply_ms=%lu",
                         (long)wait_ms, (unsigned long)apply_ms);
                vTaskDelay(pdMS_TO_TICKS(wait_ms));
            }
        }

        if (traffic_light_get_state() == TRAFFIC_STATE_STOPPED) {
            ESP_LOGW(TAG, "sync: NODE STOPPED → START (follow_root=true)");
            traffic_light_set_follow_root(true);
            traffic_light_start();
        }

        traffic_light_force_phase((uint8_t)ph, corr, dur);
        ESP_LOGI(TAG, "sync applied: phase=%u pm=%lu dur=%lu",
                 ph, (unsigned long)corr, (unsigned long)dur);
        return;
    }

    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        extern void node_handle_local_cmd(const char *cmd);
        node_handle_local_cmd(buf);
        return;
    }

    int relay = -1, state = -1, mode = 0;
    if (strcmp(buf, "traffic:start") == 0) {
        ESP_LOGW(TAG, "traffic:start from root → clear ALL_OFF + START");
        if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }
        traffic_light_set_follow_root(true);
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

static void on_node_status_from_transport(const char *json, size_t len)
{
    extern volatile uint32_t s_last_node_msg_ms;
    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t now = s_last_node_msg_ms;
    if (s_last_node_pub_ms != 0 && (now - s_last_node_pub_ms) < 2000) return;
    s_last_node_pub_ms = now;
    wifi_mqtt_publish_node_json(json, len);

    /* === FIX: ROOT замечает перезагрузку NODE (STOPPED после RUNNING) === */
    if (g_config.role == DEVICE_ROLE_MASTER) {
        static bool s_node_was_running = false;
        bool node_running = (strstr(json, "\"state\":\"RUNNING\"") != NULL);
        if (!node_running && s_node_was_running) {
            ESP_LOGW(TAG, "NODE stopped (reboot?) → s_cfg_ack=false, "
                          "re-send traffic:start + sync:");
            s_cfg_ack = false;

            transport_send_cmd("traffic:start", strlen("traffic:start"));
            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                uint8_t phase = traffic_light_get_phase();
                uint32_t pm = traffic_light_get_phase_ms();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                extern uint32_t send_sync_to_node(uint8_t, uint32_t, uint32_t);
                extern void     apply_sync_at(uint8_t, uint32_t, uint32_t, uint32_t);
                uint32_t apply_ms = send_sync_to_node(phase, pm, dur);
                if (apply_ms != 0) {
                    apply_sync_at(phase, pm, dur, apply_ms);
                }
            }
        }
        s_node_was_running = node_running;
    }
}

static void on_evt_from_node(const char *evt, size_t len)
{
    extern volatile uint32_t s_last_node_msg_ms;
    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (len == 6 && strncmp(evt, "cfg:ok", 6) == 0) {
        s_cfg_ack = true;
        ESP_LOGI(TAG, "NODE confirmed cfg OK");
        return;
    }
    if (len > 5 && strncmp(evt, "pong:", 5) == 0) return;
    if (len > 5 && strncmp(evt, "time:", 5) == 0) return;
    if (len > 6 && strncmp(evt, "fault:", 6) == 0) {
        SET_REMOTE_FLAG(true);
        wifi_mqtt_apply_remote_fault(evt, len);
        SET_REMOTE_FLAG(false);
        return;
    }
    if (len == 18 && strncmp(evt, "pedestrian:request", 18) == 0) {
        if (!traffic_light_ped_button_enabled()) return;
        ESP_LOGW(TAG, "pedestrian:request from NODE");
        traffic_light_request_pedestrian();
        transport_send_cmd("pedestrian:request", strlen("pedestrian:request"));
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
    while (1) {
        uint16_t sr = sr_get_state();
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

        if (!s_cfg_received) {
            vTaskDelay(pdMS_TO_TICKS(15000 + (esp_random() % 5000)));
        } else {
            vTaskDelay(pdMS_TO_TICKS(3000 + (esp_random() % 2000)));
        }
    }
}

static void node_link_watchdog_task(void *arg)
{
    bool     was_connected = false;
    uint32_t disconnected_since_ms = 0;
    bool     in_comm_lost_all_off = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (g_config.role == DEVICE_ROLE_MASTER) continue;

        bool connected = transport_is_lora();
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (connected && !was_connected) {
            was_connected = true;
            disconnected_since_ms = 0;
            s_last_sync_ms = 0;
            s_comm_lost = false;

            if (in_comm_lost_all_off) {
                ESP_LOGW(TAG, "LoRa restored → clear ALL_OFF, START");
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                in_comm_lost_all_off = false;
                if (traffic_light_get_state() == TRAFFIC_STATE_STOPPED) {
                    traffic_light_set_follow_root(true);
                    traffic_light_start();
                }
            }
        } else if (!connected && was_connected) {
            was_connected = false;
            disconnected_since_ms = now;
            if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING)
                traffic_light_stop();
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
            bool lost = s_last_sync_ms != 0 &&
                        (now - s_last_sync_ms > NODE_COMM_LOST_MS);
            if (lost && !s_comm_lost) {
                s_comm_lost = true;
                traffic_light_stop();
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
                in_comm_lost_all_off = true;
            } else if (!lost && s_comm_lost) {
                s_comm_lost = false;
                in_comm_lost_all_off = false;
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
                if (traffic_light_get_state() == TRAFFIC_STATE_STOPPED) {
                    traffic_light_set_follow_root(true);
                    traffic_light_start();
                }
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
    xTaskCreate(cfg_ok_tx_task, "cfg_ok_tx", 4096, NULL, 3, NULL);
    xTaskCreate(node_link_watchdog_task, "node_wd", 3072, NULL, 3, NULL);
}

// === FIX: Добавлена задача heartbeat для ROOT ===
static void root_heartbeat_task(void *arg)
{
    ESP_LOGI(TAG, "root_heartbeat_task started (5s period)");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // было 10000
        if (g_config.role == DEVICE_ROLE_MASTER) {
            esp_err_t err = transport_send_cmd("ping", 4);
            if (err == ESP_OK) {
                ESP_LOGD(TAG, "ROOT: heartbeat ping sent to NODE");
            }
        }
    }
}

void node_proto_start_root_tasks(void)
{
    xTaskCreate(root_heartbeat_task, "root_hb", 3072, NULL, 3, NULL);
}
// ================================================