/* ============================================================
   node_proto.c — NODE/ROOT протокол.

   === FIX NO-FIRST-C-WD ===
       node_link_watchdog_task: если NODE в RUNNING, но первый
       c: не пришёл за NODE_ROOT_LOST_MS от traffic:start —
       YELLOW_BLINK. Раньше `continue` при !s_c_received_since_start
       приводил к тому, что watchdog молчал, пока c: не придёт.
       Если ROOT умер сразу после traffic:start — NODE навсегда
       в RUNNING без маски и без fail-safe.

   === FIX SYNC-BY-STATUS ===
       on_node_status_from_transport (ROOT): сверяем phase/phase_ms
       из STATUS с локальной ROOT. Если расхождение > 500 мс или
       phase другой — force resync c:.

   === FIX TRAFFIC-START-KEEP-C-TIMER ===
       on_cmd_from_root(traffic:start): не сбрасывать
       s_c_received_since_start, если NODE уже RUNNING. Иначе
       повторный traffic:start (ROOT шлёт его при YELLOW_BLINK)
       сбрасывает таймер — NODE снова уходит в YELLOW_BLINK
       через 10 с, хотя уже работает по c:.
   ============================================================ */

#include "node_proto.h"
#include "app_shared.h"
#include "sync_config.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "shift_register.h"
#include "rs485.h"
#include "can_bus.h"
#include "mux_data.h"
#include "root_wd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "node_proto";

/* extern globals from blink_example_main.c */
extern volatile uint32_t s_last_sync_ms;
extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool     s_comm_lost;

/* ============================================================
   STATUS periods
   ============================================================ */
#define NODE_STATUS_PERIOD_MS 5000
#define NODE_STATUS_FAULT_MS  1000
#define STATUS_FORCE_MS       5000

/* === NODE-ROOT-WATCHDOG: 10s === */
#define NODE_ROOT_LOST_MS     10000
/* === END NODE-ROOT-WATCHDOG === */

/* ============================================================
   Shared flags
   ============================================================ */
volatile bool s_fault_from_remote_ex = false;
volatile bool s_node_running = false;

static volatile bool     s_status_now = false;
static volatile uint32_t s_status_force_until_ms = 0;
static uint32_t s_last_node_pub_ms = 0;

static volatile uint32_t s_last_root_data_ms = 0;

/* === NODE-ROOT-WATCHDOG === */
static volatile bool s_root_lost_flag = false;
static volatile bool s_c_received_since_start = false;
/* === END NODE-ROOT-WATCHDOG === */

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

/* === SIMPLIFIED-NO-CFG: stubs === */
bool node_cfg_received(void) { return true; }
bool node_cfg_confirmed_by_root(void) { return true; }
/* === END SIMPLIFIED-NO-CFG === */

void node_send_status_now(void) { s_status_now = true; }

void node_send_status_for_ms(uint32_t ms)
{
    s_status_now = true;
    s_status_force_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + ms;
}

/* ============================================================
   node_note_root_data
   ============================================================ */
static inline void node_note_root_data(void)
{
    s_last_root_data_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

/* ============================================================
   on_cmd_from_root
   ============================================================ */
static void on_cmd_from_root(const char *cmd, size_t len)
{
    link_monitor_note_ping_rx();
    node_note_root_data();

    char buf[256];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n);
    buf[n] = '\0';

    /* ----------------------------------------------------------
       c:<phase>:<pm>:<on_hex>:<blink_hex>:<dur_ms>
       ---------------------------------------------------------- */
    if (buf[0] == 'c' && buf[1] == ':') {
        unsigned int ph = 0;
        unsigned long pm = 0;
        unsigned int on_mask = 0;
        unsigned int blink_mask = 0;
        unsigned long dur = 0;

        int matched = sscanf(buf + 2, "%u:%lu:%4x:%4x:%lu",
                             &ph, &pm, &on_mask, &blink_mask, &dur);
        if (matched != 5) {
            ESP_LOGW(TAG, "c: bad format: '%s'", buf);
            return;
        }

        if (!s_node_running) {
            ESP_LOGW(TAG, "c: ignored - NODE not RUNNING (ph=%u)", ph);
            return;
        }

        s_c_received_since_start = true;

        if (s_root_lost_flag) {
            ESP_LOGW(TAG, "c: ROOT back -> clear local YELLOW_BLINK");
            s_root_lost_flag = false;
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }

        uint16_t on16    = (uint16_t)on_mask;
        uint16_t blink16 = (uint16_t)blink_mask;

        ESP_LOGI(TAG, "c: ph=%u pm=%lu on=0x%04X blink=0x%04X dur=%lu",
                 ph, pm, on16, blink16, dur);

        traffic_light_apply_mask(on16, blink16, (uint8_t)ph,
                                 (uint32_t)pm, (uint32_t)dur);
        return;
    }

    /* ----------------------------------------------------------
       fault:*
       ---------------------------------------------------------- */
    if (strncmp(buf, "fault:none", 10) == 0) {
        ESP_LOGW(TAG, "fault:none from ROOT -> clear local fault");
        s_root_lost_flag = false;
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
        return;
    }
    if (strncmp(buf, "fault:all_off", 13) == 0) {
        ESP_LOGW(TAG, "fault:all_off from ROOT -> ALL_OFF");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        node_send_status_for_ms(2000);
        return;
    }
    if (strncmp(buf, "fault:yellow_blink", 18) == 0) {
        ESP_LOGW(TAG, "fault:yellow_blink from ROOT -> YELLOW_BLINK");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
        SET_REMOTE_FLAG(false);
        return;
    }

    /* ----------------------------------------------------------
       night mode
       ---------------------------------------------------------- */
    if (strcmp(buf, "traffic:night:on") == 0) {
        traffic_light_set_night_mode(true);
        node_send_status_for_ms(STATUS_FORCE_MS);
        return;
    }
    if (strcmp(buf, "traffic:night:off") == 0) {
        traffic_light_set_night_mode(false);
        node_send_status_for_ms(STATUS_FORCE_MS);
        return;
    }

    /* ----------------------------------------------------------
       manual / priority
       ---------------------------------------------------------- */
    {
        int mph = -1;
        if (sscanf(buf, "manual:override:phase:%d", &mph) == 1 &&
            mph >= 0 && mph < g_config.traffic.sequence_count) {
            traffic_light_manual_override((uint8_t)mph);
            return;
        }
    }
    if (strcmp(buf, "manual:override:off") == 0) {
        traffic_light_manual_release();
        return;
    }
    {
        unsigned int pms = 0;
        if (sscanf(buf, "priority:extend:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_extend(pms);
            return;
        }
        if (sscanf(buf, "priority:shorten:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_shorten(pms);
            return;
        }
    }

    /* ----------------------------------------------------------
       time:sec:ms:rtt
       ---------------------------------------------------------- */
    if (strncmp(buf, "time:", 5) == 0) {
        unsigned long sec = 0;
        unsigned int ms = 0;
        unsigned long rtt = 0;
        if (sscanf(buf + 5, "%lu:%u:%lu", &sec, &ms, &rtt) >= 2) {
            uint32_t local_sec = 0;
            uint16_t local_ms = 0;
            if (time_sync_get(&local_sec, &local_ms)) {
                int32_t delta = (int32_t)((int64_t)sec * 1000 + ms
                                          - ((int64_t)local_sec * 1000 + local_ms));
                time_sync_note_delta_ms(delta);
                ESP_LOGI(TAG, "time: root=%lu.%03u local=%lu.%03u delta=%ld ms",
                         sec, ms, (unsigned long)local_sec, local_ms,
                         (long)delta);
            } else {
                ESP_LOGI(TAG, "time: first set root=%lu.%03lu", sec, ms);
            }
            time_sync_set((uint32_t)sec, (uint16_t)ms, (uint32_t)rtt);
        }
        return;
    }

    if (strcmp(buf, "timeq") == 0) {
        uint32_t t_sec = 0;
        uint16_t t_ms = 0;
        if (time_sync_get(&t_sec, &t_ms)) {
            char evt[64];
            int en = snprintf(evt, sizeof(evt), "time:%lu:%u:%lu",
                              (unsigned long)t_sec, (unsigned)t_ms,
                              (unsigned long)(esp_timer_get_time() / 1000));
            if (en > 0) transport_send_evt_noack(evt, en);
        }
        return;
    }

    /* ----------------------------------------------------------
       ping / ping:ts
       ---------------------------------------------------------- */
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
                              ts, (unsigned long)t_recv,
                              (unsigned long)t_recv);
            if (pn > 0) transport_send_evt_noack(pong, pn);
        }
        return;
    }

    /* ----------------------------------------------------------
       delay:<us>
       ---------------------------------------------------------- */
    if (strncmp(buf, "delay:", 6) == 0) {
        unsigned int d = 0;
        if (sscanf(buf + 6, "%u", &d) == 1) {
            traffic_light_set_sync_delay_us(d);
        }
        return;
    }

    /* ----------------------------------------------------------
       pedestrian:request from ROOT
       ---------------------------------------------------------- */
    if (strcmp(buf, "pedestrian:request") == 0) {
        ESP_LOGD(TAG, "pedestrian:request from ROOT (follow_root - no-op)");
        return;
    }

    /* ----------------------------------------------------------
       Local commands
       ---------------------------------------------------------- */
    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        node_handle_local_cmd(buf);
        return;
    }

    /* ----------------------------------------------------------
       traffic:start / stop / pause / resume / reset / next / mode
       === FIX TRAFFIC-START-KEEP-C-TIMER ===
       ========================================================== */
    if (strcmp(buf, "traffic:start") == 0) {
        ESP_LOGW(TAG, "traffic:start from ROOT");

        bool was_running = s_node_running;

        if (!was_running) {
            /* Только при реальном старте сбрасываем таймер c: */
            s_c_received_since_start = false;
        } else {
            ESP_LOGI(TAG, "traffic:start: NODE already RUNNING → "
                          "keep c: timer (mask_received=%d)",
                     (int)s_c_received_since_start);
        }

        if (s_root_lost_flag) {
            s_root_lost_flag = false;
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }

        traffic_light_set_follow_root(true);
        if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            SET_REMOTE_FLAG(false);
        }
        traffic_light_start();
        s_node_running = true;
        node_note_root_data();
        node_send_status_for_ms(STATUS_FORCE_MS);
        ESP_LOGI(TAG, "traffic:start applied -> RUNNING, waiting c:");
        return;
    }
    if (strcmp(buf, "traffic:stop") == 0) {
        traffic_light_stop();
        s_node_running = false;
        s_root_lost_flag = false;
        s_c_received_since_start = false;
        return;
    }
    if (strcmp(buf, "traffic:pause")  == 0) { traffic_light_pause();  return; }
    if (strcmp(buf, "traffic:resume") == 0) { traffic_light_resume(); return; }
    if (strcmp(buf, "traffic:reset")  == 0) { traffic_light_reset();  return; }
    if (strcmp(buf, "traffic:next")   == 0) { traffic_light_next_phase(); return; }

    {
        int mode = 0;
        if (sscanf(buf, "traffic:mode:%d", &mode) == 1) {
            if (mode >= 1 && mode <= 3) {
                traffic_light_set_mode((traffic_mode_t)mode);
            }
            return;
        }
    }

    /* ----------------------------------------------------------
       Direct SR control
       ---------------------------------------------------------- */
    {
        int relay = -1, state = -1;
        if (sscanf(buf, "relay:%d:%d", &relay, &state) == 2 ||
            sscanf(buf, "sr:%d:%d",    &relay, &state) == 2) {
            if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
                if (state) sr_output_on(relay);
                else       sr_output_off(relay);
            }
            return;
        }
    }
    if (strcmp(buf, "all:on")  == 0) { sr_all_on();  return; }
    if (strcmp(buf, "all:off") == 0) { sr_all_off(); return; }
}

/* ============================================================
   node_handle_local_cmd - rs485:*, can:*
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
                char c1 = p[i * 2], c2 = p[i * 2 + 1];
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
    if (matched >= 2 ||
        (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
        uint8_t data[8] = {0};
        uint8_t dlc = 0;
        size_t hl = strlen(hex_data);
        if (hl > 0 && (hl % 2) == 0 && hl <= 16) {
            dlc = hl / 2;
            for (size_t i = 0; i < dlc; i++) {
                int hi = -1, lo = -1;
                char c1 = hex_data[i * 2], c2 = hex_data[i * 2 + 1];
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
   === FIX SYNC-BY-STATUS ===
   ============================================================ */
static void on_node_status_from_transport(const char *json, size_t len)
{
    link_monitor_note_status_rx();
    if (g_config.role == DEVICE_ROLE_MASTER) {
        wifi_mqtt_set_root_comm_lost(false);
    }

    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t now = s_last_node_msg_ms;

    bool should_publish = (s_last_node_pub_ms == 0 ||
                           (now - s_last_node_pub_ms) >= 2000);
    if (should_publish) {
        s_last_node_pub_ms = now;
        wifi_mqtt_publish_node_json(json, len);
    }

    if (g_config.role != DEVICE_ROLE_MASTER || !json || len == 0) return;

    if (memmem(json, len, "\"TRAFFIC\":", 10) == NULL) {
        ESP_LOGW(TAG, "STATUS incomplete (no TRAFFIC section) - "
                      "keep s_node_running=%d", (int)s_node_running);
        return;
    }

    bool running = (memmem(json, len, "\"state\":\"RUNNING\"", 17) != NULL);
    bool was_running = s_node_running;
    s_node_running = running;

    if (running && !was_running) {
        ESP_LOGI(TAG, "STATUS: NODE RUNNING confirmed");
    } else if (!running && was_running) {
        ESP_LOGW(TAG, "STATUS: NODE no longer RUNNING");
    }

    /* === FIX SYNC-BY-STATUS === */
    if (running) {
        const char *ph_p = memmem(json, len, "\"phase\":", 8);
        const char *pm_p = memmem(json, len, "\"phase_ms\":", 11);
        if (ph_p && pm_p) {
            int node_phase = -1;
            unsigned long node_pm = 0;
            if (sscanf(ph_p + 8, "%d", &node_phase) == 1 &&
                sscanf(pm_p + 11, "%lu", &node_pm) == 1 &&
                node_phase >= 0) {

                traffic_state_t rst = traffic_light_get_state();
                if (rst == TRAFFIC_STATE_RUNNING) {
                    uint8_t  root_phase = traffic_light_get_phase();
                    uint32_t root_pm    = traffic_light_get_phase_ms();

                    int pm_diff = (int)node_pm - (int)root_pm;
                    if (pm_diff < 0) pm_diff = -pm_diff;

                    bool phase_mismatch = ((uint8_t)node_phase != root_phase);
                    bool pm_mismatch    = (pm_diff > 500);

                    if (phase_mismatch || pm_mismatch) {
                        ESP_LOGW(TAG, "STATUS mismatch: node ph=%d pm=%lu, "
                                      "root ph=%u pm=%lu → force c:",
                                 node_phase, node_pm, root_phase, root_pm);
                        send_sync_to_node_async_ex(
                            root_phase, root_pm,
                            traffic_light_get_phase_duration_ms(),
                            true, NULL);
                    }
                }
            }
        }
    }
    /* === END FIX SYNC-BY-STATUS === */

    const char *fp = memmem(json, len, "\"FAULT\":{", 9);
    if (fp) {
        const char *mp = memmem(fp, (json + len) - fp, "\"mode\":\"", 8);
        if (mp) {
            mp += 8;
            char mode[16] = {0};
            size_t k = 0;
            while (k < sizeof(mode) - 1 && (mp + k) < (json + len) &&
                   mp[k] != '"') {
                mode[k] = mp[k];
                k++;
            }
            mode[k] = '\0';

            if (strcmp(mode, "YELLOW_BLINK") == 0) {
                wifi_mqtt_apply_remote_fault("fault:yellow_blink:node", 24);
            } else if (strcmp(mode, "ALL_OFF") == 0) {
                wifi_mqtt_apply_remote_fault("fault:all_off:node", 19);
            }
        }
    }
}

/* ============================================================
   on_evt_from_node (ROOT)
   ============================================================ */
static void on_evt_from_node(const char *evt, size_t len)
{
    char buf[128];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, evt, n);
    buf[n] = '\0';

    if (len > 5 && strncmp(buf, "pong:", 5) == 0) return;
    if (len > 5 && strncmp(buf, "time:", 5) == 0) return;

    if (len > 6 && strncmp(buf, "fault:", 6) == 0) {
        SET_REMOTE_FLAG(true);
        wifi_mqtt_apply_remote_fault(evt, len);
        SET_REMOTE_FLAG(false);
        return;
    }

    if (strncmp(buf, "fault:recover_ok", 16) == 0) {
        if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
            ESP_LOGW(TAG, "NODE recover_ok but ROOT still in fault - ignore");
            return;
        }
        ESP_LOGW(TAG, "NODE recover_ok, ROOT ok -> send fault:none");
        transport_send_cmd_noack("fault:none", 10);
        return;
    }

    if (strcmp(buf, "pedestrian:request") == 0) {
        if (!traffic_light_ped_button_enabled()) return;
        ESP_LOGW(TAG, "pedestrian:request from NODE -> apply locally on ROOT");
        traffic_light_request_pedestrian_ex(true);
        return;
    }

    if (strncmp(buf, "status:autonomous", 17) == 0) return;
}

/* ============================================================
   node_send_task
   ============================================================ */
static void node_send_task(void *arg)
{
    char *json = malloc(512);
    if (!json) { vTaskDelete(NULL); return; }

    ESP_LOGI(TAG, "node_send_task started (transport=%s)",
             transport_is_lora() ? "LoRa" : "TCP");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        if (!transport_is_tx_idle()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint16_t sr = sr_get_state();
        traffic_fault_t cur_fault = traffic_light_get_fault();
        const char *state_str = traffic_light_state_str();
        const char *fault_str = traffic_light_fault_str();

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
        if (s_status_force_until_ms != 0 &&
            now_ms >= s_status_force_until_ms) {
            s_status_force_until_ms = 0;
        }

        if (force) {
            esp_err_t err = transport_send_status_noack(json, off);
            ESP_LOGI(TAG, "STATUS forced -> ROOT: %s",
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
        } else {
            esp_err_t err = transport_send_status(json, off);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "STATUS failed: %s", esp_err_to_name(err));
            }
        }

        uint32_t period;
        if (force) {
            period = 200;
        } else if (cur_fault != TRAFFIC_FAULT_NONE) {
            period = NODE_STATUS_FAULT_MS;
        } else {
            period = NODE_STATUS_PERIOD_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

/* ============================================================
   node_link_watchdog_task
   === FIX NO-FIRST-C-WD ===
   ============================================================ */
static void node_link_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "node_link_watchdog_task started (timeout=%d ms)",
             NODE_ROOT_LOST_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_config.role != DEVICE_ROLE_SLAVE) continue;

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (!s_node_running) continue;

        /* === FIX NO-FIRST-C-WD === */
        if (!s_c_received_since_start) {
            if (s_last_root_data_ms != 0 &&
                (now - s_last_root_data_ms) >= (uint32_t)NODE_ROOT_LOST_MS &&
                !s_root_lost_flag) {
                ESP_LOGE(TAG, "NODE RUNNING but no c: for %lu ms "
                              "-> YELLOW_BLINK (fail-safe)",
                         (unsigned long)(now - s_last_root_data_ms));
                s_root_lost_flag = true;
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
                SET_REMOTE_FLAG(false);
            }
            continue;
        }
        /* === END FIX NO-FIRST-C-WD === */

        if (s_last_root_data_ms == 0) continue;

        uint32_t silence = now - s_last_root_data_ms;

        if (silence >= NODE_ROOT_LOST_MS && !s_root_lost_flag) {
            ESP_LOGE(TAG, "ROOT silent %lu ms -> YELLOW_BLINK (fail-safe)",
                     (unsigned long)silence);
            s_root_lost_flag = true;
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
            SET_REMOTE_FLAG(false);
        }
    }
}

/* ============================================================
   Register callbacks / start tasks
   ============================================================ */
void node_proto_register_callbacks(void)
{
    if (g_config.role == DEVICE_ROLE_SLAVE) {
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
    xTaskCreate(node_send_task,           "node_send", 4096, NULL, 4, NULL);
    xTaskCreate(node_link_watchdog_task,  "node_wd",   3072, NULL, 3, NULL);
}

void node_proto_start_root_tasks(void)
{
    ESP_LOGI(TAG, "node_proto_start_root_tasks: no-op "
             "(heartbeat lives in root_wd.c)");
}