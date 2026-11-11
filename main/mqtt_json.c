#include "mqtt_internal.h"
#include "config_manager.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "mux.h"
#include "mux_data.h"
#include "rs485.h"
#include "can_bus.h"
#include "modem_ppp.h"
/* === FIX LORA-LINK-STATS === */
#include "lora.h"
#include "node_proto.h"
/* === END FIX LORA-LINK-STATS === */
/* === FIX BLE-NAME-CLASH === */
#include "ble_transport.h"
/* === END FIX BLE-NAME-CLASH === */
#include "esp_timer.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "mqtt_json";
extern char s_node_fault_json[1024];

extern int32_t  blink_get_rtt_samples(void);
extern uint32_t blink_get_rtt_us(void);
extern uint32_t blink_get_rtt_delay_us(void);

#define APPEND(...) do { \
    if ((size_t)off >= size) return (int)off; \
    int _n = snprintf(json + off, size - off, __VA_ARGS__); \
    if (_n < 0) return (int)off; \
    off += _n; \
} while (0)

int mqtt_build_json(char *json, size_t size) {
    if (!json || size < 64) return 0;
    int off = 0;
    uint16_t sr_state = sr_get_state();

    APPEND("{\"SR\":{\"state\":%u,\"hex\":\"0x%04X\",\"outputs\":[",
           sr_state, sr_state);
    bool first = true;
    for (int i = 0; i < SR_OUTPUT_COUNT; i++) {
        if (sr_state & (1U << i)) {
            APPEND("%s%d", first ? "" : ",", i);
            first = false;
        }
    }
    APPEND("]},");

    const char *ip = modem_ppp_is_connected() ? modem_ppp_get_ip() : "0.0.0.0";
    APPEND("\"NET\":{\"role\":\"%s\",\"ppp\":%d,\"ip\":\"%s\"},",
           g_config.role == DEVICE_ROLE_SLAVE ? "node" : "root",
           modem_ppp_is_connected() ? 1 : 0, ip);

    APPEND("\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
           "\"phase_ms\":%lu,\"phase_duration_ms\":%lu,\"phase_count\":%u},",
           traffic_light_state_str(), traffic_light_get_mode(),
           traffic_light_get_phase(),
           (unsigned long)traffic_light_get_phase_ms(),
           (unsigned long)traffic_light_get_phase_duration_ms(),
           traffic_light_get_phase_count());

    const char *fault_str = traffic_light_fault_str();
    uint8_t fault_cnt = traffic_light_get_fault_count();
    APPEND("\"FAULT\":{\"mode\":\"%s\",\"count\":%u,\"channels\":[",
           fault_str, fault_cnt);
    for (uint8_t i = 0; i < fault_cnt && i < TRAFFIC_FAULT_MAX; i++) {
        traffic_fault_info_t info;
        traffic_light_get_fault_info(i, &info);
        APPEND("%s%u", (i == 0) ? "" : ",", info.channel);
    }
    APPEND("],\"reasons\":[");
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
        APPEND("%s\"%s\"", (i == 0) ? "" : ",", safe);
    }
    APPEND("]},");

    /* === FIX LORA-LINK-STATS === */
    {
        lora_link_stats_t root_ls = {0};
        bool have_root = lora_get_last_rx_stats(&root_ls);

        APPEND("\"LINK\":{");
        APPEND("\"root\":{\"rssi\":%.1f,\"snr\":%.1f,\"sig_rssi\":%.1f,"
               "\"age_ms\":%lu,\"valid\":%d},",
               have_root ? root_ls.rssi : 0.0f,
               have_root ? root_ls.snr : 0.0f,
               have_root ? root_ls.sig_rssi : 0.0f,
               (unsigned long)(have_root ? root_ls.age_ms : 0),
               have_root ? 1 : 0);
        APPEND("\"node\":{\"rssi\":%.1f,\"snr\":%.1f,\"sig_rssi\":%.1f,"
               "\"age_ms\":%lu,\"valid\":%d}",
               s_node_link_valid ? s_node_link_rssi : 0.0f,
               s_node_link_valid ? s_node_link_snr : 0.0f,
               s_node_link_valid ? s_node_link_sig_rssi : 0.0f,
               (unsigned long)(s_node_link_valid ? s_node_link_age_ms : 0),
               s_node_link_valid ? 1 : 0);
        APPEND("},");
    }
    /* === END FIX LORA-LINK-STATS === */

    APPEND("\"NODE\":null,");
    if (s_node_fault_json[0]) APPEND("\"NODE_FAULT\":%s,", s_node_fault_json);
    else                      APPEND("\"NODE_FAULT\":null,");

    rs485_stats_t rs;  rs485_get_stats(&rs);
    can_bus_stats_t cs; can_bus_get_stats(&cs);

    APPEND("\"RS485\":{\"rx_bytes\":%lu,\"rx_frames\":%lu,\"rx_overflows\":%lu,"
           "\"tx_bytes\":%lu,\"tx_errors\":%lu},",
           (unsigned long)rs.rx_bytes, (unsigned long)rs.rx_frames,
           (unsigned long)rs.rx_overflows, (unsigned long)rs.tx_bytes,
           (unsigned long)rs.tx_errors);

    APPEND("\"CAN\":{\"rx_frames\":%lu,\"tx_frames\":%lu,\"tx_errors\":%lu,"
           "\"bus_off\":%lu,\"recovery\":%lu,\"error_passive\":%lu,\"state\":%lu},",
           (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
           (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
           (unsigned long)cs.recovery_count,
           (unsigned long)cs.error_passive_count,
           (unsigned long)cs.last_state);

    APPEND("\"RTT\":{\"us\":%lu,\"delay_us\":%lu,\"samples\":%ld},",
           (unsigned long)blink_get_rtt_us(),
           (unsigned long)blink_get_rtt_delay_us(),
           (long)blink_get_rtt_samples());

    mux_data_t data;
    bool have = mux_data_get(&data);
    for (uint8_t chx = 0; chx < 16; chx++) {
        int diag = (have && data.diag_en[chx] != 0xFF) ? data.diag_en[chx] : -1;
        float v_in = have ? data.v_in[chx] : 0.0f;
        int i_load = have ? data.i_load[chx] : 0;
        int imon_mv = have ? (int)data.imon_mv[chx] : 0;
        const char *state = have
            ? imon_state_str((imon_state_t)data.imon_state[chx]) : "UNKNOWN";
        int sr = have ? data.sr_state[chx] : 0;
        APPEND("\"ch%02d\":{\"V_IN\":%.2f,\"I_LOAD\":%d,\"V_IMON\":%d,"
               "\"STATE\":\"%s\",\"SR\":%d,\"DIAG\":%d}%s",
               chx, v_in, i_load, imon_mv, state, sr, diag,
               (chx < 15) ? "," : "");
    }
    APPEND("}");

    /* === FIX BLE-NAME-CLASH ===
       Если телефон подключён — дублируем JSON в STATUS (Notify).
       Публикуется из mqtt_publish_task раз в MQTT_PUBLISH_PERIOD_MS. */
    if (fsu_ble_is_connected()) {
        fsu_ble_notify_status(json, strlen(json));
    }
    /* === END FIX BLE-NAME-CLASH === */

    (void)TAG;

    return off;
}