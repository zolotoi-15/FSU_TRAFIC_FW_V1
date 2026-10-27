/* ============================================================
   mqtt_json.c — build_json: JSON-телеметрия root.
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "wifi_ap.h"
#include "shift_register.h"
#include "traffic_light.h"
#include "mux.h"
#include "mux_data.h"
#include "rs485.h"
#include "can_bus.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "mqtt_json";

/* s_node_fault_json — из mqtt_fault.c */
extern char s_node_fault_json[1024];

extern int32_t  blink_get_rtt_samples(void);
extern uint32_t blink_get_rtt_us(void);
extern uint32_t blink_get_rtt_delay_us(void);

int mqtt_build_json(char *json, size_t size)
{
    int off = 0;
    uint16_t sr_state = sr_get_state();

    off += snprintf(json + off, size - off,
                    "{\"SR\":{\"state\":%u,\"hex\":\"0x%04X\",\"outputs\":[",
                    sr_state, sr_state);
    bool first = true;
    for (int i = 0; i < SR_OUTPUT_COUNT; i++) {
        if (sr_state & (1U << i)) {
            off += snprintf(json + off, size - off, "%s%d", first ? "" : ",", i);
            first = false;
        }
    }
    off += snprintf(json + off, size - off, "]},");

    wifi_ap_record_t ap;
    int     rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
    uint8_t ch   = 0;
    wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&ch, &sec);

    off += snprintf(json + off, size - off,
        "\"NET\":{\"role\":\"%s\",\"sta_connected\":%d,\"has_ip\":%d,"
        "\"rssi\":%d,\"channel\":%u,\"ip\":\"%s\","
        "\"ap_running\":%d,\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\",\"ap_clients\":%d},",
        g_config.role == DEVICE_ROLE_SLAVE ? "node" : "root",
        wifi_sta_connected() ? 1 : 0,
        wifi_sta_has_ip()    ? 1 : 0,
        rssi, ch, wifi_get_ip(),
        wifi_ap_is_running() ? 1 : 0,
        wifi_get_ap_ssid(), wifi_get_ap_ip(),
        wifi_ap_client_count());

    off += snprintf(json + off, size - off,
        "\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
        "\"phase_ms\":%lu,\"phase_duration_ms\":%lu,\"phase_count\":%u},",
        traffic_light_state_str(),
        traffic_light_get_mode(),
        traffic_light_get_phase(),
        (unsigned long)traffic_light_get_phase_ms(),
        (unsigned long)traffic_light_get_phase_duration_ms(),
        traffic_light_get_phase_count());

    const char *fault_str = traffic_light_fault_str();
    uint8_t     fault_cnt = traffic_light_get_fault_count();
    off += snprintf(json + off, size - off,
        "\"FAULT\":{\"mode\":\"%s\",\"count\":%u,\"channels\":[",
        fault_str, fault_cnt);
    for (uint8_t i = 0; i < fault_cnt && i < TRAFFIC_FAULT_MAX; i++) {
        traffic_fault_info_t info;
        traffic_light_get_fault_info(i, &info);
        off += snprintf(json + off, size - off, "%s%u",
                        (i == 0) ? "" : ",", info.channel);
    }
    off += snprintf(json + off, size - off, "],\"reasons\":[");
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
        off += snprintf(json + off, size - off, "%s\"%s\"",
                        (i == 0) ? "" : ",", safe);
    }
    off += snprintf(json + off, size - off, "]},");

    off += snprintf(json + off, size - off, "\"NODE\":null,");

    if (s_node_fault_json[0]) {
        off += snprintf(json + off, size - off,
                        "\"NODE_FAULT\":%s,", s_node_fault_json);
    } else {
        off += snprintf(json + off, size - off, "\"NODE_FAULT\":null,");
    }

    rs485_stats_t rs;
    rs485_get_stats(&rs);
    can_bus_stats_t cs;
    can_bus_get_stats(&cs);

    off += snprintf(json + off, size - off,
        "\"RS485\":{\"rx_bytes\":%lu,\"rx_frames\":%lu,\"rx_overflows\":%lu,"
        "\"tx_bytes\":%lu,\"tx_errors\":%lu},",
        (unsigned long)rs.rx_bytes,
        (unsigned long)rs.rx_frames,
        (unsigned long)rs.rx_overflows,
        (unsigned long)rs.tx_bytes,
        (unsigned long)rs.tx_errors);

    off += snprintf(json + off, size - off,
        "\"CAN\":{\"rx_frames\":%lu,\"tx_frames\":%lu,\"tx_errors\":%lu,"
        "\"bus_off\":%lu,\"recovery\":%lu,\"error_passive\":%lu,\"state\":%lu},",
        (unsigned long)cs.rx_frames,
        (unsigned long)cs.tx_frames,
        (unsigned long)cs.tx_errors,
        (unsigned long)cs.bus_off_count,
        (unsigned long)cs.recovery_count,
        (unsigned long)cs.error_passive_count,
        (unsigned long)cs.last_state);

    off += snprintf(json + off, size - off,
        "\"RTT\":{\"us\":%lu,\"delay_us\":%lu,\"samples\":%ld},",
        (unsigned long)blink_get_rtt_us(),
        (unsigned long)blink_get_rtt_delay_us(),
        (long)blink_get_rtt_samples());

    mux_data_t data;
    if (mux_data_get(&data) && data.valid) {
        for (uint8_t chx = 0; chx < 16; chx++) {
            int diag = (data.diag_en[chx] == 0xFF) ? -1 : data.diag_en[chx];
            off += snprintf(json + off, size - off,
                "\"ch%02d\":{\"V_IN\":%.2f,\"I_LOAD\":%d,\"V_IMON\":%d,"
                "\"STATE\":\"%s\",\"SR\":%d,\"DIAG\":%d}%s",
                chx, data.v_in[chx], data.i_load[chx], (int)data.imon_mv[chx],
                imon_state_str((imon_state_t)data.imon_state[chx]),
                data.sr_state[chx], diag, (chx < 15) ? "," : "");
        }
    } else {
        for (uint8_t chx = 0; chx < 16; chx++) {
            off += snprintf(json + off, size - off,
                "\"ch%02d\":{\"V_IN\":0,\"I_LOAD\":0,\"V_IMON\":0,"
                "\"STATE\":\"UNKNOWN\",\"SR\":0,\"DIAG\":-1}%s",
                chx, (chx < 15) ? "," : "");
        }
    }
    off += snprintf(json + off, size - off, "}");
    return off;
}