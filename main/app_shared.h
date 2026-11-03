#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "board_pins.h"
#include "traffic_light.h"
#include "pi4ioe5v6408.h"
#include "wifi_ap.h"          /* === NEW: сюда переехали заглушки Wi-Fi === */

#ifdef __cplusplus
extern "C" {
#endif

extern pi4ioe5v6408_config_t expander_a;
extern pi4ioe5v6408_config_t expander_b;

extern volatile bool s_fault_from_remote_ex;

int32_t  blink_get_rtt_samples(void);
uint32_t blink_get_rtt_us(void);
uint32_t blink_get_rtt_delay_us(void);
void     blink_reset_rtt(void);

extern volatile uint32_t s_last_sync_ms;
extern volatile bool     s_comm_lost;
extern volatile uint32_t s_last_pong_ms;

esp_err_t board_diag_en_set(uint8_t channel, bool enable);
void      board_diag_en_all_off(void);
int       board_read_iload_raw(uint8_t channel,
                               imon_state_t *out_state,
                               int *out_mv);

/* === УБРАНО: wifi_get_ip / wifi_sta_connected / wifi_sta_has_ip /
   wifi_ap_client_count / wifi_scan_for_fsu_ap — теперь в wifi_ap.h
   как static inline заглушки. === */

#ifdef __cplusplus
}
#endif