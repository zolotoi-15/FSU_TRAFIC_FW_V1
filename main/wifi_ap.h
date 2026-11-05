/* ============================================================
   wifi_ap.h — РЕАЛЬНЫЕ прототипы (не заглушки).

   === FIX #6: убраны static inline заглушки, дублировавшие
   wifi_ap.c. Теперь единственная реализация — в wifi_ap.c. ===
   ============================================================ */

#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STA */
void        wifi_init(void);
esp_err_t   wifi_start(void);
void        wifi_stop(void);
bool        wifi_sta_connected(void);
bool        wifi_sta_has_ip(void);
const char *wifi_get_ip(void);
bool        wifi_sta_connecting(void);

/* AP */
esp_err_t   wifi_ap_start(const char *ssid, const char *password);
esp_err_t   wifi_ap_start_only(const char *ssid, const char *password);
void        wifi_ap_stop(void);
bool        wifi_ap_is_running(void);
bool        wifi_ap_running(void);
const char *wifi_get_ap_ssid(void);
const char *wifi_get_ap_ip(void);
int         wifi_ap_client_count(void);

/* STA fallback */
esp_err_t   wifi_sta_start_after_ppp_fail(void);

/* Scan */
esp_err_t   wifi_scan_for_fsu_ap(char *out_ssid, size_t out_len,
                                 int8_t *out_rssi, uint8_t *out_channel,
                                 const char *ssid_prefix);

/* Suspend/resume AP для освобождения heap */
esp_err_t   wifi_ap_suspend_for_parse(void);
esp_err_t   wifi_ap_resume_after_parse(void);

#ifdef __cplusplus
}
#endif
#endif /* WIFI_AP_H */