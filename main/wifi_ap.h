// wifi_ap.h
#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   STA (к роутеру / к AP root)
   ============================================================ */
esp_err_t wifi_start(void);
void      wifi_stop(void);

/* ============================================================
   AP (SoftAP для node)
   ============================================================ */
esp_err_t   wifi_ap_start(const char *ssid, const char *password);
void        wifi_ap_stop(void);
bool        wifi_ap_is_running(void);
const char* wifi_get_ap_ip(void);
int         wifi_ap_client_count(void);

/* ============================================================
   STA status
   ============================================================ */
bool        wifi_sta_connected(void);
bool        wifi_sta_has_ip(void);
const char* wifi_get_ip(void);

/* FIX: STA в процессе подключения (между esp_wifi_connect()
   и STA_CONNECTED/DISCONNECTED). Позволяет node_ap_retry_task
   не вмешиваться в 4-way handshake. */
bool        wifi_sta_connecting(void);

/* ============================================================
   AP status
   ============================================================ */
bool        wifi_ap_running(void);
const char* wifi_get_ap_ssid(void);

/* ============================================================
   Scan: найти AP с заданным префиксом (самый сильный).
   === Сканирует ВСЕ каналы (esp_wifi_scan_start(NULL, true)). ===
   ============================================================ */
esp_err_t wifi_scan_for_fsu_ap(char *out_ssid, size_t out_len,
                               int8_t *out_rssi,
                               const char *ssid_prefix);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_AP_H */