/* ============================================================
   wifi_ap.h — ЗАГЛУШКИ (Wi-Fi отключён).
   Все функции возвращают «нет Wi-Fi». Нужны, чтобы
   mqtt_json.c / wifi_mqtt.c / buttons.c компилировались.
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

/* Wi-Fi инициализация — no-op */
static inline void wifi_init(void) { }

/* STA — всегда «нет» */
static inline esp_err_t wifi_start(void) { return ESP_OK; }
static inline void      wifi_stop(void)  { }
static inline bool      wifi_sta_connected(void)  { return false; }
static inline bool      wifi_sta_has_ip(void)     { return false; }
static inline const char* wifi_get_ip(void)       { return "0.0.0.0"; }
static inline bool      wifi_sta_connecting(void) { return false; }

/* AP — всегда «нет» */
static inline esp_err_t wifi_ap_start(const char *ssid, const char *password)
    { (void)ssid; (void)password; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t wifi_ap_start_only(const char *ssid, const char *password)
    { (void)ssid; (void)password; return ESP_ERR_NOT_SUPPORTED; }
static inline void      wifi_ap_stop(void)        { }
static inline bool      wifi_ap_is_running(void)  { return false; }
static inline bool      wifi_ap_running(void)     { return false; }
static inline const char* wifi_get_ap_ssid(void)  { return ""; }
static inline const char* wifi_get_ap_ip(void)    { return "0.0.0.0"; }
static inline int       wifi_ap_client_count(void){ return 0; }

/* STA fallback — no-op */
static inline esp_err_t wifi_sta_start_after_ppp_fail(void) { return ESP_OK; }

/* Scan — не поддерживается */
static inline esp_err_t wifi_scan_for_fsu_ap(char *out_ssid, size_t out_len,
                                             int8_t *out_rssi, uint8_t *out_channel,
                                             const char *ssid_prefix)
    { (void)out_ssid; (void)out_len; (void)out_rssi;
      (void)out_channel; (void)ssid_prefix; return ESP_ERR_NOT_SUPPORTED; }

/* Suspend/resume — no-op */
static inline esp_err_t wifi_ap_suspend_for_parse(void)  { return ESP_OK; }
static inline esp_err_t wifi_ap_resume_after_parse(void) { return ESP_OK; }

#ifdef __cplusplus
}
#endif
#endif /* WIFI_AP_H */