#include "wifi_ap.h"
void wifi_init(void) {}
esp_err_t wifi_start(void) { return ESP_OK; }
void wifi_stop(void) {}
bool wifi_sta_connected(void) { return false; }
bool wifi_sta_has_ip(void) { return false; }
const char *wifi_get_ip(void) { return "0.0.0.0"; }
bool wifi_sta_connecting(void) { return false; }
esp_err_t wifi_ap_start(const char *s, const char *p) { (void)s;(void)p; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t wifi_ap_start_only(const char *s, const char *p) { (void)s;(void)p; return ESP_ERR_NOT_SUPPORTED; }
void wifi_ap_stop(void) {}
bool wifi_ap_is_running(void) { return false; }
bool wifi_ap_running(void) { return false; }
const char *wifi_get_ap_ssid(void) { return ""; }
const char *wifi_get_ap_ip(void) { return "0.0.0.0"; }
int wifi_ap_client_count(void) { return 0; }
esp_err_t wifi_sta_start_after_ppp_fail(void) { return ESP_OK; }
esp_err_t wifi_scan_for_fsu_ap(char *o, size_t l, int8_t *r, uint8_t *c, const char *p)
    { (void)o;(void)l;(void)r;(void)c;(void)p; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t wifi_ap_suspend_for_parse(void) { return ESP_OK; }
esp_err_t wifi_ap_resume_after_parse(void) { return ESP_OK; }