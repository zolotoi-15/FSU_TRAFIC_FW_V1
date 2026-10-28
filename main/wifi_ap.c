// wifi_ap.c
#include "wifi_ap.h"
#include "config_manager.h"
#include "modem_ppp.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"   // для wifi_sta_list_t (IDF v6.1+)
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "WIFI";

#define WIFI_STA_TX_POWER_VAL  34
#define WIFI_AP_TX_POWER_VAL   34
#define AP_MAX_STA_CONN        2

/* ============================================================
   STA state
   ============================================================ */
static bool s_sta_connected = false;
static volatile bool s_sta_connecting = false;
static char s_ip[16]        = "0.0.0.0";

static esp_timer_handle_t s_reconnect_timer = NULL;
static bool s_has_saved_ssid = false;
static int  s_retry_delay_sec = 5;

static TaskHandle_t s_monitor_task = NULL;
static volatile int64_t  s_last_connected_us = 0;
static volatile uint32_t s_disconnect_count   = 0;
static volatile int64_t  s_sta_down_since_us  = 0;

static portMUX_TYPE s_wifi_mtx = portMUX_INITIALIZER_UNLOCKED;

static inline void wifi_set_last_connected(int64_t t) {
    portENTER_CRITICAL(&s_wifi_mtx); s_last_connected_us = t; portEXIT_CRITICAL(&s_wifi_mtx);
}
static inline int64_t wifi_get_last_connected(void) {
    portENTER_CRITICAL(&s_wifi_mtx); int64_t t = s_last_connected_us; portEXIT_CRITICAL(&s_wifi_mtx); return t;
}
static inline void wifi_set_down_since(int64_t t) {
    portENTER_CRITICAL(&s_wifi_mtx); s_sta_down_since_us = t; portEXIT_CRITICAL(&s_wifi_mtx);
}
static inline int64_t wifi_get_down_since(void) {
    portENTER_CRITICAL(&s_wifi_mtx); int64_t t = s_sta_down_since_us; portEXIT_CRITICAL(&s_wifi_mtx); return t;
}
static inline void wifi_inc_disconnect(void) {
    portENTER_CRITICAL(&s_wifi_mtx); s_disconnect_count++; portEXIT_CRITICAL(&s_wifi_mtx);
}

static EventGroupHandle_t s_wifi_evt = NULL;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_GOT_IP_BIT     BIT1

/* ============================================================
   AP state
   ============================================================ */
static bool   s_ap_is_running = false;
static char   s_ap_ssid_cur[33] = {0};
static char   s_ap_ip_str[16]   = "192.168.4.1";
static esp_netif_t *s_ap_netif  = NULL;



static const char* reason_str(uint8_t r) {
    switch (r) {
        case WIFI_REASON_NO_AP_FOUND:            return "NO_AP_FOUND";
        case WIFI_REASON_AUTH_FAIL:              return "AUTH_FAIL";
        case WIFI_REASON_ASSOC_FAIL:             return "ASSOC_FAIL";
        case WIFI_REASON_HANDSHAKE_TIMEOUT:      return "HANDSHAKE_TIMEOUT";
        case WIFI_REASON_CONNECTION_FAIL:        return "CONNECTION_FAIL";
        case WIFI_REASON_BEACON_TIMEOUT:         return "BEACON_TIMEOUT";
        case WIFI_REASON_AUTH_EXPIRE:            return "AUTH_EXPIRE";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4WAY_HANDSHAKE_TIMEOUT";
        case WIFI_REASON_MIC_FAILURE:            return "MIC_FAILURE";
        case WIFI_REASON_802_1X_AUTH_FAILED:     return "802_1X_AUTH_FAILED";
        case WIFI_REASON_ASSOC_LEAVE:            return "ASSOC_LEAVE";
        default:                                 return "OTHER";
    }
}

static esp_err_t wifi_connect_start(void)
{
    s_sta_connecting = true;
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN &&
        err != ESP_ERR_WIFI_STATE) {
        ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(err));
    }
    return err;
}

static void reconnect_timer_cb(void *arg) {
    if (!s_sta_connected && !s_sta_connecting && s_has_saved_ssid) {
        ESP_LOGI(TAG, "Attempting STA reconnect...");
        wifi_connect_start();
    }
}

static void ensure_reconnect_timer(void) {
    if (s_reconnect_timer) return;
    esp_timer_create_args_t args = { .callback = reconnect_timer_cb, .name = "wifi_retry" };
    esp_timer_create(&args, &s_reconnect_timer);
}

/* ============================================================
   Event handler
   ============================================================ */
static void evt_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_has_saved_ssid) {
            s_sta_connecting = true;
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "STA idle — SSID не задан");
        }
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        s_sta_connected = true;
        s_sta_connecting = false;
        wifi_set_down_since(0);
        s_retry_delay_sec = 5;
        wifi_set_last_connected(esp_timer_get_time());
        ESP_LOGI(TAG, "STA connected");
        if (s_wifi_evt) xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        s_sta_connected = false;
        s_sta_connecting = false;
        wifi_inc_disconnect();
        strcpy(s_ip, "0.0.0.0");
        if (wifi_get_down_since() == 0) wifi_set_down_since(esp_timer_get_time());
        ESP_LOGW(TAG, "STA disconnected (reason=%u %s, rssi=%d)",
                 evt->reason, reason_str(evt->reason), evt->rssi);
        if (s_wifi_evt) xEventGroupClearBits(s_wifi_evt, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);

        if (s_has_saved_ssid) {
            ensure_reconnect_timer();
            esp_timer_stop(s_reconnect_timer);
            esp_timer_start_once(s_reconnect_timer, (uint64_t)s_retry_delay_sec * 1000000ULL);
            if (s_retry_delay_sec < 60) s_retry_delay_sec *= 2;
            if (s_retry_delay_sec > 60) s_retry_delay_sec = 60;
        }
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&evt->ip_info.ip));
        s_retry_delay_sec = 5;
        wifi_set_last_connected(esp_timer_get_time());
        wifi_set_down_since(0);
        ESP_LOGI(TAG, "Got IP %s", s_ip);
        if (s_wifi_evt) xEventGroupSetBits(s_wifi_evt, WIFI_GOT_IP_BIT);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        s_ap_is_running = true;
        uint8_t ch = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&ch, &sec);
        ESP_LOGI(TAG, "AP started: SSID='%s' IP=%s ch=%u",
                 s_ap_ssid_cur, s_ap_ip_str, ch);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STOP) {
        s_ap_is_running = false;
        ESP_LOGD(TAG, "AP stopped (mode change)");
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "AP client connected: " MACSTR " aid=%d",
                 MAC2STR(e->mac), e->aid);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGW(TAG, "AP client disconnected: " MACSTR " aid=%d",
                 MAC2STR(e->mac), e->aid);
    }
}

/* ============================================================
   Monitor task (STA watchdog)
   ============================================================ */
#define WIFI_MONITOR_PERIOD_MS     20000
#define WIFI_STUCK_TIMEOUT_US      (180LL * 1000000LL)
#define WIFI_MIN_RETRY_INTERVAL_US (15LL * 1000000LL)
#define WIFI_REBOOT_AFTER_DOWN_US  (15LL * 60 * 1000000LL)

static int64_t s_last_forced_reconnect_us = 0;

static void wifi_monitor_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    ESP_LOGI(TAG, "WiFi monitor task started");
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WIFI_MONITOR_PERIOD_MS));

        int64_t now = esp_timer_get_time();

        if (s_has_saved_ssid) {
            wifi_ap_record_t ap_info;
            bool sta_really_connected = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);

            if (!s_sta_connected && !sta_really_connected && !s_sta_connecting) {
                if (now - s_last_forced_reconnect_us > WIFI_MIN_RETRY_INTERVAL_US) {
                    ESP_LOGW(TAG, "WiFi monitor: STA down, forcing connect");
                    s_last_forced_reconnect_us = now;
                    wifi_connect_start();
                }
            } else if (sta_really_connected && s_sta_connected) {
                wifi_set_last_connected(now);
                wifi_set_down_since(0);
            }
        }

        int64_t last_conn  = wifi_get_last_connected();
        int64_t down_since = wifi_get_down_since();

        if (s_has_saved_ssid && !s_sta_connected &&
            last_conn > 0 && (now - last_conn) > WIFI_STUCK_TIMEOUT_US) {
            ESP_LOGE(TAG, "WiFi monitor: STA stuck >3min, hard reconnect");
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(200));
            wifi_connect_start();
            wifi_set_last_connected(now);
        }

        if (s_has_saved_ssid && !s_sta_connected && down_since > 0) {
            int64_t down_for = now - down_since;
            if (down_for > WIFI_REBOOT_AFTER_DOWN_US) {
                ESP_LOGE(TAG, "WiFi down for %lld s -> REBOOT",
                         (long long)(down_for / 1000000LL));
                esp_restart();
            }
        }
    }
}

/* ============================================================
   wifi_init — однократная инициализация.
   ============================================================ */
void wifi_init(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(err));
    }
    esp_event_loop_create_default();

    if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
        esp_netif_create_default_wifi_sta();
    }

    if (!s_wifi_evt) s_wifi_evt = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
    }

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
   // esp_wifi_restore();

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    evt_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, evt_handler, NULL);
}

/* ============================================================
   wifi_start — STA (для node или резерв на root)
   ============================================================ */
esp_err_t wifi_start(void)
{
    esp_err_t err;

    s_has_saved_ssid = (strlen(g_config.wifi_ssid) > 0);

    wifi_config_t sta_cfg = {0};
    if (s_has_saved_ssid) {
        strncpy((char*)sta_cfg.sta.ssid, g_config.wifi_ssid, sizeof(sta_cfg.sta.ssid) - 1);
        if (strlen(g_config.wifi_password) >= 8) {
            strncpy((char*)sta_cfg.sta.password, g_config.wifi_password,
                    sizeof(sta_cfg.sta.password) - 1);
        }
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        sta_cfg.sta.pmf_cfg.capable  = false;
        sta_cfg.sta.pmf_cfg.required = false;
        sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta_cfg.sta.threshold.rssi = -127;
    }

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) return err;

    esp_wifi_set_max_tx_power(WIFI_STA_TX_POWER_VAL);
    esp_wifi_set_protocol(WIFI_IF_STA,
                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_inactive_time(WIFI_IF_STA, 360);
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (s_has_saved_ssid) {
        ESP_LOGI(TAG, "Waiting for STA connection (up to 15 s)...");
        EventBits_t bits = xEventGroupWaitBits(s_wifi_evt, WIFI_GOT_IP_BIT,
                                               pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
        if (bits & WIFI_GOT_IP_BIT) {
            ESP_LOGI(TAG, "STA ready, IP=%s", s_ip);
        } else {
            ESP_LOGW(TAG, "STA connect timeout — IP may arrive later");
        }
    }

    if (!s_monitor_task)
        xTaskCreate(wifi_monitor_task, "wifi_mon", 4096, NULL, 4, &s_monitor_task);

    ESP_LOGI(TAG, "WiFi started: STA SSID='%s', IP=%s",
             s_has_saved_ssid ? g_config.wifi_ssid : "(none)", s_ip);
    return ESP_OK;
}

void wifi_stop(void)
{
    if (s_reconnect_timer) {
        esp_timer_stop(s_reconnect_timer);
        esp_timer_delete(s_reconnect_timer);
        s_reconnect_timer = NULL;
    }
    if (s_monitor_task) { vTaskDelete(s_monitor_task); s_monitor_task = NULL; }
    esp_wifi_stop();
    esp_wifi_deinit();
    s_sta_connected = false;
    s_sta_connecting = false;
    s_ap_is_running = false;
}

/* ============================================================
   wifi_ap_start_only — только AP (WIFI_MODE_AP), канал 6.
   Используется на root при старте, пока PPPoS не подтверждён.
   Beacon «разбужен» для ESP32-S3 (IDF v6.1+).
   ============================================================ */
esp_err_t wifi_ap_start_only(const char *ssid, const char *password)
{
    if (s_ap_is_running) {
        ESP_LOGW(TAG, "wifi_ap_start_only: already running, skip");
        return ESP_OK;
    }
    if (!ssid || strlen(ssid) == 0) return ESP_ERR_INVALID_ARG;
    if (strlen(ssid) > 32)          return ESP_ERR_INVALID_ARG;

    snprintf(s_ap_ssid_cur, sizeof(s_ap_ssid_cur), "%s", ssid);

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (!s_ap_netif) s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) return ESP_FAIL;
    }

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) return err;

    // Country Code — обязателен для ESP32-S3
    wifi_country_t country = {
        .cc = "RU", .schan = 1, .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    esp_wifi_set_country(&country);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);   // IDF v6.1: WIFI_BW20

    wifi_config_t ap_cfg = {0};
    strncpy((char*)ap_cfg.ap.ssid, s_ap_ssid_cur, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len       = strlen(s_ap_ssid_cur);
    ap_cfg.ap.channel        = 6;
    ap_cfg.ap.max_connection = AP_MAX_STA_CONN;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.pmf_cfg.capable  = false;
    ap_cfg.ap.pmf_cfg.required = false;
    ap_cfg.ap.beacon_interval = 100;

    if (password && strlen(password) >= 8)
        strncpy((char*)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password) - 1);
    else
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) return err;

    esp_wifi_set_max_tx_power(WIFI_AP_TX_POWER_VAL);

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) return err;

    // «Пробуждение» beacon для ESP32-S3 (IDF v6.1+)
    vTaskDelay(pdMS_TO_TICKS(150));

    wifi_sta_list_t *sta_list = calloc(1, sizeof(wifi_sta_list_t) +
                                          AP_MAX_STA_CONN * sizeof(wifi_ap_record_t));
    if (sta_list) {
        esp_err_t list_err = esp_wifi_ap_get_sta_list(sta_list);
        if (list_err == ESP_OK) {
            ESP_LOGI(TAG, "AP initialized: %d stations (beacon active)", sta_list->num);
        }
        free(sta_list);
    }

    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);   // refresh beacon buffer

    s_ap_is_running = true;
    ESP_LOGI(TAG, "AP-only started: SSID='%s' ch=6 (beacon awake, country=RU)",
             s_ap_ssid_cur);
    return ESP_OK;
}

/* ============================================================
   wifi_ap_start — APSTA (legacy)
   ============================================================ */
esp_err_t wifi_ap_start(const char *ssid, const char *password)
{
    if (s_ap_is_running) return ESP_OK;
    if (!ssid || strlen(ssid) == 0) return ESP_ERR_INVALID_ARG;

    snprintf(s_ap_ssid_cur, sizeof(s_ap_ssid_cur), "%s", ssid);

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (!s_ap_netif) s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) return ESP_FAIL;
    }

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) return err;

    wifi_config_t ap_cfg = {0};
    strncpy((char*)ap_cfg.ap.ssid, s_ap_ssid_cur, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len       = strlen(s_ap_ssid_cur);
    ap_cfg.ap.channel        = 6;
    ap_cfg.ap.max_connection = AP_MAX_STA_CONN;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.pmf_cfg.capable  = false;
    ap_cfg.ap.pmf_cfg.required = false;
    ap_cfg.ap.beacon_interval = 100;

    if (password && strlen(password) >= 8)
        strncpy((char*)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password) - 1);
    else
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) return err;

    esp_wifi_set_max_tx_power(WIFI_AP_TX_POWER_VAL);
    vTaskDelay(pdMS_TO_TICKS(150));
    wifi_sta_list_t *sta_list = calloc(1, sizeof(wifi_sta_list_t) +
                                          AP_MAX_STA_CONN * sizeof(wifi_ap_record_t));
    if (sta_list) { esp_wifi_ap_get_sta_list(sta_list); free(sta_list); }

    s_ap_is_running = true;
    ESP_LOGI(TAG, "AP started (APSTA): SSID='%s'", s_ap_ssid_cur);
    return ESP_OK;
}

void wifi_ap_stop(void)
{
    if (!s_ap_is_running) return;
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_ap_is_running = false;
    ESP_LOGW(TAG, "AP stopped, mode=STA");
}

/* ============================================================
   wifi_sta_start_after_ppp_fail — STA fallback, если PPPoS не поднялся.
   ============================================================ */
esp_err_t wifi_sta_start_after_ppp_fail(void)
{
    s_has_saved_ssid = (strlen(g_config.wifi_ssid) > 0);
    if (!s_has_saved_ssid) {
        ESP_LOGW(TAG, "STA fallback: no saved SSID");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "PPPoS failed → STA fallback: connecting to '%s'",
             g_config.wifi_ssid);

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) return err;

    wifi_config_t sta_cfg = {0};
    strncpy((char*)sta_cfg.sta.ssid, g_config.wifi_ssid, sizeof(sta_cfg.sta.ssid) - 1);
    if (strlen(g_config.wifi_password) >= 8) {
        strncpy((char*)sta_cfg.sta.password, g_config.wifi_password,
                sizeof(sta_cfg.sta.password) - 1);
    }
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta_cfg.sta.pmf_cfg.capable  = false;
    sta_cfg.sta.pmf_cfg.required = false;
    sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_cfg.sta.threshold.rssi = -127;

    err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK) return err;

    s_sta_connecting = true;
    esp_wifi_connect();

    if (!s_monitor_task)
        xTaskCreate(wifi_monitor_task, "wifi_mon", 4096, NULL, 4, &s_monitor_task);

    ESP_LOGI(TAG, "STA fallback started: SSID='%s'", g_config.wifi_ssid);
    return ESP_OK;
}

/* ============================================================
   Getters
   ============================================================ */
bool wifi_sta_connected(void) { return s_sta_connected; }
bool wifi_sta_has_ip(void)    { return s_sta_connected && strcmp(s_ip, "0.0.0.0") != 0; }
const char* wifi_get_ip(void) { return s_ip; }

bool wifi_sta_connecting(void) { return s_sta_connecting; }

bool wifi_ap_running(void)        { return s_ap_is_running; }
bool wifi_ap_is_running(void)     { return s_ap_is_running; }
const char* wifi_get_ap_ssid(void){ return s_ap_ssid_cur; }
const char* wifi_get_ap_ip(void)  { return s_ap_ip_str; }

int wifi_ap_client_count(void)
{
    wifi_sta_list_t *list = calloc(1, sizeof(wifi_sta_list_t) +
                                      AP_MAX_STA_CONN * sizeof(wifi_ap_record_t));
    if (!list) return -1;
    int n = -1;
    if (esp_wifi_ap_get_sta_list(list) == ESP_OK) n = list->num;
    free(list);
    return n;
}

/* ============================================================
   Scan: найти AP с заданным префиксом (самый сильный)
   ============================================================ */
esp_err_t wifi_scan_for_fsu_ap(char *out_ssid, size_t out_len,
                               int8_t *out_rssi, uint8_t *out_channel,
                               const char *ssid_prefix)
{
    if (!out_ssid || out_len == 0) return ESP_ERR_INVALID_ARG;
    out_ssid[0] = '\0';
    if (out_rssi) *out_rssi = -127;
    if (out_channel) *out_channel = 0;
    if (!ssid_prefix) ssid_prefix = "FSU_";

    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scan start failed: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) {
        ESP_LOGW(TAG, "scan: 0 AP");
        esp_wifi_clear_ap_list();
        return ESP_ERR_NOT_FOUND;
    }

    wifi_ap_record_t *list = calloc(n, sizeof(wifi_ap_record_t));
    if (!list) return ESP_ERR_NO_MEM;

    err = esp_wifi_scan_get_ap_records(&n, list);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get records failed: %s", esp_err_to_name(err));
        free(list);
        esp_wifi_clear_ap_list();
        return err;
    }

    ESP_LOGI(TAG, "scan: total %u AP (all channels)", n);

    int      best_rssi = -127;
    const wifi_ap_record_t *best = NULL;
    size_t   plen = strlen(ssid_prefix);

    for (int i = 0; i < n; i++) {
        const char *s = (const char *)list[i].ssid;
        size_t slen = strlen(s);
        if (slen < plen + 1) continue;
        if (strncmp(s, ssid_prefix, plen) != 0) continue;
        if (slen < 3 || strcmp(s + slen - 3, "_AP") != 0) continue;

        if (list[i].rssi > best_rssi) {
            best_rssi = list[i].rssi;
            best = &list[i];
        }
    }

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (best) {
        strncpy(out_ssid, (const char *)best->ssid, out_len - 1);
        out_ssid[out_len - 1] = '\0';
        if (out_rssi) *out_rssi = best->rssi;
        if (out_channel) *out_channel = best->primary;
        ESP_LOGI(TAG, "scan: chosen '%s' rssi=%d ch=%u",
                 out_ssid, best->rssi, best->primary);
        ret = ESP_OK;
    } else {
        ESP_LOGW(TAG, "scan: no AP matching '%s*_AP'", ssid_prefix);
    }

    free(list);
    esp_wifi_clear_ap_list();
    return ret;
}