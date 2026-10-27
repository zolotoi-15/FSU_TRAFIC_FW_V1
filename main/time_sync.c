#include "time_sync.h"
#include "config_manager.h"
#include "wifi_ap.h"
#include "node_link.h"
#include "ds1307.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "time_sync";

extern ds1307_handle_t rtc_handle;

static volatile uint32_t s_base_sec = 0;
static volatile uint16_t s_base_ms  = 0;
static volatile int64_t  s_t0_us    = 0;
static volatile bool     s_valid    = false;
static SemaphoreHandle_t s_mtx      = NULL;

#define TIME_SYNC_PUSH_PERIOD_MS   30000
#define TIME_SYNC_REPORT_PERIOD_MS 5000

static uint32_t s_last_push_ms   = 0;
static uint32_t s_last_report_ms = 0;

/* FIX: picolibc (CONFIG_LIBC_PICOLIBC=y) не имеет timegm(). */
static time_t timegm_compat(const struct tm *tm)
{
    static const int mdays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    int year = tm->tm_year + 1900;
    int mon  = tm->tm_mon;
    if (mon < 0 || mon > 11) return (time_t)-1;
    long days = (year - 1970) * 365L
              + (year - 1969) / 4 - (year - 1901) / 100 + (year - 1601) / 400
              + mdays[mon] + (tm->tm_mday - 1);
    if (mon > 1 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0))
        days++;
    return days * 86400L + tm->tm_hour * 3600L + tm->tm_min * 60L + tm->tm_sec;
}

static inline uint32_t now_ms_tick(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void set_base(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us)
{
    if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);

    uint32_t add_ms = rtt_us / 2000;
    uint32_t total_ms = (uint32_t)ms + add_ms;
    uint32_t sec = unix_sec + total_ms / 1000;
    uint16_t rem = (uint16_t)(total_ms % 1000);

    s_base_sec = sec;
    s_base_ms  = rem;
    s_t0_us    = esp_timer_get_time();
    s_valid    = true;

    struct timeval tv = { .tv_sec = (time_t)sec, .tv_usec = rem * 1000 };
    settimeofday(&tv, NULL);

    if (s_mtx) xSemaphoreGive(s_mtx);

    ESP_LOGI(TAG, "time set: %lu.%03u UTC (rtt=%lu us)",
             (unsigned long)sec, (unsigned)rem, (unsigned long)rtt_us);

    if (rtc_handle) {
        time_t t = (time_t)sec;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        esp_err_t err = ds1307_set_datetime(rtc_handle, &tm_utc);
        if (err != ESP_OK) {
            static bool warned = false;
            if (!warned) {
                ESP_LOGW(TAG, "DS1307 write failed (%s) — RTC may be absent; "
                              "time still valid in system clock",
                         esp_err_to_name(err));
                warned = true;
            }
        }
    }
}

esp_err_t time_sync_init(void)
{
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    if (rtc_handle) {
        struct tm tm_utc;
        memset(&tm_utc, 0, sizeof(tm_utc));
        if (ds1307_get_datetime(rtc_handle, &tm_utc) == ESP_OK &&
            (tm_utc.tm_year + 1900) >= 2024) {
            time_t t = timegm_compat(&tm_utc);
            if (t > 0) {
                set_base((uint32_t)t, 0, 0);
                ESP_LOGI(TAG, "time from DS1307: %lu", (unsigned long)t);
                return ESP_OK;
            }
        }
        ESP_LOGW(TAG, "DS1307 empty/invalid — time not set");
    } else {
        ESP_LOGW(TAG, "no RTC — time not set (waiting for SNTP/root)");
    }
    return ESP_OK;
}

bool time_sync_get(uint32_t *unix_sec, uint16_t *ms)
{
    if (!s_valid) return false;
    int64_t now = esp_timer_get_time();
    int64_t dt_us = now - s_t0_us;
    if (dt_us < 0) dt_us = 0;

    uint32_t add_ms = (uint32_t)(dt_us / 1000);
    uint32_t total  = (uint32_t)s_base_ms + add_ms;
    if (unix_sec) *unix_sec = s_base_sec + total / 1000;
    if (ms)       *ms       = (uint16_t)(total % 1000);
    return true;
}

void time_sync_set(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us)
{
    set_base(unix_sec, ms, rtt_us);
}

static void sntp_sync_cb(struct timeval *tv)
{
    if (!tv) return;
    ESP_LOGI(TAG, "SNTP sync: %ld.%06ld", (long)tv->tv_sec, (long)tv->tv_usec);
    set_base((uint32_t)tv->tv_sec, (uint16_t)(tv->tv_usec / 1000), 0);
}

esp_err_t time_sync_start_sntp(void)
{
    static bool started = false;
    if (started) return ESP_OK;
    started = true;

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_sync_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started (pool.ntp.org)");
    return ESP_OK;
}

void time_sync_root_push(uint32_t rtt_us)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!node_link_server_has_client()) return;

    /* FIX: throttle ДО проверки s_valid — иначе при !s_valid
       s_last_push_ms не обновляется и warning спамит каждые
       100 мс (phase_sync_task). */
    uint32_t now = now_ms_tick();
    if (s_last_push_ms != 0 && (now - s_last_push_ms) < TIME_SYNC_PUSH_PERIOD_MS)
        return;
    s_last_push_ms = now;

    if (!s_valid) {
        /* FIX: DEBUG, а не WARN — не засоряем лог. */
        ESP_LOGD(TAG, "root push: time not valid yet (no RTC/SNTP)");
        return;
    }

    uint32_t sec; uint16_t ms;
    if (!time_sync_get(&sec, &ms)) return;

    char cmd[64];
    int n = snprintf(cmd, sizeof(cmd), "time:%lu:%u:%lu",
                     (unsigned long)sec, (unsigned)ms,
                     (unsigned long)rtt_us);
    if (n > 0) {
        node_link_server_send_cmd(cmd, n);
        ESP_LOGD(TAG, "→ node: %s", cmd);
    }
}

void time_sync_node_report(void)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (!node_link_client_connected()) return;

    uint32_t now = now_ms_tick();
    if (s_last_report_ms != 0 && (now - s_last_report_ms) < TIME_SYNC_REPORT_PERIOD_MS)
        return;
    s_last_report_ms = now;

    uint32_t sec = 0; uint16_t ms = 0;
    bool ok = time_sync_get(&sec, &ms);
    if (!ok) return;

    char evt[80];
    int n = snprintf(evt, sizeof(evt), "time:%lu:%u:%lu",
                     (unsigned long)sec, (unsigned)ms,
                     (unsigned long)now_ms_tick());
    if (n > 0) node_link_client_send_evt(evt, n);
}