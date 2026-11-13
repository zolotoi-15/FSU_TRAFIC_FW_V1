/* ============================================================
   time_sync.c — синхронизация времени через системное время
   ESP32-S3 (NVS + опциональный DS1307 + SNTP/LoRa).

   === FIX TIME-NVS-WRITE-FAST ===
   NVS пишется не чаще раза в 10 с (было 60).

   === FIX TIME-DELTA-RESET-ON-INIT ===
   s_last_delta_ms сбрасывается в time_sync_init().

   === FIX SNTP-PENDING-DIAG ===
   time_sync_start_sntp() теперь:
     - пишет s_sntp_started_ms;
     - пишет s_sntp_sync_ms (последний callback);
     - предоставляет time_sync_is_sntp_pending() — TRUE, пока
       SNTP запущен, но callback ещё не приходил дольше
       SNTP_PENDING_WARN_MS;
     - в on_got_ip (blink_example_main.c) default netif = PPPoS
       выставляется ДО time_sync_start_sntp() → SNTP-сокет
       идёт через PPP.
   time_sync_is_sntp_pending() используется в root_wd.c как
   мягкий индикатор: если SNTP не отвечает >WAIT_TIME_SYNC_MAX_MS,
   ROOT стартует без UTC (см. root_wd.c).
   ============================================================ */

#include "time_sync.h"
#include "config_manager.h"
#include "transport.h"
#include "ds1307.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
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

static volatile int32_t  s_last_delta_ms = 0;
static volatile uint32_t s_last_delta_update_ms = 0;

#define TIME_SYNC_PUSH_PERIOD_MS   10000
#define TIME_SYNC_REPORT_PERIOD_MS 5000

static uint32_t s_last_push_ms   = 0;
static uint32_t s_last_report_ms = 0;

/* === FIX SNTP-PENDING-DIAG === */
#define SNTP_PENDING_WARN_MS   (30 * 1000)

static volatile bool     s_sntp_started    = false;
static volatile bool     s_sntp_synced     = false;
static volatile uint32_t s_sntp_started_ms = 0;
static volatile uint32_t s_sntp_sync_ms    = 0;
/* === END FIX === */

/* === FIX TIME-NVS-PERSIST + TIME-NVS-WRITE-FAST === */
#define NVS_NS_TIME        "time_save"
#define NVS_KEY_UTC        "last_utc"
#define NVS_TIME_MIN_YEAR  2024
#define NVS_WRITE_MIN_S    10

static time_t s_last_nvs_write = 0;

static bool nvs_time_read(time_t *out)
{
    if (!out) return false;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_TIME, NVS_READONLY, &nvs) != ESP_OK) return false;

    int64_t saved = 0;
    esp_err_t err = nvs_get_i64(nvs, NVS_KEY_UTC, &saved);
    nvs_close(nvs);

    if (err != ESP_OK || saved <= 0) return false;

    time_t t = (time_t)saved;
    struct tm tm_utc;
    gmtime_r(&t, &tm_utc);
    if ((tm_utc.tm_year + 1900) < NVS_TIME_MIN_YEAR) {
        ESP_LOGD(TAG, "NVS time invalid (year=%d)", tm_utc.tm_year + 1900);
        return false;
    }
    *out = t;
    return true;
}

static void nvs_time_write(time_t t)
{
    if (t <= 0) return;
    if (s_last_nvs_write != 0 &&
        (t - s_last_nvs_write) < NVS_WRITE_MIN_S) {
        ESP_LOGD(TAG, "NVS time write skipped (last=%ld, new=%ld, diff<%d s)",
                 (long)s_last_nvs_write, (long)t, NVS_WRITE_MIN_S);
        return;
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_TIME, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_i64(nvs, NVS_KEY_UTC, (int64_t)t);
    nvs_commit(nvs);
    nvs_close(nvs);

    s_last_nvs_write = t;
}

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

static void set_base_no_nvs_write(uint32_t unix_sec, uint16_t ms,
                                  uint32_t rtt_us)
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

    if (rtc_handle) {
        time_t t = (time_t)sec;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        (void)ds1307_set_datetime(rtc_handle, &tm_utc);
    }

    if (s_mtx) xSemaphoreGive(s_mtx);
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

    nvs_time_write((time_t)sec);

    if (rtc_handle) {
        time_t t = (time_t)sec;
        struct tm tm_utc;
        gmtime_r(&t, &tm_utc);
        esp_err_t err = ds1307_set_datetime(rtc_handle, &tm_utc);
        if (err != ESP_OK) {
            static bool warned = false;
            if (!warned) {
                ESP_LOGW(TAG, "DS1307 write failed (%s) — RTC may be absent; "
                              "time still valid in system clock + NVS",
                         esp_err_to_name(err));
                warned = true;
            }
        }
    }

    if (s_mtx) xSemaphoreGive(s_mtx);

    ESP_LOGI(TAG, "time set: %lu.%03u UTC (rtt=%lu us, NVS=%s, DS1307=%s)",
             (unsigned long)sec, (unsigned)rem, (unsigned long)rtt_us,
             "ok",
             rtc_handle ? "written" : "absent");
}

esp_err_t time_sync_init(void)
{
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    /* === FIX TIME-DELTA-RESET-ON-INIT === */
    s_last_delta_ms = 0;
    s_last_delta_update_ms = 0;
    /* === END FIX === */

    /* === FIX SNTP-PENDING-DIAG === */
    s_sntp_started    = false;
    s_sntp_synced     = false;
    s_sntp_started_ms = 0;
    s_sntp_sync_ms    = 0;
    /* === END FIX === */

    time_t t_nvs = 0;
    if (nvs_time_read(&t_nvs)) {
        set_base_no_nvs_write((uint32_t)t_nvs, 0, 0);
        s_last_nvs_write = t_nvs;
        ESP_LOGI(TAG, "time from NVS: %lu (no rewrite)",
                 (unsigned long)t_nvs);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "NVS: no saved time, trying DS1307");

    if (rtc_handle) {
        struct tm tm_utc;
        memset(&tm_utc, 0, sizeof(tm_utc));
        if (ds1307_get_datetime(rtc_handle, &tm_utc) == ESP_OK &&
            (tm_utc.tm_year + 1900) >= NVS_TIME_MIN_YEAR) {
            time_t t = timegm_compat(&tm_utc);
            if (t > 0) {
                set_base_no_nvs_write((uint32_t)t, 0, 0);
                s_last_nvs_write = 0;
                nvs_time_write(t);
                ESP_LOGI(TAG, "time from DS1307: %lu (saved to NVS)",
                         (unsigned long)t);
                return ESP_OK;
            }
        }
        ESP_LOGW(TAG, "DS1307 empty/invalid — time not set");
    } else {
        ESP_LOGW(TAG, "no DS1307 — time not set (waiting for SNTP/root)");
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

uint64_t time_sync_get_utc_ms(void)
{
    uint32_t sec = 0;
    uint16_t ms  = 0;
    if (!time_sync_get(&sec, &ms)) return 0;
    return ((uint64_t)sec * 1000ULL) + (uint64_t)ms;
}

void time_sync_set(uint32_t unix_sec, uint16_t ms, uint32_t rtt_us)
{
    set_base(unix_sec, ms, rtt_us);
}

void time_sync_note_delta_ms(int32_t delta_ms)
{
    s_last_delta_ms = delta_ms;
    s_last_delta_update_ms = now_ms_tick();
    ESP_LOGI(TAG, "delta noted: %ld ms", (long)delta_ms);
}

int32_t time_sync_get_delta_ms(void)
{
    return s_last_delta_ms;
}

uint32_t time_sync_get_delta_age_ms(void)
{
    if (s_last_delta_update_ms == 0) return 0;
    uint32_t now = now_ms_tick();
    if (now < s_last_delta_update_ms) return 0;
    return now - s_last_delta_update_ms;
}

/* === FIX SNTP-PENDING-DIAG === */
static void sntp_sync_cb(struct timeval *tv)
{
    if (!tv) return;
    ESP_LOGI(TAG, "SNTP sync: %ld.%06ld", (long)tv->tv_sec, (long)tv->tv_usec);
    s_sntp_synced = true;
    s_sntp_sync_ms = now_ms_tick();
    set_base((uint32_t)tv->tv_sec, (uint16_t)(tv->tv_usec / 1000), 0);
}

esp_err_t time_sync_start_sntp(void)
{
    if (s_sntp_started) return ESP_OK;
    s_sntp_started = true;

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_sync_cb);
    esp_sntp_init();

    s_sntp_started_ms = now_ms_tick();
    ESP_LOGI(TAG, "SNTP started (pool.ntp.org), started_at=%lu ms",
             (unsigned long)s_sntp_started_ms);
    return ESP_OK;
}

bool time_sync_is_sntp_pending(void)
{
    if (!s_sntp_started) return false;
    if (s_sntp_synced)    return false;
    if (s_sntp_started_ms == 0) return false;
    uint32_t now = now_ms_tick();
    return (now - s_sntp_started_ms) >= SNTP_PENDING_WARN_MS;
}
/* === END FIX SNTP-PENDING-DIAG === */

/* ============================================================
   time_sync_root_push_ex
   ============================================================ */
static void time_sync_root_push_ex(uint32_t rtt_us, bool force)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!transport_is_lora()) return;

    uint32_t now = now_ms_tick();
    if (!force && s_last_push_ms != 0 &&
        (now - s_last_push_ms) < TIME_SYNC_PUSH_PERIOD_MS)
        return;
    s_last_push_ms = now;

    if (!s_valid) {
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
        esp_err_t err = transport_send_cmd_noack(cmd, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "time_sync_root_push: %s", esp_err_to_name(err));
        } else {
            ESP_LOGD(TAG, "→ node: %s", cmd);
        }
    }
}

void time_sync_root_push(uint32_t rtt_us)
{
    time_sync_root_push_ex(rtt_us, false);
}

void time_sync_root_push_force(uint32_t rtt_us)
{
    time_sync_root_push_ex(rtt_us, true);
}

void time_sync_node_report(void)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (!transport_is_lora()) return;

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
    if (n > 0) {
        esp_err_t err = transport_send_evt(evt, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "time_sync_node_report: %s", esp_err_to_name(err));
        }
    }
}