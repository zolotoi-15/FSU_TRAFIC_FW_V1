/* ============================================================
   node_proto.c — NODE/ROOT протокол.

   === FIX RT-CHANNELS ===
   NODE: c: содержит apply_utc_ms. Если оно != 0 и UTC валиден,
   NODE откладывает применение маски до этого момента в
   node_mask_scheduler_task (шаг 5 мс). Пока маска в pending:
     - traffic_light_check_faults НЕ вызывается для каналов
       из pending (иначе ложный SHORT_GND на 150 мс, пока SR
       ещё в старой маске, а s_active_mask_from_root уже новая);
     - s_expected_mask_from_root / s_active_mask_from_root
       обновляются В МОМЕНТ ПРИМЕНЕНИЯ, а не при приходе c:.
   Реализовано через s_pending_mask_valid — mux_task и
   check_faults смотрят на этот флаг.

   Если apply_utc_ms == 0 (refresh c:, fallback) — применяем
   сразу, как раньше.

   Остальные FIX сохранены.
   ============================================================ */

#include "node_proto.h"
#include "app_shared.h"
#include "sync_config.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "shift_register.h"
#include "rs485.h"
#include "can_bus.h"
#include "mux_data.h"
#include "root_wd.h"
#include "sync_handshake.h"
#include "lora.h"
#include "lora_nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include "traffic_internal.h"

static const char *TAG = "node_proto";

extern volatile uint32_t s_last_sync_ms;
extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool     s_comm_lost;

volatile float    s_node_link_rssi     = 0.0f;
volatile float    s_node_link_snr      = 0.0f;
volatile float    s_node_link_sig_rssi = 0.0f;
volatile uint32_t s_node_link_age_ms   = 0;
volatile bool     s_node_link_valid    = false;

char s_node_channels_json[2048] = {0};
volatile bool s_node_channels_valid = false;
#define NODE_CHANNELS_PERIOD_MS  60000
static uint32_t s_last_channels_ms = 0;

#define NODE_STATUS_PERIOD_MS      8000
#define NODE_STATUS_FAULT_MS       1000
#define NODE_STATUS_SYNC_MS        300
#define STATUS_FORCE_MS            500

#define NODE_ROOT_LOST_MS          6000

#define UTC_DELTA_MAX_MS_LOCAL     3000
#define SYNC_MAX_DRIFT_MS_LOCAL    500

#define NODE_STATUS_BUF_SIZE       1024

volatile bool s_fault_from_remote_ex = false;
volatile bool s_node_running = false;

static volatile bool     s_status_now = false;
static volatile uint32_t s_status_force_until_ms = 0;
static uint32_t s_last_node_pub_ms = 0;

static volatile uint32_t s_last_root_data_ms = 0;

static volatile bool s_root_lost_flag = false;
static volatile bool s_c_received_since_start = false;

static volatile bool     s_pending_c_valid = false;
static volatile uint8_t  s_pending_c_phase = 0;
static volatile uint32_t s_pending_c_pm    = 0;
static volatile uint16_t s_pending_c_on    = 0;
static volatile uint16_t s_pending_c_blink = 0;
static volatile uint32_t s_pending_c_dur   = 0;
static volatile uint64_t s_pending_c_end   = 0;

/* === FIX RT-CHANNELS ===
   Отложенное применение маски по UTC. NODE получает c: с
   apply_utc_ms и применяет маску ровно в этот момент.
   Пока s_pending_mask_valid — mux_task/check_faults пропускают
   проверку каналов, чтобы не сгенерировать ложный SHORT_GND
   (SR ещё в старой маске, s_active_mask_from_root уже новая
   ещё не применена — но мы и не обновляем её до применения). */
typedef struct {
    uint8_t  ph;
    uint32_t pm;
    uint16_t on;
    uint16_t blink;
    uint32_t dur;
    uint64_t end_utc;
    bool     has_utc;
} pending_mask_t;

static pending_mask_t     s_pending_mask = {0};
static volatile bool      s_pending_mask_valid = false;
static uint64_t           s_pending_mask_apply_utc = 0;
static SemaphoreHandle_t  s_pending_mask_mtx = NULL;

bool node_mask_pending(void) { return s_pending_mask_valid; }
/* === END FIX RT-CHANNELS === */

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

bool node_cfg_received(void) { return true; }
bool node_cfg_confirmed_by_root(void) { return true; }

void node_send_status_now(void) { s_status_now = true; }

void node_send_status_for_ms(uint32_t ms)
{
    s_status_now = true;
    s_status_force_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + ms;
}

static inline void node_note_root_data(void)
{
    s_last_root_data_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

/* ============================================================
   node_apply_c_mask
   === FIX FAULT-HAS-TOP-PRIORITY ===
   ============================================================ */
static void node_apply_c_mask(uint8_t ph,
                              uint32_t pm,
                              uint16_t on16,
                              uint16_t blink16,
                              uint32_t dur,
                              uint64_t end_utc_ms,
                              bool has_utc_field)
{
    if (traffic_light_fault_has_priority()) {
        ESP_LOGW(TAG, "c: apply IGNORED (fault active: %s)",
                 traffic_light_fault_str());
        return;
    }

    uint32_t apply_pm = pm;
    bool used_utc = false;

    if (has_utc_field && end_utc_ms != 0) {
        uint64_t now_utc = time_sync_get_utc_ms();
        int32_t  delta   = time_sync_get_delta_ms();
        int32_t  abs_delta = delta < 0 ? -delta : delta;

        if (now_utc == 0) {
            ESP_LOGD(TAG, "c: UTC skipped (now_utc=0)");
        } else if (abs_delta > UTC_DELTA_MAX_MS_LOCAL) {
            ESP_LOGW(TAG, "c: UTC skipped (delta=%ld ms > %d) → pm=%lu",
                     (long)delta, UTC_DELTA_MAX_MS_LOCAL,
                     (unsigned long)pm);
        } else {
            int64_t remain = (int64_t)end_utc_ms - (int64_t)now_utc;
            if (remain < 0) remain = 0;
            if ((uint64_t)remain > (uint64_t)dur) remain = (int64_t)dur;
            uint32_t pm_utc = (uint32_t)((uint64_t)dur - (uint64_t)remain);

            int32_t drift = (int32_t)pm_utc - (int32_t)pm;
            if (drift < 0) drift = -drift;

            if (drift > SYNC_MAX_DRIFT_MS_LOCAL) {
                ESP_LOGW(TAG, "c: UTC drift=%ld ms (> %d) → hard pm=%lu",
                         (long)drift, SYNC_MAX_DRIFT_MS_LOCAL,
                         (unsigned long)pm);
                apply_pm = pm;
                used_utc = false;
            } else {
                apply_pm = pm_utc;
                used_utc = true;
            }
        }
    }

    ESP_LOGI(TAG, "c: ph=%u pm=%lu→%lu on=0x%04X blink=0x%04X dur=%lu%s",
             ph, (unsigned long)pm, (unsigned long)apply_pm,
             on16, blink16, (unsigned long)dur,
             used_utc ? " [utc]" : "");

    traffic_light_apply_mask(on16, blink16, ph, apply_pm, dur);
}

/* ============================================================
   === FIX RT-CHANNELS ===
   node_schedule_mask_apply — поставить маску в pending, чтобы
   применить её в apply_utc_ms.
   ============================================================ */
static void node_schedule_mask_apply(uint8_t ph, uint32_t pm,
                                     uint16_t on16, uint16_t blink16,
                                     uint32_t dur, uint64_t end_utc_ms,
                                     uint64_t apply_utc_ms)
{
    if (!s_pending_mask_mtx) {
        /* scheduler ещё не запущен — применяем сразу (fallback) */
        node_apply_c_mask(ph, pm, on16, blink16, dur, end_utc_ms,
                          end_utc_ms != 0);
        return;
    }

    if (traffic_light_fault_has_priority()) {
        ESP_LOGW(TAG, "sched c: IGNORED (fault active: %s)",
                 traffic_light_fault_str());
        return;
    }

    xSemaphoreTake(s_pending_mask_mtx, portMAX_DELAY);
    s_pending_mask.ph      = ph;
    s_pending_mask.pm      = pm;
    s_pending_mask.on      = on16;
    s_pending_mask.blink   = blink16;
    s_pending_mask.dur     = dur;
    s_pending_mask.end_utc = end_utc_ms;
    s_pending_mask.has_utc = (end_utc_ms != 0);
    s_pending_mask_apply_utc = apply_utc_ms;
    s_pending_mask_valid = true;
    xSemaphoreGive(s_pending_mask_mtx);

    ESP_LOGI(TAG, "sched c: ph=%u apply_utc=%llu (now=%llu, delta=%lld ms)",
             ph, (unsigned long long)apply_utc_ms,
             (unsigned long long)time_sync_get_utc_ms(),
             (long long)((int64_t)apply_utc_ms -
                         (int64_t)time_sync_get_utc_ms()));
}

static void node_mask_scheduler_task(void *arg)
{
    ESP_LOGI(TAG, "node_mask_scheduler_task started (step=5 ms)");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (!s_pending_mask_valid) continue;

        uint64_t now = time_sync_get_utc_ms();
        if (now == 0) {
            /* Время пропало — применяем сразу, чтобы не залипнуть.
               apply_utc_ms здесь не нужен (мы не ждём). */
            pending_mask_t p;
            xSemaphoreTake(s_pending_mask_mtx, portMAX_DELAY);
            p = s_pending_mask;
            s_pending_mask_valid = false;
            xSemaphoreGive(s_pending_mask_mtx);

            ESP_LOGW(TAG, "sched c: UTC lost → apply immediately");
            node_apply_c_mask(p.ph, p.pm, p.on, p.blink, p.dur,
                              p.end_utc, p.has_utc);
            continue;
        }

        uint64_t apply_utc;
        xSemaphoreTake(s_pending_mask_mtx, portMAX_DELAY);
        apply_utc = s_pending_mask_apply_utc;
        xSemaphoreGive(s_pending_mask_mtx);

        if (now >= apply_utc) {
            pending_mask_t p;
            xSemaphoreTake(s_pending_mask_mtx, portMAX_DELAY);
            p = s_pending_mask;
            s_pending_mask_valid = false;
            xSemaphoreGive(s_pending_mask_mtx);

            ESP_LOGI(TAG, "sched c: APPLY ph=%u (now=%llu, target=%llu)",
                     p.ph, (unsigned long long)now,
                     (unsigned long long)apply_utc);
            node_apply_c_mask(p.ph, p.pm, p.on, p.blink, p.dur,
                              p.end_utc, p.has_utc);
        }
    }
}
/* === END FIX RT-CHANNELS === */

/* ============================================================
   on_cmd_from_root
   ============================================================ */
static void on_cmd_from_root(const char *cmd, size_t len)
{
    link_monitor_note_ping_rx();
    node_note_root_data();

    if (s_root_lost_flag) {
        ESP_LOGW(TAG, "ROOT back (cmd='%.16s...') -> clear local ALL_OFF "
                      "(was root-lost)",
                 cmd ? cmd : "");
        s_root_lost_flag = false;
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
    }

    char buf[256];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n);
    buf[n] = '\0';

    if (strncmp(buf, "time:", 5) == 0) {
        unsigned long sec = 0;
        unsigned int ms = 0;
        unsigned long rtt = 0;
        if (sscanf(buf + 5, "%lu:%u:%lu", &sec, &ms, &rtt) >= 2) {
            uint32_t local_sec = 0;
            uint16_t local_ms = 0;
            if (time_sync_get(&local_sec, &local_ms)) {
                int32_t delta = (int32_t)((int64_t)sec * 1000 + ms
                                          - ((int64_t)local_sec * 1000 + local_ms));
                time_sync_note_delta_ms(delta);
                ESP_LOGI(TAG, "time: root=%lu.%03u local=%lu.%03u delta=%ld ms",
                         sec, ms, (unsigned long)local_sec, local_ms,
                         (long)delta);
            } else {
                ESP_LOGI(TAG, "time: first set root=%lu.%03lu", sec, ms);
            }
            time_sync_set((uint32_t)sec, (uint16_t)ms, (uint32_t)rtt);
        }
        sync_handshake_node_on_rx(buf, n);
        return;
    }

    if (len >= 5 && strncmp(buf, "sync:", 5) == 0) {
        sync_handshake_node_on_rx(buf, n);
        if (strncmp(buf, "sync:probe", 10) == 0) return;
        if (strncmp(buf, "sync:lock:", 10) == 0) return;
        if (strncmp(buf, "sync:done", 9) == 0) return;
        if (strncmp(buf, "sync:ack", 8) == 0) return;
    }

    if (buf[0] == 'c' && buf[1] == ':') {
        if (traffic_light_fault_has_priority()) {
            ESP_LOGW(TAG, "c: IGNORED (fault active: %s)",
                     traffic_light_fault_str());
            return;
        }

        unsigned int ph = 0;
        unsigned long pm = 0;
        unsigned int on_mask = 0;
        unsigned int blink_mask = 0;
        unsigned long dur = 0;
        unsigned long long end_utc_ms = 0;
        unsigned long long apply_utc_ms = 0;

        int matched = sscanf(buf + 2, "%u:%lu:%4x:%4x:%lu:%llu:%llu",
                             &ph, &pm, &on_mask, &blink_mask, &dur,
                             &end_utc_ms, &apply_utc_ms);
        if (matched < 5) {
            ESP_LOGW(TAG, "c: bad format: '%s'", buf);
            return;
        }

        if (!s_node_running) {
            ESP_LOGW(TAG, "c: NODE not RUNNING → stash for later "
                          "(ph=%u)", ph);
            s_pending_c_phase = (uint8_t)ph;
            s_pending_c_pm    = (uint32_t)pm;
            s_pending_c_on    = (uint16_t)on_mask;
            s_pending_c_blink = (uint16_t)blink_mask;
            s_pending_c_dur   = (uint32_t)dur;
            s_pending_c_end   = (uint64_t)end_utc_ms;
            s_pending_c_valid = true;
            return;
        }

        s_c_received_since_start = true;
        s_fault_from_remote_ex = false;

        /* === FIX RT-CHANNELS ===
           matched >= 7 → есть apply_utc_ms.
           Если он != 0 — планируем.
           Если 0 или нет поля (matched < 7) — применяем сразу
           (refresh c: / fallback). */
        if (matched >= 7 && apply_utc_ms != 0) {
            node_schedule_mask_apply((uint8_t)ph, (uint32_t)pm,
                                     (uint16_t)on_mask, (uint16_t)blink_mask,
                                     (uint32_t)dur, (uint64_t)end_utc_ms,
                                     (uint64_t)apply_utc_ms);
            return;
        }

        node_apply_c_mask((uint8_t)ph, (uint32_t)pm,
                          (uint16_t)on_mask, (uint16_t)blink_mask,
                          (uint32_t)dur, (uint64_t)end_utc_ms,
                          matched >= 6);
        return;
        /* === END FIX RT-CHANNELS === */
    }

    if (strncmp(buf, "fault:none", 10) == 0) {
        uint8_t local_cnt = traffic_light_get_fault_count();
        if (traffic_light_fault_has_priority() && local_cnt > 0) {
            ESP_LOGW(TAG, "fault:none from ROOT IGNORED: NODE has %u "
                          "active local fault(s), wait try_recovery",
                     local_cnt);
            return;
        }

        ESP_LOGW(TAG, "fault:none from ROOT -> clear local fault");
        s_root_lost_flag = false;
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_NONE);
        SET_REMOTE_FLAG(false);
        return;
    }
    if (strncmp(buf, "fault:all_off", 13) == 0) {
        ESP_LOGW(TAG, "fault:all_off from ROOT -> ALL_OFF");
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        node_send_status_for_ms(2000);
        return;
    }

    if (strncmp(buf, "fault:yellow_blink", 18) == 0) {
        unsigned int mask = 0;
        const char *p = buf + 18;
        if (*p == ':') p++;
        bool have_mask = false;
        if (sscanf(p, "0x%4x", &mask) == 1 ||
            sscanf(p, "0X%4x", &mask) == 1 ||
            sscanf(p, "%4x", &mask) == 1) {
            have_mask = true;
        }

        if (have_mask && mask != 0) {
            s_fault_yellow_mask_from_root = (uint16_t)mask;
            s_fault_mask_valid = true;
            ESP_LOGW(TAG, "fault:yellow_blink from ROOT, mask=0x%04X "
                          "→ NODE blinks by ROOT mask",
                     s_fault_yellow_mask_from_root);
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_YELLOW_BLINK);
            SET_REMOTE_FLAG(false);
        } else {
            s_fault_yellow_mask_from_root = 0;
            s_fault_mask_valid = false;
            ESP_LOGE(TAG, "fault:yellow_blink from ROOT but mask=0 → "
                          "treat as ALL_OFF (safe)");
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
        }
        node_send_status_for_ms(2000);
        return;
    }

    if (strcmp(buf, "traffic:night:on") == 0) {
        traffic_light_set_night_mode(true);
        node_send_status_for_ms(STATUS_FORCE_MS);
        return;
    }
    if (strcmp(buf, "traffic:night:off") == 0) {
        traffic_light_set_night_mode(false);
        node_send_status_for_ms(STATUS_FORCE_MS);
        return;
    }

    {
        int mph = -1;
        if (sscanf(buf, "manual:override:phase:%d", &mph) == 1 &&
            mph >= 0 && mph < g_config.traffic.sequence_count) {
            traffic_light_manual_override((uint8_t)mph);
            return;
        }
    }
    if (strcmp(buf, "manual:override:off") == 0) {
        traffic_light_manual_release();
        return;
    }
    {
        unsigned int pms = 0;
        if (sscanf(buf, "priority:extend:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_extend(pms);
            return;
        }
        if (sscanf(buf, "priority:shorten:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_shorten(pms);
            return;
        }
    }

    if (strcmp(buf, "timeq") == 0) {
        uint32_t t_sec = 0;
        uint16_t t_ms = 0;
        if (time_sync_get(&t_sec, &t_ms)) {
            char evt[64];
            int en = snprintf(evt, sizeof(evt), "time:%lu:%u:%lu",
                              (unsigned long)t_sec, (unsigned)t_ms,
                              (unsigned long)(esp_timer_get_time() / 1000));
            if (en > 0) transport_send_evt_noack(evt, en);
        }
        return;
    }

    if (strcmp(buf, "ping") == 0) {
        s_last_sync_ms = (uint32_t)(esp_timer_get_time() / 1000);
        return;
    }
    if (strncmp(buf, "ping:", 5) == 0) {
        unsigned int ts = 0;
        if (sscanf(buf + 5, "%u", &ts) == 1) {
            uint32_t t_recv = (uint32_t)esp_timer_get_time();
            char pong[64];
            int pn = snprintf(pong, sizeof(pong), "pong:%u:%lu:%lu",
                              ts, (unsigned long)t_recv,
                              (unsigned long)t_recv);
            if (pn > 0) transport_send_evt_noack(pong, pn);
        }
        return;
    }

    if (strncmp(buf, "delay:", 6) == 0) {
        unsigned int d = 0;
        if (sscanf(buf + 6, "%u", &d) == 1) {
            ESP_LOGW(TAG, "delay:%u ignored (sync_delay must be 0)", d);
        }
        return;
    }

    if (strcmp(buf, "pedestrian:request") == 0) {
        ESP_LOGD(TAG, "pedestrian:request from ROOT (follow_root - no-op)");
        return;
    }

    if (strncmp(buf, "lora.cfg:", 9) == 0) {
        unsigned long freq = 0;
        unsigned int sf = 0, bw = 0, cr = 0;
        int pwr = 0;
        if (sscanf(buf + 9, "%lu:%u:%u:%u:%d", &freq, &sf, &bw, &cr, &pwr) == 5) {
            ESP_LOGW(TAG, "lora.cfg from ROOT: freq=%lu sf=%u bw=%u cr=%u pwr=%d",
                     freq, sf, bw, cr, pwr);
            lora_cfg_nvs_t cfg = {
                .freq_hz = (uint32_t)freq, .sf = (uint8_t)sf,
                .bw_khz = (uint16_t)bw, .cr = (uint8_t)cr,
                .tx_power_dbm = (int8_t)pwr,
            };
            lora_nvs_save(&cfg);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
        return;
    }

    if (strncmp(buf, "rs485:", 6) == 0 || strncmp(buf, "can:", 4) == 0) {
        node_handle_local_cmd(buf);
        return;
    }

    if (strcmp(buf, "traffic:start") == 0) {
        ESP_LOGW(TAG, "traffic:start from ROOT");

        if (traffic_light_fault_has_priority()) {
            ESP_LOGW(TAG, "traffic:start IGNORED: fault active (%s), "
                          "wait fault:none",
                     traffic_light_fault_str());
            node_send_status_for_ms(2000);
            return;
        }

        bool was_running = s_node_running;
        if (!was_running) s_c_received_since_start = false;

        traffic_light_set_follow_root(true);

        if (traffic_light_get_state() == TRAFFIC_STATE_RECOVERING ||
            traffic_light_get_state() == TRAFFIC_STATE_STOPPED) {
            ESP_LOGW(TAG, "traffic:start: NODE recover → start_with_red");
            traffic_light_start_with_red();
        } else {
            traffic_light_start();
        }

        s_node_running = true;
        node_note_root_data();
        node_send_status_for_ms(STATUS_FORCE_MS);

        if (s_pending_c_valid) {
            ESP_LOGI(TAG, "traffic:start → apply stashed c: ph=%u",
                     s_pending_c_phase);
            node_apply_c_mask(s_pending_c_phase,
                              s_pending_c_pm,
                              s_pending_c_on,
                              s_pending_c_blink,
                              s_pending_c_dur,
                              s_pending_c_end,
                              s_pending_c_end != 0);
            s_pending_c_valid = false;
            s_c_received_since_start = true;
        }
        return;
    }
    if (strcmp(buf, "traffic:stop") == 0) {
        traffic_light_stop();
        s_node_running = false;
        s_root_lost_flag = false;
        s_c_received_since_start = false;
        s_pending_c_valid = false;
        /* pending UTC-mask тоже сбрасываем */
        s_pending_mask_valid = false;
        return;
    }
    if (strcmp(buf, "traffic:pause")  == 0) { traffic_light_pause();  return; }
    if (strcmp(buf, "traffic:resume") == 0) { traffic_light_resume(); return; }
    if (strcmp(buf, "traffic:reset")  == 0) { traffic_light_reset();  return; }
    if (strcmp(buf, "traffic:next")   == 0) { traffic_light_next_phase(); return; }

    {
        int mode = 0;
        if (sscanf(buf, "traffic:mode:%d", &mode) == 1) {
            if (mode >= 1 && mode <= 3) {
                traffic_light_set_mode((traffic_mode_t)mode);
            }
            return;
        }
    }

    {
        int relay = -1, state = -1;
        if (sscanf(buf, "relay:%d:%d", &relay, &state) == 2 ||
            sscanf(buf, "sr:%d:%d",    &relay, &state) == 2) {
            if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
                if (traffic_light_fault_has_priority()) {
                    ESP_LOGW(TAG, "relay:%d:%d IGNORED (fault active)",
                             relay, state);
                    return;
                }
                if (state) sr_output_on(relay);
                else       sr_output_off(relay);
            }
            return;
        }
    }
    if (strcmp(buf, "all:on")  == 0) {
        if (traffic_light_fault_has_priority()) {
            ESP_LOGW(TAG, "all:on IGNORED (fault active)");
            return;
        }
        sr_all_on();  return;
    }
    if (strcmp(buf, "all:off") == 0) {
        sr_all_off(); return;
    }
}

/* ============================================================
   node_handle_local_cmd
   ============================================================ */
void node_handle_local_cmd(const char *cmd)
{
    int baud = 0;
    if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
        rs485_set_baudrate((uint32_t)baud);
        return;
    }
    if (strncmp(cmd, "rs485:send:", 11) == 0) {
        const char *p = cmd + 11;
        size_t hl = strlen(p);
        if (hl > 0 && (hl % 2) == 0 && hl <= 512) {
            uint8_t buf[256];
            size_t n = hl / 2;
            for (size_t i = 0; i < n; i++) {
                int hi = -1, lo = -1;
                char c1 = p[i * 2], c2 = p[i * 2 + 1];
                if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                if (hi < 0 || lo < 0) return;
                buf[i] = (uint8_t)((hi << 4) | lo);
            }
            rs485_send(buf, n);
        }
        return;
    }

    unsigned int id = 0;
    char hex_data[32] = {0};
    int ext_flag = 0;
    int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d",
                         &id, hex_data, &ext_flag);
    if (matched >= 2 ||
        (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
        uint8_t data[8] = {0};
        uint8_t dlc = 0;
        size_t hl = strlen(hex_data);
        if (hl > 0 && (hl % 2) == 0 && hl <= 16) {
            dlc = hl / 2;
            for (size_t i = 0; i < dlc; i++) {
                int hi = -1, lo = -1;
                char c1 = hex_data[i * 2], c2 = hex_data[i * 2 + 1];
                if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
                else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
                else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
                if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
                else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
                else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
                if (hi < 0 || lo < 0) return;
                data[i] = (uint8_t)((hi << 4) | lo);
            }
        }
        can_bus_send(id, data, dlc, ext_flag != 0);
        return;
    }
    if (strcmp(cmd, "can:standby") == 0) { can_bus_set_standby(true);  return; }
    if (strcmp(cmd, "can:normal")  == 0) { can_bus_set_standby(false); return; }
}

/* ============================================================
   on_node_status_from_transport (ROOT)
   ============================================================ */
static void on_node_status_from_transport(const char *json, size_t len)
{
    link_monitor_note_status_rx();
    if (g_config.role == DEVICE_ROLE_MASTER) {
        wifi_mqtt_set_root_comm_lost(false);
    }

    s_last_node_msg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t now = s_last_node_msg_ms;

    bool should_publish = (s_last_node_pub_ms == 0 ||
                           (now - s_last_node_pub_ms) >= 2000);
    if (should_publish) {
        s_last_node_pub_ms = now;
        ESP_LOGI(TAG, "STATUS→MQTT: len=%u mqtt=%d role_found=%d",
                 (unsigned)len,
                 wifi_mqtt_is_connected() ? 1 : 0,
                 strstr(json, "\"role\":\"node\"") ? 1 : 0);
        wifi_mqtt_publish_node_json(json, len);
    }

    if (g_config.role != DEVICE_ROLE_MASTER || !json || len == 0) return;

    {
        const char *lk = (const char *)memmem(json, len, "\"LINK\":{", 8);
        if (lk) {
            const char *root_p = (const char *)memmem(lk,
                                                      (json + len) - lk,
                                                      "\"root\":{\"rssi\":",
                                                      15);
            if (root_p) {
                float rssi = 0.0f, snr = 0.0f, sig_rssi = 0.0f;
                unsigned long age = 0;
                int valid = 0;
                if (sscanf(root_p,
                           "\"root\":{\"rssi\":%f,\"snr\":%f,"
                           "\"sig_rssi\":%f,\"age_ms\":%lu,\"valid\":%d",
                           &rssi, &snr, &sig_rssi, &age, &valid) >= 4) {
                    s_node_link_rssi     = rssi;
                    s_node_link_snr      = snr;
                    s_node_link_sig_rssi = sig_rssi;
                    s_node_link_age_ms   = (uint32_t)age;
                    s_node_link_valid    = (valid != 0) && (rssi != 0.0f);
                }
            }
        }
    }

    sync_handshake_root_on_rx(json, len);

    if (memmem(json, len, "\"TRAFFIC\":", 10) == NULL) {
        ESP_LOGW(TAG, "STATUS incomplete (no TRAFFIC section) - "
                      "keep s_node_running=%d", (int)s_node_running);
        return;
    }

    bool running = (memmem(json, len, "\"state\":\"RUNNING\"", 17) != NULL);
    bool was_running = s_node_running;
    s_node_running = running;

    if (running && !was_running) {
        ESP_LOGI(TAG, "STATUS: NODE RUNNING confirmed");
    } else if (!running && was_running) {
        ESP_LOGW(TAG, "STATUS: NODE no longer RUNNING");
    }

    if (running && !traffic_light_fault_has_priority()) {
        const char *ph_p = memmem(json, len, "\"phase\":", 8);
        const char *pm_p = memmem(json, len, "\"phase_ms\":", 11);
        if (ph_p && pm_p) {
            int node_phase = -1;
            unsigned long node_pm = 0;
            if (sscanf(ph_p + 8, "%d", &node_phase) == 1 &&
                sscanf(pm_p + 11, "%lu", &node_pm) == 1 &&
                node_phase >= 0) {

                traffic_state_t rst = traffic_light_get_state();
                if (rst == TRAFFIC_STATE_RUNNING) {
                    uint8_t  root_phase = traffic_light_get_phase();
                    uint32_t root_pm    = traffic_light_get_phase_ms();

                    int pm_diff = (int)node_pm - (int)root_pm;
                    if (pm_diff < 0) pm_diff = -pm_diff;

                    bool phase_mismatch = ((uint8_t)node_phase != root_phase);
                    bool pm_mismatch    = (pm_diff > 1500);

                    if (phase_mismatch || pm_mismatch) {
                        ESP_LOGW(TAG, "STATUS mismatch: node ph=%d pm=%lu, "
                                      "root ph=%u pm=%lu → force c:",
                                 node_phase, node_pm, root_phase, root_pm);
                        send_sync_to_node_async_ex(
                            root_phase, UINT32_MAX,
                            traffic_light_get_phase_duration_ms(),
                            true, NULL);
                    }
                }
            }
        }
    }

    const char *fp = memmem(json, len, "\"FAULT\":{", 9);
    if (fp) {
        const char *mp = memmem(fp, (json + len) - fp, "\"mode\":\"", 8);
        if (mp) {
            mp += 8;
            char mode[16] = {0};
            size_t k = 0;
            while (k < sizeof(mode) - 1 && (mp + k) < (json + len) &&
                   mp[k] != '"') {
                mode[k] = mp[k];
                k++;
            }
            mode[k] = '\0';

            if (strcmp(mode, "YELLOW_BLINK") == 0) {
                wifi_mqtt_apply_remote_fault("fault:yellow_blink:node", 24);
            } else if (strcmp(mode, "ALL_OFF") == 0) {
                wifi_mqtt_apply_remote_fault("fault:all_off:node", 19);
            }
        }
    }
}

static void on_node_channels_from_transport(const char *json, size_t len)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!json || len == 0 || len >= sizeof(s_node_channels_json)) {
        ESP_LOGW(TAG, "T_CHANNELS: bad len %u", (unsigned)len);
        return;
    }
    memcpy(s_node_channels_json, json, len);
    s_node_channels_json[len] = '\0';
    s_node_channels_valid = true;
    ESP_LOGI(TAG, "T_CHANNELS: received %u bytes from NODE", (unsigned)len);

    wifi_mqtt_publish_node_channels(json, len);
}

/* ============================================================
   on_evt_from_node (ROOT)
   ============================================================ */
static void on_evt_from_node(const char *evt, size_t len)
{
    char buf[128];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, evt, n);
    buf[n] = '\0';

    if (len >= 5 && strncmp(buf, "sync:", 5) == 0) {
        if (strncmp(buf, "sync:probe", 10) == 0) {
            sync_state_t rst = sync_handshake_root_state();
            if (rst == SYNC_STATE_ALIGNED) {
                transport_send_cmd_noack("sync:ack", 8);
                return;
            }
            if (rst == SYNC_STATE_IDLE) {
                ESP_LOGW(TAG, "ROOT: got sync:probe while IDLE → start PROBE");
                sync_handshake_root_start();
                transport_send_cmd_noack("sync:ack", 8);
            } else {
                transport_send_cmd_noack("sync:ack", 8);
            }
            return;
        }
        sync_handshake_root_on_rx(buf, n);
        return;
    }

    if (len > 5 && strncmp(buf, "pong:", 5) == 0) return;
    if (len > 5 && strncmp(buf, "time:", 5) == 0) return;

    if (len > 6 && strncmp(buf, "fault:", 6) == 0) {
        if (strncmp(buf, "fault:recover_ok", 16) == 0) {
            if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) {
                ESP_LOGW(TAG, "NODE recover_ok but ROOT still in fault - ignore");
                return;
            }
            ESP_LOGW(TAG, "NODE recover_ok, ROOT ok -> send fault:none");
            transport_send_cmd_noack("fault:none", 10);
            return;
        }

        if (strncmp(buf, "fault:local:", 12) == 0) {
            const char *mode = buf + 12;
            traffic_fault_t nf = TRAFFIC_FAULT_NONE;
            if (strcmp(mode, "YELLOW_BLINK") == 0) {
                nf = TRAFFIC_FAULT_YELLOW_BLINK;
            } else if (strcmp(mode, "ALL_OFF") == 0) {
                nf = TRAFFIC_FAULT_ALL_OFF;
            }
            if (nf != TRAFFIC_FAULT_NONE) {
                ESP_LOGW(TAG, "NODE local fault '%s' → ROOT applies %s",
                         mode,
                         nf == TRAFFIC_FAULT_ALL_OFF
                             ? "ALL_OFF" : "YELLOW_BLINK");
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(nf);
                SET_REMOTE_FLAG(false);
                return;
            }
        }

        SET_REMOTE_FLAG(true);
        wifi_mqtt_apply_remote_fault(evt, len);
        SET_REMOTE_FLAG(false);
        return;
    }

    if (strcmp(buf, "pedestrian:request") == 0) {
        if (!traffic_light_ped_button_enabled()) return;
        if (traffic_light_fault_has_priority()) {
            ESP_LOGW(TAG, "pedestrian:request from NODE ignored (fault active)");
            return;
        }
        ESP_LOGW(TAG, "pedestrian:request from NODE -> apply locally on ROOT");
        traffic_light_request_pedestrian_ex(true);
        return;
    }

    if (strncmp(buf, "status:autonomous", 17) == 0) return;
}

/* ============================================================
   node_send_task
   ============================================================ */
static void node_send_task(void *arg)
{
    char *json = malloc(NODE_STATUS_BUF_SIZE);
    if (!json) { vTaskDelete(NULL); return; }

    ESP_LOGI(TAG, "node_send_task started (transport=%s, buf=%u, RT=UTC)",
             transport_is_lora() ? "LoRa" : "TCP",
             (unsigned)NODE_STATUS_BUF_SIZE);
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        if (!transport_is_tx_idle()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        bool send_channels = (s_last_channels_ms == 0 ||
                              (now_ms - s_last_channels_ms) >= NODE_CHANNELS_PERIOD_MS);
        if (send_channels && transport_is_lora()) {
            s_last_channels_ms = now_ms;

            mux_data_t md;
            bool have_md = mux_data_get(&md);

            int off = snprintf(json, NODE_STATUS_BUF_SIZE,
                               "{\"role\":\"node\",\"CHANNELS\":[");
            if (have_md) {
                for (uint8_t ch = 0; ch < 16; ch++) {
                    off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
                                    "%s{\"ch\":%u,\"V_IN\":%.2f,\"I_LOAD\":%d,"
                                    "\"V_IMON\":%d,\"STATE\":\"%s\","
                                    "\"SR\":%d,\"DIAG\":%d}",
                                    (ch == 0) ? "" : ",",
                                    ch, md.v_in[ch], md.i_load[ch],
                                    md.imon_mv[ch],
                                    imon_state_str((imon_state_t)md.imon_state[ch]),
                                    md.sr_state[ch], md.diag_en[ch]);
                }
            }
            off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off, "]}");

            esp_err_t err = transport_send_channels(json, off);
            ESP_LOGI(TAG, "T_CHANNELS -> ROOT: %s (%d bytes)",
                     err == ESP_OK ? "OK" : esp_err_to_name(err), off);

            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        uint16_t sr_state = sr_get_state();
        const char *state_str = traffic_light_state_str();
        const char *fault_str = traffic_light_fault_str();
        uint8_t fault_cnt = traffic_light_get_fault_count();

        lora_link_stats_t ls = {0};
        bool have_ls = lora_get_last_rx_stats(&ls);

        rs485_stats_t rs;
        rs485_get_stats(&rs);
        can_bus_stats_t cs;
        can_bus_get_stats(&cs);

        int off = 0;

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "{\"SR\":{\"state\":%u,\"hex\":\"0x%04X\",\"outputs\":[",
            sr_state, sr_state);
        bool first = true;
        for (int i = 0; i < SR_OUTPUT_COUNT; i++) {
            if (sr_state & (1U << i)) {
                off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
                                "%s%d", first ? "" : ",", i);
                first = false;
            }
        }
        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off, "]},");

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"NET\":{\"role\":\"node\",\"ppp\":0,\"ip\":\"0.0.0.0\"},");

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"TRAFFIC\":{\"state\":\"%s\",\"mode\":%d,\"phase\":%d,"
            "\"phase_ms\":%lu,\"phase_duration_ms\":%lu,\"phase_count\":%u},",
            state_str, traffic_light_get_mode(),
            traffic_light_get_phase(),
            (unsigned long)traffic_light_get_phase_ms(),
            (unsigned long)traffic_light_get_phase_duration_ms(),
            traffic_light_get_phase_count());

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"FAULT\":{\"mode\":\"%s\",\"count\":%u,\"channels\":[",
            fault_str, fault_cnt);
        for (uint8_t i = 0; i < fault_cnt && i < TRAFFIC_FAULT_MAX; i++) {
            traffic_fault_info_t info;
            traffic_light_get_fault_info(i, &info);
            off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off, "%s%u",
                            (i == 0) ? "" : ",", info.channel);
        }
        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off, "],\"reasons\":[");
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
            off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
                            "%s\"%s\"", (i == 0) ? "" : ",", safe);
        }
        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off, "]},");

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"LINK\":{\"root\":{\"rssi\":%.1f,\"snr\":%.1f,"
            "\"sig_rssi\":%.1f,\"age_ms\":%lu,\"valid\":%d},"
            "\"node\":{\"rssi\":0.0,\"snr\":0.0,\"sig_rssi\":0.0,"
            "\"age_ms\":0,\"valid\":0}},",
            have_ls ? ls.rssi : 0.0f,
            have_ls ? ls.snr : 0.0f,
            have_ls ? ls.sig_rssi : 0.0f,
            (unsigned long)(have_ls ? ls.age_ms : 0),
            have_ls ? 1 : 0);

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"NODE\":null,\"NODE_FAULT\":null,");

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"RS485\":{\"rx_bytes\":%lu,\"rx_frames\":%lu,\"rx_overflows\":%lu,"
            "\"tx_bytes\":%lu,\"tx_errors\":%lu},",
            (unsigned long)rs.rx_bytes, (unsigned long)rs.rx_frames,
            (unsigned long)rs.rx_overflows, (unsigned long)rs.tx_bytes,
            (unsigned long)rs.tx_errors);

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"CAN\":{\"rx_frames\":%lu,\"tx_frames\":%lu,\"tx_errors\":%lu,"
            "\"bus_off\":%lu,\"recovery\":%lu,\"error_passive\":%lu,\"state\":%lu},",
            (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
            (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
            (unsigned long)cs.recovery_count,
            (unsigned long)cs.error_passive_count,
            (unsigned long)cs.last_state);

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
            "\"RTT\":{\"us\":0,\"delay_us\":0,\"samples\":0}");

        off += snprintf(json + off, NODE_STATUS_BUF_SIZE - off,
                        ",\"role\":\"node\"}");

        if (off <= 0 || off >= NODE_STATUS_BUF_SIZE) {
            ESP_LOGE(TAG, "STATUS build overflow (off=%d), skip", off);
            vTaskDelay(pdMS_TO_TICKS(NODE_STATUS_PERIOD_MS));
            continue;
        }

        bool force = s_status_now ||
                     (s_status_force_until_ms != 0 &&
                      now_ms < s_status_force_until_ms);
        if (s_status_now) s_status_now = false;
        if (s_status_force_until_ms != 0 &&
            now_ms >= s_status_force_until_ms) {
            s_status_force_until_ms = 0;
        }

        if (force) {
            esp_err_t err = transport_send_status_noack(json, off);
            ESP_LOGI(TAG, "STATUS forced -> ROOT: %s (%d bytes)",
                     err == ESP_OK ? "OK" : esp_err_to_name(err), off);
        } else {
            esp_err_t err = transport_send_status(json, off);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "STATUS failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "STATUS -> ROOT: OK (%d bytes)", off);
            }
        }

        uint32_t period;
        if (force)                                     period = 200;
        else if (sync_handshake_node_is_syncing())     period = NODE_STATUS_SYNC_MS;
        else if (traffic_light_get_fault() != TRAFFIC_FAULT_NONE) period = NODE_STATUS_FAULT_MS;
        else                                           period = NODE_STATUS_PERIOD_MS;
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

/* ============================================================
   node_link_watchdog_task
   ============================================================ */
static void node_link_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "node_link_watchdog_task started (timeout=%d ms)",
             NODE_ROOT_LOST_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_config.role != DEVICE_ROLE_SLAVE) continue;

        uint32_t now_tick = (uint32_t)(esp_timer_get_time() / 1000);
        sync_handshake_node_tick(now_tick);
        if (sync_handshake_node_is_syncing()) {
            continue;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (!s_node_running) continue;

        if (!s_c_received_since_start) {
            if (s_last_root_data_ms != 0 &&
                (now - s_last_root_data_ms) >= (uint32_t)NODE_ROOT_LOST_MS &&
                !s_root_lost_flag) {
                ESP_LOGE(TAG, "NODE RUNNING but no c: for %lu ms "
                              "-> ALL_OFF + RECOVERING + sync_handshake",
                         (unsigned long)(now - s_last_root_data_ms));
                s_root_lost_flag = true;
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                SET_REMOTE_FLAG(false);
                traffic_light_set_state_recovering();
                s_node_running = false;
                sync_handshake_node_start();
            }
            continue;
        }

        if (s_last_root_data_ms == 0) continue;

        uint32_t silence = now - s_last_root_data_ms;

        if (silence >= NODE_ROOT_LOST_MS && !s_root_lost_flag) {
            ESP_LOGE(TAG, "ROOT silent %lu ms -> ALL_OFF + RECOVERING "
                          "+ sync_handshake",
                     (unsigned long)silence);
            s_root_lost_flag = true;
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            traffic_light_set_state_recovering();
            s_node_running = false;
            sync_handshake_node_start();
        }
    }
}

/* ============================================================
   Register callbacks / start tasks
   ============================================================ */
void node_proto_register_callbacks(void)
{
    if (g_config.role == DEVICE_ROLE_SLAVE) {
        transport_set_cmd_cb(on_cmd_from_root);
    } else {
        transport_set_evt_cb(on_evt_from_node);
        transport_set_status_cb(on_node_status_from_transport);
        transport_set_channels_cb(on_node_channels_from_transport);
    }
}

void node_proto_client_start(const char *host, uint16_t port)
{
    (void)host; (void)port;
    transport_init();
}

void node_proto_start_tasks(void)
{
    /* === FIX RT-CHANNELS: mutex + scheduler task === */
    if (!s_pending_mask_mtx) {
        s_pending_mask_mtx = xSemaphoreCreateMutex();
    }
    /* === END FIX RT-CHANNELS === */

    xTaskCreate(node_send_task,           "node_send", 4096, NULL, 4, NULL);
    xTaskCreate(node_link_watchdog_task,  "node_wd",   3072, NULL, 3, NULL);
    /* === FIX RT-CHANNELS === */
    xTaskCreate(node_mask_scheduler_task, "node_msk",  3072, NULL, 5, NULL);
    /* === END FIX RT-CHANNELS === */
}

void node_proto_start_root_tasks(void)
{
    ESP_LOGI(TAG, "node_proto_start_root_tasks: no-op "
             "(heartbeat lives in root_wd.c)");
}