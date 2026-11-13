/* ============================================================
   sync_handshake.c — ускоренное восстановление LoRa-связи
   с ритуалом синхронизации и синхронным миганием ЖДИТЕ.

   === FIX SYNC-PONG-UTC ===
   NODE в sync:pong возвращает pm_utc, пересчитанный через
   end_utc_ms из sync:lock. ROOT сравнивает по той же UTC-шкале.
   ToA/RTT/обработка исключены из drift.

   === FIX FAULT-HAS-TOP-PRIORITY ===
   Если s_fault != NONE (YELLOW_BLINK или ALL_OFF), ритуал
   sync:probe/time/lock НЕ запускается: авария имеет наивысший
   приоритет. В ALIGNED-ветке traffic:start + start_with_red
   пропускаются при активной аварии. По завершении таймаута
   ALIGNED при аварии — только sync:done + IDLE, без снятия
   fault и без старта.

   === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
   Перед traffic:start в ветках ALIGNED-timeout и disp-synced
   ROOT отправляет явный fault:none + пауза. NODE мог сам уйти
   в ALL_OFF из-за потери ROOT (node_link_watchdog_task) и
   блокирует traffic:start/c: через
   traffic_light_fault_has_priority() без fault:none.
   ============================================================ */

#include "sync_handshake.h"
#include "app_shared.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "traffic_internal.h"
#include "transport.h"
#include "root_wd.h"
#include "time_sync.h"
#include "shift_register.h"
#include "lora.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "sync_hs";

extern volatile bool     s_node_running;
extern volatile uint32_t s_last_node_msg_ms;

/* === FIX SYNC-PONG-UTC ===
   Fallback-порог для локального pm (когда UTC недоступен).
   Покрывает ToA (112 мс SF8/BW250 для 40 байт) + RTT + обработку. */
#define SYNC_ALIGN_DRIFT_LOCAL_MS    400
/* === END FIX SYNC-PONG-UTC === */

/* === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
   Пауза между fault:none и traffic:start — чтобы LoRa успел
   доставить первый пакет (ToA 10 байт ≈ 40-50 мс SF8/BW250). */
#define FAULT_NONE_TO_START_GAP_MS   80
/* === END FIX ROOT-RETURN-SENDS-FAULT-NONE === */

/* ============================================================
   Общее: мигание ЖДИТЕ + выключение остальных
   ============================================================ */

static volatile bool     s_display_blink_on   = false;
static volatile uint32_t s_display_blink_last = 0;
static volatile bool     s_display_cleared    = false;
static volatile bool     s_display_use_utc    = false;

static void display_only_blink(uint32_t now_ms, bool use_utc)
{
    uint8_t ch = g_config.traffic.pins.cross_ped.ped_display;
    if (ch >= 16) ch = SYNC_DISPLAY_CH;

    if (!s_display_cleared) {
        for (int i = 0; i < 16; i++) {
            if (i != ch) sr_output_off(i);
        }
        s_display_cleared = true;
        ESP_LOGW(TAG, "SYNC: all channels OFF except display ch%02d", ch);
    }

    bool want_on;

    if (use_utc) {
        uint64_t utc = time_sync_get_utc_ms();
        if (utc != 0) {
            want_on = ((utc / SYNC_DISPLAY_BLINK_MS) & 1ULL) == 0;
            s_display_use_utc = true;
        } else {
            want_on = s_display_blink_on;
            if (s_display_blink_last == 0 ||
                (now_ms - s_display_blink_last) >= SYNC_DISPLAY_BLINK_MS) {
                s_display_blink_last = now_ms;
                want_on = !s_display_blink_on;
            }
        }
    } else {
        want_on = s_display_blink_on;
        if (s_display_blink_last == 0 ||
            (now_ms - s_display_blink_last) >= SYNC_DISPLAY_BLINK_MS) {
            s_display_blink_last = now_ms;
            want_on = !s_display_blink_on;
        }
    }

    if (want_on != s_display_blink_on) {
        s_display_blink_on = want_on;
        if (s_display_blink_on) sr_output_on(ch);
        else                    sr_output_off(ch);
    }
}

static void display_only_stop(void)
{
    uint8_t ch = g_config.traffic.pins.cross_ped.ped_display;
    if (ch >= 16) ch = SYNC_DISPLAY_CH;
    sr_output_off(ch);
    s_display_blink_on   = false;
    s_display_blink_last = 0;
    s_display_cleared    = false;
    s_display_use_utc    = false;
    ESP_LOGI(TAG, "SYNC: display blink stopped");
}

static inline void display_force_off(void)
{
    uint8_t ch = g_config.traffic.pins.cross_ped.ped_display;
    if (ch >= 16) ch = SYNC_DISPLAY_CH;
    sr_output_off(ch);
}

/* === FIX SYNC-PONG-UTC: elapsed_ms по UTC === */
static uint32_t utc_elapsed_ms(uint64_t end_utc_ms, uint32_t dur_ms)
{
    if (end_utc_ms == 0 || dur_ms == 0) return 0;
    uint64_t now_utc = time_sync_get_utc_ms();
    if (now_utc == 0) return 0;
    if (now_utc >= end_utc_ms) return dur_ms;
    uint64_t remain = end_utc_ms - now_utc;
    if (remain > dur_ms) remain = dur_ms;
    return (uint32_t)(dur_ms - remain);
}
/* === END FIX SYNC-PONG-UTC === */

/* ============================================================
   ROOT
   ============================================================ */

static volatile sync_state_t s_root_state = SYNC_STATE_IDLE;
static volatile uint32_t     s_root_state_ms = 0;
static volatile uint32_t     s_root_last_probe_ms = 0;
static volatile uint32_t     s_root_last_time_ms  = 0;
static volatile uint32_t     s_root_last_lock_ms  = 0;
static volatile uint8_t      s_root_time_sent = 0;
static volatile uint8_t      s_root_lock_sent = 0;
static volatile uint8_t      s_root_aligned_streak = 0;
static volatile int32_t      s_root_last_drift_ms = 0;
static volatile uint32_t     s_root_sync_started_ms = 0;

static volatile uint32_t     s_root_node_pm = 0;
static volatile uint8_t      s_root_node_phase = 0xFF;
static volatile uint32_t     s_root_node_pm_ms = 0;
static volatile bool         s_root_node_status_fresh = false;

/* === FIX SYNC-PONG-UTC: параметры последнего lock === */
static volatile uint64_t     s_root_lock_end_utc_ms = 0;
static volatile uint32_t     s_root_lock_dur_ms     = 0;
static volatile uint32_t     s_root_lock_phase      = 0;
/* === END FIX SYNC-PONG-UTC === */

static volatile uint8_t  s_root_node_disp       = 0xFF;
static volatile uint32_t s_root_node_disp_ms    = 0;
static volatile uint8_t  s_root_disp_streak     = 0;

static uint8_t root_current_disp_bit(void)
{
    uint64_t utc = time_sync_get_utc_ms();
    if (utc == 0) return 0xFF;
    return (uint8_t)((utc / SYNC_DISPLAY_BLINK_MS) & 1ULL);
}

static volatile uint32_t s_root_last_phase_time_ms = 0;

sync_state_t sync_handshake_root_state(void) { return s_root_state; }
bool sync_handshake_root_is_syncing(void) { return s_root_state != SYNC_STATE_IDLE; }

static void root_set_state(sync_state_t st)
{
    if (s_root_state == st) return;
    ESP_LOGW(TAG, "ROOT sync: %d → %d", (int)s_root_state, (int)st);
    s_root_state = st;
    s_root_state_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_root_aligned_streak = 0;
    s_root_disp_streak = 0;
}

void sync_handshake_root_start(void)
{
    /* === FIX FAULT-HAS-TOP-PRIORITY ===
       При активной аварии ритуал sync:probe НЕ запускаем: каналы
       принадлежат аварии. Как только авария снята (try_recovery
       или ручной traffic:start), ритуал стартует. */
    if (traffic_light_fault_has_priority()) {
        ESP_LOGW(TAG, "ROOT sync: START ignored (fault active: %s)",
                 traffic_light_fault_str());
        return;
    }
    /* === END FIX FAULT-HAS-TOP-PRIORITY === */

    for (int i = 0; i < 16; i++) sr_output_off(i);   // все выключены
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (s_root_state != SYNC_STATE_IDLE) return;

    ESP_LOGW(TAG, "ROOT sync: START (probe)");
    s_root_sync_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_root_last_probe_ms = 0;
    s_root_time_sent = 0;
    s_root_lock_sent = 0;
    s_root_aligned_streak = 0;
    s_root_node_status_fresh = false;
    s_root_lock_end_utc_ms = 0;
    s_root_lock_dur_ms = 0;
    s_root_lock_phase = 0;

    s_root_node_disp    = 0xFF;
    s_root_node_disp_ms = 0;
    s_root_disp_streak  = 0;

    for (int i = 0; i < 16; i++) sr_output_off(i);

    s_display_cleared  = false;
    s_display_blink_on = false;
    s_display_blink_last = 0;

    s_insert_pending  = false;
    s_ped_request     = false;
    s_ped_display_done = false;

    root_set_state(SYNC_STATE_PROBE);
}

void sync_handshake_root_on_rx(const char *evt, size_t len)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!evt || len == 0) return;

    char buf[160];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, evt, n);
    buf[n] = '\0';

    if (strncmp(buf, "sync:ack", 8) == 0) {
        if (s_root_state == SYNC_STATE_PROBE) {
            ESP_LOGW(TAG, "ROOT sync: got ack in PROBE → TIME");
            s_root_time_sent = 0;
            s_root_last_time_ms = 0;
            root_set_state(SYNC_STATE_TIME);
        } else if (s_root_state == SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "ROOT sync: got ack in IDLE → start PROBE");
            sync_handshake_root_start();
            if (s_root_state == SYNC_STATE_PROBE) {
                s_root_time_sent = 0;
                s_root_last_time_ms = 0;
                root_set_state(SYNC_STATE_TIME);
            }
        } else if (s_root_state == SYNC_STATE_PHASE) {
            ESP_LOGW(TAG, "ROOT sync: got ack in PHASE → resend time:");
            s_root_time_sent = 0;
            s_root_last_time_ms = 0;
            root_set_state(SYNC_STATE_TIME);
        } else if (s_root_state == SYNC_STATE_TIME) {
            ESP_LOGD(TAG, "ROOT sync: got ack in TIME → ignore");
        }
        return;
    }

    if (strncmp(buf, "sync:probe", 10) == 0) {
        if (s_root_state == SYNC_STATE_ALIGNED) return;
        if (s_root_state == SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "ROOT sync: got probe in IDLE → start PROBE");
            sync_handshake_root_start();
        }
        transport_send_cmd_noack("sync:ack", 8);
        return;
    }

    if (strncmp(buf, "sync:pong:", 10) == 0) {
        unsigned int ts = 0, t_recv = 0, pm = 0, phase = 0;
        if (sscanf(buf + 10, "%u:%u:%u:%u", &ts, &t_recv, &pm, &phase) == 4) {
            s_root_node_pm       = pm;
            s_root_node_phase    = (uint8_t)phase;
            s_root_node_pm_ms    = (uint32_t)(esp_timer_get_time() / 1000);
            s_root_node_status_fresh = true;

            if (s_root_state == SYNC_STATE_PHASE) {
                bool utc_used = false;
                int32_t drift = 0;

                if (s_root_lock_end_utc_ms != 0 &&
                    s_root_lock_dur_ms != 0) {
                    uint32_t my_pm_utc = utc_elapsed_ms(
                        s_root_lock_end_utc_ms, s_root_lock_dur_ms);
                    if (my_pm_utc != 0 || pm != 0) {
                        drift = (int32_t)pm - (int32_t)my_pm_utc;
                        utc_used = true;
                        ESP_LOGD(TAG, "ROOT sync: pong pm_utc=%u my_pm_utc=%u "
                                      "drift=%ld [utc]",
                                 pm, my_pm_utc, (long)drift);
                    }
                }

                if (!utc_used) {
                    uint32_t my_pm = traffic_light_get_phase_ms();
                    drift = (int32_t)pm - (int32_t)my_pm;
                    ESP_LOGW(TAG, "ROOT sync: pong drift=%ld [local] "
                                  "(UTC unavailable)",
                             (long)drift);
                }

                if (drift < 0) drift = -drift;
                s_root_last_drift_ms = drift;

                int32_t threshold = utc_used ? SYNC_ALIGN_DRIFT_MS
                                             : SYNC_ALIGN_DRIFT_LOCAL_MS;

                if (drift <= threshold) {
                    s_root_aligned_streak++;
                    ESP_LOGI(TAG, "ROOT sync: pong drift=%ld ms "
                                  "(streak %u/%u)%s",
                             (long)drift, s_root_aligned_streak,
                             SYNC_ALIGN_STREAK,
                             utc_used ? " [utc]" : " [local]");
                    if (s_root_aligned_streak >= SYNC_ALIGN_STREAK) {
                        ESP_LOGW(TAG, "ROOT sync: ALIGNED (drift %ld ms)",
                                 (long)drift);
                        root_set_state(SYNC_STATE_ALIGNED);
                    }
                } else {
                    s_root_aligned_streak = 0;
                    ESP_LOGW(TAG, "ROOT sync: pong drift=%ld ms > %d → re-lock",
                             (long)drift, threshold);
                    s_root_lock_sent = 0;
                }
            }
        }
        return;
    }

    if (buf[0] == '{' && memmem(buf, n, "\"phase_ms\":", 11)) {
        const char *pm_p = memmem(buf, n, "\"phase_ms\":", 11);
        const char *ph_p = memmem(buf, n, "\"phase\":", 8);
        if (pm_p) {
            unsigned long pm = 0;
            if (sscanf(pm_p + 11, "%lu", &pm) == 1) {
                s_root_node_pm = (uint32_t)pm;
                s_root_node_pm_ms = (uint32_t)(esp_timer_get_time() / 1000);
                s_root_node_status_fresh = true;
            }
        }
        if (ph_p) {
            int ph = 0;
            if (sscanf(ph_p + 8, "%d", &ph) == 1 && ph >= 0 && ph < 256) {
                s_root_node_phase = (uint8_t)ph;
            }
        }

        const char *dp = memmem(buf, n, "\"disp\":", 7);
        if (dp) {
            int nb = -1;
            if (sscanf(dp + 7, "%d", &nb) == 1 && (nb == 0 || nb == 1)) {
                s_root_node_disp    = (uint8_t)nb;
                s_root_node_disp_ms = (uint32_t)(esp_timer_get_time() / 1000);

                uint8_t want = root_current_disp_bit();
                if (want == 0xFF) {
                    s_root_disp_streak = 0;
                } else if (nb == (int)want) {
                    if (s_root_disp_streak < 0xFF) s_root_disp_streak++;
                    ESP_LOGD(TAG, "ROOT sync: disp match bit=%d streak=%u/%u",
                             nb, s_root_disp_streak,
                             SYNC_DISPLAY_STREAK_MIN);
                } else {
                    if (s_root_disp_streak > 0) {
                        ESP_LOGW(TAG, "ROOT sync: disp MISMATCH node=%d "
                                      "root=%u → streak reset",
                                 nb, want);
                    }
                    s_root_disp_streak = 0;
                }
            }
        }
    }
}

void sync_handshake_root_tick(uint32_t now_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (s_root_state == SYNC_STATE_IDLE) return;

    /* === FIX FAULT-HAS-TOP-PRIORITY ===
       При активной аварии ритуал не трогает каналы: авария
       управляет ими. Форсим IDLE без sync:done и без start. */
    if (traffic_light_fault_has_priority()) {
        if (s_root_state != SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "ROOT sync: fault active (%s) → force IDLE "
                          "(no traffic:start)",
                     traffic_light_fault_str());
            display_only_stop();
            display_force_off();
            s_root_state = SYNC_STATE_IDLE;
            s_root_state_ms = now_ms;
        }
        return;
    }
    /* === END FIX FAULT-HAS-TOP-PRIORITY === */

    bool use_utc = (s_root_state >= SYNC_STATE_TIME);
    display_only_blink(now_ms, use_utc);

    /* Глобальный таймаут — только для PROBE и TIME (не для ALIGNED) */
    if (s_root_sync_started_ms != 0 &&
        (now_ms - s_root_sync_started_ms) > SYNC_ALIGN_TIMEOUT_MS) {
        if (s_root_state == SYNC_STATE_PROBE) {
            ESP_LOGE(TAG, "ROOT sync: PROBE timeout %u ms → force IDLE "
                          "+ traffic:start",
                     (unsigned)(now_ms - s_root_sync_started_ms));
            display_only_stop();
            display_force_off();
            /* === FIX FAULT-HAS-TOP-PRIORITY ===
               На случай, если fault поднялся ровно между проверкой
               выше и этим блоком — не стартуем. */
            if (!traffic_light_fault_has_priority()) {
                /* === FIX ROOT-RETURN-SENDS-FAULT-NONE === */
                transport_send_cmd_noack("fault:none", strlen("fault:none"));
                vTaskDelay(pdMS_TO_TICKS(FAULT_NONE_TO_START_GAP_MS));
                /* === END FIX === */

                s_fault_from_remote_ex = true;
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                s_fault_from_remote_ex = false;
                traffic_light_set_follow_root(false);
                traffic_light_start_with_red();
                transport_send_cmd_noack("traffic:start", strlen("traffic:start"));
                uint8_t ph = traffic_light_get_phase();
                send_sync_to_node_async_ex(ph, 0,
                    traffic_light_get_phase_duration_ms(), true, NULL);
            }
            root_set_state(SYNC_STATE_IDLE);
            return;
        } else if (s_root_state != SYNC_STATE_ALIGNED) {
            ESP_LOGE(TAG, "ROOT sync: TIMEOUT %u ms in state %d → restart probe",
                     (unsigned)(now_ms - s_root_sync_started_ms),
                     (int)s_root_state);
            s_root_sync_started_ms = now_ms;
            s_root_time_sent = 0;
            s_root_lock_sent = 0;
            s_root_aligned_streak = 0;
            root_set_state(SYNC_STATE_PROBE);
        }
    }

    switch (s_root_state) {
    case SYNC_STATE_PROBE:
        if (s_root_last_probe_ms == 0 ||
            (now_ms - s_root_last_probe_ms) >= SYNC_PROBE_INTERVAL_MS) {
            s_root_last_probe_ms = now_ms;
            transport_send_cmd_noack("sync:probe", 10);
        }
        break;

    case SYNC_STATE_TIME:
        if (s_root_time_sent < SYNC_TIME_TRIES) {
            if (s_root_last_time_ms == 0 ||
                (now_ms - s_root_last_time_ms) >= 500) {
                s_root_last_time_ms = now_ms;
                uint32_t sec = 0; uint16_t ms = 0;
                if (time_sync_get(&sec, &ms)) {
                    char tcmd[64];
                    int tn = snprintf(tcmd, sizeof(tcmd), "time:%lu:%u:0",
                                      (unsigned long)sec, (unsigned)ms);
                    if (tn > 0) transport_send_cmd_noack(tcmd, tn);
                }
                s_root_time_sent++;
                if (s_root_time_sent >= SYNC_TIME_TRIES) {
                    ESP_LOGW(TAG, "ROOT sync: time sent %u/%u → PHASE",
                             s_root_time_sent, SYNC_TIME_TRIES);
                    s_root_lock_sent = 0;
                    s_root_last_lock_ms = 0;
                    s_root_last_phase_time_ms = now_ms;
                    root_set_state(SYNC_STATE_PHASE);
                }
            }
        }
        break;

    case SYNC_STATE_PHASE: {
        /* Resend time: в PHASE (для тех, кто пропустил) */
        if (s_root_last_phase_time_ms == 0 ||
            (now_ms - s_root_last_phase_time_ms) >= 1000) {
            s_root_last_phase_time_ms = now_ms;
            uint32_t sec = 0; uint16_t ms = 0;
            if (time_sync_get(&sec, &ms)) {
                char tcmd[64];
                int tn = snprintf(tcmd, sizeof(tcmd), "time:%lu:%u:0",
                                  (unsigned long)sec, (unsigned)ms);
                if (tn > 0) transport_send_cmd_noack(tcmd, tn);
            }
        }

        if (s_root_lock_sent < SYNC_LOCK_TRIES) {
            if (s_root_last_lock_ms == 0 ||
                (now_ms - s_root_last_lock_ms) >= 500) {
                s_root_last_lock_ms = now_ms;
                uint8_t  ph  = traffic_light_get_phase();
                uint32_t pm  = traffic_light_get_phase_ms();
                uint32_t dur = traffic_light_get_phase_duration_ms();
                uint64_t utc = time_sync_get_utc_ms();
                uint64_t end_utc = utc + (dur > pm ? dur - pm : dur);

                s_root_lock_end_utc_ms = end_utc;
                s_root_lock_dur_ms     = dur;
                s_root_lock_phase      = ph;

                char lcmd[128];
                int ln = snprintf(lcmd, sizeof(lcmd),
                                  "sync:lock:%u:%lu:%lu:%llu",
                                  ph, (unsigned long)pm,
                                  (unsigned long)dur,
                                  (unsigned long long)end_utc);
                if (ln > 0) transport_send_cmd_noack(lcmd, ln);

                s_root_lock_sent++;
                if (s_root_lock_sent >= SYNC_LOCK_TRIES &&
                    s_root_aligned_streak == 0) {
                    ESP_LOGW(TAG, "ROOT sync: no drift match after %u locks "
                                  "→ restart TIME", SYNC_LOCK_TRIES);
                    s_root_time_sent = 0;
                    s_root_last_time_ms = 0;
                    s_root_lock_sent = 0;
                    s_root_last_lock_ms = 0;
                    root_set_state(SYNC_STATE_TIME);
                }
            }
        }
        break;
    }

    case SYNC_STATE_ALIGNED: {
        if ((now_ms - s_root_state_ms) >= SYNC_ALIGN_TIMEOUT_MS) {
            ESP_LOGE(TAG, "ROOT sync: ALIGNED timeout %u ms "
                          "(disp_streak=%u) → force IDLE + traffic:start",
                     (unsigned)(now_ms - s_root_state_ms),
                     s_root_disp_streak);

            transport_send_cmd_noack("sync:done", 9);

            /* === FIX FAULT-HAS-TOP-PRIORITY ===
               traffic:start + start_with_red только если авария
               не поднялась во время ALIGNED. */
            if (!traffic_light_fault_has_priority()) {
                /* === FIX ROOT-RETURN-SENDS-FAULT-NONE ===
                   NODE мог сам уйти в ALL_OFF из-за потери ROOT.
                   Шлём fault:none ПЕРЕД traffic:start, с паузой. */
                transport_send_cmd_noack("fault:none", strlen("fault:none"));
                vTaskDelay(pdMS_TO_TICKS(FAULT_NONE_TO_START_GAP_MS));
                /* === END FIX === */

                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                s_fault_from_remote_ex = true;
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                s_fault_from_remote_ex = false;
                traffic_light_set_follow_root(false);
                traffic_light_start_with_red();
                uint8_t ph = traffic_light_get_phase();
                send_sync_to_node_async_ex(
                    ph, 0,
                    traffic_light_get_phase_duration_ms(),
                    true, NULL);
            } else {
                ESP_LOGW(TAG, "ROOT sync: ALIGNED timeout but fault active "
                              "(%s) — skip start",
                         traffic_light_fault_str());
            }

            display_only_stop();
            display_force_off();
            root_set_state(SYNC_STATE_IDLE);
            break;
        }

        if (s_root_last_phase_time_ms == 0 ||
            (now_ms - s_root_last_phase_time_ms) >= 1000) {
            s_root_last_phase_time_ms = now_ms;
            uint32_t sec = 0; uint16_t ms = 0;
            if (time_sync_get(&sec, &ms)) {
                char tcmd[64];
                int tn = snprintf(tcmd, sizeof(tcmd), "time:%lu:%u:0",
                                  (unsigned long)sec, (unsigned)ms);
                if (tn > 0) transport_send_cmd_noack(tcmd, tn);
            }
        }

        static uint32_t s_last_display_sync_ms = 0;
        if (s_last_display_sync_ms == 0 ||
            (now_ms - s_last_display_sync_ms) >= SYNC_DISPLAY_RESYNC_MS) {
            s_last_display_sync_ms = now_ms;

            uint64_t utc = time_sync_get_utc_ms();
            if (utc != 0) {
                uint8_t  phase_bit = (uint8_t)((utc / SYNC_DISPLAY_BLINK_MS)
                                               & 1ULL);
                uint64_t next_edge = ((utc / SYNC_DISPLAY_BLINK_MS) + 1)
                                     * SYNC_DISPLAY_BLINK_MS;
                char dcmd[64];
                int dn = snprintf(dcmd, sizeof(dcmd),
                                  "sync:display:%u:%llu",
                                  phase_bit,
                                  (unsigned long long)next_edge);
                if (dn > 0) transport_send_cmd_noack(dcmd, dn);
            }
        }

        bool disp_synced = (s_root_disp_streak >= SYNC_DISPLAY_STREAK_MIN);

        if (disp_synced) {
            ESP_LOGW(TAG, "ROOT sync: NODE disp synced (streak=%u/%u) → "
                          "traffic:start + sync:done + c: → IDLE",
                     s_root_disp_streak, SYNC_DISPLAY_STREAK_MIN);

            transport_send_cmd_noack("sync:done", 9);

            /* === FIX FAULT-HAS-TOP-PRIORITY === */
            if (!traffic_light_fault_has_priority()) {
                /* === FIX ROOT-RETURN-SENDS-FAULT-NONE === */
                transport_send_cmd_noack("fault:none", strlen("fault:none"));
                vTaskDelay(pdMS_TO_TICKS(FAULT_NONE_TO_START_GAP_MS));
                /* === END FIX === */

                transport_send_cmd_noack("traffic:start",
                                         strlen("traffic:start"));
                s_fault_from_remote_ex = true;
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                s_fault_from_remote_ex = false;
                traffic_light_set_follow_root(false);
                traffic_light_start_with_red();
                uint8_t ph = traffic_light_get_phase();
                send_sync_to_node_async_ex(
                    ph, 0,
                    traffic_light_get_phase_duration_ms(),
                    true, NULL);
            } else {
                ESP_LOGW(TAG, "ROOT sync: disp synced but fault active "
                              "(%s) — skip start",
                         traffic_light_fault_str());
            }
            /* === END FIX === */

            display_only_stop();
            display_force_off();

            root_set_state(SYNC_STATE_IDLE);
        } else if ((now_ms - s_root_state_ms) >= 3000) {
            ESP_LOGW(TAG, "ROOT sync: disp streak=%u/%u → resend sync:display",
                     s_root_disp_streak, SYNC_DISPLAY_STREAK_MIN);
            uint64_t utc = time_sync_get_utc_ms();
            if (utc != 0) {
                uint8_t  phase_bit = (uint8_t)((utc / SYNC_DISPLAY_BLINK_MS)
                                               & 1ULL);
                uint64_t next_edge = ((utc / SYNC_DISPLAY_BLINK_MS) + 1)
                                     * SYNC_DISPLAY_BLINK_MS;
                char dcmd[64];
                int dn = snprintf(dcmd, sizeof(dcmd),
                                  "sync:display:%u:%llu",
                                  phase_bit,
                                  (unsigned long long)next_edge);
                if (dn > 0) transport_send_cmd_noack(dcmd, dn);
            }
            /* НЕ обновляем s_root_state_ms — иначе ALIGNED-TIMEOUT
               никогда не сработает. */
        }
        break;
    }

    default:
        break;
    }
}

/* ============================================================
   NODE
   ============================================================ */

static volatile sync_state_t s_node_state = SYNC_STATE_IDLE;
static volatile uint32_t     s_node_state_ms = 0;
static volatile uint32_t     s_node_last_probe_ms = 0;
static volatile uint32_t     s_node_last_ack_ms = 0;
static volatile uint32_t     s_node_sync_started_ms = 0;
static volatile uint32_t     s_node_lock_pm = 0;
static volatile uint8_t      s_node_lock_phase = 0;

static volatile uint64_t     s_node_lock_end_utc_ms = 0;
static volatile uint32_t     s_node_lock_dur_ms     = 0;

sync_state_t sync_handshake_node_state(void) { return s_node_state; }
bool sync_handshake_node_is_syncing(void) { return s_node_state != SYNC_STATE_IDLE; }

static void node_set_state(sync_state_t st)
{
    if (s_node_state == st) return;
    ESP_LOGW(TAG, "NODE sync: %d → %d", (int)s_node_state, (int)st);
    s_node_state = st;
    s_node_state_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

void sync_handshake_node_start(void)
{
    /* === FIX FAULT-HAS-TOP-PRIORITY ===
       При активной аварии ритуал sync:probe НЕ запускаем. */
    if (traffic_light_fault_has_priority()) {
        ESP_LOGW(TAG, "NODE sync: START ignored (fault active: %s)",
                 traffic_light_fault_str());
        return;
    }
    /* === END FIX FAULT-HAS-TOP-PRIORITY === */

    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (s_node_state != SYNC_STATE_IDLE) return;

    ESP_LOGW(TAG, "NODE sync: START (probe)");
    s_node_sync_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_node_last_probe_ms = 0;
    s_node_last_ack_ms = 0;
    s_node_lock_end_utc_ms = 0;
    s_node_lock_dur_ms = 0;

    for (int i = 0; i < 16; i++) sr_output_off(i);

    s_display_cleared  = false;
    s_display_blink_on = false;
    s_display_blink_last = 0;

    s_insert_pending   = false;
    s_ped_request      = false;
    s_ped_display_done = false;

    node_set_state(SYNC_STATE_PROBE);
}

void sync_handshake_node_on_rx(const char *cmd, size_t len)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (!cmd || len == 0) return;

    char buf[160];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, n);
    buf[n] = '\0';

    if (strncmp(buf, "sync:probe", 10) == 0) {
        if (s_node_state != SYNC_STATE_ALIGNED) {
            s_node_last_ack_ms = (uint32_t)(esp_timer_get_time() / 1000);
            transport_send_evt_noack("sync:ack", 8);
            ESP_LOGD(TAG, "NODE sync: got probe → ack (state=%d)",
                     (int)s_node_state);
        }
        return;
    }

    if (strncmp(buf, "sync:ack", 8) == 0) {
        if (s_node_state == SYNC_STATE_PROBE) {
            ESP_LOGW(TAG, "NODE sync: got ack → TIME");
            node_set_state(SYNC_STATE_TIME);
        } else if (s_node_state == SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "NODE sync: got ack while IDLE → start PROBE");
            sync_handshake_node_start();
            if (s_node_state == SYNC_STATE_PROBE) {
                node_set_state(SYNC_STATE_TIME);
            }
        }
        return;
    }

    if (strncmp(buf, "time:", 5) == 0) {
        if (s_node_state == SYNC_STATE_PROBE ||
            s_node_state == SYNC_STATE_TIME) {
            ESP_LOGW(TAG, "NODE sync: got time → PHASE");
            node_set_state(SYNC_STATE_PHASE);
        }
        return;
    }

    if (strncmp(buf, "sync:lock:", 10) == 0) {
        /* === FIX FAULT-HAS-TOP-PRIORITY ===
           Если NODE в аварии — не применяем sync:lock к каналам
           (apply_mask и так заблокирован, но и состояние не
           трогаем, чтобы не сбить s_phase_start_ms для recovery). */
        if (traffic_light_fault_has_priority()) {
            ESP_LOGW(TAG, "NODE sync: lock IGNORED (fault active: %s)",
                     traffic_light_fault_str());
            /* Отвечаем pong, чтобы ROOT не висел в PHASE — но
               без применения маски. pm_to_send — 0, ROOT получит
               drift = 0 - my_pm ≠ 0 → re-lock, но если ROOT тоже
               в аварии, он не будет спрашивать. */
            char pong[80];
            int pn = snprintf(pong, sizeof(pong),
                              "sync:pong:0:0:0:%u",
                              (unsigned)traffic_light_get_phase());
            if (pn > 0) transport_send_evt_noack(pong, pn);
            return;
        }
        /* === END FIX === */

        unsigned int ph = 0, pm = 0, dur = 0;
        unsigned long long end_utc = 0;
        if (sscanf(buf + 10, "%u:%u:%u:%llu",
                   &ph, &pm, &dur, &end_utc) == 4) {
            if (s_node_state == SYNC_STATE_IDLE) {
                ESP_LOGW(TAG, "NODE sync: lock while IDLE → force start PHASE");
                sync_handshake_node_start();
                node_set_state(SYNC_STATE_PHASE);
            } else if (s_node_state == SYNC_STATE_PROBE ||
                       s_node_state == SYNC_STATE_TIME) {
                ESP_LOGW(TAG, "NODE sync: lock in state %d → force PHASE",
                         (int)s_node_state);
                node_set_state(SYNC_STATE_PHASE);
            } else if (s_node_state == SYNC_STATE_ALIGNED) {
                ESP_LOGD(TAG, "NODE sync: lock in ALIGNED → pong only");
            }

            s_node_lock_phase = (uint8_t)ph;
            s_node_lock_pm    = pm;

            s_node_lock_end_utc_ms = (uint64_t)end_utc;
            s_node_lock_dur_ms     = (uint32_t)dur;

            if (s_mutex && xSemaphoreTake(s_mutex,
                        pdMS_TO_TICKS(100)) == pdTRUE) {
                s_phase          = (uint8_t)ph;
                uint32_t now_t   = xTaskGetTickCount() * portTICK_PERIOD_MS;
                s_phase_start_ms = now_t - pm;
                s_state          = TRAFFIC_STATE_RUNNING;
                xSemaphoreGive(s_mutex);
            }

            uint32_t pm_utc = utc_elapsed_ms((uint64_t)end_utc,
                                             (uint32_t)dur);
            uint32_t pm_to_send;
            bool utc_ok = (pm_utc != 0 || end_utc == 0);
            if (end_utc != 0 && pm_utc != 0) {
                pm_to_send = pm_utc;
            } else {
                pm_to_send = traffic_light_get_phase_ms();
                utc_ok = false;
            }

            uint32_t my_ph = traffic_light_get_phase();
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            char pong[80];
            int pn = snprintf(pong, sizeof(pong),
                              "sync:pong:%u:%u:%u:%u",
                              (unsigned)now_ms, (unsigned)now_ms,
                              (unsigned)pm_to_send, (unsigned)my_ph);
            if (pn > 0) transport_send_evt_noack(pong, pn);

            ESP_LOGW(TAG, "NODE sync: lock ph=%u pm=%u dur=%u end_utc=%llu "
                          "→ pong pm_utc=%u (%s) my_ph=%u",
                     ph, pm, dur, end_utc, pm_to_send,
                     utc_ok ? "utc" : "local", my_ph);

            if (s_node_state == SYNC_STATE_PHASE) {
                node_set_state(SYNC_STATE_ALIGNED);
            } else if (s_node_state == SYNC_STATE_ALIGNED) {
                ESP_LOGD(TAG, "NODE sync: pong resent in ALIGNED");
            }
        }
        return;
    }

    if (strncmp(buf, "sync:display:", 13) == 0) {
        /* === FIX FAULT-HAS-TOP-PRIORITY ===
           В аварии display-каналом управляет fault-мигание, а не
           sync:display. */
        if (traffic_light_fault_has_priority()) return;
        /* === END FIX === */
        unsigned int bit = 0;
        unsigned long long next_edge = 0;
        if (sscanf(buf + 13, "%u:%llu", &bit, &next_edge) == 2) {
            uint64_t utc = time_sync_get_utc_ms();
            if (utc != 0) {
                int64_t remain = (int64_t)next_edge - (int64_t)utc;
                if (remain > 0 && remain <= SYNC_DISPLAY_BLINK_MS) {
                    bool want = (bit & 1) == 0;
                    if (want != s_display_blink_on) {
                        ESP_LOGD(TAG, "NODE sync: display force %s "
                                      "(remain=%lld ms)",
                                 want ? "ON" : "OFF", (long long)remain);
                        s_display_blink_on = want;
                        uint8_t ch = g_config.traffic.pins.cross_ped.ped_display;
                        if (ch >= 16) ch = SYNC_DISPLAY_CH;
                        if (want) sr_output_on(ch);
                        else      sr_output_off(ch);
                    }
                }
            }
        }
        return;
    }

    if (strncmp(buf, "sync:done", 9) == 0) {
        if (s_node_state == SYNC_STATE_ALIGNED) {
            ESP_LOGW(TAG, "NODE sync: done → IDLE");
            display_only_stop();
            display_force_off();
            node_set_state(SYNC_STATE_IDLE);
        }
        return;
    }

    if (strcmp(buf, "traffic:start") == 0) {
        if (s_node_state == SYNC_STATE_ALIGNED) {
            ESP_LOGW(TAG, "NODE sync: traffic:start in ALIGNED → IDLE");
            display_only_stop();
            display_force_off();
            node_set_state(SYNC_STATE_IDLE);
        }
        /* === FIX FAULT-HAS-TOP-PRIORITY ===
           sync_handshake НЕ снимает fault и НЕ трогает каналы.
           Снятие fault — только через fault:none от ROOT (см.
           node_proto.c) или try_recovery. */
        return;
    }
}

void sync_handshake_node_tick(uint32_t now_ms)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (s_node_state == SYNC_STATE_IDLE) return;

    /* === FIX FAULT-HAS-TOP-PRIORITY ===
       При активной аварии ритуал не трогает каналы. Форсим IDLE
       без display. */
    if (traffic_light_fault_has_priority()) {
        if (s_node_state != SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "NODE sync: fault active (%s) → force IDLE",
                     traffic_light_fault_str());
            display_only_stop();
            display_force_off();
            s_node_state = SYNC_STATE_IDLE;
            s_node_state_ms = now_ms;
        }
        return;
    }
    /* === END FIX FAULT-HAS-TOP-PRIORITY === */

    bool use_utc = (s_node_state >= SYNC_STATE_TIME);
    display_only_blink(now_ms, use_utc);

    if (s_node_state == SYNC_STATE_ALIGNED &&
        (now_ms - s_node_state_ms) >= SYNC_ALIGN_TIMEOUT_MS) {
        ESP_LOGE(TAG, "NODE sync: ALIGNED timeout %u ms → force IDLE",
                 (unsigned)(now_ms - s_node_state_ms));
        display_only_stop();
        display_force_off();
        node_set_state(SYNC_STATE_IDLE);
        return;
    }

    /* Глобальный таймаут для PROBE/TIME/PHASE */
    if (s_node_sync_started_ms != 0 &&
        (now_ms - s_node_sync_started_ms) > SYNC_ALIGN_TIMEOUT_MS &&
        s_node_state != SYNC_STATE_ALIGNED) {
        ESP_LOGE(TAG, "NODE sync: TIMEOUT %u ms in state %d → restart probe",
                 (unsigned)(now_ms - s_node_sync_started_ms),
                 (int)s_node_state);
        s_node_sync_started_ms = now_ms;
        node_set_state(SYNC_STATE_PROBE);
    }

    switch (s_node_state) {
    case SYNC_STATE_PROBE:
        if (s_node_last_probe_ms == 0 ||
            (now_ms - s_node_last_probe_ms) >= SYNC_PROBE_INTERVAL_MS) {
            s_node_last_probe_ms = now_ms;
            transport_send_evt_noack("sync:probe", 10);
        }
        break;

    case SYNC_STATE_ALIGNED:
        /* resend pong раз в 3 с. НЕ обновляем s_node_state_ms — иначе
           ALIGNED-TIMEOUT никогда не сработает. */
        if ((now_ms - s_node_state_ms) >= 3000) {
            ESP_LOGW(TAG, "NODE sync: no done for 3s → resend pong");

            uint32_t pm_to_send;
            uint64_t end_utc = s_node_lock_end_utc_ms;
            uint32_t dur     = s_node_lock_dur_ms;
            uint32_t pm_utc  = utc_elapsed_ms(end_utc, dur);
            if (end_utc != 0 && pm_utc != 0) {
                pm_to_send = pm_utc;
            } else {
                pm_to_send = traffic_light_get_phase_ms();
            }

            uint32_t my_ph = traffic_light_get_phase();
            char pong[80];
            int pn = snprintf(pong, sizeof(pong),
                              "sync:pong:%u:%u:%u:%u",
                              (unsigned)now_ms, (unsigned)now_ms,
                              (unsigned)pm_to_send, (unsigned)my_ph);
            if (pn > 0) transport_send_evt_noack(pong, pn);
            /* НЕ трогаем s_node_state_ms */
        }
        break;

    default:
        break;
    }
}