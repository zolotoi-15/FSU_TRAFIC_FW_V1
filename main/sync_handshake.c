/* ============================================================
   sync_handshake.c — ускоренное восстановление LoRa-связи
   с ритуалом синхронизации и синхронным миганием ЖДИТЕ.

   === FIX NODE-ALIGNED-TIMEOUT (новое) ===
       Если NODE в ALIGNED дольше SYNC_ALIGN_TIMEOUT_MS и
       sync:done не пришёл — принудительно IDLE + display_off.
       ROOT к этому моменту уже вышел из sync и шлёт c:/traffic:start.
       Без этого фикса NODE висит в ALIGNED бесконечно,
       traffic_task_loop уходит в continue, blink не крутится.

   === FIX ALIGNED-TIMEOUT (ROOT) ===
       ROOT в ALIGNED дольше SYNC_ALIGN_TIMEOUT_MS без disp_streak —
       принудительный IDLE + traffic:start + c:.

   === FIX TIME-IN-ALIGNED ===
       Пока ROOT в ALIGNED — time: раз в 1000 мс. Иначе часы NODE
       уплывают, disp не совпадает.

   === FIX ALIGNED-NO-RESET-STATE-MS ===
       В ALIGNED при resend sync:display НЕ обновляем
       s_root_state_ms / s_node_state_ms — иначе таймауты не
       сработают.
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
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (s_root_state != SYNC_STATE_IDLE) return;

    ESP_LOGW(TAG, "ROOT sync: START (probe)");
    s_root_sync_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_root_last_probe_ms = 0;
    s_root_time_sent = 0;
    s_root_lock_sent = 0;
    s_root_aligned_streak = 0;
    s_root_node_status_fresh = false;

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
                uint32_t my_pm = traffic_light_get_phase_ms();
                int32_t  drift = (int32_t)pm - (int32_t)my_pm;
                if (drift < 0) drift = -drift;
                s_root_last_drift_ms = drift;

                if (drift <= SYNC_ALIGN_DRIFT_MS) {
                    s_root_aligned_streak++;
                    ESP_LOGI(TAG, "ROOT sync: pong drift=%ld ms (streak %u/%u)",
                             (long)drift, s_root_aligned_streak,
                             SYNC_ALIGN_STREAK);
                    if (s_root_aligned_streak >= SYNC_ALIGN_STREAK) {
                        ESP_LOGW(TAG, "ROOT sync: ALIGNED (drift %ld ms)",
                                 (long)drift);
                        root_set_state(SYNC_STATE_ALIGNED);
                    }
                } else {
                    s_root_aligned_streak = 0;
                    ESP_LOGW(TAG, "ROOT sync: pong drift=%ld ms > %d → re-lock",
                             (long)drift, SYNC_ALIGN_DRIFT_MS);
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
            s_fault_from_remote_ex = true;
            traffic_light_set_fault(TRAFFIC_FAULT_NONE);
            s_fault_from_remote_ex = false;
            traffic_light_set_follow_root(false);
            traffic_light_start_with_red();
            transport_send_cmd_noack("traffic:start", strlen("traffic:start"));
            uint8_t ph = traffic_light_get_phase();
            send_sync_to_node_async_ex(ph, 0,
                traffic_light_get_phase_duration_ms(), true, NULL);
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
        /* === FIX ALIGNED-TIMEOUT (ROOT) === */
        if ((now_ms - s_root_state_ms) >= SYNC_ALIGN_TIMEOUT_MS) {
            ESP_LOGE(TAG, "ROOT sync: ALIGNED timeout %u ms "
                          "(disp_streak=%u) → force IDLE + traffic:start",
                     (unsigned)(now_ms - s_root_state_ms),
                     s_root_disp_streak);

            transport_send_cmd_noack("traffic:start",
                                     strlen("traffic:start"));
            transport_send_cmd_noack("sync:done", 9);

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

            display_only_stop();
            display_force_off();
            root_set_state(SYNC_STATE_IDLE);
            break;
        }
        /* === END FIX ALIGNED-TIMEOUT === */

        /* === FIX TIME-IN-ALIGNED === */
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
        /* === END FIX TIME-IN-ALIGNED === */

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

            transport_send_cmd_noack("traffic:start", strlen("traffic:start"));
            transport_send_cmd_noack("sync:done", 9);

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
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (s_node_state != SYNC_STATE_IDLE) return;

    ESP_LOGW(TAG, "NODE sync: START (probe)");
    s_node_sync_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_node_last_probe_ms = 0;
    s_node_last_ack_ms = 0;

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

            if (s_mutex && xSemaphoreTake(s_mutex,
                        pdMS_TO_TICKS(100)) == pdTRUE) {
                s_phase          = (uint8_t)ph;
                uint32_t now_t   = xTaskGetTickCount() * portTICK_PERIOD_MS;
                s_phase_start_ms = now_t - pm;
                s_state          = TRAFFIC_STATE_RUNNING;
                xSemaphoreGive(s_mutex);
            }

            uint32_t my_pm = traffic_light_get_phase_ms();
            uint32_t my_ph = traffic_light_get_phase();
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            char pong[64];
            int pn = snprintf(pong, sizeof(pong),
                              "sync:pong:%u:%u:%u:%u",
                              (unsigned)now_ms, (unsigned)now_ms,
                              (unsigned)my_pm, (unsigned)my_ph);
            if (pn > 0) transport_send_evt_noack(pong, pn);

            if (s_node_state == SYNC_STATE_PHASE) {
                ESP_LOGW(TAG, "NODE sync: lock ph=%u pm=%u → pong (my pm=%u)",
                         ph, pm, (unsigned)my_pm);
                node_set_state(SYNC_STATE_ALIGNED);
            } else if (s_node_state == SYNC_STATE_ALIGNED) {
                ESP_LOGD(TAG, "NODE sync: pong resent in ALIGNED");
            }
        }
        return;
    }

    if (strncmp(buf, "sync:display:", 13) == 0) {
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
        return;
    }
}

void sync_handshake_node_tick(uint32_t now_ms)
{
    if (g_config.role != DEVICE_ROLE_SLAVE) return;
    if (s_node_state == SYNC_STATE_IDLE) return;

    bool use_utc = (s_node_state >= SYNC_STATE_TIME);
    display_only_blink(now_ms, use_utc);

    /* === FIX NODE-ALIGNED-TIMEOUT ===
       Если NODE в ALIGNED дольше SYNC_ALIGN_TIMEOUT_MS и
       sync:done не пришёл — принудительно IDLE + display_off.
       ROOT к этому моменту уже вышел из sync и шлёт c:/traffic:start.
       Без этого фикса NODE висит в ALIGNED бесконечно,
       traffic_task_loop уходит в continue, blink не крутится. */
    if (s_node_state == SYNC_STATE_ALIGNED &&
        (now_ms - s_node_state_ms) >= SYNC_ALIGN_TIMEOUT_MS) {
        ESP_LOGE(TAG, "NODE sync: ALIGNED timeout %u ms → force IDLE",
                 (unsigned)(now_ms - s_node_state_ms));
        display_only_stop();
        display_force_off();
        node_set_state(SYNC_STATE_IDLE);
        return;
    }
    /* === END FIX NODE-ALIGNED-TIMEOUT === */

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
        /* resend pong раз в 3 с. НЕ обновляем s_node_state_ms —
           иначе ALIGNED-TIMEOUT никогда не сработает. */
        if ((now_ms - s_node_state_ms) >= 3000) {
            ESP_LOGW(TAG, "NODE sync: no done for 3s → resend pong");
            uint32_t my_pm = traffic_light_get_phase_ms();
            uint32_t my_ph = traffic_light_get_phase();
            char pong[64];
            int pn = snprintf(pong, sizeof(pong),
                              "sync:pong:%u:%u:%u:%u",
                              (unsigned)now_ms, (unsigned)now_ms,
                              (unsigned)my_pm, (unsigned)my_ph);
            if (pn > 0) transport_send_evt_noack(pong, pn);
            /* НЕ трогаем s_node_state_ms */
        }
        break;

    default:
        break;
    }
}