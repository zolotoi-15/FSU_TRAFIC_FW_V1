/* ============================================================
   sync_handshake.c — ускоренное восстановление LoRa-связи
   с ритуалом синхронизации и синхронным миганием ЖДИТЕ.

   Жизненный цикл (ROOT):
     IDLE → (нет NODE) → PROBE
     PROBE → (получен sync:ack) → TIME
     TIME  → (отправлены time: ×3) → PHASE
     PHASE → (отправлены lock ×N, получены pong с drift <= SYNC_ALIGN_DRIFT_MS
              SYNC_ALIGN_STREAK раз подряд) → ALIGNED
     ALIGNED → (NODE мигает display синхронно SYNC_DISPLAY_STREAK_MIN
                STATUS подряд) → отправка traffic:start + sync:done + c:
                → IDLE (конфиг запущен)

   Жизненный цикл (NODE):
     IDLE → (нет ROOT) → PROBE
     PROBE → (получен sync:probe от ROOT) → ack; (sync:ack от ROOT) → TIME
     TIME  → (получен time:) → PHASE
     PHASE → (получен sync:lock) → pong; ALIGNED
     ALIGNED → (получен sync:done / traffic:start) → IDLE

   Пока state != IDLE:
     - все каналы кроме ch11 (display) выключены;
     - ch11 мигает с периодом SYNC_DISPLAY_BLINK_MS;
     - с момента SYNC_STATE_TIME мигание синхронно от UTC.

   === FIX SYNC-BLINK-NODE ===
   В NODE в состоянии PHASE/ALIGNED мигание продолжается по UTC,
   не замирает на одном бите, пока ROOT готовит sync:done.

   === FIX SYNC-DISPLAY-UTC-PHASE ===
   - PROBE: локальный таймер.
   - TIME/PHASE/ALIGNED: абсолютная фаза от UTC
     (utc / SYNC_DISPLAY_BLINK_MS) & 1 → мигание синхронно до мс.
   - ROOT в ALIGNED раз в SYNC_DISPLAY_RESYNC_MS шлёт
     sync:display:<0|1>:<utc_ms_next_edge> для форса фазы NODE.

   === FIX SYNC-DISPLAY-VERIFY ===
   ROOT запускает конфиг (ALIGNED → IDLE) ТОЛЬКО когда NODE
   несколько STATUS подряд прислал "disp", совпадающий с
   текущим битом ROOT.

   === FIX DISPLAY-OFF-AFTER-SYNC ===
   При выходе из sync (ALIGNED → IDLE) display_only_stop()
   вызывается явно И дублируется прямым sr_output_off(ped_display).

   === FIX SYNC-PROBE-ALWAYS-ANSWER ===
   - NODE отвечает sync:ack на sync:probe в любом состоянии, кроме ALIGNED.
   - ROOT принимает sync:ack в любом состоянии; если ROOT в IDLE — стартует PROBE.
   - ROOT на sync:probe от NODE отвечает sync:ack в любом состоянии, кроме ALIGNED.

   === FIX SYNC-PROBE-FORCE-IDLE-TIMEOUT ===
   Если ROOT в PROBE не получил ack за SYNC_ALIGN_TIMEOUT_MS, он
   принудительно выходит в IDLE, гасит display и запускает traffic
   с красного.

   === FIX SYNC-LOCK-ANY-STATE ===
   NODE принимает sync:lock в любом состоянии, кроме ALIGNED
   (и IDLE → принудительно PROBE→PHASE). Раньше lock принимался
   только в PHASE. Но ROOT начинает слать lock сразу после time:,
   а time: может потеряться → NODE в TIME игнорировал lock →
   бесконечный цикл PROBE→TIME→(lock ignore)→TIMEOUT→PROBE.

   === FIX SYNC-ACK-IN-PHASE ===
   ROOT в PHASE при получении sync:ack от NODE переотправляет
   time: ×3 — NODE потерял time:, надо перевести его в PHASE.

   === FIX SYNC-TIME-IN-PHASE ===
   ROOT в PHASE, помимо lock, раз в 1000 мс переотправляет time:.
   Иначе пока ROOT в PHASE (не в RUNNING) NODE не получает свежее
   время → |delta| растёт → UTC-коррекция в c: отбрасывается
   (UTC_DELTA_MAX_MS) → рассинхрон продолжается.

   === FIX NO-DRIFT-RESTART-TIME ===
   ROOT при "no drift match after SYNC_LOCK_TRIES locks"
   рестартит TIME (а не PHASE). Иначе ROOT снова шлёт lock ×5,
   а NODE (без time:) в TIME игнорирует → цикл.
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

extern volatile bool     s_node_running;      /* node_proto.c */
extern volatile uint32_t s_last_node_msg_ms;  /* blink_example_main.c */

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

    /* Гасим всё, кроме display, один раз при входе в sync */
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

/* === FIX DISPLAY-OFF-AFTER-SYNC === */
static inline void display_force_off(void)
{
    uint8_t ch = g_config.traffic.pins.cross_ped.ped_display;
    if (ch >= 16) ch = SYNC_DISPLAY_CH;
    sr_output_off(ch);
}
/* === END FIX DISPLAY-OFF-AFTER-SYNC === */

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

/* последний pm NODE, полученный в STATUS */
static volatile uint32_t     s_root_node_pm = 0;
static volatile uint8_t      s_root_node_phase = 0xFF;
static volatile uint32_t     s_root_node_pm_ms = 0;
static volatile bool         s_root_node_status_fresh = false;

/* === FIX SYNC-DISPLAY-VERIFY === */
static volatile uint8_t  s_root_node_disp       = 0xFF;
static volatile uint32_t s_root_node_disp_ms    = 0;
static volatile uint8_t  s_root_disp_streak     = 0;

static uint8_t root_current_disp_bit(void)
{
    uint64_t utc = time_sync_get_utc_ms();
    if (utc == 0) return 0xFF;
    return (uint8_t)((utc / SYNC_DISPLAY_BLINK_MS) & 1ULL);
}
/* === END FIX SYNC-DISPLAY-VERIFY === */

/* === FIX SYNC-TIME-IN-PHASE === */
static volatile uint32_t s_root_last_phase_time_ms = 0;
/* === END FIX SYNC-TIME-IN-PHASE === */

sync_state_t sync_handshake_root_state(void) { return s_root_state; }
bool sync_handshake_root_is_syncing(void) { return s_root_state != SYNC_STATE_IDLE; }

static void root_set_state(sync_state_t st)
{
    if (s_root_state == st) return;
    ESP_LOGW(TAG, "ROOT sync: %d → %d", (int)s_root_state, (int)st);
    s_root_state = st;
    s_root_state_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_root_aligned_streak = 0;
    /* === FIX SYNC-DISPLAY-VERIFY === */
    s_root_disp_streak = 0;
    /* === END FIX SYNC-DISPLAY-VERIFY === */
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

    /* === FIX SYNC-DISPLAY-VERIFY === */
    s_root_node_disp    = 0xFF;
    s_root_node_disp_ms = 0;
    s_root_disp_streak  = 0;
    /* === END FIX SYNC-DISPLAY-VERIFY === */

    /* Все каналы выключаем, ЖДИТЕ начинает мигать */
    for (int i = 0; i < 16; i++) sr_output_off(i);

    /* сбрасываем локальные флаги display, чтобы «очистка» прошла заново */
    s_display_cleared  = false;
    s_display_blink_on = false;
    s_display_blink_last = 0;

    /* сбрасываем ped-флаги на случай, если остались от предыдущей работы */
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

    /* sync:ack — NODE готов
       === FIX SYNC-PROBE-ALWAYS-ANSWER ===
       === FIX SYNC-ACK-IN-PHASE === */
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
            /* === FIX SYNC-ACK-IN-PHASE ===
               NODE потерял time: и просит probe. ROOT в PHASE —
               переотправляем time: ×3, чтобы NODE перешёл в PHASE
               и принял lock. */
            ESP_LOGW(TAG, "ROOT sync: got ack in PHASE → resend time:");
            s_root_time_sent = 0;
            s_root_last_time_ms = 0;
            root_set_state(SYNC_STATE_TIME);
        } else if (s_root_state == SYNC_STATE_TIME) {
            ESP_LOGD(TAG, "ROOT sync: got ack in TIME → ignore");
        }
        return;
    }

    /* sync:probe от NODE
       === FIX SYNC-PROBE-ALWAYS-ANSWER === */
    if (strncmp(buf, "sync:probe", 10) == 0) {
        if (s_root_state == SYNC_STATE_ALIGNED) return;
        if (s_root_state == SYNC_STATE_IDLE) {
            ESP_LOGW(TAG, "ROOT sync: got probe in IDLE → start PROBE");
            sync_handshake_root_start();
        }
        transport_send_cmd_noack("sync:ack", 8);
        return;
    }

    /* sync:pong:<ts>:<t_recv>:<pm>:<phase> — NODE ответил на sync:lock */
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

    /* STATUS от NODE — обновляем pm/phase + display-бит */
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

        /* === FIX SYNC-DISPLAY-VERIFY === */
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
        /* === END FIX SYNC-DISPLAY-VERIFY === */
    }
}

void sync_handshake_root_tick(uint32_t now_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;

    if (s_root_state == SYNC_STATE_IDLE) return;

    bool use_utc = (s_root_state >= SYNC_STATE_TIME);
    display_only_blink(now_ms, use_utc);

    /* === FIX SYNC-PROBE-FORCE-IDLE-TIMEOUT === */
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
    /* === END FIX SYNC-PROBE-FORCE-IDLE-TIMEOUT === */

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
        /* === FIX SYNC-TIME-IN-PHASE ===
           Пока ROOT в PHASE (не в RUNNING), периодический time:
           раз в 1000 мс, чтобы NODE не уплыл по часам и UTC-коррекция
           в c: работала. */
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
        /* === END FIX SYNC-TIME-IN-PHASE === */

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
                    /* === FIX NO-DRIFT-RESTART-TIME ===
                       Раньше рестартили PHASE (снова lock ×5). NODE
                       терял time: и не переходил в PHASE → замкнутый
                       круг. Теперь рестартим TIME. */
                    ESP_LOGW(TAG, "ROOT sync: no drift match after %u locks "
                                  "→ restart TIME", SYNC_LOCK_TRIES);
                    s_root_time_sent = 0;
                    s_root_last_time_ms = 0;
                    s_root_lock_sent = 0;
                    s_root_last_lock_ms = 0;
                    root_set_state(SYNC_STATE_TIME);
                    /* === END FIX NO-DRIFT-RESTART-TIME === */
                }
            }
        }
        break;
    }

    case SYNC_STATE_ALIGNED: {
        /* sync:display ресинк */
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

        /* === FIX SYNC-DISPLAY-VERIFY === */
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
            s_root_state_ms = now_ms;
        }
        /* === END FIX SYNC-DISPLAY-VERIFY === */
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

    /* === FIX SYNC-PROBE-ALWAYS-ANSWER === */
    if (strncmp(buf, "sync:probe", 10) == 0) {
        if (s_node_state != SYNC_STATE_ALIGNED) {
            s_node_last_ack_ms = (uint32_t)(esp_timer_get_time() / 1000);
            transport_send_evt_noack("sync:ack", 8);
            ESP_LOGD(TAG, "NODE sync: got probe → ack (state=%d)",
                     (int)s_node_state);
        }
        return;
    }

    /* sync:ack — ROOT ответил на наш probe */
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

    /* time: — время от ROOT */
    if (strncmp(buf, "time:", 5) == 0) {
        if (s_node_state == SYNC_STATE_PROBE ||
            s_node_state == SYNC_STATE_TIME) {
            ESP_LOGW(TAG, "NODE sync: got time → PHASE");
            node_set_state(SYNC_STATE_PHASE);
        }
        return;
    }

    /* sync:lock:<ph>:<pm>:<dur>:<end_utc>
       === FIX SYNC-LOCK-ANY-STATE === */
    if (strncmp(buf, "sync:lock:", 10) == 0) {
        unsigned int ph = 0, pm = 0, dur = 0;
        unsigned long long end_utc = 0;
        if (sscanf(buf + 10, "%u:%u:%u:%llu",
                   &ph, &pm, &dur, &end_utc) == 4) {
            /* lock содержит end_utc, самодостаточен. Принимаем в
               любом состоянии, кроме ALIGNED. Если NODE в PROBE
               или TIME — принудительно переходим в PHASE. */
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
    /* === END FIX SYNC-LOCK-ANY-STATE === */

    /* sync:display:<0|1>:<utc_ms_next_edge> */
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

    /* sync:done */
    if (strncmp(buf, "sync:done", 9) == 0) {
        if (s_node_state == SYNC_STATE_ALIGNED) {
            ESP_LOGW(TAG, "NODE sync: done → IDLE");
            display_only_stop();
            display_force_off();
            node_set_state(SYNC_STATE_IDLE);
        }
        return;
    }

    /* traffic:start — если ещё в ALIGNED, принимаем как done */
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

    /* === FIX SYNC-BLINK-NODE ===
       В любом состоянии sync (PROBE/TIME/PHASE/ALIGNED) display
       мигает. В TIME+ — по UTC. Это критично: если ROOT не присылает
       sync:done сразу после ALIGNED, display продолжает мигать, а не
       замирает на одном бите. */
    bool use_utc = (s_node_state >= SYNC_STATE_TIME);
    display_only_blink(now_ms, use_utc);
    /* === END FIX SYNC-BLINK-NODE === */

    /* Таймаут: если долго в любом состоянии и нет ack — рестарт probe */
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
            s_node_state_ms = now_ms;
        }
        break;

    default:
        break;
    }
}