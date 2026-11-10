/* ============================================================
   traffic_light.c — основной цикл traffic_task + force_phase.

   === SIMPLIFIED-NO-CFG ===
       NODE (follow_root=true):
         - не крутит apply_stage_dd;
         - не переключает фазы сам;
         - всё получает через traffic_light_apply_mask() из on_cmd_from_root.

   === FIX EXPECTED-FROM-APPLY-MASK ===
       NODE получает маску через 'c:' (apply_mask), но
       g_config.traffic.sequence на NODE — из NVS (старый,
       CFG не передаётся в NO-CFG). check_faults на NODE
       использует s_expected_mask_from_root / s_active_mask_from_root
       из apply_mask, а не g_config. Иначе expected (старый config)
       ≠ actual (новая маска из ROOT) → ложный V_IN low.

   === FIX NO-DOUBLE-SYNC-FORCE ===
       force_phase / next_phase / manual_override зовут
       phase_change_cancel_pending() после apply_stage_dd.

   === FIX STALE-ALL-OFF-WATCHDOG ===
       Если ROOT в RUNNING и fault=ALL_OFF >5 с — сбрасываем
       и принудительно resync.

   === FIX MASK-RECEIVED-RESET ===
       traffic_light_start: при s_follow_root сбрасываем
       s_mask_received. Иначе после traffic:stop/start NODE
       работал бы со старой маской в check_faults, пока не
       придёт новый c:.
   ============================================================ */

#include "traffic_internal.h"
#include "traffic_light.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
#include "time_sync.h"
#include "transport.h"
#include "root_wd.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "traffic";

extern esp_err_t board_diag_en_set(uint8_t channel, bool enable);
extern void      board_diag_en_all_off(void);
extern int       board_read_iload_raw(uint8_t channel,
                                      imon_state_t *out_state,
                                      int *out_mv);

volatile traffic_state_t s_state = TRAFFIC_STATE_STOPPED;
volatile traffic_mode_t  s_mode  = TRAFFIC_MODE_CROSS_PED;
volatile uint8_t         s_phase = 0;
volatile uint32_t        s_phase_start_ms = 0;
volatile uint32_t        s_last_recovery_ms = 0;
volatile uint32_t        s_recovery_attempts = 0;
volatile bool            s_follow_root = false;

volatile bool            s_ped_request      = false;
volatile uint32_t        s_ped_request_ms   = 0;
volatile bool            s_ped_min_recall   = false;

TaskHandle_t             s_task = NULL;
SemaphoreHandle_t        s_mutex = NULL;

volatile traffic_fault_t s_fault = TRAFFIC_FAULT_NONE;
traffic_fault_info_t     s_fault_list[TRAFFIC_FAULT_MAX];
volatile uint8_t         s_fault_count = 0;
SemaphoreHandle_t        s_fault_mutex = NULL;

volatile uint32_t        s_last_fault_notify_ms = 0;
volatile uint16_t        s_blink_mask = 0;
volatile uint16_t        s_fault_yellow_mask = 0;
volatile bool            s_blink_on = false;
volatile uint32_t        s_blink_last_phase = 0;
volatile uint32_t        s_sync_delay_us = 0;
uint8_t                  s_suspect_counter[16] = {0};
uint8_t                  s_vin_off_counter[16] = {0};
uint8_t                  s_ok_streak = 0;
volatile bool            s_mux_reading = false;
volatile uint32_t        s_last_recovery_ok_ms = 0;
volatile bool            s_fault_blink_state = false;

volatile bool            s_mask_received = false;
volatile bool            s_in_force_phase = false;

volatile bool            s_hold_active       = false;
volatile bool            s_insert_pending    = false;
volatile uint8_t         s_insert_queue[MAX_INSERT_SEQ] = {0};
volatile uint8_t         s_insert_count      = 0;
volatile uint8_t         s_insert_idx        = 0;
volatile uint32_t        s_insert_request_ms = 0;

/* === FIX EXPECTED-FROM-APPLY-MASK ===
   Маски, которые ROOT прислал через 'c:'. NODE использует их
   для check_faults, потому что g_config.traffic.sequence/stages_dd
   на NODE — из NVS (может быть старым — CFG не передаётся). */
volatile uint16_t        s_expected_mask_from_root = 0;
volatile uint16_t        s_active_mask_from_root   = 0;
/* === END FIX EXPECTED-FROM-APPLY-MASK === */

traffic_fault_notify_cb_t s_fault_notify = NULL;
traffic_phase_change_cb_t s_phase_change_cb = NULL;

static volatile bool s_night_mode = false;

static volatile bool     s_all_red_active      = false;
static volatile uint32_t s_all_red_start_ms    = 0;
static volatile uint8_t  s_all_red_next_seq    = 0xFF;
static volatile uint32_t s_all_red_next_dur_ms = 0;

static volatile bool    s_manual_override = false;
static volatile uint8_t s_manual_seq = 0;

static volatile bool     s_hold_pm_frozen = false;

bool traffic_light_get_fault_blink_state(void) { return s_fault_blink_state; }

void traffic_light_apply_fault_blink(bool on)
{
    if (s_fault != TRAFFIC_FAULT_YELLOW_BLINK) return;
    if (s_fault_blink_state == on) return;
    fault_blink_yellow(on);
}

uint32_t traffic_light_last_recovery_ok_ms(void) { return s_last_recovery_ok_ms; }
void traffic_light_set_fault_notify(traffic_fault_notify_cb_t cb) { s_fault_notify = cb; }
void traffic_light_set_phase_change_cb(traffic_phase_change_cb_t cb) { s_phase_change_cb = cb; }

void traffic_light_set_follow_root(bool en)
{
    s_follow_root = en;
    if (en) s_mask_received = false;
    ESP_LOGD(TAG, "follow_root=%d", en);
}

/* ============================================================
   traffic_light_request_pedestrian_ex
   ============================================================ */
void traffic_light_request_pedestrian_ex(bool from_remote)
{
    if (!g_config.traffic.pedestrian.button_enabled) {
        ESP_LOGW(TAG, "pedestrian request ignored: button disabled in config");
        return;
    }

    if (s_follow_root) {
        ESP_LOGI(TAG, "pedestrian request: follow_root → delegate to ROOT "
                      "(wait 'c:' per phase)");
        return;
    }

    bool in_hold = s_hold_active && g_config.traffic.hold.enabled;
    if (in_hold) {
        if (g_config.traffic.hold.on_ped_request == HOLD_PED_IGNORE) {
            ESP_LOGW(TAG, "pedestrian request ignored: hold + on_ped_request=ignore");
            return;
        }
        if (g_config.traffic.hold.on_ped_request == HOLD_PED_MIN_RECALL) {
            s_ped_min_recall = true;
            ESP_LOGW(TAG, "pedestrian request: hold + min_recall");
            return;
        }
    }

    uint8_t mode = g_config.traffic.pedestrian.request_mode;

    if (mode == PED_REQUEST_MODE_MIN_RECALL) {
        s_ped_min_recall = true;
        ESP_LOGW(TAG, "pedestrian request: MIN_RECALL (next cycle min %lu s)",
                 (unsigned long)g_config.traffic.pedestrian.min_recall_s);
        return;
    }

    if (mode == PED_REQUEST_MODE_INSERT) {
        if (s_insert_pending) {
            ESP_LOGW(TAG, "pedestrian insert: already pending → ignore");
            return;
        }
        if (g_config.traffic.pedestrian.insert_count == 0) {
            ESP_LOGW(TAG, "pedestrian insert: insert_sequence empty → fallback min_recall");
            s_ped_min_recall = true;
            return;
        }
        s_insert_pending  = true;
        s_insert_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        for (int i = 0; i < g_config.traffic.pedestrian.insert_count; i++)
            s_insert_queue[i] = g_config.traffic.pedestrian.insert_sequence[i];
        s_insert_count = g_config.traffic.pedestrian.insert_count;
        s_insert_idx   = 0;
        ESP_LOGW(TAG, "pedestrian request: INSERT pending (seq=%u, lead=%lu ms, hold=%d, safe_exit=%u)",
                 s_insert_count,
                 (unsigned long)g_config.traffic.pedestrian.insert_lead_ms,
                 (int)in_hold,
                 g_config.traffic.pedestrian.safe_exit_count);

        if (g_config.role == DEVICE_ROLE_MASTER && !from_remote) {
            transport_send_cmd("pedestrian:request", strlen("pedestrian:request"));
        }
        return;
    }

    s_ped_request = true;
    s_ped_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGW(TAG, "pedestrian request: STAGE");

    if (g_config.role == DEVICE_ROLE_MASTER && !from_remote) {
        transport_send_cmd("pedestrian:request", strlen("pedestrian:request"));
    }
}

void traffic_light_request_pedestrian(void)
{
    traffic_light_request_pedestrian_ex(false);
}

bool traffic_light_ped_button_enabled(void) { return g_config.traffic.pedestrian.button_enabled; }

void traffic_light_set_sync_delay_us(uint32_t delay_us)
{
    s_sync_delay_us = delay_us;
    ESP_LOGD(TAG, "sync_delay = %lu us (%.3f ms)",
             (unsigned long)delay_us, delay_us / 1000.0);
}
uint32_t traffic_light_get_sync_delay_us(void) { return s_sync_delay_us; }
void traffic_light_set_mux_reading(bool en) { s_mux_reading = en; }

static inline uint32_t phase_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.phase_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}

static uint32_t insert_duration_ms(uint8_t seq_idx)
{
    uint32_t dur = effective_stage_duration_ms(seq_idx);
    if (g_config.traffic.pedestrian.insert_red_ms > 0 &&
        seq_idx < g_config.traffic.sequence_count) {
        const traffic_stage_dd_t *st =
            find_stage(g_config.traffic.sequence[seq_idx]);
        if (st && st->duration_kind == DD_DUR_RED) {
            dur = g_config.traffic.pedestrian.insert_red_ms;
        }
    }
    return dur;
}

/* ============================================================
   traffic_task_loop
   === SIMPLIFIED-NO-CFG ===
   ============================================================ */
void traffic_task_loop(void *arg)
{
    ESP_LOGI(TAG, "Traffic task started (step=20ms, follow_root=%d)", s_follow_root);

    uint32_t blink_last_fault = 0;
    bool     fault_blink_on   = false;
    uint8_t  last_min_green_phase  = 0xFF;
    uint8_t  last_min_recall_phase = 0xFF;
    uint32_t last_phase_log_ms = 0;

    uint32_t all_off_since_ms = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (now - last_phase_log_ms >= 5000) {
            last_phase_log_ms = now;
            if (s_state == TRAFFIC_STATE_RUNNING ||
                s_state == TRAFFIC_STATE_NIGHT_YELLOW_BLINK) {
                const char *fault_str = "NONE";
                if (s_fault == TRAFFIC_FAULT_ALL_OFF)          fault_str = "ALL_OFF";
                else if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) fault_str = "YELLOW_BLINK";

                ESP_LOGD(TAG, "PHASE: role=%s seq=%u state=%s fault=%s "
                              "mask_recv=%d hold=%d insert=%d",
                         g_config.role == DEVICE_ROLE_MASTER ? "ROOT" : "NODE",
                         s_phase, traffic_light_state_str(), fault_str,
                         (int)s_mask_received,
                         (int)s_hold_active,
                         (int)s_insert_pending);
            }
        }

        if (s_state != TRAFFIC_STATE_RUNNING &&
            s_state != TRAFFIC_STATE_NIGHT_YELLOW_BLINK) continue;

        if (s_follow_root) {
            if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) {
                uint32_t per = g_config.traffic.phases_ms.fault_blink_period;
                if (per < BLINK_MIN_PERIOD_MS) per = 500;
                if (now - blink_last_fault >= per) {
                    blink_last_fault = now;
                    fault_blink_on = !fault_blink_on;
                    fault_blink_yellow(fault_blink_on);
                }
            }
            continue;
        }

        /* stale ALL_OFF watchdog */
        if (s_state == TRAFFIC_STATE_RUNNING &&
            s_fault == TRAFFIC_FAULT_ALL_OFF) {
            if (all_off_since_ms == 0) {
                all_off_since_ms = now;
            } else if ((now - all_off_since_ms) >= 5000) {
                ESP_LOGE(TAG, "stale ALL_OFF (RUNNING) >5s → force clear + resync");
                all_off_since_ms = 0;
                s_fault = TRAFFIC_FAULT_NONE;
                s_fault_count = 0;
                s_phase_start_ms = now;
                s_force_resync_pending = true;
                apply_stage_dd(s_phase);
            }
        } else {
            all_off_since_ms = 0;
        }

        if (s_state == TRAFFIC_STATE_NIGHT_YELLOW_BLINK) {
            uint32_t per = g_config.traffic.phases_ms.fault_blink_period;
            if (per < BLINK_MIN_PERIOD_MS) per = 500;
            if (now - blink_last_fault >= per) {
                blink_last_fault = now;
                fault_blink_on = !fault_blink_on;
                fault_blink_yellow(fault_blink_on);
            }
            continue;
        }

        if (s_all_red_active) {
            uint32_t elapsed = now - s_all_red_start_ms;
            if (elapsed < ALL_RED_MIN_MS) continue;

            uint8_t next_seq = s_all_red_next_seq;
            uint32_t next_dur = s_all_red_next_dur_ms;
            s_all_red_active = false;
            s_all_red_next_seq = 0xFF;
            ESP_LOGW(TAG, "virtual ALL-RED done (%lu ms) → seq=%u dur=%lu",
                     (unsigned long)elapsed, next_seq,
                     (unsigned long)next_dur);

            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                s_phase = next_seq;
                s_phase_start_ms = now;
                xSemaphoreGive(s_mutex);
            }
            s_in_force_phase = true;
            apply_stage_dd(next_seq);
            s_in_force_phase = false;
            phase_change_cancel_pending();
            continue;
        }

        if (s_ped_request &&
            g_config.traffic.pedestrian.request_mode == PED_REQUEST_MODE_INSERT) {
            uint32_t age = now - s_ped_request_ms;
            if (age > g_config.traffic.pedestrian.insert_timeout_ms) {
                ESP_LOGW(TAG, "pedestrian INSERT timeout (%lu ms) → cancel",
                         (unsigned long)age);
                s_ped_request = false;
            }
        }

        if (s_fault != TRAFFIC_FAULT_NONE) {
            if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) {
                uint32_t per = g_config.traffic.phases_ms.fault_blink_period;
                if (per < BLINK_MIN_PERIOD_MS) per = 500;
                if (now - blink_last_fault >= per) {
                    blink_last_fault = now;
                    fault_blink_on = !fault_blink_on;
                    fault_blink_yellow(fault_blink_on);
                }
                if (!s_fault_blink_state &&
                    (now - s_last_recovery_ms >= g_config.traffic.fault.recovery_ms)) {
                    try_recovery(now, &blink_last_fault);
                }
            }
            continue;
        }

        /* hold detection */
        {
            bool cur_hold = false;
            if (g_config.traffic.hold.enabled) {
                for (int i = 0; i < g_config.traffic.hold.stage_count; i++) {
                    if (g_config.traffic.sequence[s_phase] ==
                        g_config.traffic.hold.stage_ids[i]) {
                        cur_hold = true;
                        break;
                    }
                }
            }
            if (cur_hold != s_hold_active) {
                ESP_LOGW(TAG, "hold: %s (phase %u seq=%u)",
                         cur_hold ? "ENTER" : "EXIT",
                         g_config.traffic.sequence[s_phase], s_phase);
                s_hold_active = cur_hold;
                if (cur_hold) {
                    s_hold_pm_frozen = true;
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase_start_ms = now;
                        xSemaphoreGive(s_mutex);
                    }
                } else {
                    s_hold_pm_frozen = false;
                }
            }
        }

        if (s_hold_active && !s_insert_pending) {
            if (s_pending_sync_phase != 0xFF) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            if (g_config.traffic.hold.max_hold_ms > 0 &&
                (now - s_phase_start_ms) >= g_config.traffic.hold.max_hold_ms) {
                ESP_LOGW(TAG, "hold: max_hold_ms=%lu reached → force next",
                         (unsigned long)g_config.traffic.hold.max_hold_ms);
            } else {
                if (s_hold_pm_frozen) {
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase_start_ms = now;
                        xSemaphoreGive(s_mutex);
                    }
                }
                continue;
            }
        }

        /* INSERT block */
        if (s_insert_pending) {
            uint32_t since_req = now - s_insert_request_ms;

            if (s_insert_idx == 0 &&
                since_req < g_config.traffic.pedestrian.insert_lead_ms) {
                continue;
            }

            if (s_insert_idx == 0 && s_insert_count > 0 && !s_hold_active) {
                uint8_t first_id = s_insert_queue[0];
                bool same_phase = (s_phase < g_config.traffic.sequence_count) &&
                                  (g_config.traffic.sequence[s_phase] == first_id);
                if (same_phase) {
                    uint32_t dur_hold = effective_stage_duration_ms(s_phase);
                    uint32_t elapsed  = now - s_phase_start_ms;
                    if (elapsed < dur_hold) continue;
                }
            }

            if (s_insert_idx > 0 && s_insert_idx <= s_insert_count && !s_hold_active) {
                uint32_t dur_current = insert_duration_ms(s_phase);
                uint32_t elapsed     = now - s_phase_start_ms;
                if (elapsed < dur_current) continue;
            }

            if (s_insert_idx < s_insert_count) {
                uint8_t ins_id = s_insert_queue[s_insert_idx++];
                uint8_t seq_idx = 0xFF;
                for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                    if (g_config.traffic.sequence[i] == ins_id) {
                        seq_idx = (uint8_t)i;
                        break;
                    }
                }
                if (seq_idx == 0xFF) {
                    ESP_LOGW(TAG, "insert: stage_id=%u not in sequence, skip", ins_id);
                    continue;
                }
                uint32_t dur_ins = insert_duration_ms(seq_idx);
                ESP_LOGW(TAG, "insert: step %u/%u → id=%u (seq=%u) dur=%lu",
                         s_insert_idx, s_insert_count,
                         ins_id, seq_idx, (unsigned long)dur_ins);

                if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    s_phase = seq_idx;
                    s_phase_start_ms = now;
                    s_hold_active = false;
                    s_hold_pm_frozen = false;
                    xSemaphoreGive(s_mutex);
                }
                s_in_force_phase = true;
                apply_stage_dd(seq_idx);
                s_in_force_phase = false;
                phase_change_cancel_pending();
                send_sync_to_node_async_ex(seq_idx, 0, dur_ins, true, NULL);
                continue;
            }

            /* safe_exit + return_to_hold */
            if (g_config.traffic.pedestrian.return_to_hold &&
                g_config.traffic.hold.stage_count > 0) {

                uint8_t hold_id = g_config.traffic.hold.stage_ids[0];

                uint8_t exit_ids[MAX_INSERT_SEQ];
                uint8_t exit_count = 0;

                if (g_config.traffic.pedestrian.safe_exit_count > 0) {
                    for (int i = 0; i < g_config.traffic.pedestrian.safe_exit_count; i++)
                        exit_ids[exit_count++] =
                            g_config.traffic.pedestrian.safe_exit_sequence[i];
                } else {
                    uint8_t fb_red = 0xFF, fb_ry = 0xFF;
                    for (int i = 0; i < g_config.traffic.stage_count; i++) {
                        if (fb_red == 0xFF &&
                            g_config.traffic.stages[i].duration_kind == DD_DUR_RED) {
                            fb_red = g_config.traffic.stages[i].id;
                        }
                        if (fb_ry == 0xFF &&
                            g_config.traffic.stages[i].duration_kind == DD_DUR_RED_YELLOW) {
                            fb_ry = g_config.traffic.stages[i].id;
                        }
                        if (fb_red != 0xFF && fb_ry != 0xFF) break;
                    }
                    if (fb_red != 0xFF) exit_ids[exit_count++] = fb_red;
                    if (fb_ry  != 0xFF) exit_ids[exit_count++] = fb_ry;
                }

                uint8_t hold_seq = 0xFF;
                for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                    if (g_config.traffic.sequence[i] == hold_id) {
                        hold_seq = (uint8_t)i;
                        break;
                    }
                }

                int exit_step = (int)s_insert_idx - (int)s_insert_count;
                if (exit_step < 0) exit_step = 0;

                if (exit_step == 0 && exit_count > 0) {
                    uint32_t dur_last = insert_duration_ms(s_phase);
                    uint32_t elapsed  = now - s_phase_start_ms;
                    if (!s_hold_active && elapsed < dur_last) continue;

                    uint8_t next_id = exit_ids[0];
                    uint8_t seq_idx = 0xFF;
                    for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                        if (g_config.traffic.sequence[i] == next_id) {
                            seq_idx = (uint8_t)i;
                            break;
                        }
                    }
                    if (seq_idx == 0xFF || seq_idx == hold_seq) {
                        ESP_LOGW(TAG, "safe_exit[0]: id=%u not usable, skip", next_id);
                        s_insert_idx = s_insert_count + 1;
                        continue;
                    }
                    uint32_t dur_ex = insert_duration_ms(seq_idx);
                    ESP_LOGW(TAG, "insert DONE → safe_exit[1/%u] id=%u (seq=%u) dur=%lu",
                             exit_count, next_id, seq_idx, (unsigned long)dur_ex);

                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase = seq_idx;
                        s_phase_start_ms = now;
                        xSemaphoreGive(s_mutex);
                    }
                    s_in_force_phase = true;
                    apply_stage_dd(seq_idx);
                    s_in_force_phase = false;
                    phase_change_cancel_pending();
                    send_sync_to_node_async_ex(seq_idx, 0, dur_ex, true, NULL);
                    s_insert_idx = s_insert_count + 1;
                    continue;
                }

                if (exit_step > 0 && exit_step < exit_count) {
                    uint8_t prev_id = exit_ids[exit_step - 1];
                    uint8_t prev_seq = 0xFF;
                    for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                        if (g_config.traffic.sequence[i] == prev_id) {
                            prev_seq = (uint8_t)i;
                            break;
                        }
                    }
                    if (prev_seq != 0xFF) {
                        uint32_t dur_prev = insert_duration_ms(prev_seq);
                        uint32_t elapsed  = now - s_phase_start_ms;
                        if (!s_hold_active && elapsed < dur_prev) continue;
                    }

                    uint8_t next_id = exit_ids[exit_step];
                    uint8_t seq_idx = 0xFF;
                    for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                        if (g_config.traffic.sequence[i] == next_id) {
                            seq_idx = (uint8_t)i;
                            break;
                        }
                    }
                    if (seq_idx == 0xFF || seq_idx == hold_seq) {
                        ESP_LOGW(TAG, "safe_exit[%d]: id=%u not usable, skip",
                                 exit_step, next_id);
                        s_insert_idx = s_insert_count + exit_step + 1;
                        continue;
                    }
                    uint32_t dur_ex = insert_duration_ms(seq_idx);
                    ESP_LOGW(TAG, "insert DONE → safe_exit[%d/%u] id=%u (seq=%u) dur=%lu",
                             exit_step + 1, exit_count, next_id, seq_idx,
                             (unsigned long)dur_ex);
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase = seq_idx;
                        s_phase_start_ms = now;
                        xSemaphoreGive(s_mutex);
                    }
                    s_in_force_phase = true;
                    apply_stage_dd(seq_idx);
                    s_in_force_phase = false;
                    phase_change_cancel_pending();
                    send_sync_to_node_async_ex(seq_idx, 0, dur_ex, true, NULL);
                    s_insert_idx = s_insert_count + exit_step + 1;
                    continue;
                }

                if (hold_seq == 0xFF) {
                    ESP_LOGW(TAG, "insert: hold_id=%u not in sequence → skip",
                             hold_id);
                } else {
                    if (exit_count > 0) {
                        uint8_t last_id = exit_ids[exit_count - 1];
                        uint8_t last_seq = 0xFF;
                        for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                            if (g_config.traffic.sequence[i] == last_id) {
                                last_seq = (uint8_t)i;
                                break;
                            }
                        }
                        if (last_seq != 0xFF) {
                            uint32_t dur_last = insert_duration_ms(last_seq);
                            uint32_t elapsed  = now - s_phase_start_ms;
                            if (!s_hold_active && elapsed < dur_last) continue;
                        }
                    }
                    uint32_t dur_hold = effective_stage_duration_ms(hold_seq);
                    ESP_LOGW(TAG, "insert DONE → return_to_hold id=%u (seq=%u) dur=%lu",
                             hold_id, hold_seq, (unsigned long)dur_hold);
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase = hold_seq;
                        s_phase_start_ms = now;
                        s_hold_pm_frozen = true;
                        xSemaphoreGive(s_mutex);
                    }
                    s_in_force_phase = true;
                    apply_stage_dd(hold_seq);
                    s_in_force_phase = false;
                    phase_change_cancel_pending();
                    send_sync_to_node_async_ex(hold_seq, 0, dur_hold, true, NULL);
                    s_force_resync_pending = true;
                }
            }

            ESP_LOGW(TAG, "insert COMPLETE: idx=%u/%u → clear pending",
                     s_insert_idx, s_insert_count);
            s_insert_pending = false;
            s_insert_idx = 0;
            s_insert_count = 0;
            continue;
        }

        uint16_t bm = s_blink_mask;
        uint32_t per = phase_blink_period_ms();
        if (bm && (now - s_blink_last_phase >= per)) {
            s_blink_last_phase = now;
            s_blink_on = !s_blink_on;
            for (int ch = 0; ch < 16; ch++) {
                if (bm & (1u << ch)) {
                    if (s_blink_on) sr_output_on(ch);
                    else            sr_output_off(ch);
                }
            }
        }

        bool need_fault = false;
        {
            bool in_running = false;
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                in_running = (s_state == TRAFFIC_STATE_RUNNING);
                if (in_running) {
                    uint32_t dur_now = effective_stage_duration_ms(s_phase);
                    uint32_t elapsed = now - s_phase_start_ms;
                    if (dur_now > 0 &&
                        elapsed > dur_now + TRAFFIC_PHASE_OVERDUE_MS) {
                        ESP_LOGE(TAG, "phase %u stuck: elapsed=%lu >> dur=%lu → FAULT",
                                 s_phase, (unsigned long)elapsed,
                                 (unsigned long)dur_now);
                        if (s_fault == TRAFFIC_FAULT_NONE) {
                            s_fault = TRAFFIC_FAULT_YELLOW_BLINK;
                            s_last_recovery_ms = now;
                            s_ok_streak = 0;
                            s_last_recovery_ok_ms = 0;
                            need_fault = true;
                        }
                    }
                }
                xSemaphoreGive(s_mutex);
            }
            if (!in_running) continue;
        }
        if (need_fault) {
            apply_fault_mode(TRAFFIC_FAULT_YELLOW_BLINK);
            s_fault_blink_state = false;
            if (s_fault_notify) s_fault_notify(TRAFFIC_FAULT_YELLOW_BLINK);
            continue;
        }

        if (s_manual_override) continue;

        uint32_t dur = effective_stage_duration_ms(s_phase);

        if (s_ped_request && stage_is_transport_green(s_phase)) {
            uint32_t min_green = g_config.traffic.phases_ms.green / 2;
            if (min_green < 20000) min_green = 20000;
            if (dur > min_green) {
                dur = min_green;
                if (last_min_green_phase != s_phase) {
                    last_min_green_phase = s_phase;
                    ESP_LOGI(TAG, "PED_MIN_GREEN: green %lu → %lu",
                             (unsigned long)g_config.traffic.phases_ms.green,
                             (unsigned long)min_green);
                }
            }
        } else {
            last_min_green_phase = 0xFF;
        }

        if (s_ped_min_recall && stage_is_transport_green(s_phase)) {
            uint32_t max_green = g_config.traffic.pedestrian.min_recall_s * 1000;
            if (max_green > 0 && dur > max_green) {
                dur = max_green;
                if (last_min_recall_phase != s_phase) {
                    last_min_recall_phase = s_phase;
                    ESP_LOGI(TAG, "PED_MIN_RECALL: green %lu → %lu",
                             (unsigned long)g_config.traffic.phases_ms.green,
                             (unsigned long)max_green);
                }
            }
        } else {
            last_min_recall_phase = 0xFF;
        }

        if ((now - s_phase_start_ms) < dur) continue;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) continue;

        uint8_t next_phase;

        if (s_ped_request &&
            g_config.traffic.pedestrian.request_mode == PED_REQUEST_MODE_STAGE) {
            next_phase = next_phase_smooth_ped(s_phase);
            if (next_phase == s_phase) {
                next_phase = (s_phase + 1) % g_config.traffic.sequence_count;
            }
            ESP_LOGI(TAG, "P7: fast → ped phase (seq=%u)", next_phase);
        } else {
            next_phase = (s_phase + 1) % g_config.traffic.sequence_count;
        }

        if (s_ped_request) {
            const traffic_stage_dd_t *nx = find_stage(g_config.traffic.sequence[next_phase]);
            if (nx) {
                for (int g = 0; g < nx->group_count; g++) {
                    if (nx->groups[g] == 7) {
                        s_ped_request = false;
                        ESP_LOGI(TAG, "P7: reached pedestrian phase (seq=%u)", next_phase);
                        break;
                    }
                }
            }
        }

        uint8_t prev_seq = s_phase;
        if (need_virtual_all_red(prev_seq, next_phase)) {
            uint32_t next_dur_calc = effective_stage_duration_ms(next_phase);
            xSemaphoreGive(s_mutex);

            set_all_off();
            s_blink_mask = 0;
            s_blink_on = false;
            s_blink_last_phase = 0;

            s_all_red_active      = true;
            s_all_red_start_ms    = now;
            s_all_red_next_seq    = next_phase;
            s_all_red_next_dur_ms = next_dur_calc;

            ESP_LOGW(TAG, "virtual ALL-RED START (%lu ms) before seq=%u",
                     (unsigned long)ALL_RED_MIN_MS, next_phase);

            /* === FIX ALL-RED-NOTIFY ===
               Явно сообщаем NODE: all-red, все каналы OFF.
               После FIX ALL-RED-GREEN-ONLY в traffic_config.c
               для sequence [3,4,5,1,2,6,7,1,2] all-red не
               вставляется, но на будущее — оставляем. */
            send_all_off_to_node(ALL_RED_MIN_MS);
            /* === END FIX ALL-RED-NOTIFY === */

            continue;
        }

        s_phase = next_phase;
        s_phase_start_ms = now;
        s_hold_pm_frozen = false;
        xSemaphoreGive(s_mutex);
        apply_stage_dd(s_phase);

        if (s_ped_min_recall && stage_is_pedestrian(s_phase)) {
            ESP_LOGW(TAG, "MIN_RECALL: ped phase min %lu s applied",
                     (unsigned long)g_config.traffic.pedestrian.min_recall_s);
            s_ped_min_recall = false;
            last_min_recall_phase = 0xFF;
        }

        ESP_LOGI(TAG, "Phase → seq=%u stage=%u dur=%lu%s",
                 s_phase, g_config.traffic.sequence[s_phase],
                 (unsigned long)stage_duration_ms(s_phase),
                 stage_is_pedestrian(s_phase) ? " (PED)" : "");
    }
}

/* ============================================================
   Остальные функции traffic_light.c
   ============================================================ */

esp_err_t traffic_light_init(void)
{
    if (s_task != NULL) return ESP_OK;
    s_mutex = xSemaphoreCreateMutex();
    s_fault_mutex = xSemaphoreCreateMutex();
    if (!s_mutex || !s_fault_mutex) return ESP_ERR_NO_MEM;
    set_all_off();
    xTaskCreate(traffic_task_loop, "traffic_task", 8192, NULL, 4, &s_task);
    ESP_LOGI(TAG, "Traffic initialized (DD, stages=%u seq=%u ped_button=%d, "
                  "follow_root=%d, hold=%d/%u, insert_seq=%u, safe_exit=%u, "
                  "conflicts=%u)",
             g_config.traffic.stage_count, g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.button_enabled, s_follow_root,
             g_config.traffic.hold.enabled, g_config.traffic.hold.stage_count,
             g_config.traffic.pedestrian.insert_count,
             g_config.traffic.pedestrian.safe_exit_count,
             g_config.traffic.conflict_count);
    return ESP_OK;
}

void traffic_light_set_pins_single(const traffic_pins_single_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.single.red    = pins->pin_red;
    g_config.traffic.pins.single.yellow = pins->pin_yellow;
    g_config.traffic.pins.single.green  = pins->pin_green;
}

void traffic_light_set_pins_cross(const traffic_pins_cross_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.cross.a_north.red    = pins->a_north_red;
    g_config.traffic.pins.cross.a_north.yellow = pins->a_north_yellow;
    g_config.traffic.pins.cross.a_north.green  = pins->a_north_green;
    g_config.traffic.pins.cross.a_south.red    = pins->a_south_red;
    g_config.traffic.pins.cross.a_south.yellow = pins->a_south_yellow;
    g_config.traffic.pins.cross.a_south.green  = pins->a_south_green;
    g_config.traffic.pins.cross.b_west.red     = pins->b_west_red;
    g_config.traffic.pins.cross.b_west.yellow  = pins->b_west_yellow;
    g_config.traffic.pins.cross.b_west.green   = pins->b_west_green;
    g_config.traffic.pins.cross.b_east.red     = pins->b_east_red;
    g_config.traffic.pins.cross.b_east.yellow  = pins->b_east_yellow;
    g_config.traffic.pins.cross.b_east.green   = pins->b_east_green;
}

void traffic_light_set_pins_cross_ped(const traffic_pins_cross_ped_t *pins)
{
    if (!pins) return;
    g_config.traffic.pins.cross_ped.axis_a.red     = pins->axis_a_red;
    g_config.traffic.pins.cross_ped.axis_a.yellow  = pins->axis_a_yellow;
    g_config.traffic.pins.cross_ped.axis_a.green   = pins->axis_a_green;
    g_config.traffic.pins.cross_ped.axis_a2.red    = pins->axis_a2_red;
    g_config.traffic.pins.cross_ped.axis_a2.yellow = pins->axis_a2_yellow;
    g_config.traffic.pins.cross_ped.axis_a2.green  = pins->axis_a2_green;
    g_config.traffic.pins.cross_ped.axis_a3.red    = pins->axis_a3_red;
    g_config.traffic.pins.cross_ped.axis_a3.yellow = pins->axis_a3_yellow;
    g_config.traffic.pins.cross_ped.axis_a3.green  = pins->axis_a3_green;
    g_config.traffic.pins.cross_ped.ped_red        = pins->ped_red;
    g_config.traffic.pins.cross_ped.ped_green      = pins->ped_green;
}

esp_err_t traffic_light_set_mode(traffic_mode_t mode)
{
    if (mode != TRAFFIC_MODE_SINGLE &&
        mode != TRAFFIC_MODE_CROSS &&
        mode != TRAFFIC_MODE_CROSS_PED) return ESP_ERR_INVALID_ARG;
    if (s_state != TRAFFIC_STATE_STOPPED) {
        ESP_LOGW(TAG, "Cannot change mode while running");
        return ESP_ERR_INVALID_STATE;
    }
    s_mode = mode;
    g_config.traffic.mode = (uint8_t)mode;
    s_phase = 0;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_fault_yellow_mask_from_root = 0;
    s_fault_mask_valid            = false;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    s_hold_active = false;
    s_hold_pm_frozen = false;
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
    set_all_off();
    ESP_LOGI(TAG, "Mode set to %d", mode);
    return ESP_OK;
}

/* === FIX MASK-RECEIVED-RESET === */
esp_err_t traffic_light_start(void)
{
    if (g_config.traffic.sequence_count == 0) {
        ESP_LOGE(TAG, "No sequence in config");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (s_state == TRAFFIC_STATE_RUNNING) {
        ESP_LOGI(TAG, "START ignored — already RUNNING (phase=%u)", s_phase);
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }

    s_state = TRAFFIC_STATE_RUNNING;
    s_phase = 0;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_night_mode = false;
    s_hold_pm_frozen = false;

    /* === FIX MASK-RECEIVED-RESET ===
       При старте NODE (follow_root) сбрасываем s_mask_received.
       Иначе после traffic:start NODE работал бы со старой маской
       в check_faults, пока не придёт новый c:. */
    if (s_follow_root) {
        s_mask_received = false;
        s_expected_mask_from_root = 0;
        s_active_mask_from_root   = 0;
    }
    /* === END FIX MASK-RECEIVED-RESET === */

    traffic_fault_t f = s_fault;
    if (f == TRAFFIC_FAULT_NONE) {
        s_fault_count = 0;
        s_fault_yellow_mask = 0;
        s_recovery_attempts = 0;
        s_ok_streak = 0;
        s_last_recovery_ms = s_phase_start_ms;
    } else {
        s_last_recovery_ms = s_phase_start_ms;
        s_fault_blink_state = false;
    }
    xSemaphoreGive(s_mutex);

    if (s_follow_root) {
        ESP_LOGI(TAG, "START (follow_root=true) — STANDBY, "
                      "waiting 'c:' from ROOT (channels OFF)");
        set_all_off();
    } else if (f == TRAFFIC_FAULT_NONE) {
        apply_stage_dd(0);
    } else {
        apply_fault_mode(f);
    }

    ESP_LOGI(TAG, "START (mode=%d, seq=%u, fault=%s, follow_root=%d, hold=%d)",
             s_mode, g_config.traffic.sequence_count,
             traffic_light_fault_str(), s_follow_root,
             (int)s_hold_active);
    return ESP_OK;
}
/* === END FIX MASK-RECEIVED-RESET === */

esp_err_t traffic_light_stop(void)
{
    if (!s_mutex) {
        s_state = TRAFFIC_STATE_STOPPED;
        set_all_off();
        ESP_LOGI(TAG, "STOP (no mutex yet)");
        return ESP_OK;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_state = TRAFFIC_STATE_STOPPED;
    s_phase = 0;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_fault_yellow_mask_from_root = 0;
    s_fault_mask_valid            = false;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_last_recovery_ms = 0;
    s_phase_start_ms = 0;
    s_blink_mask = 0;
    s_blink_on = false;
    s_blink_last_phase = 0;
    s_last_fault_notify_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    s_mask_received = false;
    s_hold_active = false;
    s_hold_pm_frozen = false;
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
    s_night_mode = false;
    s_all_red_active = false;
    s_manual_override = false;
    s_expected_mask_from_root = 0;
    s_active_mask_from_root   = 0;
    xSemaphoreGive(s_mutex);
    set_all_off();
    ESP_LOGI(TAG, "STOP");
    return ESP_OK;
}

esp_err_t traffic_light_pause(void)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_state != TRAFFIC_STATE_RUNNING) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_state = TRAFFIC_STATE_PAUSED;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "PAUSE");
    return ESP_OK;
}

esp_err_t traffic_light_resume(void)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_state != TRAFFIC_STATE_PAUSED) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_state = TRAFFIC_STATE_RUNNING;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "RESUME");
    return ESP_OK;
}

esp_err_t traffic_light_reset(void)
{
    if (!s_mutex) {
        s_state = TRAFFIC_STATE_STOPPED;
        s_phase = 0;
        s_mask_received = false;
        set_all_off();
        ESP_LOGI(TAG, "RESET (no mutex yet)");
        return ESP_OK;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_phase = 0;
    s_state = TRAFFIC_STATE_STOPPED;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_fault_yellow_mask = 0;
    s_fault_yellow_mask_from_root = 0;
    s_fault_mask_valid            = false;
    s_recovery_attempts = 0;
    s_ok_streak = 0;
    s_last_recovery_ok_ms = 0;
    s_last_recovery_ms = 0;
    s_phase_start_ms = 0;
    s_blink_mask = 0;
    s_blink_on = false;
    s_blink_last_phase = 0;
    s_last_fault_notify_ms = 0;
    s_fault_blink_state = false;
    s_ped_request = false;
    s_ped_min_recall = false;
    s_mask_received = false;
    s_hold_active = false;
    s_hold_pm_frozen = false;
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
    s_night_mode = false;
    s_all_red_active = false;
    s_manual_override = false;
    s_expected_mask_from_root = 0;
    s_active_mask_from_root   = 0;
    xSemaphoreGive(s_mutex);
    set_all_off();
    ESP_LOGI(TAG, "RESET");
    return ESP_OK;
}

esp_err_t traffic_light_next_phase(void)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_state != TRAFFIC_STATE_RUNNING) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_phase = (s_phase + 1) % g_config.traffic.sequence_count;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_hold_active = false;
    s_hold_pm_frozen = false;
    s_insert_pending = false;
    uint8_t ph = s_phase;
    xSemaphoreGive(s_mutex);
    s_in_force_phase = true;
    apply_stage_dd(ph);
    s_in_force_phase = false;
    phase_change_cancel_pending();
    ESP_LOGI(TAG, "NEXT → seq=%u", ph);
    return ESP_OK;
}

esp_err_t traffic_light_force_phase(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms)
{
    if (g_config.traffic.sequence_count == 0) return ESP_ERR_INVALID_STATE;
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (phase >= g_config.traffic.sequence_count)
        phase = phase % g_config.traffic.sequence_count;

    if (duration_ms > 0 && phase_ms >= duration_ms) {
        ESP_LOGW(TAG, "force_phase: pm=%lu >= dur=%lu → set 0",
                 (unsigned long)phase_ms, (unsigned long)duration_ms);
        phase_ms = 0;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "force_phase: mutex timeout %d ms", MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool in_fault = (s_fault != TRAFFIC_FAULT_NONE);

    if (s_phase == phase && s_state == TRAFFIC_STATE_RUNNING) {
        s_phase_start_ms = now - phase_ms;
        if (in_fault) {
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    s_phase          = phase;
    s_phase_start_ms = now - phase_ms;
    s_state          = TRAFFIC_STATE_RUNNING;
    xSemaphoreGive(s_mutex);

    if (!in_fault) {
        s_in_force_phase = true;
        apply_stage_dd(phase);
        s_in_force_phase = false;
        phase_change_cancel_pending();
    }
    return ESP_OK;
}

/* ============================================================
   traffic_light_apply_mask
   ============================================================ */
void traffic_light_apply_mask(uint16_t on_mask, uint16_t blink_mask,
                              uint8_t phase, uint32_t phase_ms,
                              uint32_t duration_ms)
{
    if (!s_mutex) return;

    if (s_follow_root && s_sync_delay_us > 0) {
        uint32_t delay_ms = s_sync_delay_us / 1000;
        if (delay_ms > 0 && delay_ms < 2000) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return;

    if (traffic_mask_has_conflict(on_mask)) {
        ESP_LOGE(TAG, "APPLY MASK: CONFLICT 0x%04X → ALL_OFF "
                      "(GOST Р 52289 п.7.4.1)", (unsigned)on_mask);
        set_all_off();
        s_blink_mask = 0;
        s_blink_on = false;
        s_blink_last_phase = 0;
        s_phase = phase;
        s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        s_state = TRAFFIC_STATE_RUNNING;
        s_expected_mask_from_root = 0;
        s_active_mask_from_root   = 0;
        xSemaphoreGive(s_mutex);
        return;
    }

    uint16_t blink_orig = blink_mask;
    blink_mask &= ~on_mask;
    if (blink_orig != blink_mask) {
        ESP_LOGW(TAG, "APPLY MASK: on=0x%04X blink_orig=0x%04X → blink=0x%04X",
                 (unsigned)on_mask, (unsigned)blink_orig,
                 (unsigned)blink_mask);
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (duration_ms > 0 && phase_ms >= duration_ms) {
        ESP_LOGW(TAG, "apply_mask: pm=%lu >= dur=%lu → set 0",
                 (unsigned long)phase_ms, (unsigned long)duration_ms);
        phase_ms = 0;
    }

    s_phase            = phase;
    s_phase_start_ms   = now - phase_ms;
    s_state            = TRAFFIC_STATE_RUNNING;
    s_blink_mask       = blink_mask;
    s_blink_on         = false;
    s_blink_last_phase = 0;

    s_expected_mask_from_root = on_mask;
    s_active_mask_from_root   = on_mask | blink_mask;

    {
        bool new_phase_is_hold = false;
        if (g_config.traffic.hold.enabled) {
            for (int i = 0; i < g_config.traffic.hold.stage_count; i++) {
                if (phase < g_config.traffic.sequence_count &&
                    g_config.traffic.sequence[phase] ==
                    g_config.traffic.hold.stage_ids[i]) {
                    new_phase_is_hold = true;
                    break;
                }
            }
        }
        if (!new_phase_is_hold) {
            s_hold_pm_frozen = false;
        }
    }

    if (!s_mask_received) {
        ESP_LOGI(TAG, "APPLY MASK: first mask from ROOT → fault detection enabled");
        s_mask_received = true;
    }
    xSemaphoreGive(s_mutex);

    for (int ch = 0; ch < 16; ch++) {
        if (!(on_mask & (1u << ch)) &&
            !(blink_mask & (1u << ch))) {
            sr_output_off(ch);
        }
    }
    for (int ch = 0; ch < 16; ch++) {
        if (on_mask & (1u << ch))
            sr_output_on(ch);
    }

    ESP_LOGD(TAG, "APPLY MASK ph=%u pm=%lu dur=%lu on=0x%04X blink=0x%04X",
             phase, (unsigned long)phase_ms, (unsigned long)duration_ms,
             (unsigned)on_mask, (unsigned)blink_mask);
}

esp_err_t traffic_light_force_phase_masked(uint8_t phase,
                                           uint32_t phase_ms,
                                           uint32_t duration_ms)
{
    esp_err_t err = traffic_light_force_phase(phase, phase_ms, duration_ms);
    if (err != ESP_OK) return err;
    return ESP_OK;
}

uint16_t traffic_light_get_phase_on_mask(void)
{
    if (g_config.traffic.sequence_count == 0) return 0;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[s_phase]);
    if (!st) return 0;
    uint16_t m = 0;
    for (int k = 0; k < st->on_count; k++)
        m |= (1u << st->outputs_on[k]);
    return m;
}

uint16_t traffic_light_get_phase_blink_mask(void)
{
    if (g_config.traffic.sequence_count == 0) return 0;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[s_phase]);
    if (!st) return 0;
    uint16_t m = 0;
    for (int k = 0; k < st->blink_count; k++)
        m |= (1u << st->outputs_blink[k]);
    return m;
}

void traffic_light_get_stage_masks(uint8_t phase,
                                   uint16_t *out_on_mask,
                                   uint16_t *out_blink_mask)
{
    if (out_on_mask)    *out_on_mask = 0;
    if (out_blink_mask) *out_blink_mask = 0;
    if (g_config.traffic.sequence_count == 0) return;
    if (phase >= g_config.traffic.sequence_count) return;

    const traffic_stage_dd_t *st =
        find_stage(g_config.traffic.sequence[phase]);
    if (!st) return;

    uint16_t on = 0, blink = 0;
    for (int k = 0; k < st->on_count; k++)
        on |= (1u << st->outputs_on[k]);
    for (int k = 0; k < st->blink_count; k++)
        blink |= (1u << st->outputs_blink[k]);
    blink &= ~on;

    if (out_on_mask)    *out_on_mask = on;
    if (out_blink_mask) *out_blink_mask = blink;
}

traffic_state_t traffic_light_get_state(void) { return s_state; }
traffic_mode_t  traffic_light_get_mode(void)  { return s_mode;  }
uint8_t         traffic_light_get_phase(void) { return s_phase; }
uint8_t         traffic_light_get_phase_count(void) { return g_config.traffic.sequence_count; }

uint32_t traffic_light_get_phase_ms(void)
{
    if (!s_mutex) return 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return 0;
    uint32_t result = 0;
    if (s_state == TRAFFIC_STATE_RUNNING ||
        s_state == TRAFFIC_STATE_NIGHT_YELLOW_BLINK) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        result = now - s_phase_start_ms;
    }
    xSemaphoreGive(s_mutex);
    return result;
}

uint32_t traffic_light_get_phase_duration_ms(void)
{
    return stage_duration_ms(s_phase);
}

const char *traffic_light_state_str(void)
{
    switch (s_state) {
        case TRAFFIC_STATE_RUNNING: return "RUNNING";
        case TRAFFIC_STATE_PAUSED:  return "PAUSED";
        case TRAFFIC_STATE_NIGHT_YELLOW_BLINK: return "NIGHT_YELLOW_BLINK";
        default:                    return "STOPPED";
    }
}

traffic_fault_t traffic_light_get_fault(void) { return s_fault; }

const char *traffic_light_fault_str(void)
{
    switch (s_fault) {
        case TRAFFIC_FAULT_YELLOW_BLINK: return "YELLOW_BLINK";
        case TRAFFIC_FAULT_ALL_OFF:      return "ALL_OFF";
        default:                         return "NONE";
    }
}

uint8_t traffic_light_get_fault_count(void) { return s_fault_count; }

void traffic_light_get_fault_info(uint8_t idx, traffic_fault_info_t *out)
{
    if (!out || idx >= s_fault_count) return;
    *out = s_fault_list[idx];
}

uint16_t traffic_light_active_sr_mask(void)   { return traffic_light_active_sr_mask_internal(); }
uint16_t traffic_light_expected_sr_mask(void) { return traffic_light_expected_sr_mask_internal(); }

bool traffic_light_test_channel(uint8_t ch)
{
    ESP_LOGI(TAG, "Testing ch%02d...", ch);
    uint16_t saved = sr_get_state();
    uint16_t yellow = 0;
    switch (s_mode) {
    case TRAFFIC_MODE_SINGLE:
        yellow = (1u << g_config.traffic.pins.single.yellow); break;
    case TRAFFIC_MODE_CROSS:
        yellow = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                 (1u << g_config.traffic.pins.cross.a_south.yellow) |
                 (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                 (1u << g_config.traffic.pins.cross.b_east.yellow); break;
    case TRAFFIC_MODE_CROSS_PED:
        yellow = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                 (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                 (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow); break;
    }
    sr_write16(saved & yellow);
    board_diag_en_all_off();
    esp_rom_delay_us(500);
    sr_output_on(ch);
    esp_rom_delay_us(50);

    imon_state_t st = IMON_STATE_UNKNOWN;
    int mv = 0;
    int il = board_read_iload_raw(ch, &st, &mv);

    sr_output_off(ch);
    sr_write16(saved);

    bool ok = (st == IMON_STATE_OK && il >= 0 &&
               mv >= g_config.traffic.fault.imin_mv);
    ESP_LOGI(TAG, "Test ch%02d: %s (V_IMON=%d, I=%d)",
             ch, ok ? "OK" : "FAIL", mv, il);
    return ok;
}

uint32_t traffic_light_stage_duration_ms(uint8_t seq_idx)
{
    return stage_duration_ms(seq_idx);
}

esp_err_t traffic_light_set_night_mode(bool enable)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (enable) {
        if (s_state == TRAFFIC_STATE_NIGHT_YELLOW_BLINK) {
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "NIGHT MODE ON (ГОСТ Р 52289 п.4.2.6)");
        s_state = TRAFFIC_STATE_NIGHT_YELLOW_BLINK;
        s_fault = TRAFFIC_FAULT_NONE;
        s_fault_count = 0;
        s_night_mode = true;
        s_blink_mask = 0;
        s_blink_on = false;
        s_blink_last_phase = 0;
        s_mask_received = false;
        s_hold_pm_frozen = false;
        xSemaphoreGive(s_mutex);

        uint16_t yellow = 0;
        switch (s_mode) {
        case TRAFFIC_MODE_SINGLE:
            yellow = (1u << g_config.traffic.pins.single.yellow); break;
        case TRAFFIC_MODE_CROSS:
            yellow = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                     (1u << g_config.traffic.pins.cross.a_south.yellow) |
                     (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                     (1u << g_config.traffic.pins.cross.b_east.yellow); break;
        case TRAFFIC_MODE_CROSS_PED:
            yellow = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                     (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                     (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow); break;
        }
        s_fault_yellow_mask = yellow;
        set_all_off();
        s_fault_blink_state = false;
        if (s_fault_notify) s_fault_notify(TRAFFIC_FAULT_NONE);
        return ESP_OK;
    }

    if (s_state != TRAFFIC_STATE_NIGHT_YELLOW_BLINK) {
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    ESP_LOGW(TAG, "NIGHT MODE OFF → STOPPED (wait traffic:start)");
    s_night_mode = false;
    s_state = TRAFFIC_STATE_STOPPED;
    s_fault_yellow_mask = 0;
    s_fault_blink_state = false;
    xSemaphoreGive(s_mutex);
    set_all_off();
    return ESP_OK;
}

bool traffic_light_is_night_mode(void) { return s_night_mode; }

esp_err_t traffic_light_manual_override(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return ESP_ERR_INVALID_ARG;
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    ESP_LOGW(TAG, "MANUAL OVERRIDE (ДПС): seq=%u", seq_idx);
    s_manual_override = true;
    s_manual_seq = seq_idx;
    s_state = TRAFFIC_STATE_RUNNING;
    s_phase = seq_idx;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_fault = TRAFFIC_FAULT_NONE;
    s_fault_count = 0;
    s_hold_pm_frozen = false;
    xSemaphoreGive(s_mutex);
    s_in_force_phase = true;
    apply_stage_dd(seq_idx);
    s_in_force_phase = false;
    phase_change_cancel_pending();
    return ESP_OK;
}

esp_err_t traffic_light_manual_release(void)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    ESP_LOGW(TAG, "MANUAL OVERRIDE OFF → resume normal sequence");
    s_manual_override = false;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

bool traffic_light_is_manual_override(void) { return s_manual_override; }

#define PRIORITY_MAX_EXTEND_MS  60000

esp_err_t traffic_light_priority_extend(uint32_t ms)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_state != TRAFFIC_STATE_RUNNING ||
        !stage_is_transport_green(s_phase)) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - s_phase_start_ms;
    uint32_t total   = elapsed + ms;
    if (total > PRIORITY_MAX_EXTEND_MS) total = PRIORITY_MAX_EXTEND_MS;
    s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - total;
    xSemaphoreGive(s_mutex);
    ESP_LOGW(TAG, "PRIORITY EXTEND: seq=%u +%lu ms → total=%lu ms",
             s_phase, (unsigned long)ms, (unsigned long)total);
    return ESP_OK;
}

esp_err_t traffic_light_priority_shorten(uint32_t ms)
{
    if (!s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_state != TRAFFIC_STATE_RUNNING ||
        !stage_is_transport_green(s_phase)) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (s_phase_start_ms + ms < now) {
        s_phase_start_ms = now - effective_stage_duration_ms(s_phase);
    } else {
        s_phase_start_ms += ms;
    }
    xSemaphoreGive(s_mutex);
    ESP_LOGW(TAG, "PRIORITY SHORTEN: seq=%u -%lu ms", s_phase,
             (unsigned long)ms);
    return ESP_OK;
}