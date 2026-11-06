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

/* === NEW: HOLD/INSERT: state === */
volatile bool            s_hold_active       = false;
volatile bool            s_insert_pending    = false;
volatile uint8_t         s_insert_queue[MAX_INSERT_SEQ] = {0};
volatile uint8_t         s_insert_count      = 0;
volatile uint8_t         s_insert_idx        = 0;
volatile uint32_t        s_insert_request_ms = 0;
/* === END NEW: HOLD/INSERT === */

traffic_fault_notify_cb_t s_fault_notify = NULL;
traffic_phase_change_cb_t s_phase_change_cb = NULL;

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
    ESP_LOGI(TAG, "follow_root=%d", en);
}

/* ============================================================
   traffic_light_request_pedestrian

   === FIX PED-INSERT-DEDUP ===
   При s_hold_active + HOLD_PED_INSERT: если s_insert_pending уже
   true — игнорируем второй P7.

   === FIX NODE-FOLLOW-ROOT ===
   На NODE с follow_root=true не играем insert локально — ROOT
   сам проиграет insert и разошлёт sync: для каждой фазы. NODE
   иначе получит двойной insert (свой + от ROOT через sync:).
   ============================================================ */
void traffic_light_request_pedestrian(void)
{
    if (!g_config.traffic.pedestrian.button_enabled) {
        ESP_LOGW(TAG, "pedestrian request ignored: button disabled in config");
        return;
    }

    /* === FIX NODE-FOLLOW-ROOT === */
    if (s_follow_root) {
        ESP_LOGI(TAG, "pedestrian request: follow_root → delegate to ROOT "
                      "(wait sync: per phase)");
        return;
    }
    /* === END FIX NODE-FOLLOW-ROOT === */

    if (s_hold_active && g_config.traffic.hold.enabled) {
        if (g_config.traffic.hold.on_ped_request == HOLD_PED_IGNORE) {
            ESP_LOGW(TAG, "pedestrian request ignored: hold + on_ped_request=ignore");
            return;
        }
        if (g_config.traffic.hold.on_ped_request == HOLD_PED_MIN_RECALL) {
            s_ped_min_recall = true;
            ESP_LOGW(TAG, "pedestrian request: hold + min_recall");
            return;
        }
        /* HOLD_PED_INSERT */
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
        ESP_LOGW(TAG, "pedestrian request: INSERT pending (seq=%u, lead=%lu ms)",
                 s_insert_count,
                 (unsigned long)g_config.traffic.pedestrian.insert_lead_ms);
        return;
    }

    uint8_t mode = g_config.traffic.pedestrian.request_mode;

    if (mode == PED_REQUEST_MODE_MIN_RECALL) {
        s_ped_min_recall = true;
        ESP_LOGW(TAG, "pedestrian request: MIN_RECALL (next cycle min %lu s)",
                 (unsigned long)g_config.traffic.pedestrian.min_recall_s);
        return;
    }

    s_ped_request = true;
    s_ped_request_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (mode == PED_REQUEST_MODE_INSERT) {
        ESP_LOGW(TAG, "pedestrian request: INSERT (timeout %lu ms)",
                 (unsigned long)g_config.traffic.pedestrian.insert_timeout_ms);
    } else {
        ESP_LOGW(TAG, "pedestrian request: STAGE");
    }
}

bool traffic_light_ped_button_enabled(void) { return g_config.traffic.pedestrian.button_enabled; }

void traffic_light_set_sync_delay_us(uint32_t delay_us)
{
    s_sync_delay_us = delay_us;
    ESP_LOGI(TAG, "sync_delay = %lu us (%.3f ms)",
             (unsigned long)delay_us, delay_us / 1000.0);
}
uint32_t traffic_light_get_sync_delay_us(void) { return s_sync_delay_us; }
void traffic_light_set_mux_reading(bool en) { s_mux_reading = en; }

static inline uint32_t phase_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.phase_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}

/* ============================================================
   insert_duration_ms — длительность фазы insert.

   === FIX INSERT-RED-SHORT ===
   Для insert-фазы с duration_kind=red используем
   pedestrian.insert_red_ms (если > 0), чтобы не ждать полный
   phases_ms.red (например, 8000) — иначе insert очень длинный.
   Для остальных kind — обычный effective_stage_duration_ms.
   ============================================================ */
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
/* === END FIX INSERT-RED-SHORT === */

void traffic_task_loop(void *arg)
{
    ESP_LOGI(TAG, "Traffic task started (step=20ms, follow_root=%d)", s_follow_root);

    uint32_t blink_last_fault = 0;
    bool     fault_blink_on   = false;
    uint8_t  last_min_green_phase  = 0xFF;
    uint8_t  last_min_recall_phase = 0xFF;
    uint32_t last_phase_log_ms = 0;
    uint32_t last_hold_log_ms  = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (now - last_phase_log_ms >= 5000) {
            last_phase_log_ms = now;
            if (s_state == TRAFFIC_STATE_RUNNING) {
                const char *fault_str = "NONE";
                if (s_fault == TRAFFIC_FAULT_ALL_OFF)          fault_str = "ALL_OFF";
                else if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) fault_str = "YELLOW_BLINK";

                uint32_t utc_sec = 0; uint16_t utc_ms = 0;
                bool utc_ok = time_sync_get(&utc_sec, &utc_ms);
                uint32_t pm = (now - s_phase_start_ms);

                ESP_LOGI(TAG, "PHASE: role=%s seq=%u stage=%u %s%s "
                              "pm=%lu/%lu state=%s fault=%s "
                              "utc=%lu.%03u delta=%ld mask_recv=%d mask_valid=%d "
                              "hold=%d insert=%d",
                         g_config.role == DEVICE_ROLE_MASTER ? "ROOT" : "NODE",
                         s_phase,
                         g_config.traffic.sequence[s_phase],
                         stage_is_pedestrian(s_phase) ? "PED" : "TRANS",
                         s_follow_root ? " (follow_root)" : " (local)",
                         (unsigned long)pm,
                         (unsigned long)effective_stage_duration_ms(s_phase),
                         traffic_light_state_str(),
                         fault_str,
                         utc_ok ? (unsigned long)utc_sec : 0u,
                         utc_ok ? utc_ms : 0,
                         (long)time_sync_get_delta_ms(),
                         (int)s_mask_received,
                         (int)s_fault_mask_valid,
                         (int)s_hold_active,
                         (int)s_insert_pending);
            } else {
                ESP_LOGI(TAG, "PHASE: role=%s state=%s (seq=%u fault=%s)",
                         g_config.role == DEVICE_ROLE_MASTER ? "ROOT" : "NODE",
                         traffic_light_state_str(), s_phase,
                         s_fault == TRAFFIC_FAULT_NONE ? "NONE" :
                         s_fault == TRAFFIC_FAULT_ALL_OFF ? "ALL_OFF" : "YELLOW_BLINK");
            }
        }

        if (s_state != TRAFFIC_STATE_RUNNING) continue;

        /* NODE: только blink + fault */
        if (s_follow_root) {
            if (!s_mask_received && s_fault == TRAFFIC_FAULT_NONE) {
                continue;
            }

            if (s_fault == TRAFFIC_FAULT_NONE) {
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
            } else {
                if (s_blink_mask != 0) {
                    s_blink_mask = 0;
                    s_blink_on = false;
                    s_blink_last_phase = 0;
                }
            }

            if (s_fault == TRAFFIC_FAULT_YELLOW_BLINK) {
                uint32_t per_f = g_config.traffic.phases_ms.fault_blink_period;
                if (per_f < BLINK_MIN_PERIOD_MS) per_f = 500;
                if (now - blink_last_fault >= per_f) {
                    blink_last_fault = now;
                    fault_blink_on = !fault_blink_on;
                    fault_blink_yellow(fault_blink_on);
                }
            }
            continue;
        }

        /* ROOT: обычная логика */
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

        /* === HOLD: определяем, в hold ли текущая фаза === */
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
            }
        }

        /* === FIX HOLD-STABLE: не выходим из hold, если phase_sync_task
           собирается применить force_phase (pending) — иначе s_phase
           меняется извне и hold мерцает ENTER/EXIT. === */
        if (s_hold_active && !s_insert_pending) {
            if (s_pending_sync_phase != 0xFF) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (g_config.traffic.hold.max_hold_ms > 0 &&
                (now - s_phase_start_ms) >= g_config.traffic.hold.max_hold_ms) {
                ESP_LOGW(TAG, "hold: max_hold_ms=%lu reached → force next",
                         (unsigned long)g_config.traffic.hold.max_hold_ms);
                /* пропускаем вниз — обычное переключение */
            } else {
                if (now - last_hold_log_ms >= 5000) {
                    last_hold_log_ms = now;
                    ESP_LOGI(TAG, "hold: phase %u (seq=%u) pm=%lu → waiting",
                             g_config.traffic.sequence[s_phase], s_phase,
                             (unsigned long)(now - s_phase_start_ms));
                }
                continue;
            }
        }
        /* === END FIX HOLD-STABLE === */

        /* ============================================================
           INSERT: проигрывание insert_sequence

           === FIX INSERT-DURATION ===
           Каждая фаза insert проигрывается с её duration_ms
           (из phases_ms), а не пролетает всю очередь за 80 ms.
           insert_lead_ms применяется ТОЛЬКО к первой фазе
           (s_insert_idx == 0).
           === END FIX INSERT-DURATION ===
           ============================================================ */
        if (s_insert_pending) {
            uint32_t since_req = now - s_insert_request_ms;

            /* lead_ms — только перед ПЕРВОЙ insert-фазой */
            if (s_insert_idx == 0 &&
                since_req < g_config.traffic.pedestrian.insert_lead_ms) {
                continue;
            }

            /* Ждём окончания dur ТЕКУЩЕЙ insert-фазы (со 2-й итерации).
               s_insert_idx > 0 && s_insert_idx <= s_insert_count —
               значит, уже применена хотя бы одна insert-фаза, и
               s_phase указывает на неё. */
            if (s_insert_idx > 0 && s_insert_idx <= s_insert_count) {
                uint32_t dur_current = insert_duration_ms(s_phase);
                uint32_t elapsed     = now - s_phase_start_ms;
                if (elapsed < dur_current) {
                    continue;  /* ещё рано переключать */
                }
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
                ESP_LOGW(TAG, "insert: phase id=%u (seq=%u) dur=%lu",
                         ins_id, seq_idx, (unsigned long)dur_ins);

                if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    s_phase = seq_idx;
                    s_phase_start_ms = now;
                    s_hold_active = false;
                    xSemaphoreGive(s_mutex);
                }

                /* === FIX INSERT-NO-DOUBLE-SYNC ===
                   apply_stage_dd под s_in_force_phase — не дёргает
                   s_phase_change_cb. Потом гасим pending (на случай
                   гонки с phase_sync_task). sync: шлём сами с force. */
                s_in_force_phase = true;
                apply_stage_dd(seq_idx);
                s_in_force_phase = false;
                phase_change_cancel_pending();

                send_sync_to_node_async_ex(seq_idx, 0, dur_ins, true, NULL);
                /* === END FIX INSERT-NO-DOUBLE-SYNC === */
                continue;
            }

            /* конец очереди — return_to_hold */
            if (g_config.traffic.pedestrian.return_to_hold &&
                g_config.traffic.hold.stage_count > 0) {
                /* Ждём окончания dur ПОСЛЕДНЕЙ insert-фазы */
                uint32_t dur_last = insert_duration_ms(s_phase);
                uint32_t elapsed  = now - s_phase_start_ms;
                if (elapsed < dur_last) {
                    continue;
                }

                uint8_t hold_id = g_config.traffic.hold.stage_ids[0];
                uint8_t seq_idx = 0xFF;
                for (int i = 0; i < g_config.traffic.sequence_count; i++) {
                    if (g_config.traffic.sequence[i] == hold_id) {
                        seq_idx = (uint8_t)i;
                        break;
                    }
                }
                if (seq_idx != 0xFF) {
                    uint32_t dur_hold = effective_stage_duration_ms(seq_idx);
                    ESP_LOGW(TAG, "insert: return to hold phase id=%u (seq=%u)",
                             hold_id, seq_idx);
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        s_phase = seq_idx;
                        s_phase_start_ms = now;
                        xSemaphoreGive(s_mutex);
                    }
                    s_in_force_phase = true;
                    apply_stage_dd(seq_idx);
                    s_in_force_phase = false;
                    phase_change_cancel_pending();
                    send_sync_to_node_async_ex(seq_idx, 0, dur_hold, true, NULL);
                }
            }
            s_insert_pending = false;
            s_insert_idx = 0;
            s_insert_count = 0;
            continue;
        }
        /* === END INSERT === */

        /* обычный blink_mask (не hold, не insert) */
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

        uint32_t dur = effective_stage_duration_ms(s_phase);

        if (s_ped_request && stage_is_transport_green(s_phase)) {
            uint32_t min_green = g_config.traffic.phases_ms.green / 2;
            if (dur > min_green) {
                dur = min_green;
                if (last_min_green_phase != s_phase) {
                    last_min_green_phase = s_phase;
                    ESP_LOGI(TAG, "PED_MIN_GREEN: green %lu → %lu (request)",
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
                    ESP_LOGI(TAG, "PED_MIN_RECALL: green %lu → %lu (recall %lu s)",
                             (unsigned long)g_config.traffic.phases_ms.green,
                             (unsigned long)max_green,
                             (unsigned long)g_config.traffic.pedestrian.min_recall_s);
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

        s_phase = next_phase;
        s_phase_start_ms = now;
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

esp_err_t traffic_light_init(void)
{
    if (s_task != NULL) return ESP_OK;
    s_mutex = xSemaphoreCreateMutex();
    s_fault_mutex = xSemaphoreCreateMutex();
    if (!s_mutex || !s_fault_mutex) return ESP_ERR_NO_MEM;
    set_all_off();
    xTaskCreate(traffic_task_loop, "traffic_task", 8192, NULL, 4, &s_task);
    ESP_LOGI(TAG, "Traffic initialized (DD, stages=%u seq=%u ped_button=%d, "
                  "follow_root=%d, hold=%d/%u, insert_seq=%u)",
             g_config.traffic.stage_count, g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.button_enabled, s_follow_root,
             g_config.traffic.hold.enabled, g_config.traffic.hold.stage_count,
             g_config.traffic.pedestrian.insert_count);
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
        mode != TRAFFIC_MODE_CROSS_PED)
        return ESP_ERR_INVALID_ARG;
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
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
    set_all_off();
    ESP_LOGI(TAG, "Mode set to %d", mode);
    return ESP_OK;
}

esp_err_t traffic_light_start(void)
{
    if (g_config.traffic.sequence_count == 0) {
        ESP_LOGE(TAG, "No sequence in config");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_mutex) {
        ESP_LOGW(TAG, "START before init — ignored");
        return ESP_ERR_INVALID_STATE;
    }
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
                      "waiting sync: from ROOT (channels OFF)");
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
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
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
    s_insert_pending = false;
    s_insert_count = 0;
    s_insert_idx = 0;
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
    s_insert_pending = false;
    uint8_t ph = s_phase;
    xSemaphoreGive(s_mutex);
    apply_stage_dd(ph);
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
        if (!s_follow_root) {
            if (phase_ms > 0) {
                s_phase_start_ms = now - phase_ms;
                xSemaphoreGive(s_mutex);
                ESP_LOGI(TAG, "force_phase: same phase=%u (ROOT), resync pm=%lu",
                         phase, (unsigned long)phase_ms);
                return ESP_OK;
            }
            ESP_LOGD(TAG, "force_phase: same phase=%u (root), no change", phase);
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
        s_phase_start_ms = now - phase_ms;
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "force_phase: same phase=%u (NODE), pm=%lu sync",
                 phase, (unsigned long)phase_ms);
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
    }

    ESP_LOGI(TAG, "FORCE phase=%u phase_ms=%lu (fault=%s)",
             phase, (unsigned long)phase_ms,
             in_fault ? "ACTIVE" : "NONE");
    return ESP_OK;
}

void traffic_light_apply_mask(uint16_t on_mask, uint16_t blink_mask,
                              uint8_t phase, uint32_t phase_ms,
                              uint32_t duration_ms)
{
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
        return;

    uint16_t blink_orig = blink_mask;
    blink_mask &= ~on_mask;
    if (blink_orig != blink_mask) {
        ESP_LOGW(TAG, "APPLY MASK: on=0x%04X blink_orig=0x%04X → blink=0x%04X "
                      "(blink ∩ on removed)",
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
    if (!s_mask_received) {
        ESP_LOGI(TAG, "APPLY MASK: first mask from ROOT → fault detection enabled");
        s_mask_received = true;
    }
    xSemaphoreGive(s_mutex);

    uint16_t active = traffic_light_active_sr_mask_internal();
    for (int ch = 0; ch < 16; ch++) {
        if ((active & (1u << ch)) && !(on_mask & (1u << ch)))
            sr_output_off(ch);
    }
    for (int ch = 0; ch < 16; ch++) {
        if (on_mask & (1u << ch))
            sr_output_on(ch);
    }

    ESP_LOGI(TAG, "APPLY MASK ph=%u pm=%lu dur=%lu on=0x%04X blink=0x%04X",
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
    if (s_state == TRAFFIC_STATE_RUNNING) {
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