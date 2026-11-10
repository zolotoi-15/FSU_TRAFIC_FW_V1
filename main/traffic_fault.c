/* ============================================================
   traffic_fault.c — аварии: apply_fault_mode, try_recovery,
   check_faults, set_fault, build_fault_report.

   === FIX NODE-EXPECTED-FROM-APPLY-MASK ===
       На NODE (s_follow_root && s_mask_received) check_faults
       использует s_expected_mask_from_root / s_active_mask_from_root
       из apply_mask (c: от ROOT), а не g_config.traffic.sequence.
       Иначе NODE (g_config из NVS — старый) ожидает не ту маску,
       что прислал ROOT → ложный V_IN low.

   === FIX RECOVERY-MUX-MUTEX ===
       try_recovery берёт mux_get_mutex() вокруг board_read_iload_raw().

   === FIX RECOVERY-EVT-ERR ===
       transport_send_evt("fault:recover_ok") проверяет ошибку.

   === FIX HOLD-PM-FROZEN-PRESERVE ===
       apply_fault_mode не сбрасывает s_hold_pm_frozen.

   === FIX NODE-REMOTE-FAULT-NO-LOCAL-CHECK ===
       На NODE (follow_root) check_faults полностью НЕ выполняется,
       если fault был установлен удалённо (s_fault_from_remote_ex
       = true при apply_fault_mode). Это защита от
       самоподдерживающегося цикла:
         ROOT → fault:yellow_blink → NODE (s_fault=YELLOW_BLINK)
         → STATUS с FAULT.mode=YELLOW_BLINK → ROOT применяет
         fault:yellow_blink:node → ROOT в YELLOW_BLINK
         → ROOT снова шлёт fault:yellow_blink → ...
       Раньше NODE, получив fault от ROOT, мог сам найти
       V_IN low (по устаревшему g_config) и слать свой FAULT
       в STATUS — ROOT подтверждал. Цикл не разрывался.

   === FIX NODE-CHECK-FAULTS-SKIP-REMOTE ===
       На NODE, если s_fault != NONE (в т.ч. YELLOW_BLINK от
       ROOT), check_faults не должен пытаться найти НОВЫЕ fault —
       он вернётся сразу. NODE «подтверждает» fault только через
       try_recovery (recover_ok) и STATUS.FAULT.

   === FIX CHECK-FAULTS-GRACE-AFTER-APPLY-MASK ===
       После apply_mask (новый c:) даём grace-период
       (g_config.traffic.fault.phase_grace_ms), в течение
       которого check_faults не выполняется. Иначе NODE может
       проверить маску ДО того, как SR-регистр реально
       обновился (sr_output_off/on идут последовательно,
       а mux_task читает ILOAD параллельно) → ложный V_IN low
       на каналах, которые только что были включены.
   ============================================================ */

#include "traffic_internal.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
#include "transport.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "traffic_fault";

extern esp_err_t board_diag_en_set(uint8_t channel, bool enable);
extern void      board_diag_en_all_off(void);
extern int       board_read_iload_raw(uint8_t channel,
                                      imon_state_t *out_state,
                                      int *out_mv);

extern volatile bool s_mask_received;
extern volatile bool s_fault_from_remote_ex;

volatile uint16_t s_fault_yellow_mask_from_root = 0;
volatile bool     s_fault_mask_valid            = false;

static inline uint32_t fault_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.fault_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}

/* ============================================================
   apply_fault_mode
   === FIX HOLD-PM-FROZEN-PRESERVE ===
   ============================================================ */
void apply_fault_mode(traffic_fault_t fault)
{
    set_all_off();
    s_blink_mask       = 0;
    s_blink_on         = false;
    s_blink_last_phase = 0;
    s_fault_yellow_mask = 0;
    /* s_hold_pm_frozen НЕ трогаем — см. FIX HOLD-PM-FROZEN-PRESERVE */

    if (fault == TRAFFIC_FAULT_ALL_OFF) {
        ESP_LOGE(TAG, "FAULT → ALL_OFF");
        return;
    }

    if (s_follow_root && s_fault_mask_valid) {
        s_fault_yellow_mask = s_fault_yellow_mask_from_root;
        ESP_LOGE(TAG, "FAULT → YELLOW_BLINK (mask=0x%04X from ROOT)",
                 s_fault_yellow_mask);
        return;
    }

    switch (s_mode) {
    case TRAFFIC_MODE_SINGLE:
        s_fault_yellow_mask = (1u << g_config.traffic.pins.single.yellow);
        break;
    case TRAFFIC_MODE_CROSS:
        s_fault_yellow_mask = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                              (1u << g_config.traffic.pins.cross.a_south.yellow) |
                              (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                              (1u << g_config.traffic.pins.cross.b_east.yellow);
        break;
    case TRAFFIC_MODE_CROSS_PED:
        s_fault_yellow_mask = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                              (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                              (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow);
        break;
    }
    ESP_LOGE(TAG, "FAULT → YELLOW_BLINK (mask=0x%04X by s_mode)",
             s_fault_yellow_mask);
}

void fault_blink_yellow(bool on)
{
    if (s_mux_reading) return;
    uint16_t m = s_fault_yellow_mask;
    if (!m) return;
    for (int ch = 0; ch < 16; ch++) {
        if (m & (1u << ch)) {
            if (on) sr_output_on(ch);
            else    sr_output_off(ch);
        }
    }
    s_fault_blink_state = on;
}

/* ============================================================
   try_recovery
   === FIX RECOVERY-MUX-MUTEX ===
   === FIX RECOVERY-EVT-ERR ===
   ============================================================ */
void try_recovery(uint32_t now_ms, uint32_t *blink_last)
{
    if (transport_is_cfg_rx_in_progress()) return;

    traffic_fault_info_t snap[TRAFFIC_FAULT_MAX];
    uint8_t cnt = 0;
    if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        cnt = s_fault_count;
        if (cnt > TRAFFIC_FAULT_MAX) cnt = TRAFFIC_FAULT_MAX;
        memcpy(snap, s_fault_list, cnt * sizeof(traffic_fault_info_t));
        xSemaphoreGive(s_fault_mutex);
    }
    if (cnt == 0) { s_last_recovery_ms = now_ms; return; }

    bool all_ok = true;
    uint8_t first_bad = 0xFF;

    for (uint8_t i = 0; i < cnt; i++) {
        uint8_t ch = snap[i].channel;
        if (ch >= 16) continue;

        SemaphoreHandle_t mux_mtx = mux_get_mutex();
        if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);

        bool was_on = sr_get_output(ch);
        if (!was_on) {
            sr_output_on(ch);
            vTaskDelay(pdMS_TO_TICKS(MUX_FAULT_SETTLE_MS));
        }

        imon_state_t st = IMON_STATE_UNKNOWN;
        int mv_raw = 0;
        int iload  = board_read_iload_raw(ch, &st, &mv_raw);

        if (!was_on) sr_output_off(ch);
        if (mux_mtx) xSemaphoreGive(mux_mtx);

        bool ok = (st == IMON_STATE_OK &&
                   iload >= 0 &&
                   mv_raw >= g_config.traffic.fault.imin_mv);

        ESP_LOGI(TAG, "recovery: ch%02d test st=%-7s mv=%d il=%d %s",
                 ch, imon_state_str(st), mv_raw, iload,
                 ok ? "OK" : "FAIL");

        if (!ok) {
            all_ok = false;
            if (first_bad == 0xFF) first_bad = ch;
        }
    }

    bool fire = false;
    bool node_needs_root_confirm = false;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (all_ok) {
            s_recovery_attempts++;
            if (++s_ok_streak < RECOVERY_OK_STREAK) {
                ESP_LOGW(TAG, "Recovery attempt OK (streak %u/%u)",
                         s_ok_streak, RECOVERY_OK_STREAK);
                s_last_recovery_ms = now_ms;
                xSemaphoreGive(s_mutex);
                return;
            }
            s_ok_streak = 0;
            for (int ch = 0; ch < 16; ch++) {
                s_vin_off_counter[ch] = 0;
                s_suspect_counter[ch] = 0;
            }
            if (s_follow_root) {
                node_needs_root_confirm = true;
                s_last_recovery_ms = now_ms;
            } else {
                ESP_LOGW(TAG, "Recovery CONFIRMED → resume phase %u", s_phase);
                s_fault = TRAFFIC_FAULT_NONE;
                s_fault_count = 0;
                s_fault_yellow_mask = 0;
                s_fault_yellow_mask_from_root = 0;
                s_fault_mask_valid            = false;
                s_last_fault_notify_ms = 0;
                s_last_recovery_ok_ms = now_ms;
                s_fault_blink_state = false;
                s_phase_start_ms = now_ms;
                s_last_recovery_ms = now_ms;
                fire = true;
            }
        } else {
            s_ok_streak = 0;
            s_recovery_attempts++;
            s_last_recovery_ms = now_ms;
            ESP_LOGE(TAG, "recovery failed: ch%02d still broken", first_bad);
        }
        xSemaphoreGive(s_mutex);
    }

    if (node_needs_root_confirm) {
        /* === FIX RECOVERY-EVT-ERR === */
        esp_err_t err = transport_send_evt("fault:recover_ok", 16);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "recovery: NODE ok but ROOT unreachable (%s)",
                     esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG, "recovery: NODE ok → request ROOT");
        }
        /* === END FIX RECOVERY-EVT-ERR === */
        return;
    }

    if (fire) apply_stage_dd(s_phase);
    if (fire && s_fault_notify) s_fault_notify(TRAFFIC_FAULT_NONE);
}

const char *reason_str(uint8_t reason)
{
    switch (reason) {
    case TRAFFIC_REASON_VIN_ON_OFF:       return "V_IN on OFF";
    case TRAFFIC_REASON_SHORT_GND:        return "short to GND";
    case TRAFFIC_REASON_NO_LOAD:          return "no load";
    case TRAFFIC_REASON_OVERLOAD:         return "overload";
    case TRAFFIC_REASON_UNDERLOAD:        return "underload";
    case TRAFFIC_REASON_SHORTED_TOGETHER: return "shorted together";
    case TRAFFIC_REASON_NO_DIAG:          return "no DIAG_EN";
    case TRAFFIC_REASON_VIN_LOW:          return "V_IN low";
    default:                              return "unknown";
    }
}

void fill_reason_text(traffic_fault_info_t *fi, const char *detail, uint8_t paired)
{
    if (paired != 0xFF) {
        snprintf(fi->text, sizeof(fi->text),
                 "ch%02d: %s (paired with ch%02d)",
                 fi->channel, detail, paired);
    } else {
        snprintf(fi->text, sizeof(fi->text),
                 "ch%02d: %s", fi->channel, detail);
    }
}

uint8_t classify_channel(uint8_t ch, bool should_be_on,
                         const mux_data_t *data, const char **out_detail)
{
    *out_detail = NULL;
    if (should_be_on) {
        if (data->imon_state[ch] == IMON_STATE_FAULT) {
            *out_detail = "short to GND";
            return TRAFFIC_REASON_SHORT_GND;
        }
        if (data->imon_state[ch] == IMON_STATE_SUSPECT) {
            *out_detail = "V_IMON suspect";
            return TRAFFIC_REASON_NO_LOAD;
        }
        if (data->imon_state[ch] == IMON_STATE_NO_DIAG) {
            *out_detail = "DIAG_EN missing";
            return TRAFFIC_REASON_NO_DIAG;
        }
        if (data->v_in[ch] < g_config.traffic.fault.vin_on_min_v) {
            *out_detail = "V_IN low while ON";
            return TRAFFIC_REASON_VIN_LOW;
        }
        if (data->imon_mv[ch] < g_config.traffic.fault.imin_mv) {
            *out_detail = "no current";
            return TRAFFIC_REASON_NO_LOAD;
        }
        if (data->i_load[ch] > TRAFFIC_OVERLOAD_MA) {
            *out_detail = "overload";
            return TRAFFIC_REASON_OVERLOAD;
        }
        if (data->i_load[ch] > TRAFFIC_UNDERLOAD_MIN_MA &&
            data->i_load[ch] < TRAFFIC_UNDERLOAD_MAX_MA) {
            *out_detail = "underload";
            return TRAFFIC_REASON_UNDERLOAD;
        }
    } else {
        if (data->v_in[ch] >= g_config.traffic.fault.vin_off_min_v) {
            *out_detail = "V_IN present while OFF";
            return TRAFFIC_REASON_VIN_ON_OFF;
        }
    }
    return TRAFFIC_REASON_NONE;
}

/* ============================================================
   traffic_light_check_faults
   === FIX NODE-EXPECTED-FROM-APPLY-MASK ===
   === FIX NODE-REMOTE-FAULT-NO-LOCAL-CHECK ===
   === FIX NODE-CHECK-FAULTS-SKIP-REMOTE ===
   === FIX CHECK-FAULTS-GRACE-AFTER-APPLY-MASK ===
   ============================================================ */
void traffic_light_check_faults(const mux_data_t *data)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return;
    if (!data || !data->valid) return;

    /* === FIX NODE-REMOTE-FAULT-NO-LOCAL-CHECK ===
       На NODE, если fault был установлен удалённо (ROOT прислал
       fault:yellow_blink), не пытаемся найти НОВЫЕ fault локально.
       Раньше NODE мог подтвердить чужой fault своим V_IN low
       (по устаревшему g_config) — ROOT применял его повторно,
       возникал самоподдерживающийся цикл YELLOW_BLINK. */
    if (s_follow_root && s_fault_from_remote_ex) {
        return;
    }
    /* === END FIX NODE-REMOTE-FAULT-NO-LOCAL-CHECK === */

    if (s_fault != TRAFFIC_FAULT_NONE) return;

    if (s_follow_root && !s_mask_received) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - s_phase_start_ms < g_config.traffic.fault.phase_grace_ms) return;

    /* === FIX NODE-EXPECTED-FROM-APPLY-MASK === */
    uint16_t active;
    uint16_t expected;
    if (s_follow_root && s_mask_received) {
        active   = s_active_mask_from_root;
        expected = s_expected_mask_from_root;
    } else {
        active   = traffic_light_active_sr_mask_internal();
        expected = traffic_light_expected_sr_mask_internal();
    }
    /* === END FIX NODE-EXPECTED-FROM-APPLY-MASK === */

    uint16_t blink    = s_blink_mask;

    uint8_t n_faults = 0;
    traffic_fault_info_t list[TRAFFIC_FAULT_MAX];
    bool force_all_off = false;
    bool vin_off_flag[16] = {0};

    for (uint8_t ch = 0; ch < 16; ch++) {
        if (!(active & (1u << ch))) continue;
        if (blink & (1u << ch)) continue;

        bool should_be_on = (expected & (1u << ch)) != 0;

        const char *detail = NULL;
        uint8_t reason = classify_channel(ch, should_be_on, data, &detail);

        if (!should_be_on && reason == TRAFFIC_REASON_VIN_ON_OFF) {
            if (++s_vin_off_counter[ch] >= g_config.traffic.fault.vin_off_confirmations) {
                vin_off_flag[ch] = true;
                force_all_off = true;
            } else {
                reason = TRAFFIC_REASON_NONE;
                detail = NULL;
            }
        } else if (should_be_on) {
            s_vin_off_counter[ch] = 0;
        }

        if (reason == TRAFFIC_REASON_NO_LOAD &&
            data->imon_state[ch] == IMON_STATE_SUSPECT) {
            if (++s_suspect_counter[ch] < 3) {
                reason = TRAFFIC_REASON_NONE;
                detail = NULL;
            }
        } else if (should_be_on) {
            s_suspect_counter[ch] = 0;
        }

        if (reason == TRAFFIC_REASON_NONE) continue;

        if (n_faults < TRAFFIC_FAULT_MAX) {
            traffic_fault_info_t *fi = &list[n_faults];
            memset(fi, 0, sizeof(*fi));
            fi->channel = ch;
            fi->phase   = s_phase;
            fi->mode    = s_mode;
            fi->type    = (reason == TRAFFIC_REASON_VIN_ON_OFF)
                          ? TRAFFIC_FAULT_TYPE_VIN_ON_OFF
                          : TRAFFIC_FAULT_TYPE_LOST;
            fi->reason  = reason;
            fi->paired_channel = 0xFF;
            fill_reason_text(fi, detail ? detail : reason_str(reason), 0xFF);
        }
        n_faults++;

        ESP_LOGW(TAG, "FAULT ch%02d: %s", ch,
                 detail ? detail : reason_str(reason));
    }

    if (n_faults == 0) return;

    traffic_fault_t nf = force_all_off
        ? TRAFFIC_FAULT_ALL_OFF
        : TRAFFIC_FAULT_YELLOW_BLINK;

    bool fire = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        bool already = (s_fault == nf);
        uint8_t prev_count = s_fault_count;
        if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_fault_count = (n_faults < TRAFFIC_FAULT_MAX) ? n_faults : TRAFFIC_FAULT_MAX;
            memcpy(s_fault_list, list, s_fault_count * sizeof(traffic_fault_info_t));
            xSemaphoreGive(s_fault_mutex);
        }
        s_fault = nf;
        s_last_recovery_ms = now_ms;
        if (!already || s_fault_count != prev_count) {
            fire = true;
            s_ok_streak = 0;
            s_last_recovery_ok_ms = 0;
        }
        if (fire) s_last_fault_notify_ms = now_ms;
        xSemaphoreGive(s_mutex);
        if (fire) {
            apply_fault_mode(nf);
            s_fault_blink_state = false;
        }
    }
    if (fire && s_fault_notify) s_fault_notify(nf);
}

/* ============================================================
   traffic_light_set_fault
   ============================================================ */
void traffic_light_set_fault(traffic_fault_t fault)
{
    if (fault == TRAFFIC_FAULT_YELLOW_BLINK &&
        s_state == TRAFFIC_STATE_RUNNING &&
        g_config.traffic.sequence_count > 0 &&
        s_phase < g_config.traffic.sequence_count &&
        stage_is_pedestrian(s_phase)) {
        static uint32_t s_last_block_warn_ms = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_last_block_warn_ms == 0 ||
            (now - s_last_block_warn_ms) > 5000) {
            s_last_block_warn_ms = now;
            ESP_LOGW(TAG, "set_fault(YELLOW_BLINK) BLOCKED: "
                          "pedestrian phase %u active (GOST Р 52289)",
                     s_phase);
        }
        return;
    }

    bool fire = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "set_fault: mutex timeout");
        return;
    }
    bool already = (s_fault == fault);
    s_fault = fault;
    if (fault != TRAFFIC_FAULT_NONE) {
        s_last_recovery_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (!already) {
            fire = true;
            s_ok_streak = 0;
            s_last_recovery_ok_ms = 0;
        }
        if (fire) s_last_fault_notify_ms = s_last_recovery_ms;
    } else {
        s_fault_yellow_mask = 0;
        s_fault_yellow_mask_from_root = 0;
        s_fault_mask_valid            = false;
        s_fault_count = 0;
        s_last_fault_notify_ms = 0;
        s_fault_blink_state = false;
        s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    xSemaphoreGive(s_mutex);

    if (fault != TRAFFIC_FAULT_NONE) {
        if (fire) {
            apply_fault_mode(fault);
            s_fault_blink_state = false;
        }
    } else {
        if (s_state == TRAFFIC_STATE_RUNNING) {
            apply_stage_dd(s_phase);
        }
    }
    if (fire && s_fault_notify) s_fault_notify(fault);
}

static const char *fault_mode_str(traffic_fault_t f)
{
    switch (f) {
        case TRAFFIC_FAULT_YELLOW_BLINK: return "yellow_blink";
        case TRAFFIC_FAULT_ALL_OFF:      return "all_off";
        default:                         return "none";
    }
}

size_t traffic_light_build_fault_report(char *buf, size_t size)
{
    if (!buf || size < 16) return 0;

    traffic_fault_info_t snap[TRAFFIC_FAULT_MAX];
    uint8_t cnt = 0;
    traffic_fault_t f = s_fault;

    if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        cnt = s_fault_count;
        if (cnt > TRAFFIC_FAULT_MAX) cnt = TRAFFIC_FAULT_MAX;
        memcpy(snap, s_fault_list, cnt * sizeof(traffic_fault_info_t));
        xSemaphoreGive(s_fault_mutex);
    }

    int off = snprintf(buf, size, "fault:%s:", fault_mode_str(f));

    if (f == TRAFFIC_FAULT_YELLOW_BLINK) {
        off += snprintf(buf + off, size - off, "0x%04X:",
                        (unsigned)s_fault_yellow_mask);
    }

    bool first = true;
    for (uint8_t i = 0; i < cnt; i++) {
        if (!first) off += snprintf(buf + off, size - off, ",");
        off += snprintf(buf + off, size - off, "%u", snap[i].channel);
        first = false;
    }
    off += snprintf(buf + off, size - off, ":");

    first = true;
    for (uint8_t i = 0; i < cnt; i++) {
        if (!first) off += snprintf(buf + off, size - off, ",");
        off += snprintf(buf + off, size - off, "%u", snap[i].reason);
        first = false;
    }
    return (off > 0) ? (size_t)off : 0;
}