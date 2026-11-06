/* ============================================================
   traffic_fault.c — аварии: apply_fault_mode, try_recovery,
   check_faults, set_fault, build_fault_report.

   === FIX P0-1: NODE не проверяет fault, пока не получена первая
       маска (sync:) от ROOT.
   === FIX 1: apply_fault_mode сбрасывает s_blink_mask / s_blink_on /
       s_blink_last_phase — иначе в follow_root одновременно мигают
       старый s_blink_mask (зелёный) и fault_blink_yellow (жёлтый).
   === FIX 4: для NODE в follow_root маска жёлтых берётся из
       s_fault_yellow_mask_from_root (передаётся ROOT в
       fault:yellow_blink:0xXXXX:...). s_fault_mask_valid отличает
       «маска не передана» от «передана 0x0000» (валидная пустая).
       Fallback — по s_mode (legacy / ROOT).
   === FIX 5: fault мигает даже без s_mask_received.
   === FIX GOST-PED-YELLOW-BLINK:
       set_fault(YELLOW_BLINK) блокируется, если активна
       пешеходная фаза (ГОСТ Р 52289: жёлтое мигание не
       должно заставать пешеходов в переходе).
   ============================================================ */

#include "traffic_internal.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
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

/* === FIX P0-1: флаг «NODE получил первую маску от ROOT» === */
extern volatile bool s_mask_received;
/* === END FIX P0-1 === */

/* === FIX 4: маска жёлтых, переданная ROOT в fault:yellow_blink:...
   s_fault_mask_valid отличает «не передана» от «передана 0x0000». */
volatile uint16_t s_fault_yellow_mask_from_root = 0;
volatile bool     s_fault_mask_valid            = false;
/* === END FIX 4 === */

static inline uint32_t fault_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.fault_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}

/* ============================================================
   apply_fault_mode

   === FIX 1: сбрасываем s_blink_mask / s_blink_on / s_blink_last_phase.
   === FIX 4: маска жёлтых:
       - если s_follow_root && s_fault_mask_valid
         → используем s_fault_yellow_mask_from_root (ROOT прислал);
       - иначе — вычисляем по s_mode (ROOT или legacy NODE).
   ============================================================ */
void apply_fault_mode(traffic_fault_t fault)
{
    set_all_off();

    /* === FIX 1: гасим blink-состояние === */
    s_blink_mask       = 0;
    s_blink_on         = false;
    s_blink_last_phase = 0;
    /* === END FIX 1 === */

    s_fault_yellow_mask = 0;

    if (fault == TRAFFIC_FAULT_ALL_OFF) {
        ESP_LOGE(TAG, "FAULT → ALL_OFF");
        return;
    }

    /* === FIX 4: приоритет — маска от ROOT (NODE в follow_root).
       s_fault_mask_valid позволяет принять даже 0x0000 (валидная
       пустая маска, ROOT без жёлтых в s_mode). === */
    if (s_follow_root && s_fault_mask_valid) {
        s_fault_yellow_mask = s_fault_yellow_mask_from_root;
        ESP_LOGE(TAG, "FAULT → YELLOW_BLINK (mask=0x%04X from ROOT)",
                 s_fault_yellow_mask);
        return;
    }
    /* === END FIX 4 === */

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

void try_recovery(uint32_t now_ms, uint32_t *blink_last)
{
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

        bool was_on = sr_get_output(ch);
        if (!was_on) {
            sr_output_on(ch);
            vTaskDelay(pdMS_TO_TICKS(MUX_FAULT_SETTLE_MS));
        }

        imon_state_t st = IMON_STATE_UNKNOWN;
        int mv_raw = 0;
        int iload  = board_read_iload_raw(ch, &st, &mv_raw);

        if (!was_on) {
            sr_output_off(ch);
        }

        bool ok = (st == IMON_STATE_OK &&
                   iload >= 0 &&
                   mv_raw >= g_config.traffic.fault.imin_mv);

        ESP_LOGI(TAG, "recovery: ch%02d test st=%-7s mv=%d il=%d %s",
                 ch, imon_state_str(st), mv_raw, iload,
                 ok ? "OK" : "FAIL");

        if (!ok) {
            all_ok = false;
            if (first_bad == 0xFF) first_bad = ch;
            ESP_LOGW(TAG, "recovery: ch%02d not ready (st=%s mv=%d il=%d)",
                     ch, imon_state_str(st), mv_raw, iload);
        }
    }

    bool fire = false;
    bool node_needs_root_confirm = false;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (all_ok) {
            s_recovery_attempts++;
            ESP_LOGI(TAG, "recovery: ALL channels OK (streak=%u/%u)",
                     s_ok_streak + 1, RECOVERY_OK_STREAK);
            if (++s_ok_streak < RECOVERY_OK_STREAK) {
                ESP_LOGW(TAG, "Recovery attempt #%lu OK (streak %u/%u) — wait",
                         (unsigned long)s_recovery_attempts,
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
                ESP_LOGW(TAG, "Recovery #%lu CONFIRMED → resume phase %u",
                         (unsigned long)s_recovery_attempts, s_phase);
                s_fault = TRAFFIC_FAULT_NONE;
                s_fault_count = 0;
                s_fault_yellow_mask = 0;
                s_fault_yellow_mask_from_root = 0;   /* FIX 4 */
                s_fault_mask_valid            = false; /* FIX 4 */
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
        extern esp_err_t transport_send_evt(const char *evt, size_t len);
        ESP_LOGW(TAG, "recovery: NODE ok → request ROOT (fault:recover_ok)");
        transport_send_evt("fault:recover_ok", 16);
        return;
    }

    if (fire) {
        apply_stage_dd(s_phase);
    }

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

void traffic_light_check_faults(const mux_data_t *data)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return;
    if (!data || !data->valid) return;
    if (s_fault != TRAFFIC_FAULT_NONE) return;

    /* === FIX P0-1 / FIX 5: NODE в follow_root, но маска ещё не пришла
       от ROOT — каналы заведомо OFF, проверять нечего. Но если уже
       есть fault — не блокируемся (fault мигает независимо). === */
    if (s_follow_root && !s_mask_received && s_fault == TRAFFIC_FAULT_NONE) {
        return;
    }
    /* === END FIX P0-1 / FIX 5 === */

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - s_phase_start_ms < g_config.traffic.fault.phase_grace_ms) return;

    uint16_t active   = traffic_light_active_sr_mask_internal();
    uint16_t expected = traffic_light_expected_sr_mask_internal();
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

    for (uint8_t a = 0; a < 16 && n_faults < TRAFFIC_FAULT_MAX; a++) {
        if (!(active & (1u << a))) continue;
        if (blink & (1u << a)) continue;
        for (uint8_t b = (uint8_t)(a + 1); b < 16 && n_faults < TRAFFIC_FAULT_MAX; b++) {
            if (!(active & (1u << b))) continue;
            if (blink & (1u << b)) continue;
            bool a_on = (expected & (1u << a)) != 0;
            bool b_on = (expected & (1u << b)) != 0;
            if (a_on == b_on) continue;
            uint8_t on_ch  = a_on ? a : b;
            uint8_t off_ch = a_on ? b : a;
            if (data->v_in[on_ch] >= g_config.traffic.fault.vin_on_min_v &&
                data->v_in[off_ch] >= g_config.traffic.fault.vin_off_min_v &&
                !vin_off_flag[off_ch] &&
                s_vin_off_counter[off_ch] >= 1) {
                bool updated = false;
                for (uint8_t k = 0; k < n_faults; k++) {
                    if (list[k].channel == off_ch) {
                        list[k].reason = TRAFFIC_REASON_SHORTED_TOGETHER;
                        list[k].paired_channel = on_ch;
                        fill_reason_text(&list[k], "shorted to another channel", on_ch);
                        updated = true;
                        break;
                    }
                }
                if (!updated && n_faults < TRAFFIC_FAULT_MAX) {
                    traffic_fault_info_t *fi = &list[n_faults];
                    memset(fi, 0, sizeof(*fi));
                    fi->channel = off_ch;
                    fi->phase   = s_phase;
                    fi->mode    = s_mode;
                    fi->type    = TRAFFIC_FAULT_TYPE_VIN_ON_OFF;
                    fi->reason  = TRAFFIC_REASON_SHORTED_TOGETHER;
                    fi->paired_channel = on_ch;
                    fill_reason_text(fi, "shorted to another channel", on_ch);
                    n_faults++;
                }
            }
        }
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

   === FIX GOST-PED-YELLOW-BLINK ===
   ГОСТ Р 52289: в режиме жёлтого мигания запрещено оставлять
   пешеходный переход в активной пешеходной фазе — пешеходы
   должны быть уверены, что их сигнал управляется.
   Блокируем переход в YELLOW_BLINK, если текущая фаза —
   пешеходная. ALL_OFF (полный отказ) — разрешён всегда,
   т.к. это безопасное состояние «всё выключено».
   ============================================================ */
void traffic_light_set_fault(traffic_fault_t fault)
{
    /* === FIX GOST-PED-YELLOW-BLINK === */
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
    /* === END FIX GOST-PED-YELLOW-BLINK === */

    bool fire = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "set_fault: mutex timeout %d ms, fault=%d lost",
                 MUTEX_TIMEOUT_MS, (int)fault);
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
        s_fault_yellow_mask_from_root = 0;   /* FIX 4 */
        s_fault_mask_valid            = false; /* FIX 4 */
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

    /* === FIX 4: для yellow_blink добавляем маску жёлтых, чтобы NODE
       мигал теми же каналами, что ROOT. Формат:
         fault:yellow_blink:0x0092:<ch>,<ch>:<reason>,<reason>
       ROOT (s_mode) вычисляет s_fault_yellow_mask в apply_fault_mode.
       NODE (follow_root) использует её через s_fault_yellow_mask_from_root. */
    if (f == TRAFFIC_FAULT_YELLOW_BLINK) {
        off += snprintf(buf + off, size - off, "0x%04X:",
                        (unsigned)s_fault_yellow_mask);
    }
    /* === END FIX 4 === */

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