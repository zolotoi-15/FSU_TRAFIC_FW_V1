/* ============================================================
   traffic_fault.c — аварии: apply_fault_mode, try_recovery,
   check_faults, set_fault, build_fault_report.

   === FIX FAULT-HAS-TOP-PRIORITY ===
   Авария (YELLOW_BLINK / ALL_OFF) — наивысший приоритет над
   управлением каналами.

   === FIX FAULT-MASK-ZERO-FALLBACK ===
   apply_fault_mode при mask=0 фолбэкает на маску по s_mode.

   === FIX SHORT-GND-CONFIRM ===
   SHORT_GND (SR=1, V_IN < vin_on_min_v) подтверждается двумя
   подряд циклами check_faults.

   === FIX FAULT-ROOT-PUSH-IN-SET-FAULT ===
   traffic_light_set_fault на ROOT ВСЕГДА вызывает
   root_push_fault_to_node(fault) при fire.

   === FIX CHECK-FAULTS-APPLY-MODE ===
   traffic_light_check_faults при переходе NONE → YELLOW_BLINK/
   ALL_OFF вызывает apply_fault_mode ДО установки s_fault.

   === FIX SET-FAULT-APPLY-MODE-BEFORE-SET ===
   traffic_light_set_fault вызывает apply_fault_mode ДО
   s_fault = fault.

   === FIX RT-CHANNELS ===
   КЛЮЧЕВАЯ ПРАВКА. В traffic_light_check_faults добавлена
   ранняя защита:
     if (role==SLAVE && node_mask_pending()) return;
   Пока NODE ждёт apply_utc_ms (c: уже пришёл, маска ещё не
   применена), SR находится в СТАРОЙ маске, а
   s_expected_mask_from_root — уже в НОВОЙ. Без этой защиты
   check_faults на NODE за ~150 мс до применения маски
   сгенерирует ложный SHORT_GND (SR=1 по старой маске, но
   канал «не должен быть ON» по новой → V_IN present while OFF
   или наоборот).

   Защита на уровне check_faults (а не только в mux_task.c) —
   чтобы любой вызов (mux_task, будущие места) был безопасен.
   ============================================================ */

#include "traffic_internal.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "node_proto.h"      /* === FIX RT-CHANNELS: node_mask_pending() === */
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

extern volatile uint32_t s_fault_blink_last_ms;
extern volatile bool     s_fault_blink_on;

volatile uint16_t s_fault_yellow_mask_from_root = 0;
volatile bool     s_fault_mask_valid            = false;

void     apply_fault_mode(traffic_fault_t fault);
void     fault_blink_yellow(bool on);
void     try_recovery(uint32_t now_ms, uint32_t *blink_last);
const char *reason_str(uint8_t reason);
void     fill_reason_text(traffic_fault_info_t *fi, const char *detail, uint8_t paired);
uint8_t  classify_channel(uint8_t ch, bool should_be_on,
                          const mux_data_t *data, const char **out_detail);

/* === FIX FAULT-BLINK-PERIOD-EXPORT === */
uint32_t fault_blink_period_ms(void)
{
    uint32_t p = g_config.traffic.phases_ms.fault_blink_period;
    return (p >= BLINK_MIN_PERIOD_MS) ? p : 500;
}
/* === END FIX FAULT-BLINK-PERIOD-EXPORT === */

static uint8_t  s_last_channel_fault[16]    = {0};
static uint32_t s_last_channel_log_ms[16]   = {0};

static const char *reason_ru(uint8_t reason)
{
    switch (reason) {
    case TRAFFIC_REASON_VIN_ON_OFF: return "V_IN on OFF (напряжение на выключенном канале)";
    case TRAFFIC_REASON_SHORT_GND:  return "short to GND (короткое замыкание)";
    case TRAFFIC_REASON_NO_LOAD:    return "no load (отсутствует ток)";
    case TRAFFIC_REASON_OVERLOAD:   return "overload (превышен ток)";
    case TRAFFIC_REASON_UNDERLOAD:  return "underload (недогруз)";
    case TRAFFIC_REASON_NO_DIAG:    return "no DIAG_EN (нет диагностики)";
    case TRAFFIC_REASON_VIN_LOW:    return "V_IN low (просадка питания)";
    default:                        return "none";
    }
}

/* ============================================================
   apply_fault_mode
   ============================================================ */
void apply_fault_mode(traffic_fault_t fault)
{
    set_all_off();
    s_blink_mask       = 0;
    s_blink_on         = false;
    s_blink_last_phase = 0;
    s_fault_yellow_mask = 0;

    s_fault_blink_last_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_fault_blink_on      = false;

    if (fault == TRAFFIC_FAULT_ALL_OFF) {
        ESP_LOGE(TAG, "FAULT → ALL_OFF");
        wifi_mqtt_publish_event("fault_mode", "all_off", NULL, 0);
        return;
    }

    /* === FIX FAULT-MASK-ZERO-FALLBACK === */
    if (s_follow_root && s_fault_mask_valid) {
        if (s_fault_yellow_mask_from_root == 0) {
            ESP_LOGE(TAG, "FAULT → YELLOW_BLINK: ROOT mask=0, "
                          "fallback to local s_mode mask");
            /* fallthrough к switch(s_mode) */
        } else {
            s_fault_yellow_mask = s_fault_yellow_mask_from_root;
            ESP_LOGE(TAG, "FAULT → YELLOW_BLINK (mask=0x%04X from ROOT)",
                     s_fault_yellow_mask);
            wifi_mqtt_publish_event("fault_mode", "yellow_blink:from_root", NULL, 0);
            return;
        }
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

    if (s_fault_yellow_mask == 0) {
        ESP_LOGE(TAG, "FAULT → YELLOW_BLINK: mask=0 after s_mode=%d, "
                      "treat as ALL_OFF", s_mode);
        s_fault = TRAFFIC_FAULT_ALL_OFF;
        wifi_mqtt_publish_event("fault_mode", "all_off:mask_zero", NULL, 0);
        return;
    }

    ESP_LOGE(TAG, "FAULT → YELLOW_BLINK (mask=0x%04X by s_mode)",
             s_fault_yellow_mask);
    wifi_mqtt_publish_event("fault_mode", "yellow_blink", NULL, 0);
}
/* === END FIX FAULT-MASK-ZERO-FALLBACK === */

/* ============================================================
   fault_blink_yellow
   === FIX FAULT-BLINK-EMPTY-MASK ===
   ============================================================ */
void fault_blink_yellow(bool on)
{
    if (s_mux_reading) return;
    uint16_t m = s_fault_yellow_mask;

    if (!m) {
        switch (s_mode) {
        case TRAFFIC_MODE_SINGLE:
            m = (1u << g_config.traffic.pins.single.yellow); break;
        case TRAFFIC_MODE_CROSS:
            m = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                (1u << g_config.traffic.pins.cross.a_south.yellow) |
                (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                (1u << g_config.traffic.pins.cross.b_east.yellow); break;
        case TRAFFIC_MODE_CROSS_PED:
            m = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow); break;
        }
        s_fault_yellow_mask = m;
    }
    if (!m) return;

    for (int ch = 0; ch < 16; ch++) {
        if (m & (1u << ch)) {
            if (on) sr_output_on(ch);
            else    sr_output_off(ch);
        }
    }
    s_fault_blink_state = on;
}
/* === END FIX FAULT-BLINK-EMPTY-MASK === */

/* ============================================================
   try_recovery
   ============================================================ */
void try_recovery(uint32_t now_ms, uint32_t *blink_last)
{
    (void)blink_last;
    if (transport_is_cfg_rx_in_progress()) return;

    /* === FIX RT-CHANNELS ===
       Если на NODE pending-маска (c: пришёл, apply_utc_ms ещё не
       наступил), recovery не имеет смысла: каналы ещё не в новой
       маске. Ждём применения — иначе try_recovery «починит»
       несуществующую проблему. */
    if (g_config.role == DEVICE_ROLE_SLAVE && node_mask_pending()) {
        s_last_recovery_ms = now_ms;
        return;
    }
    /* === END FIX RT-CHANNELS === */

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
                s_short_gnd_counter[ch] = 0;
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
        esp_err_t err = transport_send_evt("fault:recover_ok", 16);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "recovery: NODE ok but ROOT unreachable (%s)",
                     esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG, "recovery: NODE ok → request ROOT");
        }
        wifi_mqtt_publish_event("recover", "node:request_root_confirm", NULL, 0);
        return;
    }

    if (fire) {
        apply_stage_dd(s_phase);
        wifi_mqtt_publish_event("recover", "auto:recovery_ok", NULL, 0);
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
    bool sr_on = (data->sr_state[ch] != 0);
    float v_in = data->v_in[ch];
    int imon_mv = data->imon_mv[ch];
    int i_load  = data->i_load[ch];

    if (sr_on && v_in < g_config.traffic.fault.vin_on_min_v) {
        *out_detail = "short to GND (SR=1, V_IN low)";
        return TRAFFIC_REASON_SHORT_GND;
    }

    if (!sr_on && v_in >= g_config.traffic.fault.vin_off_min_v) {
        *out_detail = "V_IN present while OFF (short between outputs)";
        return TRAFFIC_REASON_VIN_ON_OFF;
    }

    if (!should_be_on) return TRAFFIC_REASON_NONE;

    if (data->imon_state[ch] == IMON_STATE_NO_DIAG) {
        *out_detail = "DIAG_EN missing";
        return TRAFFIC_REASON_NO_DIAG;
    }
    if (data->imon_state[ch] == IMON_STATE_FAULT) {
        *out_detail = "short to GND / thermal";
        return TRAFFIC_REASON_SHORT_GND;
    }
    if (data->imon_state[ch] == IMON_STATE_SUSPECT) {
        *out_detail = "overload (V_IMON high)";
        return TRAFFIC_REASON_OVERLOAD;
    }
    if (sr_on && v_in >= g_config.traffic.fault.vin_on_min_v &&
        imon_mv < g_config.traffic.fault.imin_mv) {
        *out_detail = "no current";
        return TRAFFIC_REASON_NO_LOAD;
    }
    if (i_load > TRAFFIC_OVERLOAD_MA) {
        *out_detail = "overload";
        return TRAFFIC_REASON_OVERLOAD;
    }
    if (i_load > TRAFFIC_UNDERLOAD_MIN_MA &&
        i_load < TRAFFIC_UNDERLOAD_MAX_MA) {
        *out_detail = "underload";
        return TRAFFIC_REASON_UNDERLOAD;
    }
    return TRAFFIC_REASON_NONE;
}

/* ============================================================
   traffic_light_check_faults
   === FIX SHORT-GND-CONFIRM ===
   === FIX CHECK-FAULTS-APPLY-MODE ===
   === FIX RT-CHANNELS ===
   ============================================================ */
void traffic_light_check_faults(const mux_data_t *data)
{
    if (s_state != TRAFFIC_STATE_RUNNING) return;
    if (!data || !data->valid) return;

    /* === FIX RT-CHANNELS ===
       NODE: пока pending-маска не применена, SR в СТАРОЙ маске,
       а s_expected_mask_from_root — уже в НОВОЙ (если бы мы её
       обновили сразу). Но мы её НЕ обновляем до применения
       (см. node_proto.c → node_apply_c_mask вызывается только
       из scheduler). Тем не менее, если пользователь отправил
       новый c:, s_expected_mask_from_root мог быть обновлён на
       предыдущей итерации (см. traffic_light_apply_mask) —
       защищаемся на всякий случай. Любой вызов check_faults
       безопасен. */
    if (g_config.role == DEVICE_ROLE_SLAVE && node_mask_pending()) {
        ESP_LOGD(TAG, "check_faults: RT-mask pending → skip");
        return;
    }
    /* === END FIX RT-CHANNELS === */

    if (g_config.role == DEVICE_ROLE_SLAVE && s_fault_from_remote_ex) {
        return;
    }

    if (s_fault != TRAFFIC_FAULT_NONE) return;

    uint16_t active;
    uint16_t expected;
    if (g_config.role == DEVICE_ROLE_SLAVE) {
        if (!s_mask_received) return;
        active   = s_active_mask_from_root;
        expected = s_expected_mask_from_root;
        if (active == 0 && expected == 0) return;
    } else {
        active   = traffic_light_active_sr_mask_internal();
        expected = traffic_light_expected_sr_mask_internal();
    }

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool in_grace = (now_ms - s_phase_start_ms <
                     g_config.traffic.fault.phase_grace_ms);

    uint16_t blink = s_blink_mask;

    uint8_t n_faults = 0;
    traffic_fault_info_t list[TRAFFIC_FAULT_MAX];
    bool force_all_off = false;

    for (uint8_t ch = 0; ch < 16; ch++) {
        bool ch_active = true;
        if (ch < g_config.sr.count) ch_active = g_config.sr.outputs[ch].active;
        if (!ch_active) continue;

        bool sr_on = (data->sr_state[ch] != 0);
        float v_in = data->v_in[ch];

        bool is_active_mask = (active & (1u << ch)) != 0;
        bool is_blink = (blink & (1u << ch)) != 0;

        if (!is_active_mask && !is_blink) {
            const char *detail = NULL;
            uint8_t r = classify_channel(ch, false, data, &detail);
            if (r == TRAFFIC_REASON_VIN_ON_OFF) {
                if (++s_vin_off_counter[ch] >=
                    g_config.traffic.fault.vin_off_confirmations) {
                    force_all_off = true;
                    if (n_faults < TRAFFIC_FAULT_MAX) {
                        traffic_fault_info_t *fi = &list[n_faults];
                        memset(fi, 0, sizeof(*fi));
                        fi->channel = ch;
                        fi->phase = s_phase;
                        fi->mode = s_mode;
                        fi->type = TRAFFIC_FAULT_TYPE_VIN_ON_OFF;
                        fi->reason = r;
                        fi->paired_channel = 0xFF;
                        fill_reason_text(fi, detail, 0xFF);
                    }
                    n_faults++;
                }
            } else {
                s_vin_off_counter[ch] = 0;
            }
            continue;
        }
        if (is_blink) continue;

        bool should_be_on = (expected & (1u << ch)) != 0;

        const char *detail = NULL;
        uint8_t reason = TRAFFIC_REASON_NONE;

        if (sr_on && v_in < g_config.traffic.fault.vin_on_min_v) {
            /* === FIX SHORT-GND-CONFIRM === */
            if (++s_short_gnd_counter[ch] >= 2) {
                reason = TRAFFIC_REASON_SHORT_GND;
                detail = "short to GND (SR=1, V_IN low)";
            }
        } else if (!sr_on && v_in >= g_config.traffic.fault.vin_off_min_v) {
            s_short_gnd_counter[ch] = 0;
            reason = TRAFFIC_REASON_VIN_ON_OFF;
            detail = "V_IN present while OFF";
        } else {
            s_short_gnd_counter[ch] = 0;
            if (!in_grace && should_be_on) {
                reason = classify_channel(ch, should_be_on, data, &detail);
            }
        }
        /* === END FIX SHORT-GND-CONFIRM === */

        if (!should_be_on && reason == TRAFFIC_REASON_VIN_ON_OFF) {
            if (++s_vin_off_counter[ch] >=
                g_config.traffic.fault.vin_off_confirmations) {
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

    {
        uint8_t now_fault[16] = {0};
        const char *now_detail[16] = {0};
        for (uint8_t i = 0; i < n_faults && i < TRAFFIC_FAULT_MAX; i++) {
            uint8_t ch = list[i].channel;
            if (ch < 16) {
                now_fault[ch] = list[i].reason;
                now_detail[ch] = list[i].text;
            }
        }
        for (uint8_t ch = 0; ch < 16; ch++) {
            uint8_t prev = s_last_channel_fault[ch];
            uint8_t cur  = now_fault[ch];
            if (prev == cur) continue;

            if (cur == 0) {
                ESP_LOGW(TAG, "ch%02d: FAULT CLEARED (was %s)",
                         ch, reason_str(prev));
                wifi_mqtt_publish_event("fault_clear", reason_str(prev), NULL, 0);
            } else {
                ESP_LOGE(TAG, "ch%02d: FAULT %s [%s]", ch,
                         reason_ru(cur),
                         now_detail[ch] ? now_detail[ch] : reason_str(cur));
                wifi_mqtt_publish_event("fault", reason_ru(cur), NULL, 0);
            }
            s_last_channel_fault[ch] = cur;
            s_last_channel_log_ms[ch] = now_ms;
        }
    }

    if (n_faults == 0) return;

    bool fault_on_yellow = false;
    {
        uint16_t yellow_mask = 0;
        switch (s_mode) {
        case TRAFFIC_MODE_SINGLE:
            yellow_mask = (1u << g_config.traffic.pins.single.yellow); break;
        case TRAFFIC_MODE_CROSS:
            yellow_mask = (1u << g_config.traffic.pins.cross.a_north.yellow) |
                          (1u << g_config.traffic.pins.cross.a_south.yellow) |
                          (1u << g_config.traffic.pins.cross.b_west.yellow)  |
                          (1u << g_config.traffic.pins.cross.b_east.yellow); break;
        case TRAFFIC_MODE_CROSS_PED:
            yellow_mask = (1u << g_config.traffic.pins.cross_ped.axis_a.yellow)  |
                          (1u << g_config.traffic.pins.cross_ped.axis_a2.yellow) |
                          (1u << g_config.traffic.pins.cross_ped.axis_a3.yellow); break;
        }
        for (uint8_t i = 0; i < n_faults && i < TRAFFIC_FAULT_MAX; i++) {
            if (yellow_mask & (1u << list[i].channel)) {
                fault_on_yellow = true;
                break;
            }
        }
    }

    traffic_fault_t nf = (force_all_off || fault_on_yellow)
        ? TRAFFIC_FAULT_ALL_OFF
        : TRAFFIC_FAULT_YELLOW_BLINK;

    bool fire = false;
    bool was_none = false;
    bool already = false;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        already = (s_fault == nf);
        was_none = (s_fault == TRAFFIC_FAULT_NONE);
        uint8_t prev_count = s_fault_count;
        if (xSemaphoreTake(s_fault_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_fault_count = (n_faults < TRAFFIC_FAULT_MAX) ? n_faults : TRAFFIC_FAULT_MAX;
            memcpy(s_fault_list, list, s_fault_count * sizeof(traffic_fault_info_t));
            xSemaphoreGive(s_fault_mutex);
        }

        /* === FIX CHECK-FAULTS-APPLY-MODE ===
           apply_fault_mode вызываем ДО s_fault = nf, чтобы
           set_all_off() внутри него увидел fault_active() == false
           и реально выключил каналы. */
        if (!already && was_none) {
            apply_fault_mode(nf);
        }
        /* === END FIX CHECK-FAULTS-APPLY-MODE === */

        s_fault = nf;
        s_last_recovery_ms = now_ms;
        if (!already || s_fault_count != prev_count) {
            fire = true;
            s_ok_streak = 0;
            s_last_recovery_ok_ms = 0;
        }
        if (fire) s_last_fault_notify_ms = now_ms;
        if (fire && g_config.role == DEVICE_ROLE_MASTER) {
            root_push_fault_to_node(nf);
        }
        xSemaphoreGive(s_mutex);
    }
    if (fire && s_fault_notify) s_fault_notify(nf);
}

/* ============================================================
   traffic_light_set_fault
   === FIX SET-FAULT-APPLY-MODE-BEFORE-SET ===
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
        if (g_config.role == DEVICE_ROLE_MASTER) {
            root_push_fault_to_node(fault);
        }
        return;
    }

    bool fire = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "set_fault: mutex timeout");
        return;
    }

    bool already = (s_fault == fault);

    /* === FIX SET-FAULT-APPLY-MODE-BEFORE-SET ===
       apply_fault_mode ВЫЗЫВАЕТСЯ ДО s_fault = fault, иначе
       set_all_off() внутри него увидит fault_active() == true и
       пропустит выключение каналов.

       Для fault == NONE apply_fault_mode не вызывается — там
       apply_stage_dd(s_phase) в конце. */
    if (fault != TRAFFIC_FAULT_NONE) {
        if (!already) {
            apply_fault_mode(fault);
        }
        s_fault = fault;
        s_last_recovery_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (!already) {
            fire = true;
            s_ok_streak = 0;
            s_last_recovery_ok_ms = 0;
        }
        if (fire) s_last_fault_notify_ms = s_last_recovery_ms;
    } else {
        s_fault = fault;
        s_fault_yellow_mask = 0;
        s_fault_yellow_mask_from_root = 0;
        s_fault_mask_valid            = false;
        s_fault_count = 0;
        s_last_fault_notify_ms = 0;
        s_fault_blink_state = false;
        s_phase_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        for (int ch = 0; ch < 16; ch++) {
            s_vin_off_counter[ch]   = 0;
            s_suspect_counter[ch]   = 0;
            s_short_gnd_counter[ch] = 0;
        }
    }
    /* === END FIX SET-FAULT-APPLY-MODE-BEFORE-SET === */

    xSemaphoreGive(s_mutex);

    if (fault != TRAFFIC_FAULT_NONE) {
        if (fire) {
            s_fault_blink_state = false;
        }
    } else {
        if (s_state == TRAFFIC_STATE_RUNNING) {
            apply_stage_dd(s_phase);
        }
    }

    if (fire && g_config.role == DEVICE_ROLE_MASTER) {
        root_push_fault_to_node(fault);
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