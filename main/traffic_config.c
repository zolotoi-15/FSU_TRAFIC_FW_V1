/* ============================================================
   traffic_config.c — data-driven helpers.

   === FIX APPLY-MASK-FOLLOW-ROOT ===
       apply_stage_dd для NODE в follow_root НЕ применяет маску
       по stage_dd — только set_all_off. ROOT пришлёт sync: с
       маской. Это защита от CONFLICT на NODE, даже если
       stage_dd в конфиге противоречив.

   === FIX CONFLICT-LOG-ONCE ===
       Лог CONFLICT не спамит — не чаще 1 раза в 5 с на стадию.

   Остальные FIX сохранены: GOST-RED-YELLOW-LIMIT,
   GOST-CONFLICT-MATRIX, GOST-ALL-RED, PED-BLINK-MASK,
   INSERT-NO-DOUBLE-SYNC.
   ============================================================ */

#include "traffic_internal.h"
#include "shift_register.h"
#include "config_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "traffic_cfg";

const traffic_stage_dd_t *find_stage(uint8_t id)
{
    for (int i = 0; i < g_config.traffic.stage_count; i++) {
        if (g_config.traffic.stages[i].id == id)
            return &g_config.traffic.stages[i];
    }
    return NULL;
}

bool stage_is_pedestrian(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return false;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) return false;
    for (int g = 0; g < st->group_count; g++) {
        if (st->groups[g] == 7 || st->groups[g] == 8) return true;
    }
    return false;
}

bool stage_is_transport_green(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return false;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) return false;
    for (int g = 0; g < st->group_count; g++) {
        if (st->groups[g] == 3) return true;
    }
    return false;
}

uint8_t next_phase_smooth_ped(uint8_t cur_seq)
{
    if (cur_seq >= g_config.traffic.sequence_count) return 0;

    for (int i = 1; i <= g_config.traffic.sequence_count; i++) {
        uint8_t idx = (cur_seq + i) % g_config.traffic.sequence_count;
        const traffic_stage_dd_t *s2 = find_stage(g_config.traffic.sequence[idx]);
        if (!s2) continue;
        for (int g = 0; g < s2->group_count; g++) {
            if (s2->groups[g] == 7) return idx;
        }
    }
    return (cur_seq + 1) % g_config.traffic.sequence_count;
}

/* === FIX GOST-RED-YELLOW-LIMIT === */
uint32_t stage_duration_ms(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return 1000;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) return 1000;

    uint32_t dur;
    switch (st->duration_kind) {
    case DD_DUR_RED:          dur = g_config.traffic.phases_ms.red;          break;
    case DD_DUR_RED_YELLOW:   dur = g_config.traffic.phases_ms.red_yellow;   break;
    case DD_DUR_GREEN:        dur = g_config.traffic.phases_ms.green;        break;
    case DD_DUR_GREEN_BLINK:  dur = g_config.traffic.phases_ms.green_blink;  break;
    case DD_DUR_YELLOW:       dur = g_config.traffic.phases_ms.yellow;       break;
    case DD_DUR_YELLOW_BLINK: dur = g_config.traffic.phases_ms.yellow_blink; break;
    case DD_DUR_PED_GREEN:    dur = g_config.traffic.phases_ms.ped_green;    break;
    case DD_DUR_PED_BLINK:    dur = g_config.traffic.phases_ms.ped_blink;    break;
    case DD_DUR_EXPLICIT:
    default:                  dur = st->duration_ms;                         break;
    }

    if (st->duration_kind == DD_DUR_RED_YELLOW && dur > 2000) {
        static uint32_t s_last_warn_ms = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_last_warn_ms == 0 || (now - s_last_warn_ms) > 5000) {
            s_last_warn_ms = now;
            ESP_LOGW(TAG, "red_yellow=%lu ms > 2000 ms → clamp to 2000 "
                          "(GOST Р 52289)",
                     (unsigned long)dur);
        }
        dur = 2000;
    }

    return dur;
}

uint32_t effective_stage_duration_ms(uint8_t seq_idx)
{
    uint32_t dur = stage_duration_ms(seq_idx);
    if (s_ped_min_recall && stage_is_pedestrian(seq_idx)) {
        uint32_t min_ms = g_config.traffic.pedestrian.min_recall_s * 1000;
        if (dur < min_ms) return min_ms;
    }
    return dur;
}

uint16_t traffic_light_active_sr_mask_internal(void)
{
    uint16_t m = 0;
    for (int i = 0; i < g_config.traffic.sequence_count; i++) {
        const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[i]);
        if (!st) continue;
        for (int k = 0; k < st->on_count; k++)    m |= (1u << st->outputs_on[k]);
        for (int k = 0; k < st->blink_count; k++) m |= (1u << st->outputs_blink[k]);
    }
    return m;
}

uint16_t traffic_light_stage_sr_mask_internal(void)
{
    if (g_config.traffic.sequence_count == 0) return 0;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[s_phase]);
    if (!st) return 0;
    uint16_t m = 0;
    for (int k = 0; k < st->on_count; k++)    m |= (1u << st->outputs_on[k]);
    for (int k = 0; k < st->blink_count; k++) m |= (1u << st->outputs_blink[k]);
    return m;
}

uint16_t traffic_light_expected_sr_mask_internal(void)
{
    return traffic_light_stage_sr_mask_internal();
}

void set_all_off(void)
{
    for (int i = 0; i < 16; i++) sr_output_off(i);
}

/* === FIX GOST-CONFLICT-MATRIX === */
bool traffic_mask_has_conflict(uint16_t on_mask)
{
    if (g_config.traffic.conflict_count == 0) return false;
    bool conflict = false;
    for (int i = 0; i < g_config.traffic.conflict_count; i++) {
        uint8_t a = g_config.traffic.conflicting_channels[i].a;
        uint8_t b = g_config.traffic.conflicting_channels[i].b;
        if ((on_mask & (1u << a)) && (on_mask & (1u << b))) {
            ESP_LOGE(TAG, "CONFLICT: ch%02d + ch%02d both ON "
                          "(GOST Р 52289 п.7.4.1)", a, b);
            conflict = true;
        }
    }
    return conflict;
}

/* === FIX GOST-ALL-RED === */
bool need_virtual_all_red(uint8_t from_seq, uint8_t to_seq)
{
    if (from_seq == to_seq) return false;
    if (from_seq >= g_config.traffic.sequence_count) return false;
    if (to_seq   >= g_config.traffic.sequence_count) return false;

    const traffic_stage_dd_t *s1 = find_stage(g_config.traffic.sequence[from_seq]);
    const traffic_stage_dd_t *s2 = find_stage(g_config.traffic.sequence[to_seq]);
    if (!s1 || !s2) return false;

    bool g1 = (s1->duration_kind == DD_DUR_GREEN ||
               s1->duration_kind == DD_DUR_GREEN_BLINK);
    bool g2 = (s2->duration_kind == DD_DUR_GREEN ||
               s2->duration_kind == DD_DUR_GREEN_BLINK);
    if (!g1 || !g2) return false;

    int steps = (to_seq > from_seq)
                    ? (to_seq - from_seq)
                    : (to_seq + g_config.traffic.sequence_count - from_seq);
    for (int i = 1; i < steps; i++) {
        uint8_t idx = (from_seq + i) % g_config.traffic.sequence_count;
        const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[idx]);
        if (!st) continue;
        if (st->duration_kind == DD_DUR_RED) {
            return false;
        }
    }

    ESP_LOGW(TAG, "need_virtual_all_red: seq %u → %u (both green, "
                  "no all-red between) → insert virtual all-red "
                  "(GOST Р 52289)", from_seq, to_seq);
    return true;
}

/* ============================================================
   apply_stage_dd

   === FIX APPLY-MASK-FOLLOW-ROOT ===
       Для NODE в follow_root НЕ применяем маску по stage_dd —
       только set_all_off. ROOT пришлёт sync: с маской.
       Это защита от CONFLICT на NODE, даже если stage_dd
       в конфиге противоречив (тестовый стенд MESH).

   === FIX CONFLICT-LOG-ONCE ===
       Лог CONFLICT не спамит — не чаще 1 раза в 5 с на стадию.
   ============================================================ */
void apply_stage_dd(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) {
        ESP_LOGE(TAG, "Stage id=%u not found", g_config.traffic.sequence[seq_idx]);
        return;
    }

    /* === FIX APPLY-MASK-FOLLOW-ROOT ===
       NODE в follow_root не применяет stage_dd — ждёт sync: от ROOT.
       Это защита от CONFLICT, даже если stage_dd в конфиге противоречив. */
    if (s_follow_root) {
        set_all_off();
        s_blink_mask = 0;
        s_blink_on = false;
        s_blink_last_phase = 0;
        ESP_LOGD(TAG, "apply_stage_dd: NODE follow_root → all off "
                      "(wait sync:)");
        return;
    }
    /* === END FIX APPLY-MASK-FOLLOW-ROOT === */

    uint16_t will_be_on = 0;
    for (int i = 0; i < st->on_count; i++)
        will_be_on |= (1u << st->outputs_on[i]);

    if (traffic_mask_has_conflict(will_be_on)) {
        /* === FIX CONFLICT-LOG-ONCE: не спамим === */
        static uint32_t s_last_conflict_warn_ms = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_last_conflict_warn_ms == 0 ||
            (now - s_last_conflict_warn_ms) > 5000) {
            s_last_conflict_warn_ms = now;
            ESP_LOGE(TAG, "Stage id=%u (seq=%u) has conflicting mask 0x%04X "
                          "→ forcing ALL_OFF (GOST Р 52289)",
                     st->id, seq_idx, (unsigned)will_be_on);
        }
        /* === END FIX CONFLICT-LOG-ONCE === */
        set_all_off();
        s_blink_mask = 0;
        s_blink_on = false;
        s_blink_last_phase = 0;
        return;
    }

    uint16_t active = traffic_light_active_sr_mask_internal();
    for (int ch = 0; ch < 16; ch++) {
        if (active & (1u << ch)) sr_output_off(ch);
    }

    uint16_t on_mask = 0;
    for (int i = 0; i < st->on_count; i++) {
        on_mask |= (1u << st->outputs_on[i]);
        sr_output_on(st->outputs_on[i]);
    }

    uint16_t blink_mask = 0;
    for (int i = 0; i < st->blink_count; i++) {
        blink_mask |= (1u << st->outputs_blink[i]);
    }
    blink_mask &= ~on_mask;

    /* === FIX PED-BLINK-MASK === */
    s_blink_mask = blink_mask;
    s_blink_on = false;
    s_blink_last_phase = 0;
    /* === END FIX PED-BLINK-MASK === */

    uint32_t dur = stage_duration_ms(seq_idx);
    ESP_LOGD(TAG, "Stage → %u (%s) kind=%u dur=%lu on=0x%04X blink=0x%04X%s",
             st->id, st->name, st->duration_kind,
             (unsigned long)dur,
             (unsigned)on_mask, (unsigned)blink_mask,
             stage_is_pedestrian(seq_idx) ? " (PED)" : "");

    /* === FIX INSERT-NO-DOUBLE-SYNC === */
    if (!s_follow_root && s_phase_change_cb && !s_in_force_phase) {
        s_phase_change_cb(seq_idx, dur);
    }
    /* === END FIX INSERT-NO-DOUBLE-SYNC === */
}