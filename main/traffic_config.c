/* ============================================================
   traffic_config.c — data-driven helpers.

   === FIX NODE-BLINK-AND-ON ===
       apply_stage_dd() сначала выключает ВСЕ каналы (active
       mask), потом включает outputs_on.
   === FIX INSERT-NO-DOUBLE-SYNC ===
       apply_stage_dd не дёргает s_phase_change_cb, если
       s_in_force_phase == true (вызывающий сам отправит sync:).
       Используется в traffic_light_force_phase и в insert-блоке.
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

uint32_t stage_duration_ms(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return 1000;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) return 1000;

    switch (st->duration_kind) {
    case DD_DUR_RED:          return g_config.traffic.phases_ms.red;
    case DD_DUR_RED_YELLOW:   return g_config.traffic.phases_ms.red_yellow;
    case DD_DUR_GREEN:        return g_config.traffic.phases_ms.green;
    case DD_DUR_GREEN_BLINK:  return g_config.traffic.phases_ms.green_blink;
    case DD_DUR_YELLOW:       return g_config.traffic.phases_ms.yellow;
    case DD_DUR_YELLOW_BLINK: return g_config.traffic.phases_ms.yellow_blink;
    case DD_DUR_PED_GREEN:    return g_config.traffic.phases_ms.ped_green;
    case DD_DUR_PED_BLINK:    return g_config.traffic.phases_ms.ped_blink;
    case DD_DUR_EXPLICIT:
    default:                  return st->duration_ms;
    }
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

void apply_stage_dd(uint8_t seq_idx)
{
    if (seq_idx >= g_config.traffic.sequence_count) return;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[seq_idx]);
    if (!st) {
        ESP_LOGE(TAG, "Stage id=%u not found", g_config.traffic.sequence[seq_idx]);
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
    s_blink_mask = blink_mask;

    s_blink_on = false;
    s_blink_last_phase = 0;

    uint32_t dur = stage_duration_ms(seq_idx);
    ESP_LOGD(TAG, "Stage → %u (%s) kind=%u dur=%lu on=0x%04X blink=0x%04X%s",
             st->id, st->name, st->duration_kind,
             (unsigned long)dur,
             (unsigned)on_mask, (unsigned)blink_mask,
             stage_is_pedestrian(seq_idx) ? " (PED)" : "");

    /* === FIX INSERT-NO-DOUBLE-SYNC: не дёргаем cb, если вызывающий
       сам отправит sync: (force_phase / insert). === */
    if (!s_follow_root && s_phase_change_cb && !s_in_force_phase) {
        s_phase_change_cb(seq_idx, dur);
    }
    /* === END FIX INSERT-NO-DOUBLE-SYNC === */
}