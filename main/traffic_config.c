/* ============================================================
   traffic_config.c — data-driven helpers.
   Стадии, длительности, next_phase_smooth_ped, apply_stage_dd.
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

/* ------------------------------------------------------------ */
/* find_stage / stage_is_*                                      */
/* ------------------------------------------------------------ */
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

/* ------------------------------------------------------------ */
/* next_phase_smooth_ped — ищем ближайший group 7 (ped_green).  */
/* ------------------------------------------------------------ */
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

/* ------------------------------------------------------------ */
/* stage_duration_ms / effective_stage_duration_ms              */
/* ------------------------------------------------------------ */
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

/* ------------------------------------------------------------ */
/* SR-маски                                                     */
/* ------------------------------------------------------------ */
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

uint16_t traffic_light_expected_sr_mask_internal(void)
{
    if (g_config.traffic.sequence_count == 0) return 0;
    const traffic_stage_dd_t *st = find_stage(g_config.traffic.sequence[s_phase]);
    if (!st) return 0;
    uint16_t m = 0;
    for (int k = 0; k < st->on_count; k++)    m |= (1u << st->outputs_on[k]);
    for (int k = 0; k < st->blink_count; k++) m |= (1u << st->outputs_blink[k]);
    return m;
}

/* ------------------------------------------------------------ */
/* set_all_off / apply_stage_dd                                 */
/* ------------------------------------------------------------ */
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

    for (int i = 0; i < st->on_count; i++) {
        sr_output_on(st->outputs_on[i]);
    }

    s_blink_mask = 0;
    for (int i = 0; i < st->blink_count; i++) {
        s_blink_mask |= (1u << st->outputs_blink[i]);
    }

    s_blink_on = false;
    s_blink_last_phase = 0;

    uint32_t dur = stage_duration_ms(seq_idx);
    ESP_LOGI(TAG, "Stage → %u (%s) kind=%u dur=%lu on=%u blink=%u%s",
             st->id, st->name, st->duration_kind,
             (unsigned long)dur,
             st->on_count, st->blink_count,
             stage_is_pedestrian(seq_idx) ? " (PED)" : "");

    if (!s_follow_root && s_phase_change_cb) {
        s_phase_change_cb(seq_idx, dur);
    }
}