#pragma once

#include "traffic_light.h"
#include "config_manager.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_MIN_PERIOD_MS   50
#define FAULT_NOTIFY_REPEAT_MS  5000
#define RECOVERY_OK_STREAK   3
#define MUX_FAULT_SETTLE_MS  12
#define MUTEX_TIMEOUT_MS     1000

/* === Базовое состояние traffic === */
extern volatile traffic_state_t s_state;
extern volatile traffic_mode_t  s_mode;
extern volatile uint8_t         s_phase;
extern volatile uint32_t        s_phase_start_ms;
extern volatile uint32_t        s_last_recovery_ms;
extern volatile uint32_t        s_recovery_attempts;
extern volatile bool            s_follow_root;

/* === Пешеходный запрос === */
extern volatile bool            s_ped_request;
extern volatile uint32_t        s_ped_request_ms;
extern volatile bool            s_ped_min_recall;

/* === Задача и мьютексы === */
extern TaskHandle_t             s_task;
extern SemaphoreHandle_t        s_mutex;

/* === Fault state === */
extern volatile traffic_fault_t s_fault;
extern traffic_fault_info_t     s_fault_list[TRAFFIC_FAULT_MAX];
extern volatile uint8_t         s_fault_count;
extern SemaphoreHandle_t        s_fault_mutex;

/* === Blink / fault blink === */
extern volatile uint32_t        s_last_fault_notify_ms;
extern volatile uint16_t        s_blink_mask;
extern volatile uint16_t        s_fault_yellow_mask;
extern volatile bool            s_blink_on;
extern volatile uint32_t        s_blink_last_phase;
extern volatile uint32_t        s_sync_delay_us;
extern uint8_t                  s_suspect_counter[16];
extern uint8_t                  s_vin_off_counter[16];
extern uint8_t                  s_short_gnd_counter[16];   /* === FIX SHORT-GND-CONFIRM === */
extern uint8_t                  s_ok_streak;
extern volatile bool            s_mux_reading;
extern volatile uint32_t        s_last_recovery_ok_ms;
extern volatile bool            s_fault_blink_state;

extern volatile uint32_t        s_fault_blink_last_ms;
extern volatile bool            s_fault_blink_on;

extern volatile bool            s_mask_received;
extern volatile bool            s_in_force_phase;

extern volatile uint16_t        s_fault_yellow_mask_from_root;
extern volatile bool            s_fault_mask_valid;

/* === HOLD/INSERT === */
extern volatile bool            s_hold_active;
extern volatile bool            s_insert_pending;
extern volatile uint8_t         s_insert_queue[MAX_INSERT_SEQ];
extern volatile uint8_t         s_insert_count;
extern volatile uint8_t         s_insert_idx;
extern volatile uint32_t        s_insert_request_ms;

extern volatile uint8_t         s_pending_sync_phase;
void phase_change_cancel_pending(void);

/* === Callbacks === */
extern traffic_fault_notify_cb_t s_fault_notify;
extern traffic_phase_change_cb_t s_phase_change_cb;
extern volatile uint16_t s_expected_mask_from_root;
extern volatile uint16_t s_active_mask_from_root;

extern volatile bool s_ped_display_done;

void root_push_fault_to_node(traffic_fault_t fault);

/* ============================================================
   traffic_config.c
   ============================================================ */
const traffic_stage_dd_t *find_stage(uint8_t id);
bool     stage_is_pedestrian(uint8_t seq_idx);
bool     stage_is_transport_green(uint8_t seq_idx);
uint8_t  next_phase_smooth_ped(uint8_t cur_seq);
uint32_t stage_duration_ms(uint8_t seq_idx);
uint32_t effective_stage_duration_ms(uint8_t seq_idx);
uint16_t traffic_light_active_sr_mask_internal(void);
uint16_t traffic_light_stage_sr_mask_internal(void);
uint16_t traffic_light_expected_sr_mask_internal(void);
void     apply_stage_dd(uint8_t seq_idx);
void     set_all_off(void);

bool     stage_is_ped_green_or_blink(uint8_t seq_idx);
bool     traffic_mask_has_conflict(uint16_t on_mask);

/* === FIX HOLD-PHASE ===
   TRUE, если фаза seq_idx входит в hold.stage_ids (например,
   зелёный транспортный, который держится до нажатия кнопки).
   В этом состоянии traffic_task_loop НЕ переходит на следующую
   фазу, пока s_ped_request || s_insert_pending не подняты, либо
   пока не истечёт hold.max_hold_ms (>0). */
bool     stage_is_hold(uint8_t seq_idx);
/* === END FIX HOLD-PHASE === */

#define ALL_RED_MIN_MS   2000
bool need_virtual_all_red(uint8_t from_seq, uint8_t to_seq);

/* ============================================================
   traffic_fault.c
   ============================================================ */
void     apply_fault_mode(traffic_fault_t fault);
void     fault_blink_yellow(bool on);
uint32_t fault_blink_period_ms(void);   /* === FIX FAULT-BLINK-PERIOD-EXPORT === */
void     try_recovery(uint32_t now_ms, uint32_t *blink_last);
const char *reason_str(uint8_t reason);
void     fill_reason_text(traffic_fault_info_t *fi, const char *detail, uint8_t paired);
uint8_t  classify_channel(uint8_t ch, bool should_be_on,
                          const mux_data_t *data, const char **out_detail);

/* === FIX FAULT-HAS-TOP-PRIORITY === */
static inline bool fault_active(void) { return s_fault != TRAFFIC_FAULT_NONE; }

/* ============================================================
   traffic_light.c — callbacks (extern)
   ============================================================ */
void     traffic_task_loop(void *arg);

#ifdef __cplusplus
}
#endif