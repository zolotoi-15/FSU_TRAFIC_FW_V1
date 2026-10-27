#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "mux_data.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRAFFIC_MODE_SINGLE    = 1,
    TRAFFIC_MODE_CROSS     = 2,
    TRAFFIC_MODE_CROSS_PED = 3,
} traffic_mode_t;

typedef enum {
    TRAFFIC_STATE_STOPPED = 0,
    TRAFFIC_STATE_RUNNING = 1,
    TRAFFIC_STATE_PAUSED  = 2,
} traffic_state_t;

typedef enum {
    TRAFFIC_FAULT_NONE         = 0,
    TRAFFIC_FAULT_YELLOW_BLINK = 1,
    TRAFFIC_FAULT_ALL_OFF      = 2,
} traffic_fault_t;

typedef enum {
    TRAFFIC_FAULT_TYPE_NONE       = 0,
    TRAFFIC_FAULT_TYPE_LOST       = 1,
    TRAFFIC_FAULT_TYPE_VIN_ON_OFF = 2,
} traffic_fault_type_t;

typedef enum {
    TRAFFIC_REASON_NONE              = 0,
    TRAFFIC_REASON_VIN_ON_OFF        = 1,
    TRAFFIC_REASON_SHORT_GND         = 2,
    TRAFFIC_REASON_NO_LOAD           = 3,
    TRAFFIC_REASON_OVERLOAD          = 4,
    TRAFFIC_REASON_UNDERLOAD         = 5,
    TRAFFIC_REASON_SHORTED_TOGETHER  = 6,
    TRAFFIC_REASON_NO_DIAG           = 7,
    TRAFFIC_REASON_VIN_LOW           = 8,
} traffic_fault_reason_t;

#define TRAFFIC_OVERLOAD_MA        1000
#define TRAFFIC_UNDERLOAD_MIN_MA   0
#define TRAFFIC_UNDERLOAD_MAX_MA   50

#define TRAFFIC_REASON_STR_LEN     96

#define TRAFFIC_PHASE_OVERDUE_MS   5000

typedef struct {
    uint8_t  channel;
    uint8_t  phase;
    uint8_t  mode;
    uint8_t  type;
    uint8_t  reason;
    uint8_t  paired_channel;
    char     text[TRAFFIC_REASON_STR_LEN];
} traffic_fault_info_t;

#define TRAFFIC_FAULT_MAX   16

typedef struct {
    uint8_t pin_red, pin_yellow, pin_green;
} traffic_pins_single_t;

typedef struct {
    uint8_t a_north_red,  a_north_yellow,  a_north_green;
    uint8_t a_south_red,  a_south_yellow,  a_south_green;
    uint8_t b_west_red,   b_west_yellow,   b_west_green;
    uint8_t b_east_red,   b_east_yellow,   b_east_green;
} traffic_pins_cross_t;

typedef struct {
    uint8_t axis_a_red,  axis_a_yellow,  axis_a_green;
    uint8_t axis_a2_red, axis_a2_yellow, axis_a2_green;
    uint8_t axis_a3_red, axis_a3_yellow, axis_a3_green;
    uint8_t ped_red,     ped_green;
} traffic_pins_cross_ped_t;

esp_err_t traffic_light_init(void);

void traffic_light_set_pins_single(const traffic_pins_single_t *pins);
void traffic_light_set_pins_cross(const traffic_pins_cross_t *pins);
void traffic_light_set_pins_cross_ped(const traffic_pins_cross_ped_t *pins);

esp_err_t traffic_light_set_mode(traffic_mode_t mode);

esp_err_t traffic_light_start(void);
esp_err_t traffic_light_stop(void);
esp_err_t traffic_light_pause(void);
esp_err_t traffic_light_resume(void);
esp_err_t traffic_light_reset(void);

esp_err_t traffic_light_next_phase(void);

esp_err_t traffic_light_force_phase(uint8_t phase, uint32_t phase_ms,
                                    uint32_t duration_ms);

traffic_state_t traffic_light_get_state(void);
traffic_mode_t  traffic_light_get_mode(void);
uint8_t         traffic_light_get_phase(void);
uint32_t        traffic_light_get_phase_ms(void);
uint8_t         traffic_light_get_phase_count(void);
uint32_t        traffic_light_get_phase_duration_ms(void);
const char     *traffic_light_state_str(void);

traffic_fault_t traffic_light_get_fault(void);
const char     *traffic_light_fault_str(void);
uint8_t         traffic_light_get_fault_count(void);
void            traffic_light_get_fault_info(uint8_t idx, traffic_fault_info_t *out);

uint16_t traffic_light_expected_sr_mask(void);
uint16_t traffic_light_active_sr_mask(void);

void traffic_light_check_faults(const mux_data_t *data);
void traffic_light_set_fault(traffic_fault_t fault);
bool traffic_light_test_channel(uint8_t ch);

size_t traffic_light_build_fault_report(char *buf, size_t size);

typedef void (*traffic_fault_notify_cb_t)(traffic_fault_t fault);
void traffic_light_set_fault_notify(traffic_fault_notify_cb_t cb);

/* node не двигает фазу сам — только по sync: от root */
void traffic_light_set_follow_root(bool en);

/* NEW: PED_BUTTON — вызывная кнопка (P7).
   Поведение зависит от g_config.traffic.pedestrian.request_mode:
     STAGE      — вставить пешеходную фазу следующей (текущее поведение);
     INSERT     — вставить в текущий цикл (с таймаутом);
     MIN_RECALL — гарантировать min зелёный в след. цикле.
   Если button_enabled=false — запрос игнорируется. */
void     traffic_light_request_pedestrian(void);

/* NEW: PED_BUTTON — true, если кнопка включена в конфиге. */
bool     traffic_light_ped_button_enabled(void);

/* callback при смене фазы (только на root, s_follow_root=false). */
typedef void (*traffic_phase_change_cb_t)(uint8_t phase, uint32_t dur_ms);
void traffic_light_set_phase_change_cb(traffic_phase_change_cb_t cb);

/* RTT-компенсация network_delay (ping/pong). */
void     traffic_light_set_sync_delay_us(uint32_t delay_us);
uint32_t traffic_light_get_sync_delay_us(void);
void traffic_light_set_mux_reading(bool en);

uint32_t traffic_light_last_recovery_ok_ms(void);

bool traffic_light_get_fault_blink_state(void);
void traffic_light_apply_fault_blink(bool on);

#ifdef __cplusplus
}
#endif