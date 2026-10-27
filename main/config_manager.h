// config_manager.h
#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define MAX_PWM_CHANNELS 3
#define MAX_NAME_LEN     32

/* ============================================================
   Роль устройства (master/slave)
   ============================================================ */
typedef enum {
    DEVICE_ROLE_MASTER = 0,
    DEVICE_ROLE_SLAVE  = 1,
} device_role_t;

/* ============================================================
   ESP-NOW
   ============================================================ */
typedef struct {
    bool     enabled;
    uint8_t  role;
    uint8_t  channel;
    uint8_t  slave_mac[6];
    uint16_t status_period_ms;
} espnow_config_t;

/* ============================================================
   PWM канал
   ============================================================ */
typedef struct {
    char     name[MAX_NAME_LEN];
    int      gpio;
    uint32_t frequency_hz;
    uint8_t  duty_percent;
    bool     enabled;

    bool     schedule_enabled;
    uint8_t  days_mask;
    uint8_t  start_hour, start_minute;
    uint8_t  end_hour,   end_minute;

    uint8_t  duty_min;
    uint8_t  duty_max;
    uint8_t  duty_step;
    uint16_t duty_interval_sec;
} pwm_channel_config_t;

typedef struct {
    bool     enabled;
    uint8_t  days_mask;
    uint8_t  start_hour, start_minute;
    uint8_t  end_hour,   end_minute;
    uint8_t  duty_min,  duty_max, duty_step;
    uint16_t duty_interval_sec;
} daynight_config_t;

typedef struct {
    bool     enabled;
    uint8_t  threshold_percent;
    uint8_t  threshold2_percent;
    uint8_t  threshold3_percent;
    uint8_t  days_mask;
    uint8_t  start_hour, start_minute;
    uint8_t  end_hour,   end_minute;
} sequential_config_t;

/* ============================================================
   DD: сигнальные группы
   ============================================================ */
#define MAX_SIGNAL_GROUPS   16
typedef struct {
    uint8_t id;
    char    name[32];
    char    type[16];
    uint8_t minRed;
} signal_group_t;

/* ============================================================
   DD: IGM
   ============================================================ */
#define MAX_TRANSITIONS     32
typedef struct {
    uint8_t  clearing;
    uint8_t  entering;
    uint16_t time_cs;
} traffic_transition_t;

typedef struct {
    uint8_t  id;
    char     name[32];
    bool     safety;
    bool     is_default;
    traffic_transition_t items[MAX_TRANSITIONS];
    uint8_t  item_count;
} igm_config_t;

/* ============================================================
   DD: stages (справочно)
   ============================================================ */
#define MAX_STAGES          16
#define MAX_STAGE_GROUPS    4
typedef struct {
    uint8_t id;
    char    name[32];
    uint8_t groups[MAX_STAGE_GROUPS];
    uint8_t group_count;
} stage_ref_t;

/* ============================================================
   DD: детекторы
   ============================================================ */
#define MAX_DETECTORS       8
typedef struct {
    uint8_t id;
    char    name[32];
} detector_ref_t;

/* ============================================================
   DD: планы
   ============================================================ */
#define MAX_PLANS           4
typedef struct {
    uint8_t id;
    char    name[32];
    char    algorithm_type[16];
    uint8_t matrix;
    uint8_t sequence[MAX_STAGES];
    uint8_t sequence_count;
    bool    coordinated;
} plan_ref_t;

/* ============================================================
   DD: расписание
   ============================================================ */
#define MAX_SCHEDULE_DAYS   8
typedef struct {
    uint8_t id;
    uint8_t plan;
    char    time[16];
    uint8_t spare_plan;
} schedule_plan_t;

typedef struct {
    uint8_t id;
    schedule_plan_t plans[4];
    uint8_t plan_count;
} schedule_day_t;

typedef struct {
    schedule_day_t days[MAX_SCHEDULE_DAYS];
    uint8_t day_count;
} schedule_config_t;

/* ============================================================
   DD: controlSources
   ============================================================ */
#define MAX_CONTROL_SOURCES 8
typedef struct {
    uint8_t  id;
    char     name[32];
    uint16_t priority;
    char     type[16];
    char     url[96];
} control_source_t;

/* ============================================================
   DD: модули (справочно)
   ============================================================ */
#define MAX_KSK1_SWITCHES   32
typedef struct {
    uint8_t id;
    char    name[16];
    uint8_t addr;
    uint8_t signalGroup;
    bool    control;
    uint8_t heads;
    char    color[8];
} ksk1_switch_t;

typedef struct {
    uint32_t baudRate;
    char     portName[32];
    bool     rs485enabled;
    bool     rs485rtsAfterSend;
} systembus_port_t;

typedef struct {
    char     deviceName[32];
} keyboard_config_t;

typedef struct {
    char     controlSource[16];
    uint16_t controlTimeout;
    char     url[96];
} spectr_its_config_t;

typedef struct {
    char     ti_host[32];
    uint16_t ti_port;
    uint8_t  channel;
    uint8_t  detector;
} traficam_channel_t;

typedef struct {
    traficam_channel_t channels[4];
    uint8_t            channel_count;
} traficam_config_t;

typedef struct {
    char url[128];
} traffic_stream_t;

typedef struct {
    systembus_port_t  port;
    ksk1_switch_t     switches[MAX_KSK1_SWITCHES];
    uint8_t           switch_count;
    keyboard_config_t keyboard;
    spectr_its_config_t spectr_its;
    traficam_config_t traficam;
    traffic_stream_t  streams[2];
    uint8_t           stream_count;
} modules_config_t;

/* ============================================================
   DD: тип длительности фазы
   ============================================================ */
typedef enum {
    DD_DUR_EXPLICIT     = 0,
    DD_DUR_RED          = 1,
    DD_DUR_RED_YELLOW   = 2,
    DD_DUR_GREEN        = 3,
    DD_DUR_GREEN_BLINK  = 4,
    DD_DUR_YELLOW       = 5,
    DD_DUR_YELLOW_BLINK = 6,
    DD_DUR_PED_GREEN    = 7,
    DD_DUR_PED_BLINK    = 8,
} traffic_duration_kind_t;

#define DD_DURATION_KIND_MAX  9

/* ============================================================
   DD: этап data-driven сценария
   ============================================================ */
#define MAX_DD_STAGES       16
#define MAX_ON_OUTPUTS      8
#define MAX_BLINK_OUTPUTS   4

typedef struct {
    uint8_t  id;
    char     name[32];
    uint8_t  groups[MAX_STAGE_GROUPS];
    uint8_t  group_count;
    uint32_t duration_ms;
    uint8_t  duration_kind;
    uint8_t  outputs_on[MAX_ON_OUTPUTS];
    uint8_t  on_count;
    uint8_t  outputs_blink[MAX_BLINK_OUTPUTS];
    uint8_t  blink_count;
} traffic_stage_dd_t;

/* ============================================================
   DD: конфигурация traffic
   ============================================================ */
#define MAX_SEQUENCE        16
#define MAX_OUTPUTS         16

typedef struct {
    uint8_t  red, yellow, green;
} traffic_color_pins_t;

typedef struct {
    traffic_color_pins_t single;
    struct {
        traffic_color_pins_t a_north, a_south, b_west, b_east;
    } cross;
    struct {
        traffic_color_pins_t axis_a, axis_a2, axis_a3;
        uint8_t ped_red, ped_green;
    } cross_ped;
} traffic_pins_config_t;

typedef struct {
    uint32_t red;
    uint32_t red_yellow;
    uint32_t green;
    uint32_t green_blink;
    uint32_t yellow;
    uint32_t yellow_blink;
    uint32_t ped_green;
    uint32_t ped_blink;
    uint32_t fault_blink_period;
    uint32_t phase_blink_period;
} traffic_phases_ms_t;

typedef struct {
    int      imin_mv;
    float    vin_on_min_v;
    float    vin_off_min_v;
    uint32_t phase_grace_ms;
    uint32_t recovery_ms;
    uint8_t  vin_off_confirmations;
} traffic_fault_config_t;

/* ============================================================
   NEW: PED_BUTTON — конфигурация вызывной кнопки пешехода (P7).
   ============================================================ */
typedef enum {
    PED_REQUEST_MODE_STAGE      = 0,   /* вставить пешеходную фазу следующей */
    PED_REQUEST_MODE_INSERT     = 1,   /* вставить в текущий цикл (min green) */
    PED_REQUEST_MODE_MIN_RECALL = 2,   /* гарантировать min зелёный в след. цикле */
} ped_request_mode_t;

typedef struct {
    bool     button_enabled;      /* false — вызывной кнопки нет */
    uint8_t  request_mode;        /* ped_request_mode_t */
    uint32_t insert_timeout_ms;   /* для INSERT: сколько ждать окна */
    uint32_t min_recall_s;        /* для MIN_RECALL: минимальный зелёный */
} pedestrian_button_config_t;

typedef struct {
    uint8_t  mode;
    bool     enabled;
    bool     auto_start;

    pedestrian_button_config_t pedestrian;   /* NEW: PED_BUTTON */

    traffic_pins_config_t   pins;
    traffic_phases_ms_t     phases_ms;
    traffic_fault_config_t  fault;

    traffic_stage_dd_t      stages[MAX_DD_STAGES];
    uint8_t                 stage_count;

    traffic_transition_t    transitions[MAX_TRANSITIONS];
    uint8_t                 transition_count;

    uint8_t                 sequence[MAX_SEQUENCE];
    uint8_t                 sequence_count;
} traffic_config_t;

/* ============================================================
   Shift register outputs
   ============================================================ */
typedef struct {
    uint8_t ch;
    char    name[16];
    bool    active;
} sr_output_cfg_t;

typedef struct {
    sr_output_cfg_t outputs[MAX_OUTPUTS];
    uint8_t         count;
} sr_config_t;

/* ============================================================
   Вся конфигурация устройства
   ============================================================ */
typedef struct {
    float version;
    uint32_t id;
    char name[64];

    device_role_t role;
    espnow_config_t espnow;

    char wifi_ssid[32];
    char wifi_password[64];
    char ap_ssid[32];
    char ap_password[64];
    char timezone[32];

    bool     mqtt_enabled;
    char     mqtt_uri[96];
    uint16_t mqtt_port;
    char     mqtt_user[32];
    char     mqtt_password[64];
    uint16_t mqtt_timepause_min;

    pwm_channel_config_t channels[MAX_PWM_CHANNELS];
    daynight_config_t    daynight;
    sequential_config_t  sequential;

    traffic_config_t     traffic;
    sr_config_t          sr;

    signal_group_t   signal_groups[MAX_SIGNAL_GROUPS];
    uint8_t          signal_group_count;
    igm_config_t     igm;
    stage_ref_t      stages[MAX_STAGES];
    uint8_t          stage_count;
    detector_ref_t   detectors[MAX_DETECTORS];
    uint8_t          detector_count;
    plan_ref_t       plans[MAX_PLANS];
    uint8_t          plan_count;
    schedule_config_t schedule;
    control_source_t control_sources[MAX_CONTROL_SOURCES];
    uint8_t          control_source_count;
    modules_config_t modules;
} app_config_t;

extern app_config_t g_config;

void config_init(void);
void config_load(void);
void config_save(void);
void config_reset_defaults(void);
void config_apply_timezone(void);
bool config_load_json(const char *json, size_t len);
bool config_role_is_locked(void);

esp_err_t      config_save_role(device_role_t role);
device_role_t  config_load_role(void);

#endif