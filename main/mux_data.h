#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   Состояние TPS27S100B по уровню V_IMON
   ============================================================ */
typedef enum {
    IMON_STATE_OK       = 0,   /* норма, линейный режим */
    IMON_STATE_FAULT    = 1,   /* fault: overload / thermal / short */
    IMON_STATE_SUSPECT  = 2,   /* обрыв, pull-up, паразитная связь */
    IMON_STATE_NO_DIAG  = 3,   /* DIAG_EN не приходит на TPS */
    IMON_STATE_UNKNOWN  = 4,   /* ADC не прочитан */
} imon_state_t;

const char *imon_state_str(imon_state_t s);

/* ============================================================
   Пороги V_IMON (мВ)
   ============================================================ */
#define IMON_TH_NO_DIAG_MV       10       /* < 10 мВ — нет DIAG_EN */
#define IMON_TH_SUSPECT_MV       3000     /* FIX: было 2500 — поднято, чтобы убрать ложные SUSPECT */
#define IMON_TH_FAULT_MV         4300     /* 4.3 В — fault (V_IMON(H) min) */

/* ============================================================
   Общие данные с задачи mux_task.
   ============================================================ */
typedef struct {
    float    v_in[16];         /* напряжение, В */
    int      i_load[16];       /* ток, мА */
    uint8_t  sr_state[16];     /* состояние реле 0/1 */
    uint8_t  diag_en[16];      /* состояние DIAG_EN 0/1/0xFF */
    uint8_t  imon_state[16];   /* imon_state_t */
    int16_t  imon_mv[16];      /* сырое V_IMON, мВ */
    uint32_t timestamp;
    bool     valid;
} mux_data_t;

void mux_data_init(void);
void mux_data_set(const mux_data_t *d);
bool mux_data_get(mux_data_t *out);

#ifdef __cplusplus
}
#endif