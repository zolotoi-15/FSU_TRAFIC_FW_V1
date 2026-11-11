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
    IMON_STATE_SUSPECT  = 2,   /* перегрузка: 3.0..4.2 В (I > 1.5 А) */
    IMON_STATE_NO_DIAG  = 3,   /* DIAG_EN не приходит на TPS */
    IMON_STATE_UNKNOWN  = 4,   /* ADC не прочитан */
} imon_state_t;

const char *imon_state_str(imon_state_t s);

/* ============================================================
   Пороги V_IMON (мВ) — из даташита TPS27S100B
     V_IMON(lin)   = 0..4 В    — линейный режим
     V_IMON(H)     = 4.3..4.9 В — fault mode (min 4.3, typ 4.75)
   ============================================================ */
#define IMON_TH_NO_DIAG_MV       10       /* < 10 мВ — нет DIAG_EN */
#define IMON_TH_SUSPECT_MV       3000     /* 3.0 В — начало перегрузки (I > 1.5 А при R_IMON=1k) */

/* === FIX TPS27S100B-VIN-FIRST ===
   Было 4300. V_IMON(H) min = 4.3 В (даташит), но с шумом ADC ±50 мВ
   порог 4300 — на грани, fault можно не поймать. Снижено до 4200
   с запасом. Не ниже 4000 — иначе V_IMON(lin) max 4 В начнёт
   ложно срабатывать. */
#define IMON_TH_FAULT_MV         4200
/* === END FIX TPS27S100B-VIN-FIRST === */

/* === FIX TPS27S100B-VIN-FIRST ===
   Единая классификация V_IMON. Используется в board_hw.c и
   mux_task.c вместо дублирования. */
static inline imon_state_t imon_classify(int mv, int imin_mv) {
    if (mv < 0) return IMON_STATE_UNKNOWN;
    if (mv < imin_mv) return IMON_STATE_NO_DIAG;
    if (mv >= IMON_TH_FAULT_MV) return IMON_STATE_FAULT;
    if (mv >= IMON_TH_SUSPECT_MV) return IMON_STATE_SUSPECT;
    return IMON_STATE_OK;
}
/* === END FIX TPS27S100B-VIN-FIRST === */

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