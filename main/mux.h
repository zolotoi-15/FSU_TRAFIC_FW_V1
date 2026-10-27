#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MUX_DEFAULT_AVG_SAMPLES     8
#define MUX_SETTLE_DELAY_US         50
#define MUX_CHANNEL_COUNT           16

#define MUX1_R_TOP                  100000.0f
#define MUX1_R_BOTTOM               13300.0f
#define MUX1_DIVIDER_RATIO          ((MUX1_R_TOP + MUX1_R_BOTTOM) / MUX1_R_BOTTOM)

#define TPS27S100B_K_IMON           500.0f
#define TPS27S100B_R_IMON           1000.0f
#define TPS27S100B_IMON_FAULT_MV    4300

esp_err_t mux_init(void);
void      mux_enable_mux2(bool enable);
void      mux_select_channel(uint8_t channel);

/* ============================================================
   FIX #2: публичный мьютекс мультиплексора + ADC.
   board_read_iload_raw и mux_task должны брать его вокруг
   пары «mux_select_channel + mux_read_sig2_avg_mv», иначе
   mux_task переключит мультиплексор между выбором и чтением
   ADC → ложный OK в try_recovery.
   ============================================================ */
SemaphoreHandle_t mux_get_mutex(void);

void mux_calibrate_imon_offset(int samples);
int  mux_get_imon_offset_mv(uint8_t channel);
void mux_set_imon_offset_mv(uint8_t channel, int offset_mv);

int  mux_read_sig1(void);
int  mux_read_sig2(void);
void mux_read_both(int *sig1, int *sig2);

int  mux_read_sig1_avg(int n);
int  mux_read_sig2_avg(int n);
void mux_read_both_avg(int *sig1, int *sig2, int n);

int  mux_read_sig1_mv(void);
int  mux_read_sig2_mv(void);
void mux_read_both_mv(int *mv1, int *mv2);

int  mux_read_sig1_avg_mv(int n);
int  mux_read_sig2_avg_mv(int n);
void mux_read_both_avg_mv(int *mv1, int *mv2, int n);

int  mux_read_vin_mv(int n);
int  mux_read_iload_ma(int n);
int  mux_read_iload_ma_ch(uint8_t channel, int n);
bool mux_imon_is_fault(int mv);

#ifdef __cplusplus
}
#endif