#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"      /* ← добавить */
#include "freertos/semphr.h"        /* ← добавить */
#include "mux_data.h"   /* imon_state_t */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t board_hw_init(void);
void      board_diag_en_self_test(void);

esp_err_t board_diag_en_set(uint8_t channel, bool enable);
void      board_diag_en_all_off(void);
int       board_read_iload_raw(uint8_t channel,
                               imon_state_t *out_state,
                               int *out_mv);

/* === FIX I2C-EXPANDER-MUTEX-P7 ===
   Мьютекс для всех I2C-транзакций expander_a/b. buttons.c
   берёт его вокруг p7_read_raw, чтобы не пересечься с
   board_diag_en_set (read-modify-write OUTPUT_PORT того же
   dev_handle → read_register может вернуть мусор). */
SemaphoreHandle_t board_expander_mutex(void);
/* === END FIX I2C-EXPANDER-MUTEX-P7 === */

#ifdef __cplusplus
}
#endif