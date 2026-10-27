#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "mux_data.h"   /* imon_state_t */

esp_err_t board_hw_init(void);
void      board_diag_en_self_test(void);

esp_err_t board_diag_en_set(uint8_t channel, bool enable);
void      board_diag_en_all_off(void);
int       board_read_iload_raw(uint8_t channel,
                               imon_state_t *out_state,
                               int *out_mv);