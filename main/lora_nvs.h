/* lora_nvs.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t freq_hz;
    uint8_t  sf;
    uint16_t bw_khz;
    uint8_t  cr;
    int8_t   tx_power_dbm;
} lora_cfg_nvs_t;

bool lora_nvs_load(lora_cfg_nvs_t *out);
void lora_nvs_save(const lora_cfg_nvs_t *cfg);
void lora_nvs_erase(void);

#ifdef __cplusplus
}
#endif