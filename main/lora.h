#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

esp_err_t lora_init(void);
void lora_set_frequency(uint32_t frequency);
void lora_set_tx_power(int8_t level);
void lora_set_spreading_factor(uint8_t sf);   /* SF5..SF9 для LLCC68 */
void lora_set_bandwidth(uint32_t sbw);
void lora_enable_crc(void);
esp_err_t lora_send_packet(uint8_t *buf, uint8_t size);
void lora_receive(void);
uint8_t lora_receive_packet(uint8_t *buf, uint8_t size);
void lora_reset(void);