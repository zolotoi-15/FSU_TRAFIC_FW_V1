#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lora_init(void);
void      lora_receive(void);
esp_err_t lora_send_packet(uint8_t *buf, uint8_t size);
uint8_t   lora_receive_packet(uint8_t *buf, uint8_t size);

/* Callback for received LoRa packets (raw bytes) */
typedef void (*lora_raw_rx_cb_t)(const uint8_t *data, size_t len);
void lora_set_rx_callback(lora_raw_rx_cb_t cb);

/* Diagnostic helpers */
bool lora_is_ready(void);

/* FIX: опрос IRQ для polling-режима (когда DIO1 не подключён
   или используется polling вместо прерываний).
   Вызывать периодически (например, раз в 10 мс) из задачи приёма. */
void lora_poll(void);

#ifdef __cplusplus
}
#endif