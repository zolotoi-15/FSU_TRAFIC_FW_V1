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

/* C-совместимый raw-callback: (data, len). 
   Внутри lora.cpp адаптируется к espp::Sx126x::RxPacket. */
typedef void (*lora_raw_rx_cb_t)(const uint8_t *data, size_t len);
void lora_set_rx_callback(lora_raw_rx_cb_t cb);

/* Diagnostic helpers */
bool lora_is_ready(void);

/* Опрос IRQ — обязателен, если DIO1 не подключён.
   Вызывать из задачи каждые ~10 мс. */
void lora_poll(void);

#ifdef __cplusplus
}
#endif