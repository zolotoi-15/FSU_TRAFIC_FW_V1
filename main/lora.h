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

typedef void (*lora_raw_rx_cb_t)(const uint8_t *data, size_t len);
void lora_set_rx_callback(lora_raw_rx_cb_t cb);

bool lora_is_ready(void);
void lora_poll(void);

/* === FIX TX-BUSY: true, пока радио в TX. ack_tx_task ждёт, пока
   не станет false, прежде чем отправить ACK. === */
bool lora_is_tx_active(void);
/* === END FIX TX-BUSY === */

#ifdef __cplusplus
}
#endif