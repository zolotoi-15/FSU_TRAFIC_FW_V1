#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callback получает каждый принятый байт.
   Вызывается из контекста rs485_rx_task (НЕ из ISR).
   Внутри callback можно вызывать FreeRTOS API. */
typedef void (*rs485_byte_cb_t)(uint8_t byte);

/* Callback на целый принятый блок (более эффективно для протоколов
   с кадрами). Вызывается, когда в буфере накопилось >= min_len байт
   или истёк idle-таймаут после последнего байта. */
typedef void (*rs485_frame_cb_t)(const uint8_t *data, size_t len);

esp_err_t rs485_init(uint32_t baudrate);
esp_err_t rs485_set_baudrate(uint32_t baudrate);

esp_err_t rs485_send(const uint8_t *data, size_t len);
int       rs485_read(uint8_t *buf, size_t len, uint32_t timeout_ms);

void rs485_set_re(bool receive_enable);

/* ---------- Фоновый приём ---------- */
/* Регистрирует byte-callback и запускает rs485_rx_task.
   Можно вызвать несколько раз — перезапишет callback. */
void rs485_set_byte_cb(rs485_byte_cb_t cb);

/* Регистрирует frame-callback. idle_timeout_ms — сколько ждать
   тишины после последнего байта, чтобы считать кадр завершённым. */
void rs485_set_frame_cb(rs485_frame_cb_t cb, uint32_t idle_timeout_ms);

/* Запуск/остановка фоновой задачи. Вызывается автоматически из
   первого set_*_cb, но можно явно. */
esp_err_t rs485_rx_task_start(void);
void      rs485_rx_task_stop(void);

/* Статистика (для MQTT/диагностики) */
typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t rx_overflows;
    uint32_t tx_bytes;
    uint32_t tx_errors;
} rs485_stats_t;
void rs485_get_stats(rs485_stats_t *out);
void rs485_reset_stats(void);

#ifdef __cplusplus
}
#endif