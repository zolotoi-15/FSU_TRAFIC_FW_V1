#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callback получает кадр в контексте задачи (не ISR).
   Внутри можно вызывать FreeRTOS API. */
typedef void (*can_rx_cb_t)(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended);

esp_err_t can_bus_init(uint32_t bitrate);
esp_err_t can_bus_start(void);
esp_err_t can_bus_stop(void);
esp_err_t can_bus_set_standby(bool standby);

/* Отправка кадра (блокирует до завершения передачи или таймаута). */
esp_err_t can_bus_send(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended);

/* Регистрация callback. Запускает внутреннюю задачу-диспетчер,
   которая вызывается из task-контекста, а не из ISR. */
void can_bus_set_rx_cb(can_rx_cb_t cb);

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t tx_errors;
    uint32_t bus_off_count;
    uint32_t recovery_count;
    uint32_t error_passive_count;
    uint32_t last_state;
} can_bus_stats_t;
void can_bus_get_stats(can_bus_stats_t *out);
void can_bus_reset_stats(void);

#ifdef __cplusplus
}
#endif