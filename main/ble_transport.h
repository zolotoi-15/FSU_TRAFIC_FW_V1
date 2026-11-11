#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === FIX BLE-NAME-CLASH ===
   Функции ble_transport_* конфликтуют с внутренними
   nimble/transport.h (void ble_transport_init(void)).
   Переименованы в fsu_ble_*. */

esp_err_t fsu_ble_init(void);
bool      fsu_ble_is_connected(void);

esp_err_t fsu_ble_notify_status(const char *json, size_t len);
esp_err_t fsu_ble_notify_response(const char *json, size_t len);

bool fsu_ble_send_raw(const uint8_t *data, size_t len);
/* === END FIX BLE-NAME-CLASH === */

#ifdef __cplusplus
}
#endif