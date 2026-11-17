#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   BLE transport — GATT-сервер для настройки/мониторинга.

   === FIX BLE-NAME-CLASH ===
   Функции переименованы ble_transport_* → fsu_ble_*, чтобы не
   конфликтовать с внутренними nimble/transport.h
   (void ble_transport_init(void)).

   === FIX BLE-DISABLED-STUBS ===
   При CONFIG_BT_ENABLED=n файл ble_transport.c не собирается
   (см. CMakeLists.txt), но код в mqtt_json.c / blink_example_main.c
   всё равно вызывает fsu_ble_*. Добавлены static inline-стабы,
   чтобы не оборачивать каждый вызов в #if CONFIG_BT_ENABLED.
   ============================================================ */

#if CONFIG_BT_ENABLED

esp_err_t fsu_ble_init(void);
bool      fsu_ble_is_connected(void);

esp_err_t fsu_ble_notify_status(const char *json, size_t len);
esp_err_t fsu_ble_notify_response(const char *json, size_t len);

bool      fsu_ble_send_raw(const uint8_t *data, size_t len);

#else  /* !CONFIG_BT_ENABLED */

#include "esp_err.h"

static inline esp_err_t fsu_ble_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static inline bool fsu_ble_is_connected(void)
{
    return false;
}
static inline esp_err_t fsu_ble_notify_status(const char *json, size_t len)
{
    (void)json; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}
static inline esp_err_t fsu_ble_notify_response(const char *json, size_t len)
{
    (void)json; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}
static inline bool fsu_ble_send_raw(const uint8_t *data, size_t len)
{
    (void)data; (void)len;
    return false;
}

#endif /* CONFIG_BT_ENABLED */

#ifdef __cplusplus
}
#endif