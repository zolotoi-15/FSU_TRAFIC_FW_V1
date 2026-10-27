#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Запуск PPPoS через A7680C (SIMCom A76XX).
   apn      — строка APN оператора (например "internet").
   timeout_ms — сколько ждать IP после ATD*99#.
   Возвращает ESP_OK, если PPP up и IP получен. */
esp_err_t modem_ppp_start(const char *apn, uint32_t timeout_ms);

/* Остановить PPP, перевести модем в command mode. */
esp_err_t modem_ppp_stop(void);

/* Состояние / IP. */
bool        modem_ppp_is_connected(void);
const char *modem_ppp_get_ip(void);

#ifdef __cplusplus
}
#endif