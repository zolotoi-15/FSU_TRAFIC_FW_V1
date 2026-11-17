#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   ble_config.h — парсинг JSON-команд из BLE (CMD/CFG Write).

   Команды:
     {"type":"config.get","rid":"1"}
     {"type":"config.set","rid":"2","data":{...}}
     {"type":"lora.get","rid":"3"}
     {"type":"lora.set","rid":"4","freq_hz":869850000,"sf":8,
      "bw_khz":250,"cr":5,"tx_power_dbm":13,"relay":true}
     {"type":"sync.get","rid":"5"}
     {"type":"sync.set","rid":"6","lead_ms":250}
     {"type":"traffic.start","rid":"7"}
     {"type":"traffic.stop","rid":"8"}
     {"type":"reboot","rid":"9"}

   Ответ — в RESP (Notify): {"type":"...","rid":"...","data":...}
   или {"type":"error","rid":"...","detail":"..."}

   === FIX BLE-DISABLED-STUBS ===
   При CONFIG_BT_ENABLED=n файл ble_config.c не собирается
   (см. CMakeLists.txt). Добавлен static inline-стаб, чтобы
   код, вызывающий ble_config_handle_cmd(), не тянул #if.
   ============================================================ */

#if CONFIG_BT_ENABLED

void ble_config_handle_cmd(const char *json, size_t len);

#else  /* !CONFIG_BT_ENABLED */

static inline void ble_config_handle_cmd(const char *json, size_t len)
{
    (void)json; (void)len;
    /* BLE отключён — команды не обрабатываются */
}

#endif /* CONFIG_BT_ENABLED */

#ifdef __cplusplus
}
#endif