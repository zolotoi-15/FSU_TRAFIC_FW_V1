#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === FIX BLE-CONFIG ===
   Парсинг JSON-команд из BLE (CMD/CFG Write).
   Команды:
     {"type":"config.get","rid":"1"}
     {"type":"config.set","rid":"2","data":{...}}
     {"type":"lora.get","rid":"3"}
     {"type":"lora.set","rid":"4","freq_hz":869850000,"sf":8,
      "bw_khz":250,"cr":5,"tx_power_dbm":13,"relay":true}
     {"type":"traffic.start","rid":"5"}
     {"type":"traffic.stop","rid":"6"}
     {"type":"reboot","rid":"7"}

   Ответ — в RESP (Notify): {"type":"...","rid":"...","data":...}
   или {"type":"error","rid":"...","detail":"..."} */

void ble_config_handle_cmd(const char *json, size_t len);

#ifdef __cplusplus
}
#endif