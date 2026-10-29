#pragma once
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Регистрация callback'ов протокола.
 *
 * Внутри вызывает transport_set_cmd_cb() / transport_set_cfg_cb() /
 * transport_set_status_cb() / transport_set_evt_cb(), которые
 * работают поверх активного транспорта (LoRa или TCP fallback).
 *
 * Должна вызываться ОДИН раз после config_init(), до старта задач.
 */
void node_proto_register_callbacks(void);

/**
 * @brief Инициализация клиентского транспорта (для NODE).
 *
 * Выполняет:
 *   1. transport_init()        — пытается поднять LoRa, при неудаче
 *                                оставляет TCP как основной.
 *   2. node_link_client_start() — TCP-клиент стартует ВСЕГДА, как
 *                                резервный канал.
 *
 * @param host  IP root-устройства (используется только TCP-клиентом).
 * @param port  TCP-порт root (обычно 3333).
 */
void node_proto_client_start(const char *host, uint16_t port);

/**
 * @brief Старт рабочих задач протокола (NODE).
 *
 * Запускает:
 *   - node_send_task         — status: раз в секунду через активный транспорт;
 *   - node_ap_retry_task     — повторное подключение к FSU_*_AP при обрыве;
 *   - node_link_watchdog_task — 30 с без sync → ALL_OFF.
 */
void node_proto_start_tasks(void);

/**
 * @brief Обработчик локальных команд (rs485:*, can:*).
 *
 * Вызывается из on_cmd_from_root() для команд, которые не нужно
 * пробрасывать дальше. Экспортируется, чтобы другие модули могли
 * выполнять те же команды локально.
 */
void node_handle_local_cmd(const char *cmd);

#ifdef __cplusplus
}
#endif