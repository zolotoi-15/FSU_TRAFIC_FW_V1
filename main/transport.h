#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*transport_rx_cb_t)(const char *data, size_t len);

/* === FIX CONNECT-ANY-ORDER === */
typedef void (*transport_ack_cb_t)(uint8_t seq, uint8_t type);
void transport_set_ack_cb(transport_ack_cb_t cb);
/* === END FIX CONNECT-ANY-ORDER === */

esp_err_t transport_init(void);
bool      transport_is_lora(void);
bool      transport_is_tcp(void);

esp_err_t transport_send_cmd(const char *cmd, size_t len);
esp_err_t transport_send_cmd_noack(const char *cmd, size_t len);

esp_err_t transport_send_cfg(const char *json, size_t len);
esp_err_t transport_send_status(const char *json, size_t len);
esp_err_t transport_send_status_noack(const char *json, size_t len);

esp_err_t transport_send_evt(const char *evt, size_t len);
esp_err_t transport_send_evt_noack(const char *evt, size_t len);

/* === FIX T_CHANNELS === */
esp_err_t transport_send_channels(const char *json, size_t len);
/* === END FIX T_CHANNELS === */

void transport_set_cmd_cb(transport_rx_cb_t cb);
void transport_set_cfg_cb(transport_rx_cb_t cb);
void transport_set_status_cb(transport_rx_cb_t cb);
void transport_set_evt_cb(transport_rx_cb_t cb);
/* === FIX T_CHANNELS === */
void transport_set_channels_cb(transport_rx_cb_t cb);
/* === END FIX T_CHANNELS === */

void transport_force_lora(void);
void transport_force_tcp(void);

void transport_reset_cfg_accum(void);
bool transport_is_cfg_rx_in_progress(void);
bool transport_is_tx_idle(void);
void transport_clear_cfg_expected(void);

esp_err_t transport_send_retry_request(uint8_t type, uint8_t start_frag_idx);

/* === FIX BLE-TRANSPORT ===
   Единая точка приёма сырых пакетов (LoRa RX + BLE RX).
   BLE-сервер в ble_transport.c вызывает transport_handle_raw_rx
   при получении Write в характеристику CFG/CMD — те же байты,
   что и по LoRa (PKT_MAGIC, фрагментация, ACK). */
void transport_handle_raw_rx(const uint8_t *data, size_t len);

/* Отправка сырого пакета через BLE (если подключён) или LoRa.
   Используется, если нужен обход фрагментации (не основной путь). */
esp_err_t transport_send_raw(uint8_t type, const uint8_t *payload, size_t len);

/* true, если BLE-соединение активно (телефон подключён) */
bool transport_is_ble_active(void);
/* === END FIX BLE-TRANSPORT === */

#ifdef __cplusplus
}
#endif