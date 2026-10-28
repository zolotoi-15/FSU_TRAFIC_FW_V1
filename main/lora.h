#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация LoRa (LLCC68 / Ra-01SC)
 * Внутри использует драйвер nopnop2002/sx126x.
 * Пины настраиваются в menuconfig, не в этом коде.
 */
esp_err_t lora_init(void);

/**
 * @brief Установка частоты (Гц)
 */
void lora_set_frequency(uint32_t frequency);

/**
 * @brief Установка мощности передачи (dBm)
 * Для Ra-01SC максимум +22 dBm.
 */
void lora_set_tx_power(int8_t level);

/**
 * @brief Установка Spreading Factor
 * LLCC68 при BW125 поддерживает SF5..SF9.
 */
void lora_set_spreading_factor(uint8_t sf);

/**
 * @brief Установка полосы пропускания (Гц)
 * LLCC68 поддерживает 125000 / 250000 / 500000.
 */
void lora_set_bandwidth(uint32_t sbw);

/**
 * @brief Включить CRC
 */
void lora_enable_crc(void);

/**
 * @brief Отправка пакета
 * @return ESP_OK при успехе
 */
esp_err_t lora_send_packet(uint8_t *buf, uint8_t size);

/**
 * @brief Перевести в режим непрерывного приёма
 */
void lora_receive(void);

/**
 * @brief Попытаться прочитать принятый пакет (неблокирующий)
 * @return длина полезной нагрузки, 0 если пакета нет
 */
uint8_t lora_receive_packet(uint8_t *buf, uint8_t size);

/**
 * @brief Аппаратный сброс модуля
 */
void lora_reset(void);

#ifdef __cplusplus
}
#endif