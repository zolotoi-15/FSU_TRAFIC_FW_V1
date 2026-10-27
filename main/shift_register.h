#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== Константы ===================== */

/* Количество выходов (2 x SN74HC595 = 16) */
#define SR_OUTPUT_COUNT     16

/* ===================== Инициализация ===================== */

/**
 * @brief Инициализировать GPIO для двух SN74HC595.
 *
 * Настраивает SR_DATA, SR_CLOCK, SR_LATCH как выходы,
 * создаёт мьютекс, сбрасывает все выходы в LOW.
 *
 * @return ESP_OK при успехе, ESP_ERR_NO_MEM если не удалось создать мьютекс
 */
esp_err_t sr_init(void);

/* ===================== Запись целиком ===================== */

/**
 * @brief Записать 16 бит во оба регистра (каскад).
 *
 * Биты сдвигаются MSB-first: первый отправленный бит попадёт
 * в QH' второго регистра (выход 15), последний — в QA первого (выход 0).
 *
 * @param value 16-битное значение (бит 0 → выход 0, бит 15 → выход 15)
 */
void sr_write16(uint16_t value);

/**
 * @brief Записать один байт (младшие 8 бит, выходы 0..7).
 *
 * @param value 8-битное значение
 */
void sr_write8(uint8_t value);

/* ===================== Управление отдельным выходом ===================== */

/**
 * @brief Установить состояние выхода по номеру.
 *
 * Не затрагивает другие выходы — меняет только бит `output`
 * во внутреннем состоянии и перезаписывает все 16 бит.
 *
 * @param output Номер выхода 0..15
 * @param state  true = HIGH, false = LOW
 * @return ESP_OK, ESP_ERR_INVALID_ARG (output >= 16)
 */
esp_err_t sr_set_output(uint8_t output, bool state);

/**
 * @brief Включить выход (HIGH) по номеру.
 *
 * @param output Номер выхода 0..15
 * @return ESP_OK, ESP_ERR_INVALID_ARG
 */
esp_err_t sr_output_on(uint8_t output);

/**
 * @brief Выключить выход (LOW) по номеру.
 *
 * @param output Номер выхода 0..15
 * @return ESP_OK, ESP_ERR_INVALID_ARG
 */
esp_err_t sr_output_off(uint8_t output);

/**
 * @brief Переключить (toggle) выход по номеру.
 *
 * @param output Номер выхода 0..15
 * @return ESP_OK, ESP_ERR_INVALID_ARG
 */
esp_err_t sr_output_toggle(uint8_t output);

/**
 * @brief Прочитать текущее состояние выхода (из внутреннего кэша).
 *
 * @param output Номер выхода 0..15
 * @return true = HIGH, false = LOW или неверный номер
 */
bool sr_get_output(uint8_t output);

/* ===================== Групповое управление ===================== */

/**
 * @brief Получить текущее состояние всех выходов (16 бит).
 *
 * @return Битовая маска: бит 0 → выход 0, ..., бит 15 → выход 15
 */
uint16_t sr_get_state(void);

/**
 * @brief Установить все выходы одной маской.
 *
 * @param mask 16-битная маска
 */
void sr_set_state(uint16_t mask);

/**
 * @brief Выключить все выходы.
 */
void sr_all_off(void);

/**
 * @brief Включить все выходы.
 */
void sr_all_on(void);

#ifdef __cplusplus
}
#endif