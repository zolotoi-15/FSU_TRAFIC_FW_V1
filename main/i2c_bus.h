#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/* Общая шина I2C для всей платы */
extern i2c_master_bus_handle_t g_i2c_bus;

/**
 * @brief Инициализировать I2C-мастер на пинах I2C_SDA / I2C_SCL.
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Просканировать шину и вывести в лог найденные адреса.
 */
void i2c_bus_scan(void);

void i2c_probe_expander(void);

/* static esp_err_t i2c_read_reg8(...)  ← УДАЛЕНО (используется только внутри i2c_bus.c) */

void i2c_dump_regs(uint8_t addr);
void i2c_dump_all(void);