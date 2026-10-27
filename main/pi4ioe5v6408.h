#ifndef PI4IOE5V6408_H
#define PI4IOE5V6408_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   Регистры PI4IOE5V6408
   ============================================================ */
#define REG_DEVICE_ID_CTRL      0x01
#define REG_IO_DIRECTION        0x03
#define REG_OUTPUT_PORT         0x05
#define REG_OUTPUT_HIGH_Z       0x07
#define REG_INPUT_DEFAULT_STATE 0x09
#define REG_PULL_ENABLE         0x0B
#define REG_PULL_SELECT         0x0D
#define REG_INPUT_STATUS        0x0F
#define REG_INTERRUPT_MASK      0x11
#define REG_INTERRUPT_STATUS    0x13

#define BIT_SOFTWARE_RESET      0x01

/* ============================================================
   Конфигурация
   ============================================================ */
typedef struct {
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t   clk_speed;
    i2c_port_t i2c_port;
    uint8_t    device_addr;
    bool       invert_outputs;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    bool owns_bus;
    bool initialized;
} pi4ioe5v6408_config_t;

/* ============================================================
   API
   ============================================================ */

/* --- Инициализация --- */
esp_err_t pi4ioe5v6408_init(pi4ioe5v6408_config_t *config);
esp_err_t pi4ioe5v6408_init_with_bus(pi4ioe5v6408_config_t *config,
                                     i2c_master_bus_handle_t bus_handle);
esp_err_t pi4ioe5v6408_deinit(pi4ioe5v6408_config_t *config);

/* --- Низкоуровневые --- */
esp_err_t pi4ioe5v6408_write_register(pi4ioe5v6408_config_t *config,
                                      uint8_t reg, uint8_t data);
esp_err_t pi4ioe5v6408_read_register(pi4ioe5v6408_config_t *config,
                                     uint8_t reg, uint8_t *data);

/* --- Отдельные пины --- */
esp_err_t pi4ioe5v6408_set_pin_direction(pi4ioe5v6408_config_t *config,
                                         uint8_t pin, bool is_input);
esp_err_t pi4ioe5v6408_set_pin_output(pi4ioe5v6408_config_t *config,
                                      uint8_t pin, bool state);
esp_err_t pi4ioe5v6408_set_pin_output_physical(pi4ioe5v6408_config_t *config,
                                               uint8_t pin, bool phys_state);
esp_err_t pi4ioe5v6408_get_pin_input(pi4ioe5v6408_config_t *config,
                                     uint8_t pin, bool *phys_state);
esp_err_t pi4ioe5v6408_set_pin_pull(pi4ioe5v6408_config_t *config,
                                    uint8_t pin, uint8_t pull_type, bool enable);
esp_err_t pi4ioe5v6408_set_pin_high_z(pi4ioe5v6408_config_t *config,
                                      uint8_t pin, bool high_z);

/* --- Весь порт --- */
esp_err_t pi4ioe5v6408_set_port_direction(pi4ioe5v6408_config_t *config,
                                          uint8_t direction_mask);
esp_err_t pi4ioe5v6408_set_port_output(pi4ioe5v6408_config_t *config,
                                       uint8_t output_mask);
esp_err_t pi4ioe5v6408_set_port_output_physical(pi4ioe5v6408_config_t *config,
                                                uint8_t physical_mask);
esp_err_t pi4ioe5v6408_get_port_input(pi4ioe5v6408_config_t *config,
                                      uint8_t *input_mask);

/* --- Служебные --- */
esp_err_t pi4ioe5v6408_software_reset(pi4ioe5v6408_config_t *config);
esp_err_t pi4ioe5v6408_ping(pi4ioe5v6408_config_t *config);
bool      pi4ioe5v6408_is_initialized(pi4ioe5v6408_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* PI4IOE5V6408_H */