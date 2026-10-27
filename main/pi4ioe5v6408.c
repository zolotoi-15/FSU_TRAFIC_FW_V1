#include "pi4ioe5v6408.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PI4IOE5V6408";

/* ============================================================
   Инициализация с уже созданной I2C-шиной
   ============================================================ */
esp_err_t pi4ioe5v6408_init_with_bus(pi4ioe5v6408_config_t *config,
                                     i2c_master_bus_handle_t bus_handle)
{
    if (config == NULL || bus_handle == NULL) return ESP_ERR_INVALID_ARG;
    if (config->initialized) return ESP_OK;

    config->bus_handle = bus_handle;
    config->owns_bus   = false;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = config->device_addr,
        .scl_speed_hz    = config->clk_speed,
    };
    esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_cfg,
                                              &config->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add device at 0x%02X: %s",
                 config->device_addr, esp_err_to_name(ret));
        return ret;
    }

    /* Проверка наличия устройства чтением Device ID */
    uint8_t device_id = 0;
    ret = pi4ioe5v6408_read_register(config, REG_DEVICE_ID_CTRL, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read Device ID @0x%02X", config->device_addr);
        i2c_master_bus_rm_device(config->dev_handle);
        config->dev_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "@0x%02X Device ID register: 0x%02X",
             config->device_addr, device_id);

    /* 1. Все выходы в 0 */
    ret = pi4ioe5v6408_write_register(config, REG_OUTPUT_PORT, 0x00);
    if (ret != ESP_OK) goto fail;

    /* 2. Отключить high-Z */
    ret = pi4ioe5v6408_write_register(config, REG_OUTPUT_HIGH_Z, 0x00);
    if (ret != ESP_OK) goto fail;

    /* 3. Отключить подтяжки */
    ret = pi4ioe5v6408_write_register(config, REG_PULL_ENABLE, 0x00);
    if (ret != ESP_OK) goto fail;

    /* 4. Все пины — выходы */
    ret = pi4ioe5v6408_set_port_direction(config, 0xFF);
    if (ret != ESP_OK) goto fail;

    /* 5. Маскировать все прерывания */
    ret = pi4ioe5v6408_write_register(config, REG_INTERRUPT_MASK, 0xFF);
    if (ret != ESP_OK) goto fail;

    config->initialized = true;
    ESP_LOGI(TAG, "PI4IOE5V6408 initialized @0x%02X", config->device_addr);
    return ESP_OK;

fail:
    ESP_LOGW(TAG, "@0x%02X init failed: %s",
             config->device_addr, esp_err_to_name(ret));
    i2c_master_bus_rm_device(config->dev_handle);
    config->dev_handle = NULL;
    return ret;
}

/* ============================================================
   Инициализация с созданием собственной I2C-шины
   ============================================================ */
static void i2c_scan_bus(i2c_master_bus_handle_t bus)
{
    ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_master_dev_handle_t dev;
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = addr,
            .scl_speed_hz    = 100000,
        };
        if (i2c_master_bus_add_device(bus, &cfg, &dev) == ESP_OK) {
            uint8_t dummy;
            if (i2c_master_transmit(dev, &dummy, 1, 50) == ESP_OK ||
                i2c_master_receive(dev, &dummy, 1, 50) == ESP_OK) {
                ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
            }
            i2c_master_bus_rm_device(dev);
        }
    }
}

esp_err_t pi4ioe5v6408_init(pi4ioe5v6408_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    if (config->initialized) return ESP_OK;

    if (config->bus_handle == NULL) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = config->i2c_port,
            .sda_io_num = config->sda_pin,
            .scl_io_num = config->scl_pin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t ret = i2c_new_master_bus(&bus_cfg, &config->bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
            return ret;
        }
        i2c_scan_bus(config->bus_handle);
        config->owns_bus = true;
    } else {
        config->owns_bus = false;
    }
    return pi4ioe5v6408_init_with_bus(config, config->bus_handle);
}

/* ============================================================
   Деинициализация
   ============================================================ */
esp_err_t pi4ioe5v6408_deinit(pi4ioe5v6408_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;

    if (config->dev_handle) {
        i2c_master_bus_rm_device(config->dev_handle);
        config->dev_handle = NULL;
    }
    if (config->owns_bus && config->bus_handle) {
        i2c_del_master_bus(config->bus_handle);
        config->bus_handle = NULL;
        config->owns_bus   = false;
    }
    config->initialized = false;
    ESP_LOGI(TAG, "PI4IOE5V6408 @0x%02X deinitialized", config->device_addr);
    return ESP_OK;
}

/* ============================================================
   Низкоуровневые операции
   ============================================================ */
esp_err_t pi4ioe5v6408_write_register(pi4ioe5v6408_config_t *config,
                                      uint8_t reg, uint8_t data)
{
    if (config == NULL || config->dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(config->dev_handle, buf, sizeof(buf),
                               pdMS_TO_TICKS(1000));
}

esp_err_t pi4ioe5v6408_read_register(pi4ioe5v6408_config_t *config,
                                     uint8_t reg, uint8_t *data)
{
    if (config == NULL || data == NULL || config->dev_handle == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_master_transmit(config->dev_handle, &reg, 1,
                                        pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    return i2c_master_receive(config->dev_handle, data, 1,
                              pdMS_TO_TICKS(1000));
}

/* ============================================================
   Управление пинами
   ============================================================ */
esp_err_t pi4ioe5v6408_set_pin_direction(pi4ioe5v6408_config_t *config,
                                         uint8_t pin, bool is_input)
{
    if (config == NULL || pin > 7) return ESP_ERR_INVALID_ARG;

    uint8_t dir = 0;
    esp_err_t ret = pi4ioe5v6408_read_register(config, REG_IO_DIRECTION, &dir);
    if (ret != ESP_OK) return ret;

    if (is_input) dir &= ~(1 << pin);
    else          dir |=  (1 << pin);

    return pi4ioe5v6408_write_register(config, REG_IO_DIRECTION, dir);
}

esp_err_t pi4ioe5v6408_set_pin_output(pi4ioe5v6408_config_t *config,
                                      uint8_t pin, bool state)
{
    return pi4ioe5v6408_set_pin_output_physical(config, pin, state);
}

esp_err_t pi4ioe5v6408_set_pin_output_physical(pi4ioe5v6408_config_t *config,
                                               uint8_t pin, bool phys_state)
{
    if (config == NULL || pin > 7) return ESP_ERR_INVALID_ARG;

    uint8_t out = 0;
    esp_err_t ret = pi4ioe5v6408_read_register(config, REG_OUTPUT_PORT, &out);
    if (ret != ESP_OK) return ret;

    if (phys_state) out |=  (1 << pin);
    else            out &= ~(1 << pin);

    return pi4ioe5v6408_write_register(config, REG_OUTPUT_PORT, out);
}

esp_err_t pi4ioe5v6408_get_pin_input(pi4ioe5v6408_config_t *config,
                                     uint8_t pin, bool *phys_state)
{
    if (config == NULL || phys_state == NULL || pin > 7)
        return ESP_ERR_INVALID_ARG;

    uint8_t in = 0;
    esp_err_t ret = pi4ioe5v6408_read_register(config, REG_INPUT_STATUS, &in);
    if (ret != ESP_OK) return ret;

    *phys_state = (in & (1 << pin)) != 0;
    return ESP_OK;
}

/* ============================================================
   Подтяжки и high-Z
   ============================================================ */
esp_err_t pi4ioe5v6408_set_pin_pull(pi4ioe5v6408_config_t *config,
                                    uint8_t pin, uint8_t pull_type, bool enable)
{
    if (config == NULL || pin > 7 || pull_type > 1) return ESP_ERR_INVALID_ARG;

    uint8_t select = 0, enable_reg = 0;
    esp_err_t ret = pi4ioe5v6408_read_register(config, REG_PULL_SELECT, &select);
    if (ret != ESP_OK) return ret;

    ret = pi4ioe5v6408_read_register(config, REG_PULL_ENABLE, &enable_reg);
    if (ret != ESP_OK) return ret;

    if (pull_type) select     |=  (1 << pin);
    else           select     &= ~(1 << pin);

    if (enable)    enable_reg |=  (1 << pin);
    else           enable_reg &= ~(1 << pin);

    ret = pi4ioe5v6408_write_register(config, REG_PULL_SELECT, select);
    if (ret != ESP_OK) return ret;

    return pi4ioe5v6408_write_register(config, REG_PULL_ENABLE, enable_reg);
}

esp_err_t pi4ioe5v6408_set_pin_high_z(pi4ioe5v6408_config_t *config,
                                      uint8_t pin, bool high_z)
{
    if (config == NULL || pin > 7) return ESP_ERR_INVALID_ARG;

    uint8_t hiz = 0;
    esp_err_t ret = pi4ioe5v6408_read_register(config, REG_OUTPUT_HIGH_Z, &hiz);
    if (ret != ESP_OK) return ret;

    if (high_z) hiz |=  (1 << pin);
    else        hiz &= ~(1 << pin);

    return pi4ioe5v6408_write_register(config, REG_OUTPUT_HIGH_Z, hiz);
}

/* ============================================================
   Групповые операции
   ============================================================ */
esp_err_t pi4ioe5v6408_set_port_direction(pi4ioe5v6408_config_t *config,
                                          uint8_t direction_mask)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    return pi4ioe5v6408_write_register(config, REG_IO_DIRECTION, direction_mask);
}

esp_err_t pi4ioe5v6408_set_port_output(pi4ioe5v6408_config_t *config,
                                       uint8_t output_mask)
{
    return pi4ioe5v6408_set_port_output_physical(config, output_mask);
}

esp_err_t pi4ioe5v6408_set_port_output_physical(pi4ioe5v6408_config_t *config,
                                                uint8_t physical_mask)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    return pi4ioe5v6408_write_register(config, REG_OUTPUT_PORT, physical_mask);
}

esp_err_t pi4ioe5v6408_get_port_input(pi4ioe5v6408_config_t *config,
                                      uint8_t *input_mask)
{
    if (config == NULL || input_mask == NULL) return ESP_ERR_INVALID_ARG;
    return pi4ioe5v6408_read_register(config, REG_INPUT_STATUS, input_mask);
}

/* ============================================================
   Reset / Ping / Status
   ============================================================ */
esp_err_t pi4ioe5v6408_software_reset(pi4ioe5v6408_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGW(TAG, "@0x%02X software reset", config->device_addr);
    esp_err_t ret = pi4ioe5v6408_write_register(config, REG_DEVICE_ID_CTRL,
                                                BIT_SOFTWARE_RESET);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t id = 0;
    for (int i = 0; i < 10; i++) {
        if (pi4ioe5v6408_read_register(config, REG_DEVICE_ID_CTRL, &id) == ESP_OK) {
            ESP_LOGW(TAG, "@0x%02X reset OK, ID=0x%02X",
                     config->device_addr, id);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t pi4ioe5v6408_ping(pi4ioe5v6408_config_t *config)
{
    if (config == NULL || config->dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t dummy;
    return pi4ioe5v6408_read_register(config, REG_DEVICE_ID_CTRL, &dummy);
}

bool pi4ioe5v6408_is_initialized(pi4ioe5v6408_config_t *config)
{
    return config && config->initialized && config->dev_handle != NULL;
}