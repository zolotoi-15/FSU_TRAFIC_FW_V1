#include "i2c_bus.h"
#include "board_pins.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_bus";
static const uint8_t regs[] = {0x01,0x03,0x05,0x07,0x09,0x0B,0x0D,0x0F,0x11,0x13};

i2c_master_bus_handle_t g_i2c_bus = NULL;

esp_err_t i2c_bus_init(void)
{
    if (g_i2c_bus) {
        return ESP_OK;   /* уже инициализирована */
    }

    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = I2C_SDA,
        .scl_io_num        = I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&cfg, &g_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus ready: SDA=GPIO%d, SCL=GPIO%d, port=%d",
             I2C_SDA, I2C_SCL, I2C_NUM_0);
    return ESP_OK;
}

void i2c_bus_scan(void)
{
    if (!g_i2c_bus) {
        ESP_LOGE(TAG, "bus not initialized");
        return;
    }

    ESP_LOGI(TAG, "Scanning I2C bus 0x08..0x77 ...");
    int found = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        esp_err_t err = i2c_master_probe(g_i2c_bus, addr, 50);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "no I2C devices found");
    } else {
        ESP_LOGI(TAG, "scan done, %d device(s) found", found);
    }
}

/* ---------- static: только для внутреннего использования ---------- */
static esp_err_t i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t *out)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t dev;
    esp_err_t err = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    err = i2c_master_transmit_receive(dev, &reg, 1, out, 1, 100);
    i2c_master_bus_rm_device(dev);
    return err;
}

void i2c_probe_expander(void)
{
    const uint8_t addrs[] = { 0x21, 0x24 };
    for (size_t i = 0; i < sizeof(addrs); i++) {
        esp_err_t err = i2c_master_probe(g_i2c_bus, addrs[i], 50);
        ESP_LOGI(TAG, "probe 0x%02X: %s", addrs[i], err == ESP_OK ? "ACK" : "NACK");
    }

    uint8_t id;
    if (i2c_read_reg8(0x43, 0x01, &id) == ESP_OK) ESP_LOGI(TAG, "0x43 reg01 = 0x%02X", id);
    if (i2c_read_reg8(0x44, 0x01, &id) == ESP_OK) ESP_LOGI(TAG, "0x44 reg01 = 0x%02X", id);
}

void i2c_dump_regs(uint8_t addr)
{
    ESP_LOGI(TAG, "Dump regs of 0x%02X:", addr);
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(g_i2c_bus, &cfg, &dev) != ESP_OK) return;

    for (size_t i = 0; i < sizeof(regs); i++) {
        uint8_t reg = regs[i], val = 0;
        esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 100);
        if (err == ESP_OK) ESP_LOGI(TAG, "  [0x%02X] = 0x%02X", reg, val);
        else               ESP_LOGW(TAG, "  [0x%02X] read failed: %s", reg, esp_err_to_name(err));
    }
    i2c_master_bus_rm_device(dev);
}

void i2c_dump_all(void)
{
    i2c_dump_regs(0x43);
    i2c_dump_regs(0x44);
}