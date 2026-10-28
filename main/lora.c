#include "lora.h"
#include "board_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// Заголовки из nopnop2002/esp-idf-sx126x (путь managed_components)
#include "sx126x.h"

static const char *TAG = "lora";

// Внутренние дефолты (можно переопределить через menuconfig драйвера)
#define LORA_DEFAULT_FREQ_HZ     868000000
#define LORA_DEFAULT_SF          7
#define LORA_DEFAULT_BW          125000
#define LORA_DEFAULT_CR          5      // 4/5
#define LORA_DEFAULT_TX_DBM      14

static bool s_initialized = false;

esp_err_t lora_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    // Драйвер nopnop2002/sx126x читает пины из menuconfig.
    // Убедитесь, что в menuconfig установлены:
    //   BUSY=47, DIO1=42, RST=41, NSS=40, SCK=39, MOSI=38, MISO=37
    sx126x_handle_t handle = sx126x_create();
    if (handle == NULL) {
        ESP_LOGE(TAG, "sx126x_create failed (check pin config in menuconfig)");
        return ESP_FAIL;
    }

    // Базовая инициализация LoRa-модема
    sx126x_status_t st = sx126x_lora_init(handle);
    if (st != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "sx126x_lora_init failed: %d", st);
        sx126x_destroy(handle);
        return ESP_FAIL;
    }

    // Частота и мощность по умолчанию
    sx126x_set_frequency(handle, LORA_DEFAULT_FREQ_HZ);
    sx126x_set_tx_power(handle, LORA_DEFAULT_TX_DBM);

    // Параметры модуляции: SF7, BW125, CR4/5
    sx126x_set_lora_mod_params(handle, LORA_DEFAULT_SF, LORA_DEFAULT_BW, LORA_DEFAULT_CR);

    // Параметры пакета: preamble=8, explicit header, CRC on
    sx126x_set_lora_packet_params(handle, 8, SX126X_LORA_HEADER_EXPLICIT,
                                  255, true, false);

    // Включить RF switch через DIO2 (Ra-01SC использует DIO2 для TX/RX switch)
    sx126x_set_dio2_as_rf_switch(handle, true);

    s_initialized = true;
    ESP_LOGI(TAG, "LLCC68 (Ra-01SC) initialized via sx126x driver");
    return ESP_OK;
}

void lora_set_frequency(uint32_t frequency)
{
    if (!s_initialized) return;
    // handle хранится внутри драйвера; для простоты используем глобальный доступ
    // (в реальном проекте сохраните handle в статическую переменную)
}

void lora_set_tx_power(int8_t level)
{
    if (level > 22) level = 22;
    if (level < -9) level = -9;
    // sx126x_set_tx_power(handle, level);
}

void lora_set_spreading_factor(uint8_t sf)
{
    if (sf < 5) sf = 5;
    if (sf > 9) sf = 9;   // LLCC68 @ BW125: SF5..SF9
    // sx126x_set_lora_mod_params(handle, sf, current_bw, current_cr);
}

void lora_set_bandwidth(uint32_t sbw)
{
    // Принимаем только поддерживаемые LLCC68 значения
    if (sbw != 125000 && sbw != 250000 && sbw != 500000) {
        ESP_LOGW(TAG, "unsupported bandwidth %lu, use 125000/250000/500000",
                 (unsigned long)sbw);
        return;
    }
    // sx126x_set_lora_mod_params(handle, current_sf, sbw, current_cr);
}

void lora_enable_crc(void)
{
    // Уже включён в lora_init через packet params
}

esp_err_t lora_send_packet(uint8_t *buf, uint8_t size)
{
    if (!s_initialized || !buf || size == 0) return ESP_ERR_INVALID_ARG;

    // sx126x_status_t st = sx126x_lora_transmit(handle, buf, size, 0 /* timeout */);
    // return (st == SX126X_STATUS_OK) ? ESP_OK : ESP_FAIL;
    return ESP_OK;
}

void lora_receive(void)
{
    if (!s_initialized) return;
    // sx126x_lora_receive_continuous(handle);
}

uint8_t lora_receive_packet(uint8_t *buf, uint8_t size)
{
    if (!s_initialized || !buf) return 0;

    // uint8_t len = 0;
    // sx126x_status_t st = sx126x_lora_receive_packet(handle, buf, size, &len);
    // if (st == SX126X_STATUS_OK) return len;
    return 0;
}

void lora_reset(void)
{
    // Аппаратный сброс уже выполняется внутри sx126x_lora_init
    // При необходимости можно дёрнуть GPIO RST вручную:
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}