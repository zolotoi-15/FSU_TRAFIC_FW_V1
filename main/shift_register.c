#include "shift_register.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "sr";
static uint16_t s_sr_state = 0;
static SemaphoreHandle_t s_sr_mutex = NULL;

/* ---------- Низкоуровневая запись БЕЗ мьютекса ---------- */
static void sr_write16_unlocked(uint16_t value)
{
    gpio_set_level(SR_LATCH, 0);
    
   //for (int i = 15; i >= 0; i--) {
       // gpio_set_level(SR_DATA, (value >> i) & 1);
        for (int i = 0; i < 16; i++) {      // LSB-first
    gpio_set_level(SR_DATA, (value >> i) & 1);
        gpio_set_level(SR_CLOCK, 1);
        gpio_set_level(SR_CLOCK, 0);
    }
    gpio_set_level(SR_LATCH, 1);
    gpio_set_level(SR_LATCH, 0);

    s_sr_state = value;
}

/* ---------- Публичная версия С мьютексом ---------- */
void sr_write16(uint16_t value)
{
    if (s_sr_mutex) xSemaphoreTake(s_sr_mutex, portMAX_DELAY);
    sr_write16_unlocked(value);
    if (s_sr_mutex) xSemaphoreGive(s_sr_mutex);
}

void sr_write8(uint8_t value)
{
    sr_write16((uint16_t)value);
}

esp_err_t sr_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << SR_DATA) | (1ULL << SR_CLOCK) | (1ULL << SR_LATCH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    gpio_set_level(SR_LATCH, 0);
    gpio_set_level(SR_CLOCK, 0);
    gpio_set_level(SR_DATA, 0);

    s_sr_mutex = xSemaphoreCreateMutex();
    if (!s_sr_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    s_sr_state = 0;
    sr_write16_unlocked(0x0000);

    ESP_LOGI(TAG, "SN74HC595 x2 initialized: DATA=%d CLOCK=%d LATCH=%d (%d outputs)",
             SR_DATA, SR_CLOCK, SR_LATCH, SR_OUTPUT_COUNT);
    return ESP_OK;
}

esp_err_t sr_set_output(uint8_t output, bool state)
{
    if (output >= SR_OUTPUT_COUNT) return ESP_ERR_INVALID_ARG;
    if (s_sr_mutex) xSemaphoreTake(s_sr_mutex, portMAX_DELAY);

    uint16_t new_state = s_sr_state;
    if (state) new_state |=  (1U << output);
    else       new_state &= ~(1U << output);

    sr_write16_unlocked(new_state);   // ← всегда пишем

    if (s_sr_mutex) xSemaphoreGive(s_sr_mutex);
    return ESP_OK;
}

esp_err_t sr_output_on(uint8_t output)
{
    return sr_set_output(output, true);
}

esp_err_t sr_output_off(uint8_t output)
{
    return sr_set_output(output, false);
}

esp_err_t sr_output_toggle(uint8_t output)
{
    if (output >= SR_OUTPUT_COUNT) return ESP_ERR_INVALID_ARG;

    if (s_sr_mutex) xSemaphoreTake(s_sr_mutex, portMAX_DELAY);
    uint16_t new_state = s_sr_state ^ (1U << output);
    sr_write16_unlocked(new_state);       // ← БЕЗ мьютекса
    if (s_sr_mutex) xSemaphoreGive(s_sr_mutex);
    return ESP_OK;
}

bool sr_get_output(uint8_t output)
{
    if (output >= SR_OUTPUT_COUNT) return false;
    return (s_sr_state & (1U << output)) != 0;
}

uint16_t sr_get_state(void)
{
    return s_sr_state;
}

void sr_set_state(uint16_t mask)
{
    sr_write16(mask);
}

void sr_all_off(void)
{
    sr_write16(0x0000);
}

void sr_all_on(void)
{
    sr_write16(0xFFFF);
}