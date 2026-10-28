#include "connectivity.h"
#include "app_shared.h"
#include "wifi_ap.h"
#include "wifi_mqtt.h"
#include "modem_ppp.h"
#include "rs485.h"
#include "can_bus.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "connectivity";

#define MODEM_APN             "internet"
#define MODEM_PPP_TIMEOUT_MS  8000
#define MODEM_RETRY_PERIOD_MS 5000

#define MODEM_MAX_FAILED_ATTEMPTS  3

static void modem_retry_task(void *arg)
{
    ESP_LOGI(TAG, "modem_retry_task started (max %d failed attempts, "
                  "then STA fallback)",
             MODEM_MAX_FAILED_ATTEMPTS);

    int failed_attempts = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_PERIOD_MS));

        bool wifi_ok = wifi_sta_has_ip();
        bool ppp_ok  = modem_ppp_is_connected();
        if (wifi_ok || ppp_ok) {
            failed_attempts = 0;
            continue;
        }

        if (failed_attempts >= MODEM_MAX_FAILED_ATTEMPTS) {
            ESP_LOGE(TAG, "modem_retry_task: %d failed attempts → "
                          "STOP PPPoS, enable STA fallback",
                     failed_attempts);
            // === FIX: включить STA как резерв ===
            wifi_sta_start_after_ppp_fail();
            vTaskDelete(NULL);
        }

        ESP_LOGW(TAG, "no IP → try PPPoS (attempt %d/%d)",
                 failed_attempts + 1, MODEM_MAX_FAILED_ATTEMPTS);
        esp_err_t mret = modem_ppp_start(MODEM_APN, MODEM_PPP_TIMEOUT_MS);
        if (mret == ESP_OK) {
            failed_attempts = 0;
            esp_netif_t *ppp_netif = esp_netif_get_handle_from_ifkey("PPP_DEF");
            if (ppp_netif) {
                esp_netif_set_default_netif(ppp_netif);
                ESP_LOGI(TAG, "Default netif → PPPoS (AP continues on ch=6)");
            }
            if (g_config.role == DEVICE_ROLE_MASTER && !wifi_mqtt_is_connected())
                wifi_mqtt_init();
        } else {
            failed_attempts++;
            ESP_LOGW(TAG, "PPPoS failed (attempt %d/%d)",
                     failed_attempts, MODEM_MAX_FAILED_ATTEMPTS);
        }
    }
}

static void rs485_on_frame(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    ESP_LOGI(TAG, "RS485 RX %u bytes: %02X %02X %02X %02X",
             (unsigned)len, data[0],
             len > 1 ? data[1] : 0, len > 2 ? data[2] : 0,
             len > 3 ? data[3] : 0);
}

static void can_on_rx(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended)
{
    ESP_LOGI(TAG, "CAN RX id=0x%lX dlc=%u", (unsigned long)id, dlc);
}

void connectivity_register_rs485_can(void)
{
    if (rs485_init(9600) == ESP_OK) {
        rs485_set_frame_cb(rs485_on_frame, 20);
        ESP_LOGI(TAG, "RS485 background RX started");
    }
    if (can_bus_init(CAN_BITRATE_DEFAULT) == ESP_OK &&
        can_bus_start() == ESP_OK) {
        can_bus_set_rx_cb(can_on_rx);
        ESP_LOGI(TAG, "CAN background RX started");
    }
}

void connectivity_start_tasks(void)
{
    // === FIX: modem_retry_task только на ROOT (MASTER) ===
    if (g_config.role == DEVICE_ROLE_MASTER) {
        xTaskCreate(modem_retry_task, "modem_retry", 4096, NULL, 3, NULL);
    } else {
        ESP_LOGI(TAG, "NODE: modem_retry_task NOT started (no PPPoS on node)");
    }
}