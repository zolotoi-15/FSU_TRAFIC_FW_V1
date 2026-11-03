/* ============================================================
   connectivity.c — PPPoS (модем) + RS485/CAN.
   Wi-Fi STA fallback отключён — только PPP для MQTT.

   === NEW: PPP watchdog ===
   Если MQTT не подключён дольше MQTT_DOWN_RESTART_MS (5 мин),
   wifi_mqtt_force_restart_modem() уже вызвал modem_ppp_stop().
   Здесь modem_retry_task поднимает PPP заново.

   Дополнительно: если PPP «подключён», но MQTT down > 5 мин —
   принудительно останавливаем PPP и перезапускаем модем.
   ============================================================ */

#include "connectivity.h"
#include "app_shared.h"
#include "wifi_mqtt.h"
#include "modem_ppp.h"
#include "rs485.h"
#include "can_bus.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "connectivity";

#define MODEM_APN             "internet"
#define MODEM_PPP_TIMEOUT_MS  8000
#define MODEM_RETRY_PERIOD_MS 5000

/* === NEW: PPP watchdog === */
#define PPP_RETRY_MAX_DOWN_MS    (5 * 60 * 1000)   /* 5 мин PPP down → force reset */
#define PPP_CHECK_PERIOD_MS      10000

/* === NEW: счётчик последовательных неудач PPP === */
static int s_ppp_fail_count = 0;

/* === NEW: внешние геттеры из wifi_mqtt.c === */
extern uint32_t wifi_mqtt_get_down_ms(void);
extern void     wifi_mqtt_force_restart_modem(void);

/* ============================================================
   modem_retry_task — поднимает PPP, если его нет.
   Если PPP поднят, но MQTT down > 5 мин → принудительный
   перезапуск модема.
   ============================================================ */
static void modem_retry_task(void *arg)
{
    ESP_LOGI(TAG, "modem_retry_task started (NO Wi-Fi STA fallback)");

    uint32_t ppp_down_since_ms = 0;   /* когда PPP «упал» (0 = PPP up) */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MODEM_RETRY_PERIOD_MS));

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        /* ─── PPP уже поднят ─── */
        if (modem_ppp_is_connected()) {
            s_ppp_fail_count = 0;
            ppp_down_since_ms = 0;

            /* Проверка: PPP up, но MQTT down > 5 мин → рестарт модема */
            if (g_config.role == DEVICE_ROLE_MASTER) {
                uint32_t mqtt_down = wifi_mqtt_get_down_ms();
                if (mqtt_down > PPP_RETRY_MAX_DOWN_MS) {
                    ESP_LOGE(TAG, "PPP up but MQTT down %lu ms → "
                                  "force restart modem",
                             (unsigned long)mqtt_down);
                    wifi_mqtt_force_restart_modem();   /* = modem_ppp_stop() */
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ppp_down_since_ms = now;
                    continue;
                }
            }
            continue;
        }

        /* ─── PPP down ─── */
        if (ppp_down_since_ms == 0) {
            ppp_down_since_ms = now;
            ESP_LOGW(TAG, "PPPoS down, starting recovery timer");
        }

        uint32_t down_for = now - ppp_down_since_ms;
        if (down_for > PPP_RETRY_MAX_DOWN_MS) {
            ESP_LOGE(TAG, "PPPoS down > %lu ms → hard reset modem",
                     (unsigned long)down_for);
            /* modem_ppp_stop() уже мог быть вызван из wifi_mqtt.c,
               но на всякий случай — ещё раз */
            modem_ppp_stop();
            vTaskDelay(pdMS_TO_TICKS(5000));
            ppp_down_since_ms = 0;   /* сброс — попробуем заново */
            continue;
        }

        ESP_LOGW(TAG, "no IP → try PPPoS (attempt %d)", s_ppp_fail_count + 1);
        esp_err_t mret = modem_ppp_start(MODEM_APN, MODEM_PPP_TIMEOUT_MS);
        if (mret == ESP_OK) {
            s_ppp_fail_count = 0;
            ppp_down_since_ms = 0;
            esp_netif_t *ppp_netif = esp_netif_get_handle_from_ifkey("PPP_DEF");
            if (ppp_netif) {
                esp_netif_set_default_netif(ppp_netif);
                ESP_LOGI(TAG, "Default netif → PPPoS");
            }
            if (g_config.role == DEVICE_ROLE_MASTER && !wifi_mqtt_is_connected())
                wifi_mqtt_init();
        } else {
            s_ppp_fail_count++;
            ESP_LOGW(TAG, "PPPoS failed (%d), retry in %d ms",
                     s_ppp_fail_count, MODEM_RETRY_PERIOD_MS);
        }
    }
}

/* ============================================================
   RS485 / CAN
   ============================================================ */
static void rs485_on_frame(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    ESP_LOGI(TAG, "RS485 RX %u bytes", (unsigned)len);
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
    if (g_config.role == DEVICE_ROLE_MASTER) {
        xTaskCreate(modem_retry_task, "modem_retry", 4096, NULL, 3, NULL);
    } else {
        ESP_LOGI(TAG, "NODE: modem_retry_task NOT started");
    }
}