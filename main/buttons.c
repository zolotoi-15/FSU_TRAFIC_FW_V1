#include "buttons.h"
#include "app_shared.h"
#include "board_pins.h"
#include "config_manager.h"
#include "pi4ioe5v6408.h"
#include "traffic_light.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "root_wd.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buttons";

#define BOOT_HOLD_MS          3000
#define BOOT_POLL_MS          50
#define LED_MASTER_PERIOD_MS  500
#define LED_SLAVE_PERIOD_MS   100
#define LED_CONFIRM_BLINKS    3
#define LED_CONFIRM_PERIOD_MS 150
#define BTN_EXPANDER_B_PIN    7
#define BTN_POLL_MS           10
#define BTN_DEBOUNCE_COUNT    2
#define BTN_IDLE_POLL_MS      50

static volatile device_role_t s_role = DEVICE_ROLE_MASTER;
static volatile bool          s_role_changing = false;

static inline void led_set(bool on) { gpio_set_level(LED_PIN, on ? 1 : 0); }

static void role_led_task(void *arg) {
    uint32_t last = 0; bool on = false;
    while (1) {
        if (s_role_changing) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        uint32_t period = (g_config.role == DEVICE_ROLE_MASTER)
                          ? LED_MASTER_PERIOD_MS : LED_SLAVE_PERIOD_MS;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last >= period) { last = now; on = !on; led_set(on); }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void role_button_task(void *arg) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    uint32_t press_start = 0;
    bool pressed = false, long_fired = false;

    while (1) {
        bool raw = (gpio_get_level(BOOT_PIN) == 0);
        if (raw && !pressed) {
            pressed = true;
            press_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
            long_fired = false;
            s_role_changing = true;
            led_set(true);
        } else if (raw && pressed && !long_fired) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - press_start >= BOOT_HOLD_MS) {
                long_fired = true;
                device_role_t cur = g_config.role;
                device_role_t nr = (cur == DEVICE_ROLE_MASTER)
                                   ? DEVICE_ROLE_SLAVE : DEVICE_ROLE_MASTER;
                ESP_LOGW(TAG, "ROLE CHANGE: %s → %s",
                         cur == DEVICE_ROLE_MASTER ? "MASTER" : "SLAVE",
                         nr == DEVICE_ROLE_MASTER ? "MASTER" : "SLAVE");
                config_save_role(nr);
                for (int i = 0; i < LED_CONFIRM_BLINKS; i++) {
                    led_set(false); vTaskDelay(pdMS_TO_TICKS(LED_CONFIRM_PERIOD_MS));
                    led_set(true);  vTaskDelay(pdMS_TO_TICKS(LED_CONFIRM_PERIOD_MS));
                }
                s_role_changing = false;
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else if (!raw && pressed) {
            if (!long_fired && g_config.role == DEVICE_ROLE_MASTER) {
                uint8_t mac[6];
                esp_read_mac(mac, ESP_MAC_WIFI_STA);
                char mac_str[18];
                snprintf(mac_str, sizeof(mac_str),
                         "%02X:%02X:%02X:%02X:%02X:%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                mqtt_app_publish_wrapper(mac_str);
            }
            pressed = false;
            s_role_changing = false;
            led_set(false);
        }
        vTaskDelay(pdMS_TO_TICKS(BOOT_POLL_MS));
    }
}

static bool p7_read_raw(void) {
    uint8_t in = 0;
    esp_err_t err = pi4ioe5v6408_read_register(&expander_b, REG_INPUT_STATUS, &in);
    if (err != ESP_OK) return false;
    return (in & (1 << BTN_EXPANDER_B_PIN)) == 0;
}

/* ============================================================
   P7 — вызывная кнопка пешехода.
   === FIX: оба устройства (ROOT и NODE) применяют локально +
   оповещают друг друга. ROOT дополнительно шлёт sync: через
   phase_change_cb (см. fault_handler.c / root_wd.c).
   === FIX: NODE получив pedestrian:request от ROOT — применяет
   локально (см. node_proto.c).
   ============================================================ */
static void button_p7_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(100));
    bool last_raw = false, stable = false;
    int same_count = 0;

    while (1) {
        if (!traffic_light_ped_button_enabled()) {
            vTaskDelay(pdMS_TO_TICKS(BTN_IDLE_POLL_MS));
            last_raw = false; stable = false; same_count = 0;
            continue;
        }
        bool raw = p7_read_raw();
        if (raw == last_raw) {
            if (same_count < BTN_DEBOUNCE_COUNT) same_count++;
            if (same_count == BTN_DEBOUNCE_COUNT && raw != stable) {
                stable = raw;
                if (stable == true) {
                    if (s_cfg_pushing || transport_is_cfg_rx_in_progress()) {
                        ESP_LOGW(TAG, "P7 ignored (CFG in progress)");
                    } else if (g_config.role == DEVICE_ROLE_SLAVE && s_comm_lost) {
                        ESP_LOGW(TAG, "P7 ignored (NODE comm lost)");
                    } else {
                        ESP_LOGI(TAG, "P7 pressed (role=%s, request_mode=%u)",
                                 g_config.role == DEVICE_ROLE_MASTER
                                     ? "ROOT" : "NODE",
                                 g_config.traffic.pedestrian.request_mode);

                        /* 1. Локально — оба устройства */
                        traffic_light_request_pedestrian();

                        /* 2. Оповестить партнёра.
                           ROOT → CMD (NODE применит локально)
                           NODE → EVT (ROOT применит локально) */
                        if (g_config.role == DEVICE_ROLE_MASTER) {
                            transport_send_cmd("pedestrian:request",
                                               strlen("pedestrian:request"));
                        } else {
                            transport_send_evt("pedestrian:request",
                                               strlen("pedestrian:request"));
                        }
                        /* 3. Дальше — естественная смена фазы:
                           ROOT: phase_change_cb → pending sync
                                 → phase_sync_task → sync: на NODE
                           NODE: ждёт sync: от ROOT */
                    }
                }
            }
        } else {
            last_raw = raw;
            same_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

void buttons_start_tasks(void) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    s_role = g_config.role;
    xTaskCreate(role_led_task,    "role_led",  2048, NULL, 2, NULL);
    xTaskCreate(role_button_task, "role_btn",  3072, NULL, 2, NULL);
    if (traffic_light_ped_button_enabled()) {
        xTaskCreate(button_p7_task, "btn_p7", 3072, NULL, 3, NULL);
    }
}