#include "eth_w5500.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth_mac_w5500.h"  // Обязательно для IDF 6.1
#include "esp_eth_phy_w5500.h"  // Обязательно для IDF 6.1

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth_netif_glue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "eth_w5500";

static esp_eth_handle_t             s_eth_handle = NULL;
static esp_eth_netif_glue_handle_t  s_eth_glue   = NULL;
static esp_netif_t                 *s_eth_netif  = NULL;

/* ---------- Обработчики событий ---------- */
static void eth_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:    ESP_LOGI(TAG, "Link up");   break;
    case ETHERNET_EVENT_DISCONNECTED: ESP_LOGW(TAG, "Link down"); break;
    case ETHERNET_EVENT_START:        ESP_LOGI(TAG, "Started");   break;
    case ETHERNET_EVENT_STOP:         ESP_LOGI(TAG, "Stopped");   break;
    default: break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t base,
                                 int32_t id, void *data)
{
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
}

/* ---------- Инициализация ---------- */
esp_err_t eth_w5500_app_init(void)
{
    ESP_LOGI(TAG, "Initializing W5500 on SPI3_HOST");

    /* 1. Установка GPIO ISR service (если ещё не установлен) */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isr_ret));
        return isr_ret;
    }

    /* 2. SPI-шина */
    spi_bus_config_t buscfg = {
        .miso_io_num = ETH_MISO,
        .mosi_io_num = ETH_MOSI,
        .sclk_io_num = ETH_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t ret = spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3. Конфигурация SPI-устройства */
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 2 * 1000 * 1000,
        .queue_size = 32,
        .spics_io_num = ETH_CS,
    };

    /* 4. Конфигурация W5500 (IDF 6.1: int_gpio_num внутри base) */
    eth_w5500_config_t w5500_cfg = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_cfg.base.int_gpio_num = ETH_INT;

    /* Если INT не используется (пин < 0), задаём период опроса */
    if (ETH_INT < 0) {
        w5500_cfg.base.poll_period_ms = 100; // 100 мс
    }

    /* 5. MAC */
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_stack_size = 4096;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    if (!mac) {
        ESP_LOGE(TAG, "esp_eth_mac_new_w5500 failed");
        return ESP_FAIL;
    }

    /* 6. PHY */
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num   = ETH_RST;
    phy_cfg.reset_timeout_ms = 100;
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
    if (!phy) {
        ESP_LOGE(TAG, "esp_eth_phy_new_w5500 failed");
        return ESP_FAIL;
    }

spi_device_handle_t test_dev;
spi_device_interface_config_t test_cfg = {
    .mode = 0, .clock_speed_hz = 1000000, .spics_io_num = ETH_CS,
};
spi_bus_add_device(ETH_SPI_HOST, &test_cfg, &test_dev);

uint8_t tx[4] = {0x00, 0x39, 0x00, 0x00};
uint8_t rx[4] = {0};
spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
spi_device_polling_transmit(test_dev, &t);
ESP_LOGI(TAG, "W5500 VERSIONR = 0x%02X (ожидается 0x04)", rx[3]);
spi_bus_remove_device(test_dev);

    /* 7. Установка Ethernet драйвера */
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_cfg, &s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 8. Netif + glue */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    s_eth_glue  = esp_eth_new_netif_glue(s_eth_handle);
    esp_netif_attach(s_eth_netif, s_eth_glue);

    /* 9. Обработчики событий (регистрировать до запуска драйвера) */
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &got_ip_event_handler, NULL));

    ESP_LOGI(TAG, "W5500 initialized");
    return ESP_OK;
}

esp_err_t eth_w5500_app_start(void)
{
    if (!s_eth_handle) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = esp_eth_start(s_eth_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Ethernet started, waiting for link...");
    }
    return ret;
}