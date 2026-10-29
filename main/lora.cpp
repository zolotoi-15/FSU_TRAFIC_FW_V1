#include "lora.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "sx126x.hpp"
#include <memory>
#include <span>
#include <chrono>
#include <system_error>
#include <cstring>

static const char *TAG = "lora";
static spi_device_handle_t s_spi_handle = nullptr;
static std::unique_ptr<espp::Sx126x> s_radio;
static lora_raw_rx_cb_t s_raw_cb = nullptr;

/* --- SPI wrappers (must return bool) --- */
static bool spi_write(const uint8_t *data, size_t len) {
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    return spi_device_transmit(s_spi_handle, &t) == ESP_OK;
}

static bool spi_write_then_read(const uint8_t *write_data, size_t write_len,
                                uint8_t *read_data, size_t read_len) {
    if (spi_device_acquire_bus(s_spi_handle, portMAX_DELAY) != ESP_OK)
        return false;

    spi_transaction_t tw = {};
    tw.length = write_len * 8;
    tw.tx_buffer = write_data;
    tw.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    esp_err_t err = spi_device_transmit(s_spi_handle, &tw);
    if (err != ESP_OK) { spi_device_release_bus(s_spi_handle); return false; }

    esp_rom_delay_us(10);

    spi_transaction_t tr = {};
    tr.length = read_len * 8;
    tr.rx_buffer = read_data;
    err = spi_device_transmit(s_spi_handle, &tr);

    spi_device_release_bus(s_spi_handle);
    return err == ESP_OK;
}

static bool is_busy() { return gpio_get_level(LORA_BUSY) != 0; }
static void reset(bool level) { gpio_set_level(LORA_RST, level ? 1 : 0); }

/* --- receive callback from espp --- */
static void on_rx_packet(const espp::Sx126x::RxPacket &pkt) {
    ESP_LOGI(TAG, "on_rx_packet: %d bytes, rssi=%.1f, snr=%.1f",
             (int)pkt.data.size(), pkt.status.rssi, pkt.status.snr);
    if (s_raw_cb && !pkt.data.empty()) {
        s_raw_cb(pkt.data.data(), pkt.data.size());
    } else if (!pkt.data.empty()) {
        ESP_LOGI(TAG, "RX (raw): %.*s",
                 (int)pkt.data.size(), (const char *)pkt.data.data());
    }
}

extern "C" esp_err_t lora_init(void) {
    if (s_radio) return ESP_OK;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = LORA_MOSI;
    bus_cfg.miso_io_num = LORA_MISO;
    bus_cfg.sclk_io_num = LORA_SCK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 4 * 1000 * 1000;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = LORA_CS;
    dev_cfg.queue_size = 7;
    spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_handle);

    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_BUSY, GPIO_MODE_INPUT);

    espp::Sx126x::Config cfg;
    cfg.variant = espp::Sx126x::Variant::LLCC68;
    cfg.write = spi_write;
    cfg.write_then_read = spi_write_then_read;
    cfg.is_busy = is_busy;
    cfg.reset = reset;
    cfg.tcxo_voltage = 0.0f;
    cfg.use_dio2_as_rf_switch = true;
    cfg.use_dcdc_regulator = false;
    cfg.auto_init = false;
    cfg.radio_config.frequency_hz = 868000000;
    cfg.radio_config.tx_power_dbm = 14;
    cfg.radio_config.spreading_factor = espp::Sx126x::SpreadingFactor::SF7;
    cfg.radio_config.bandwidth = espp::Sx126x::Bandwidth::BW_125_KHZ;
    cfg.radio_config.coding_rate = espp::Sx126x::CodingRate::CR_4_5;
    cfg.radio_config.sync_word = 0x12;
    cfg.on_receive = on_rx_packet;
    cfg.log_level = espp::Logger::Verbosity::DEBUG;

    s_radio = std::make_unique<espp::Sx126x>(cfg);

    std::error_code ec;
    if (!s_radio->initialize(ec)) {
        ESP_LOGE(TAG, "Sx126x init failed: %s", ec.message().c_str());
        s_radio.reset();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LoRa ready (LLCC68, 868 MHz, SF7/BW125)");

    s_radio->start_receive(ec);
    if (ec) {
        ESP_LOGE(TAG, "start_receive failed: %s", ec.message().c_str());
    } else {
        ESP_LOGI(TAG, "LoRa RX mode enabled");
    }

    return ESP_OK;
}

extern "C" void lora_receive(void) {
    if (!s_radio) return;
    std::error_code ec;
    s_radio->start_receive(ec);
    if (ec) {
        ESP_LOGE(TAG, "lora_receive: %s", ec.message().c_str());
    } else {
        ESP_LOGI(TAG, "LoRa RX mode (re)enabled");
    }
}

extern "C" esp_err_t lora_send_packet(uint8_t *buf, uint8_t size) {
    if (!s_radio || !buf || size == 0) return ESP_ERR_INVALID_ARG;
    std::error_code ec;
    bool ok = s_radio->transmit(std::span<const uint8_t>(buf, size),
                                std::chrono::milliseconds(2000), ec);
    ESP_LOGI(TAG, "tx ok=%d ec=%s", (int)ok, ec.message().c_str());
    std::error_code ec2;
    s_radio->start_receive(ec2);
    if (!ok) {
        ESP_LOGW(TAG, "tx failed: %s", ec.message().c_str());
        return ESP_FAIL;
    }
    return ESP_OK;
}

extern "C" uint8_t lora_receive_packet(uint8_t *buf, uint8_t size) {
    (void)buf; (void)size;
    return 0;
}

extern "C" void lora_set_rx_callback(lora_raw_rx_cb_t cb) { s_raw_cb = cb; }
extern "C" bool lora_is_ready(void) { return (bool)s_radio; }

/* ============================================================
   FIX: lora_poll — опрос IRQ для polling-режима.
   Когда DIO1 не подключён или используется polling,
   нужно периодически вызывать service_interrupts(),
   иначе RX_DONE не будет обработан и callback не сработает.
   ============================================================ */
extern "C" void lora_poll(void) {
    if (!s_radio) return;
    std::error_code ec;
    s_radio->service_interrupts(ec);
}