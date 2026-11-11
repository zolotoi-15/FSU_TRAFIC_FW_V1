#include "lora.h"
#include "lora_nvs.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "sx126x.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <memory>
#include <span>
#include <chrono>
#include <system_error>
#include <cstring>

static const char *TAG = "lora";
static spi_device_handle_t s_spi_handle = nullptr;
static std::unique_ptr<espp::Sx126x> s_radio;
static lora_raw_rx_cb_t s_raw_cb = nullptr;
static SemaphoreHandle_t s_spi_mutex = NULL;

/* === FIX TX-BUSY === */
static volatile bool s_tx_active = false;
/* === END FIX TX-BUSY === */

/* === FIX LORA-LINK-STATS === */
static volatile float    s_last_rssi     = 0.0f;
static volatile float    s_last_snr      = 0.0f;
static volatile float    s_last_sig_rssi = 0.0f;
static volatile uint32_t s_last_rx_ms    = 0;
static volatile bool     s_have_rx_stats = false;
/* === END FIX LORA-LINK-STATS === */

/* === FIX LORA-PARAMS-SYNC ===
   Параметры модуляции (fallback, если NVS пуст).
   SF9 / BW250 / CR4/5 / 869.85 МГц / 13 dBm.
   ============================================================ */
#define LORA_FREQ_HZ        869850000   /* 869.85 МГц — g4, 100% скважности, 25 мВт */
#define LORA_TX_POWER_DBM   13          /* 13 dBm = 20 мВт (в пределах 25 мВт g4) */
#define LORA_SF             8           /* SF7..SF12 */
#define LORA_BW_KHZ         250         /* 125/250/500 */
#define LORA_CR             5           /* 5=4/5, 6=4/6, 7=4/7, 8=4/8 */

#define LORA_SYNC_WORD      0x12
#define LORA_PREAMBLE_LEN   12
#define LORA_USE_DIO2_SWITCH true
#define LORA_USE_DCDC       true
#define LORA_CRC_ON         true
#define LORA_RX_BOOST_GAIN  true

#if   LORA_SF == 7
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF7
#define LORA_SF_STR    "SF7"
#elif LORA_SF == 8
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF8
#define LORA_SF_STR    "SF8"
#elif LORA_SF == 9
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF9
#define LORA_SF_STR    "SF9"
#elif LORA_SF == 10
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF10
#define LORA_SF_STR    "SF10"
#elif LORA_SF == 11
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF11
#define LORA_SF_STR    "SF11"
#elif LORA_SF == 12
#define LORA_SF_ENUM   espp::Sx126x::SpreadingFactor::SF12
#define LORA_SF_STR    "SF12"
#else
#error "LORA_SF must be 7..12"
#endif

#if   LORA_BW_KHZ == 125
#define LORA_BW_ENUM   espp::Sx126x::Bandwidth::BW_125_KHZ
#define LORA_BW_STR    "BW125"
#elif LORA_BW_KHZ == 250
#define LORA_BW_ENUM   espp::Sx126x::Bandwidth::BW_250_KHZ
#define LORA_BW_STR    "BW250"
#elif LORA_BW_KHZ == 500
#define LORA_BW_ENUM   espp::Sx126x::Bandwidth::BW_500_KHZ
#define LORA_BW_STR    "BW500"
#else
#error "LORA_BW_KHZ must be 125, 250 or 500"
#endif

#if   LORA_CR == 5
#define LORA_CR_ENUM   espp::Sx126x::CodingRate::CR_4_5
#define LORA_CR_STR    "CR4/5"
#elif LORA_CR == 6
#define LORA_CR_ENUM   espp::Sx126x::CodingRate::CR_4_6
#define LORA_CR_STR    "CR4/6"
#elif LORA_CR == 7
#define LORA_CR_ENUM   espp::Sx126x::CodingRate::CR_4_7
#define LORA_CR_STR    "CR4/7"
#elif LORA_CR == 8
#define LORA_CR_ENUM   espp::Sx126x::CodingRate::CR_4_8
#define LORA_CR_STR    "CR4/8"
#else
#error "LORA_CR must be 5..8"
#endif
/* === END FIX LORA-PARAMS-SYNC === */

#define LORA_SPI_TX_TIMEOUT_MS  3000
#define LORA_SPI_POLL_TIMEOUT_MS 200

/* === FIX TX-RX-GAP === */
#define LORA_TX_RX_GAP_MS       5
/* === END FIX TX-RX-GAP === */

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

static void on_rx_packet(const espp::Sx126x::RxPacket &pkt) {
    /* === FIX TX-BUSY === */
    s_tx_active = false;
    /* === END FIX TX-BUSY === */

    /* === FIX LORA-LINK-STATS === */
    s_last_rssi     = pkt.status.rssi;
    s_last_snr      = pkt.status.snr;
    s_last_sig_rssi = pkt.status.signal_rssi;
    s_last_rx_ms    = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_have_rx_stats = true;
    /* === END FIX LORA-LINK-STATS === */

    ESP_LOGI(TAG, "on_rx_packet: %d bytes, rssi=%.1f, snr=%.1f, sig_rssi=%.1f",
             (int)pkt.data.size(), pkt.status.rssi, pkt.status.snr,
             pkt.status.signal_rssi);
    if (s_raw_cb && !pkt.data.empty()) {
        s_raw_cb(pkt.data.data(), pkt.data.size());
    }
}

static void log_lora_config(uint32_t freq_hz, int8_t pwr, uint8_t sf,
                            uint16_t bw_khz, uint8_t cr) {
    ESP_LOGI(TAG, "LoRa config:");
    ESP_LOGI(TAG, "  freq      = %lu Hz", (unsigned long)freq_hz);
    ESP_LOGI(TAG, "  tx_power  = %d dBm", pwr);
    ESP_LOGI(TAG, "  SF        = SF%u", sf);
    ESP_LOGI(TAG, "  BW        = BW%u (%u kHz)", bw_khz, bw_khz);
    ESP_LOGI(TAG, "  CR        = 4/%u", cr);
    ESP_LOGI(TAG, "  sync_word = 0x%02X", LORA_SYNC_WORD);
    ESP_LOGI(TAG, "  preamble  = %d", LORA_PREAMBLE_LEN);
    ESP_LOGI(TAG, "  crc       = %s", LORA_CRC_ON ? "ON" : "OFF");
    ESP_LOGI(TAG, "  rx_boost  = %s", LORA_RX_BOOST_GAIN ? "yes" : "no");
    ESP_LOGI(TAG, "  dio2_sw   = %s", LORA_USE_DIO2_SWITCH ? "yes" : "no");
    ESP_LOGI(TAG, "  dcdc      = %s", LORA_USE_DCDC ? "yes" : "no");
    ESP_LOGI(TAG, "  pins: CS=%d RST=%d BUSY=%d DIO0=%d",
             LORA_CS, LORA_RST, LORA_BUSY, LORA_DIO0);
}

extern "C" esp_err_t lora_init(void) {
    if (s_radio) return ESP_OK;

    if (!s_spi_mutex) {
        s_spi_mutex = xSemaphoreCreateRecursiveMutex();
        if (!s_spi_mutex) {
            ESP_LOGE(TAG, "s_spi_mutex alloc failed");
            return ESP_ERR_NO_MEM;
        }
    }

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
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);

    /* === FIX LORA-NVS-CONFIG ===
       Читаем сохранённые параметры из NVS (устанавливаются через
       BLE lora.set). Если нет — fallback на define-значения. */
    uint32_t freq_hz = LORA_FREQ_HZ;
    uint8_t  sf      = LORA_SF;
    uint16_t bw_khz  = LORA_BW_KHZ;
    uint8_t  cr      = LORA_CR;
    int8_t   pwr     = LORA_TX_POWER_DBM;

    lora_cfg_nvs_t saved;
    if (lora_nvs_load(&saved)) {
        freq_hz = saved.freq_hz;
        sf      = saved.sf;
        bw_khz  = saved.bw_khz;
        cr      = saved.cr;
        pwr     = saved.tx_power_dbm;
        ESP_LOGI(TAG, "LoRa params from NVS: freq=%lu sf=%u bw=%u cr=%u pwr=%d",
                 (unsigned long)freq_hz, sf, bw_khz, cr, pwr);
    } else {
        ESP_LOGI(TAG, "LoRa params from defines (NVS empty)");
    }

    espp::Sx126x::SpreadingFactor sf_enum;
    switch (sf) {
    case 7: sf_enum = espp::Sx126x::SpreadingFactor::SF7; break;
    case 8: sf_enum = espp::Sx126x::SpreadingFactor::SF8; break;
    case 9: sf_enum = espp::Sx126x::SpreadingFactor::SF9; break;
    case 10: sf_enum = espp::Sx126x::SpreadingFactor::SF10; break;
    case 11: sf_enum = espp::Sx126x::SpreadingFactor::SF11; break;
    case 12: sf_enum = espp::Sx126x::SpreadingFactor::SF12; break;
    default: sf_enum = espp::Sx126x::SpreadingFactor::SF9; break;
    }
    espp::Sx126x::Bandwidth bw_enum;
    switch (bw_khz) {
    case 125: bw_enum = espp::Sx126x::Bandwidth::BW_125_KHZ; break;
    case 250: bw_enum = espp::Sx126x::Bandwidth::BW_250_KHZ; break;
    case 500: bw_enum = espp::Sx126x::Bandwidth::BW_500_KHZ; break;
    default:  bw_enum = espp::Sx126x::Bandwidth::BW_250_KHZ; break;
    }
    espp::Sx126x::CodingRate cr_enum;
    switch (cr) {
    case 5: cr_enum = espp::Sx126x::CodingRate::CR_4_5; break;
    case 6: cr_enum = espp::Sx126x::CodingRate::CR_4_6; break;
    case 7: cr_enum = espp::Sx126x::CodingRate::CR_4_7; break;
    case 8: cr_enum = espp::Sx126x::CodingRate::CR_4_8; break;
    default: cr_enum = espp::Sx126x::CodingRate::CR_4_5; break;
    }
    /* === END FIX LORA-NVS-CONFIG === */

    espp::Sx126x::Config cfg;
    cfg.variant = espp::Sx126x::Variant::LLCC68;
    cfg.write = spi_write;
    cfg.write_then_read = spi_write_then_read;
    cfg.is_busy = is_busy;
    cfg.reset = reset;
    cfg.tcxo_voltage = 0.0f;
    cfg.use_dio2_as_rf_switch = LORA_USE_DIO2_SWITCH;
    cfg.use_dcdc_regulator = LORA_USE_DCDC;
    cfg.auto_init = false;

    cfg.radio_config.frequency_hz       = freq_hz;
    cfg.radio_config.tx_power_dbm       = pwr;
    cfg.radio_config.spreading_factor   = sf_enum;
    cfg.radio_config.bandwidth          = bw_enum;
    cfg.radio_config.coding_rate        = cr_enum;
    cfg.radio_config.sync_word          = LORA_SYNC_WORD;
    cfg.radio_config.preamble_length    = LORA_PREAMBLE_LEN;
    cfg.radio_config.crc_enabled        = LORA_CRC_ON;
    cfg.radio_config.rx_boosted_gain    = LORA_RX_BOOST_GAIN;

    cfg.on_receive = on_rx_packet;
    cfg.log_level = espp::Logger::Verbosity::WARN;

    s_radio = std::make_unique<espp::Sx126x>(cfg);

    if (s_spi_mutex) xSemaphoreTakeRecursive(s_spi_mutex, portMAX_DELAY);

    std::error_code ec;
    if (!s_radio->initialize(ec)) {
        if (s_spi_mutex) xSemaphoreGiveRecursive(s_spi_mutex);
        ESP_LOGE(TAG, "Sx126x init failed: %s", ec.message().c_str());
        s_radio.reset();
        return ESP_FAIL;
    }

    log_lora_config(freq_hz, pwr, sf, bw_khz, cr);

    auto toa_64 = s_radio->time_on_air(64);
    auto toa_25 = s_radio->time_on_air(25);
    ESP_LOGI(TAG, "ToA: 25B=%lld ms, 64B=%lld ms",
             (long long)toa_25.count(), (long long)toa_64.count());

    ESP_LOGI(TAG, "LoRa ready (LLCC68, %lu MHz, SF%u/BW%u/CR4/%u, %d dBm)",
             (unsigned long)(freq_hz / 1000000),
             sf, bw_khz, cr, pwr);

    s_radio->start_receive(ec);
    if (ec) ESP_LOGE(TAG, "start_receive failed: %s", ec.message().c_str());
    else    ESP_LOGI(TAG, "LoRa RX mode enabled");

    if (s_spi_mutex) xSemaphoreGiveRecursive(s_spi_mutex);
    return ESP_OK;
}

extern "C" void lora_receive(void) {
    if (!s_radio) return;
    if (s_spi_mutex &&
        xSemaphoreTakeRecursive(s_spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "lora_receive: spi mutex timeout");
        return;
    }
    std::error_code ec;
    s_radio->start_receive(ec);
    if (ec) ESP_LOGE(TAG, "lora_receive: %s", ec.message().c_str());
    else    ESP_LOGI(TAG, "LoRa RX mode (re)enabled");
    if (s_spi_mutex) xSemaphoreGiveRecursive(s_spi_mutex);
}

extern "C" esp_err_t lora_send_packet(uint8_t *buf, uint8_t size) {
    if (!s_radio || !buf || size == 0) return ESP_ERR_INVALID_ARG;
    if (s_spi_mutex &&
        xSemaphoreTakeRecursive(s_spi_mutex,
                                pdMS_TO_TICKS(LORA_SPI_TX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "lora_send_packet: spi mutex timeout (%d ms)",
                 LORA_SPI_TX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    s_tx_active = true;

    std::error_code ec;
    bool ok = s_radio->transmit(std::span<const uint8_t>(buf, size),
                                std::chrono::milliseconds(2000), ec);
    ESP_LOGI(TAG, "tx ok=%d size=%u ec=%s", (int)ok, (unsigned)size,
             ec.message().c_str());

    if (ok) {
        vTaskDelay(pdMS_TO_TICKS(LORA_TX_RX_GAP_MS));
    }

    std::error_code ec2;
    s_radio->start_receive(ec2);
    if (ec2) ESP_LOGW(TAG, "start_receive after tx: %s", ec2.message().c_str());

    s_tx_active = false;

    if (s_spi_mutex) xSemaphoreGiveRecursive(s_spi_mutex);

    if (!ok) { ESP_LOGW(TAG, "tx failed: %s", ec.message().c_str()); return ESP_FAIL; }
    return ESP_OK;
}

extern "C" uint8_t lora_receive_packet(uint8_t *buf, uint8_t size) {
    (void)buf; (void)size;
    return 0;
}

extern "C" void lora_set_rx_callback(lora_raw_rx_cb_t cb) { s_raw_cb = cb; }
extern "C" bool lora_is_ready(void) { return (bool)s_radio; }

extern "C" bool lora_is_tx_active(void) { return s_tx_active; }

/* === FIX LORA-LINK-STATS === */
extern "C" bool lora_get_last_rx_stats(lora_link_stats_t *out)
{
    if (!out) return false;
    if (!s_have_rx_stats) {
        out->rssi = 0.0f;
        out->snr = 0.0f;
        out->sig_rssi = 0.0f;
        out->age_ms = 0;
        return false;
    }
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    out->rssi     = s_last_rssi;
    out->snr      = s_last_snr;
    out->sig_rssi = s_last_sig_rssi;
    out->age_ms   = (now >= s_last_rx_ms) ? (now - s_last_rx_ms) : 0;
    return true;
}
/* === END FIX LORA-LINK-STATS === */

extern "C" void lora_poll(void) {
    if (!s_radio) return;
    if (s_spi_mutex &&
        xSemaphoreTakeRecursive(s_spi_mutex,
                                pdMS_TO_TICKS(LORA_SPI_POLL_TIMEOUT_MS)) != pdTRUE) {
        return;
    }

    std::error_code ec;
    uint16_t irq = s_radio->service_interrupts(ec);
    if (ec) ESP_LOGW(TAG, "service_interrupts: %s", ec.message().c_str());

    if (irq & espp::Sx126x::IRQ_CRC_ERR)    ESP_LOGW(TAG, "IRQ: CRC_ERR");
    if (irq & espp::Sx126x::IRQ_HEADER_ERR) ESP_LOGW(TAG, "IRQ: HEADER_ERR");
    if (irq & espp::Sx126x::IRQ_RX_DONE)    ESP_LOGD(TAG, "IRQ: RX_DONE");

    if (s_spi_mutex) xSemaphoreGiveRecursive(s_spi_mutex);
}