#include "lora.h"
#include "board_pins.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "lora";
static spi_device_handle_t s_lora_spi;

/* --- SX126x/LLCC68 команды (opcodes) --- */
#define CMD_SET_STANDBY          0x80
#define CMD_SET_SLEEP            0x84
#define CMD_SET_TX               0x83
#define CMD_SET_RX               0x82
#define CMD_SET_RF_FREQUENCY     0x86
#define CMD_SET_PACKET_TYPE      0x8A
#define CMD_SET_MODULATION_PARAMS 0x8B
#define CMD_SET_PACKET_PARAMS    0x8C
#define CMD_SET_DIO_IRQ_PARAMS   0x08
#define CMD_GET_IRQ_STATUS       0x12
#define CMD_CLEAR_IRQ_STATUS     0x02
#define CMD_WRITE_BUFFER         0x0E
#define CMD_READ_BUFFER          0x1E
#define CMD_SET_TX_PARAMS        0x8E
#define CMD_SET_BUFFER_BASE_ADDR 0x8F
#define CMD_SET_DIO2_AS_RF_SWITCH 0x9D

/* --- Статусы --- */
#define IRQ_TX_DONE              0x0001
#define IRQ_RX_DONE              0x0002
#define IRQ_CRC_ERR              0x0040
#define IRQ_TIMEOUT              0x0200

/* --- Низкоуровневые операции --- */

static void lora_wait_busy(void)
{
    uint32_t timeout = 10000;
   /* while (gpio_get_level(LORA_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }*/
    if (timeout == 0) {
        ESP_LOGW(TAG, "BUSY timeout");
    }
}

static void lora_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    lora_wait_busy();
    uint8_t *tx = malloc(1 + len);
    tx[0] = cmd;
    if (len > 0 && data) memcpy(&tx[1], data, len);

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(s_lora_spi, &t);
    free(tx);
}

static void lora_read_cmd(uint8_t cmd, uint8_t *data, size_t len)
{
    lora_wait_busy();
    uint8_t *tx = calloc(1, 1 + len);
    uint8_t *rx = calloc(1, 1 + len);
    tx[0] = cmd;

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_lora_spi, &t);
    if (data) memcpy(data, &rx[1], len);
    free(tx);
    free(rx);
}

/* --- Сброс --- */

void lora_reset(void)
{
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    lora_wait_busy();
}

/* --- Инициализация --- */

esp_err_t lora_init(void)
{
    /* 1. GPIO BUSY */
   // gpio_reset_pin(LORA_BUSY);
  //  gpio_set_direction(LORA_BUSY, GPIO_MODE_INPUT);

    /* 2. GPIO RST */
    gpio_reset_pin(LORA_RST);
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 1);

    /* 3. SPI шина */
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO,
        .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* 4. SPI устройство (LLCC68 макс. 10 МГц) */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,  /* 4 МГц */
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &devcfg, &s_lora_spi));

    /* 5. Аппаратный сброс */
    lora_reset();

    /* 6. Проверка связи: читаем статус (команда GetStatus = 0xC0) */
   // lora_wait_busy();
    uint8_t status = 0;
    lora_read_cmd(0xC0, &status, 1);
    ESP_LOGI(TAG, "LLCC68 status: 0x%02X", status);

    /* 7. Установка в Standby (RC) */
    uint8_t standby_cfg = 0x00;
    lora_write_cmd(CMD_SET_STANDBY, &standby_cfg, 1);
   // lora_wait_busy();

    ESP_LOGI(TAG, "LLCC68 initialized");
    return ESP_OK;
}

/* --- Частота --- */

void lora_set_frequency(uint32_t frequency)
{
    /* Frf = frequency * 2^25 / 32 MHz */
    uint32_t frf = (uint32_t)(((uint64_t)frequency << 25) / 32000000);
    uint8_t buf[4] = {
        (uint8_t)(frf >> 24),
        (uint8_t)(frf >> 16),
        (uint8_t)(frf >> 8),
        (uint8_t)(frf),
    };
    lora_write_cmd(CMD_SET_RF_FREQUENCY, buf, 4);
}

/* --- Мощность --- */

void lora_set_tx_power(int8_t level)
{
    /* LLCC68: max +22 дБм (Ra-01SC) */
    if (level > 22) level = 22;
    if (level < -9) level = -9;

    uint8_t buf[2] = { (uint8_t)level, 0x04 };  /* 0x04 = 200us ramp */
    lora_write_cmd(CMD_SET_TX_PARAMS, buf, 2);
}

/* --- Spreading Factor (LLCC68: SF5..SF9 для BW125) --- */

void lora_set_spreading_factor(uint8_t sf)
{
    if (sf < 5) sf = 5;
    if (sf > 9) sf = 9;

    /* SetPacketType = LoRa */
    uint8_t pkt_type = 0x01;
    lora_write_cmd(CMD_SET_PACKET_TYPE, &pkt_type, 1);

    /* SetModulationParams: SF, BW=125kHz(0x04), CR=4/5(0x01), LDRO=0 */
    uint8_t mod_params[4] = { sf, 0x04, 0x01, 0x00 };
    lora_write_cmd(CMD_SET_MODULATION_PARAMS, mod_params, 4);
}

/* --- Полоса пропускания --- */

void lora_set_bandwidth(uint32_t sbw)
{
    /* Для простоты используем 125 кГц */
    (void)sbw;
}

/* --- CRC --- */

void lora_enable_crc(void)
{
    /* SetPacketParams: preamble=8, explicit header, payload=255, CRC on, normal IQ */
    uint8_t pkt_params[6] = { 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00 };
    lora_write_cmd(CMD_SET_PACKET_PARAMS, pkt_params, 6);
}

/* --- Отправка --- */

esp_err_t lora_send_packet(uint8_t *buf, uint8_t size)
{
    //lora_wait_busy();

    /* SetBufferBaseAddress */
    uint8_t base[2] = { 0x00, 0x00 };
    lora_write_cmd(CMD_SET_BUFFER_BASE_ADDR, base, 2);

    /* WriteBuffer */
   // lora_wait_busy();
    uint8_t *tx = malloc(1 + size);
    tx[0] = CMD_WRITE_BUFFER;
    memcpy(&tx[1], buf, size);
    spi_transaction_t t = {
        .length = (1 + size) * 8,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(s_lora_spi, &t);
    free(tx);

    /* SetPacketParams (payload length) */
    uint8_t pkt_params[6] = { 0x00, 0x08, 0x00, size, 0x01, 0x00 };
    lora_write_cmd(CMD_SET_PACKET_PARAMS, pkt_params, 6);

    /* Clear IRQ */
    uint8_t clear[2] = { 0xFF, 0xFF };
    lora_write_cmd(CMD_CLEAR_IRQ_STATUS, clear, 2);

    /* SetTx: timeout = 0 (no timeout) */
    uint8_t tx_cfg[3] = { 0x00, 0x00, 0x00 };
    lora_write_cmd(CMD_SET_TX, tx_cfg, 3);

    /* Ждём TxDone */
    uint32_t timeout = 5000;
    while (timeout--) {
        uint8_t irq[2] = { 0 };
        lora_read_cmd(CMD_GET_IRQ_STATUS, irq, 2);
        uint16_t irq_status = (irq[0] << 8) | irq[1];
        if (irq_status & IRQ_TX_DONE) {
            lora_write_cmd(CMD_CLEAR_IRQ_STATUS, (uint8_t[]){ 0x01, 0x00 }, 2);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Возврат в Standby */
    uint8_t standby_cfg = 0x00;
    lora_write_cmd(CMD_SET_STANDBY, &standby_cfg, 1);

    return ESP_OK;
}

/* --- Приём --- */

void lora_receive(void)
{
   // lora_wait_busy();

    /* Clear IRQ */
    uint8_t clear[2] = { 0xFF, 0xFF };
    lora_write_cmd(CMD_CLEAR_IRQ_STATUS, clear, 2);

    /* SetRx: timeout = 0 (continuous) */
    uint8_t rx_cfg[3] = { 0xFF, 0xFF, 0xFF };
    lora_write_cmd(CMD_SET_RX, rx_cfg, 3);
}

uint8_t lora_receive_packet(uint8_t *buf, uint8_t size)
{
    uint8_t irq[2] = { 0 };
    lora_read_cmd(CMD_GET_IRQ_STATUS, irq, 2);
    uint16_t irq_status = (irq[0] << 8) | irq[1];

    if (!(irq_status & IRQ_RX_DONE)) {
        return 0;
    }

    /* Clear IRQ */
    lora_write_cmd(CMD_CLEAR_IRQ_STATUS, (uint8_t[]){ 0x02, 0x00 }, 2);

    /* CRC error */
    if (irq_status & IRQ_CRC_ERR) {
        lora_write_cmd(CMD_CLEAR_IRQ_STATUS, (uint8_t[]){ 0x40, 0x00 }, 2);
        return 0;
    }

    /* GetRxBufferStatus: payload_len, rx_start_ptr */
   // lora_wait_busy();
    uint8_t status_buf[2] = { 0 };
    lora_read_cmd(0x13, status_buf, 2);
    uint8_t payload_len = status_buf[0];
    uint8_t rx_ptr = status_buf[1];

    if (payload_len > size) payload_len = size;

    /* SetBufferBaseAddress */
    uint8_t base[2] = { 0x00, 0x00 };
    lora_write_cmd(CMD_SET_BUFFER_BASE_ADDR, base, 2);

    /* ReadBuffer */
   // lora_wait_busy();
    uint8_t *tx = calloc(1, 2 + payload_len);
    uint8_t *rx = calloc(1, 2 + payload_len);
    tx[0] = CMD_READ_BUFFER;
    tx[1] = rx_ptr;

    spi_transaction_t t = {
        .length = (2 + payload_len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_lora_spi, &t);
    memcpy(buf, &rx[2], payload_len);
    free(tx);
    free(rx);

    /* Возврат в приём */
    lora_receive();

    return payload_len;
}