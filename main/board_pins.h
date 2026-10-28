#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

/* ======================= System ======================= */
#define BOOT_PIN            GPIO_NUM_0

/* ======================= I2C +========================== */
#define I2C_SDA             GPIO_NUM_8
#define I2C_SCL             GPIO_NUM_9

/* ============= SPI Ethernet (W5500) — SPI3 +============ */
#define ETH_SPI_HOST        SPI3_HOST
#define ETH_MOSI            GPIO_NUM_11
#define ETH_MISO            GPIO_NUM_12
#define ETH_SCLK            GPIO_NUM_13
#define ETH_CS              GPIO_NUM_6
#define ETH_RST             GPIO_NUM_7
#define ETH_INT             GPIO_NUM_18

/* ============= SPI LoRa (Ra-01) — SPI2 ================ */
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_MISO           GPIO_NUM_37
#define LORA_MOSI           GPIO_NUM_38
#define LORA_SCK            GPIO_NUM_39
#define LORA_CS             GPIO_NUM_40
#define LORA_RST            GPIO_NUM_41
#define LORA_RF_EN          GPIO_NUM_5
#define LORA_DIO0           GPIO_NUM_42
#define LORA_BUSY           GPIO_NUM_47


/* ============ 4G Modem (SIM7681E) — UART1 =========== */
#define MODEM_UART_NUM      UART_NUM_1
#define MODEM_TX            GPIO_NUM_21   // ESP32 TX -> модем RX
#define MODEM_RX            GPIO_NUM_10   // ESP32 RX <- модем TX
#define MODEM_RST           GPIO_NUM_36   // ВНИМАНИЕ: GPIO36 input-only!
                                          // Для сброса используйте другой GPIO

/* ===================== RS-485 ========================= */
#define RS485_RO            GPIO_NUM_17
#define RS485_DI            GPIO_NUM_14
#define RS485_UART_NUM      UART_NUM_2
#define RS485_RE_SR_PIN     15   /* RE через shift register output 15 */

/* ====================== CAN =========================== */
#define CAN_TX              GPIO_NUM_1
#define CAN_RX              GPIO_NUM_2
#define CAN_RS_SR_PIN       14   /* RS через shift register output 14 */
#define CAN_BITRATE_DEFAULT 500000

/* ================= Shift Register +===================== */
#define SR_DATA             GPIO_NUM_33
#define SR_CLOCK            GPIO_NUM_34
#define SR_LATCH            GPIO_NUM_35

/* =================== Analog Mux +======================= */
#define MUX_S0              GPIO_NUM_16
#define MUX_S1              GPIO_NUM_3
#define MUX_S2              GPIO_NUM_45
#define MUX_S3              GPIO_NUM_46
#define MUX_SIG             GPIO_NUM_5
#define MUX_SIG2            GPIO_NUM_4
#define MUX_EN2             GPIO_NUM_15

/* ===================== LED +============================ */
#define LED_PIN             GPIO_NUM_48

/* =================== LoRa RF ========================== */
#define LORA_FREQUENCY      868E6

