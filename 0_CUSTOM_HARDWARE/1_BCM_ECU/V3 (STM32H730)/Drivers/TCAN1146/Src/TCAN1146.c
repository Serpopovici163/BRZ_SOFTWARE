#include "tcan1146.h"
#include "string.h"

// Register Addresses (from datasheet)
#define TCAN_REG_DEVICE_CTRL  0x00
#define TCAN_REG_WAKE_CONFIG  0x03
#define TCAN_REG_WAKE_ID      0x04  // Start of 4-byte ID filter
#define TCAN_REG_WAKE_PAYLOAD 0x08  // Start of 8-byte payload filter

// Helper: SPI Read/Write
static void _spi_write(TCAN1146 *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(dev->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void _spi_read(TCAN1146 *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    reg |= 0x80;  // Set read bit
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(dev->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// Initialize TCAN1146 + Wake-on-CAN
void TCAN1146_Init(TCAN1146 *dev, uint32_t wake_id, uint8_t *wake_payload, uint8_t payload_len) {
    // Set device to Standby to configure registers
    TCAN1146_SetMode(dev, TCAN_MODE_STANDBY);

    // Configure Wake-on-CAN:
    // 1. Write 29-bit ID (extended) to registers 0x04-0x07
    uint8_t id_bytes[4] = {
        (wake_id >> 21) & 0xFF,  // Bits 28-21
        (wake_id >> 13) & 0xFF,  // Bits 20-13
        (wake_id >> 5)  & 0xFF,  // Bits 12-5
        (wake_id << 3)  & 0xF8   // Bits 4-0 + 0b111 (extended ID)
    };
    _spi_write(dev, TCAN_REG_WAKE_ID, id_bytes, 4);

    // 2. Write payload filter (up to 8 bytes)
    if (wake_payload && payload_len > 0) {
        payload_len = (payload_len > 8) ? 8 : payload_len;
        _spi_write(dev, TCAN_REG_WAKE_PAYLOAD, wake_payload, payload_len);
    }

    // 3. Enable Wake-on-CAN (bit 4 in WAKE_CONFIG)
    uint8_t wake_config = 0x10;  // Wake on matching ID + payload
    _spi_write(dev, TCAN_REG_WAKE_CONFIG, &wake_config, 1);

    // Set to Normal mode (ready for CAN)
    TCAN1146_SetMode(dev, TCAN_MODE_NORMAL);
}

// Change Mode (Normal, Standby, Listen-Only, Sleep)
void TCAN1146_SetMode(TCAN1146 *dev, TCAN_Mode mode) {
    uint8_t ctrl_reg;
    _spi_read(dev, TCAN_REG_DEVICE_CTRL, &ctrl_reg, 1);

    // Clear mode bits (bits 1-0)
    ctrl_reg &= ~0x03;

    switch (mode) {
        case TCAN_MODE_NORMAL:
            ctrl_reg |= 0x00;  // Normal mode
            break;
        case TCAN_MODE_STANDBY:
            ctrl_reg |= 0x01;  // Standby
            break;
        case TCAN_MODE_LISTEN_ONLY:
            ctrl_reg |= 0x02;  // Listen-Only
            break;
        case TCAN_MODE_SLEEP:
            ctrl_reg |= 0x03;  // Sleep
            break;
    }

    _spi_write(dev, TCAN_REG_DEVICE_CTRL, &ctrl_reg, 1);
}
