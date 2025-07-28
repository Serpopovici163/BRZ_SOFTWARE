#include "adc108s022.h"
#include "stm32h7xx_hal.h"

// Static function for software CS control
static void _cs_assert(ADC108S022 *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void _cs_deassert(ADC108S022 *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t _build_control_byte(uint8_t channel) {
    return (channel & 0x07) << 3;  // 0b000DDD00 (DDD = channel)
}

void ADC108S022_Init(ADC108S022 *dev) {
    // Ensure CS pin is initialized externally (CubeMX/your code)
    _cs_deassert(dev);  // Start with CS high
}

uint16_t ADC108S022_ReadChannel(ADC108S022 *dev, uint8_t channel) {
    uint8_t tx_data[2] = { _build_control_byte(channel), 0x00 };
    uint8_t rx_data[2] = { 0 };

    _cs_assert(dev);  // CS low
    HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, 2, HAL_MAX_DELAY);
    _cs_deassert(dev);  // CS high

    return ((rx_data[0] & 0x03) << 8) | rx_data[1];  // 10-bit value
}
