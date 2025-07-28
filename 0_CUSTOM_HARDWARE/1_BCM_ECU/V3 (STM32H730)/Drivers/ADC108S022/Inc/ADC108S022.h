#ifndef ADC108S022_H
#define ADC108S022_H

#include <stdint.h>
#include "stm32h7xx_hal.h"  // For GPIO functions

typedef struct {
    SPI_HandleTypeDef *hspi;  // STM32 SPI handle (e.g., hspi1)
    GPIO_TypeDef *cs_port;    // CS GPIO port (e.g., GPIOA)
    uint16_t cs_pin;          // CS pin (e.g., GPIO_PIN_4)
} ADC108S022;

void ADC108S022_Init(ADC108S022 *dev);
uint16_t ADC108S022_ReadChannel(ADC108S022 *dev, uint8_t channel);

#endif // ADC108S022_H
