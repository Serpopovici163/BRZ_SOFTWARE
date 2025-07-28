#ifndef TCAN1146_H
#define TCAN1146_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    TCAN_MODE_NORMAL,
    TCAN_MODE_STANDBY,
    TCAN_MODE_LISTEN_ONLY,
    TCAN_MODE_SLEEP
} TCAN_Mode;

typedef struct {
    SPI_HandleTypeDef *hspi;  // SPI handle (e.g., &hspi1)
    GPIO_TypeDef *cs_port;    // CS GPIO port (e.g., GPIOA)
    uint16_t cs_pin;          // CS pin (e.g., GPIO_PIN_4)
} TCAN1146;

// Public Functions
void TCAN1146_Init(TCAN1146 *dev, uint32_t wake_id, uint8_t *wake_payload, uint8_t payload_len);
void TCAN1146_SetMode(TCAN1146 *dev, TCAN_Mode mode);

#endif // TCAN1146_H
