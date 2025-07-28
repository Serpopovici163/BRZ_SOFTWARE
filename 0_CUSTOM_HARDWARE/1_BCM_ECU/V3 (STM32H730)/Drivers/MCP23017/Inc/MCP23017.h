#pragma once
#include "stm32h7xx_hal.h"

// MCP23017 Registers (Bank=0)
#define MCP_IODIRA     0x00  // I/O Direction (1=input, 0=output)
#define MCP_GPINTENA   0x04  // Interrupt-on-Change Enable (Port A)
#define MCP_GPINTENB   0x05  // Interrupt-on-Change Enable (Port B)
#define MCP_INTCONA    0x08  // Interrupt Control (Port A: 0=change, 1=compare to DEFVAL)
#define MCP_DEFVALA    0x06  // Default Value (for interrupt comparison)
#define MCP_GPIOA      0x12  // GPIO Port A
#define MCP_INTFA      0x0E  // Interrupt Flag (Port A: pin that triggered interrupt)
#define MCP_INTCAPA    0x10  // Interrupt Capture (Port A: GPIO state at interrupt)

typedef struct {
    I2C_HandleTypeDef *i2c;      // STM32 I2C handle (e.g., &hi2c1)
    uint8_t i2c_addr;            // I2C address (e.g., 0x20 for A2/A1/A0=GND)
    GPIO_TypeDef *rst_port;      // RST pin GPIO port (e.g., GPIOA)
    uint16_t rst_pin;            // RST pin number (e.g., GPIO_PIN_5)
    GPIO_TypeDef *inta_port;     // INTA pin GPIO port
    uint16_t inta_pin;           // INTA pin number
    GPIO_TypeDef *intb_port;     // INTB pin GPIO port
    uint16_t intb_pin;           // INTB pin number
} MCP23017;

// Public Functions
void MCP23017_Init(MCP23017 *hdev);
void MCP23017_Reset(MCP23017 *dev);
void MCP23017_PinMode(MCP23017 *hdev, uint8_t pin, uint8_t output);  // output=0: input, output=1: output
void MCP23017_WritePin(MCP23017 *hdev, uint8_t pin, uint8_t state);  // state=0: LOW, state=1: HIGH
uint8_t MCP23017_ReadPin(MCP23017 *hdev, uint8_t pin);               // returns 0 or 1
void MCP23017_EnableInterrupt(MCP23017 *hdev, uint8_t pin);          // Enable interrupt-on-change for pin
