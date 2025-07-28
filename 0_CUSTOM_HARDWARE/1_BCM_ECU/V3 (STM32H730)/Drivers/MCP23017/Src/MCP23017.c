#include "mcp23017.h"

// Private: Write to MCP23017 register
static void _write_reg(MCP23017 *hdev, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(hdev->hi2c, hdev->i2c_addr << 1, data, 2, HAL_MAX_DELAY);
}

// Private: Read from MCP23017 register
static uint8_t _read_reg(MCP23017 *hdev, uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(hdev->hi2c, hdev->i2c_addr << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hdev->hi2c, hdev->i2c_addr << 1, &value, 1, HAL_MAX_DELAY);
    return value;
}

// Reset MCP23017 via RST pin (active-low)
void MCP23017_Reset(MCP23017 *dev) {
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);  // Assert reset
    HAL_Delay(1);  // Hold reset for 1ms (min 300ns per datasheet)
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);    // Release reset
    HAL_Delay(5);  // Wait for device to stabilize
}

// Initialize MCP23017 (enable interrupts, set defaults)
void MCP23017_Init(MCP23017 *dev) {
    // Hardware reset
    MCP23017_Reset(dev);

    // Set all pins as inputs by default
    _write_reg(dev, MCP_IODIRA, 0xFF);      // Port A as inputs
    _write_reg(dev, MCP_IODIRA + 1, 0xFF);  // Port B as inputs

    // Enable interrupts on all pins (trigger on any change)
    _write_reg(dev, MCP_GPINTENA, 0xFF);    // Port A: interrupt-on-change
    _write_reg(dev, MCP_GPINTENB, 0xFF);    // Port B: interrupt-on-change
    _write_reg(dev, MCP_INTCONA, 0x00);     // Trigger on change (not comparison)
    _write_reg(dev, MCP_INTCONA + 1, 0x00); // Port B same behavior
}

// Set pin direction (input=0, output=1)
void MCP23017_PinMode(MCP23017 *hdev, uint8_t pin, uint8_t output) {
    uint8_t iodir_reg = (pin < 8) ? MCP_IODIRA : MCP_IODIRA + 1;
    uint8_t mask = 1 << (pin % 8);
    uint8_t current = _read_reg(hdev, iodir_reg);
    _write_reg(hdev, iodir_reg, output ? (current & ~mask) : (current | mask));
}

// Write pin state (LOW=0, HIGH=1)
void MCP23017_WritePin(MCP23017 *hdev, uint8_t pin, uint8_t state) {
    uint8_t gpio_reg = (pin < 8) ? MCP_GPIOA : MCP_GPIOA + 1;
    uint8_t mask = 1 << (pin % 8);
    uint8_t current = _read_reg(hdev, gpio_reg);
    _write_reg(hdev, gpio_reg, state ? (current | mask) : (current & ~mask));
}

// Read pin state (returns 0 or 1)
uint8_t MCP23017_ReadPin(MCP23017 *hdev, uint8_t pin) {
    uint8_t gpio_reg = (pin < 8) ? MCP_GPIOA : MCP_GPIOA + 1;
    return (_read_reg(hdev, gpio_reg) >> (pin % 8)) & 1;
}

// Enable interrupt for a specific pin
void MCP23017_EnableInterrupt(MCP23017 *hdev, uint8_t pin) {
    uint8_t gpinten_reg = (pin < 8) ? MCP_GPINTENA : MCP_GPINTENB;
    uint8_t mask = 1 << (pin % 8);
    uint8_t current = _read_reg(hdev, gpinten_reg);
    _write_reg(hdev, gpinten_reg, current | mask);  // Enable interrupt for this pin
}
