#pragma once
#include "stm32h7xx_hal.h"

// Defaults (adjust as needed)
#define WS2812B_MAX_LEDS    24    // Max LEDs per strip (for buffer sizing)
#define WS2812B_PWM_FREQ    800000 // 800 kHz (1.25 µs period)
#define WS2812B_0_BIT       33     // 0.4 µs high (33% duty)
#define WS2812B_1_BIT       66     // 0.8 µs high (66% duty)

typedef struct {
    TIM_HandleTypeDef *htim;      // PWM timer (e.g., TIM1)
    uint32_t channel;             // Timer channel (e.g., TIM_CHANNEL_1)
    DMA_HandleTypeDef *hdma;      // DMA handle
    uint16_t num_leds;            // Number of LEDs in this strip (variable)
    uint16_t pwm_buffer[24 * WS2812B_MAX_LEDS]; // Dynamically sized buffer
} WS2812B_Strip;

// Public Functions
void WS2812B_Init(WS2812B_Strip *strip);
void WS2812B_SetRGB(WS2812B_Strip *strip, uint16_t led_idx, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_Update(WS2812B_Strip *strip);
