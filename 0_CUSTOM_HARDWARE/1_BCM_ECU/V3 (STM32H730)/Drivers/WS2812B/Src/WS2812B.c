#include "ws2812b.h"

// Convert RGB to PWM buffer (WS2812B uses GRB order)
static void _rgb_to_pwm_buffer(uint16_t *buf, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = ((g << 16) | (r << 8) | b);
    for (int i = 0; i < 24; i++) {
        buf[i] = (grb & (1 << (23 - i))) ? WS2812B_1_BIT : WS2812B_0_BIT;
    }
}

// Initialize hardware (assumes struct is pre-configured)
void WS2812B_Init(WS2812B_Strip *strip) {
    // Configure timer for 800 kHz PWM
    strip->htim->Instance->ARR = (SystemCoreClock / WS2812B_PWM_FREQ) - 1;
    strip->htim->Instance->PSC = 0;
    HAL_TIM_PWM_Start_DMA(strip->htim, strip->channel,
                         (uint32_t *)strip->pwm_buffer,
                         24 * strip->num_leds);  // Use strip->num_leds
}

// Set LED color (call for each LED)
void WS2812B_SetRGB(WS2812B_Strip *strip, uint16_t led_idx, uint8_t r, uint8_t g, uint8_t b) {
    if (led_idx < strip->num_leds) {  // Bounds check
        _rgb_to_pwm_buffer(&strip->pwm_buffer[24 * led_idx], r, g, b);
    }
}

// Update strip (transmit PWM buffer via DMA)
void WS2812B_Update(WS2812B_Strip *strip) {
    HAL_TIM_PWM_Start_DMA(strip->htim, strip->channel,
                         (uint32_t *)strip->pwm_buffer,
                         24 * strip->num_leds);  // Use strip->num_leds
}
