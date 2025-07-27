/*
 * neopixel.h
 *
 *  Created on: May 27, 2024
 *      Author: sdfuh
 */

//https://github.com/hey-frnk/STM32_HAL_NeoPixel.git

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

#include <stdint.h>

void NEO_setLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void NEO_showLEDs();
void NEO_clearLEDs();

#endif /* INC_NEOPIXEL_H_ */
