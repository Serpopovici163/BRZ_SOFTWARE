#pragma once
#include "state_manager.h"

#include <stdint.h>

// Lighting State Struct
typedef struct {
    volatile uint8_t ANIMATION_ID;
    volatile uint8_t ANIMATION_STEP;    // Range: 0-9
    volatile uint8_t BRAKE_LIGHT_STATE; // Range: 0-2
    volatile uint8_t INDICATOR_STATE;   // Range: 0-3
    volatile uint8_t RUN_LIGHT_STATE;   // Range: 0-1
    volatile uint8_t BLACKOUT_STATE;    // Range: 0-1
} LightingState;

// Global State Access (extern)
extern volatile LightingState lighting_state;

// Public API
void LIGHT_UpdateAll(void);  // Main light state computer
