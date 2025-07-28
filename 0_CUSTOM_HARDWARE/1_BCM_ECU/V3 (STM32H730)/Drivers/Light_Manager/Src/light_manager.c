#include "light_manager.h"
#include "state_manager.h"

uint8_t MOS_LIGHT_STATE[16]; //Array including states of all 16 MOSFET switches
uint8_t EXT_INPUT_STATE[7]; //Array including states of all external inputs on I/O expander

void LIGHT_UpdateAll() {
    // Example: Set brake light based on module/fault state
    if (module_state.FAULT_STATE != FAULT_OK) {
        lighting_state.BRAKE_LIGHT_STATE = 2;  // Fault mode
    } else {
        lighting_state.BRAKE_LIGHT_STATE = (module_state.MODULE_STATE == MODULE_ACTIVE) ? 1 : 0;
    }

    // Add other light logic here...
}

void _HandleAnimationStep(void) {
    if (lighting_state.ANIMATION_ID != 0) {
        lighting_state.ANIMATION_STEP = (lighting_state.ANIMATION_STEP + 1) % 10;
    }
}
