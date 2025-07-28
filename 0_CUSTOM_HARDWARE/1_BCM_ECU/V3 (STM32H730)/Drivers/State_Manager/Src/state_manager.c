#include "state_manager.h"
#include <string.h>

// Global State Variables
volatile ModuleState module_state;

void STATE_MANAGER_Init(void) {
    // Initialize module state
    module_state.MODULE_STATE = MODULE_STANDBY;
    module_state.CAN_STATE = CAN_STANDBY;
    module_state.FAULT_STATE = FAULT_OK;
}
