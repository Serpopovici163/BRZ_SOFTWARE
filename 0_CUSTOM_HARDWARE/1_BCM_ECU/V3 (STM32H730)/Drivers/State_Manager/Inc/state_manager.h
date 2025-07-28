#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <stdint.h>

// Module State Enums
typedef enum {
    MODULE_ACTIVE,
    MODULE_KEEPON,
    MODULE_STANDBY
} ModuleFuncState;

typedef enum {
    CAN_ACTIVE,
    CAN_TIMEOUT,
    CAN_STANDBY
} CanState;

typedef enum {
    FAULT_OK,
    FAULT_I_FAULT,
    FAULT_STAT_FAULT,
    FAULT_COMM_FAULT,
    FAULT_MULT_FAULT
} FaultState;

// Module State Struct
typedef struct {
    volatile ModuleFuncState MODULE_STATE;
    volatile CanState CAN_STATE;
    volatile FaultState FAULT_STATE;
} ModuleState;

// Global State Access (extern)
extern volatile ModuleState module_state;

// Function Prototypes
void STATE_MANAGER_Init(void);
void STATE_MANAGER_ResetLighting(void);

#endif // STATE_MANAGER_H
