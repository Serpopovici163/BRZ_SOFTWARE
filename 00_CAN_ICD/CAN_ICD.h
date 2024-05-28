// This file is generated from an Excel file containing CAN IDs and variable names

#define CAN_ID_LIGHTING 0x000 // Car lighting (running lights, turn signals + hazards, hand brake)
#define CAN_ID_SPEED 0x001 // Individual wheel speed sensor data
#define CAN_ID_ENGINE_1 0x002 // Oil temp, rpm, fault code
#define CAN_ID_ENGINE_2 0x003 // Fuel consumption, fuel level
#define CAN_ID_SRS 0x004 // TBD, general fault codes
#define CAN_ID_ABS_TRACTION 0x005 // TBD, general fault codes and activation
#define CAN_ID_STEERING_WHEEL 0x51 // Key presses + telemetry
#define CAN_ID_HW_COMMANDS 0x52 // target ID + command ID (command IDs are HW-specific)
#define CAN_ID_HEARTBEATS 0x101 // hardware ID + status (board-specific status codes)
#define CAN_ID_TELEM_REQUEST 0x102 // used to request data from a specific computer (target-board-id, followed by list of var-ids)
#define CAN_ID_TELEM_RESPONSE 0x103 // used to respond to data request (target-board-id, followed by list of var-values)
