#ifndef CAN_CONTROLLER_DEVICE_MCP2515_CONFIG_H_
#define CAN_CONTROLLER_DEVICE_MCP2515_CONFIG_H_

#include <stdint.h>

typedef struct mcp2515_config {
    uint32_t osc_clock_hz;
} mcp2515_config_t;

#endif // CAN_CONTROLLER_DEVICE_MCP2515_CONFIG_H_
