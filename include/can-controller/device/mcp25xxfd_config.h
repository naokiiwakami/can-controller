#ifndef CAN_CONTROLLER_DEVICE_MCP25XXFD_CONFIG_H_
#define CAN_CONTROLLER_DEVICE_MCP25XXFD_CONFIG_H_

#include <stdint.h>

typedef struct mcp25xxfd_system_config {
  uint8_t osc_pll_enabled;
} mcp25xxfd_system_config_t;

typedef struct mcp25xxfd_can_config {
  // CAN
  uint8_t fd_mode;
  uint8_t brs_enabled;
  // bit timing
  uint32_t nominal_baud_rate_prescale;
  uint32_t data_baud_rate_prescale;
} mcp25xxfd_can_config_t;

#endif // CAN_CONTROLLER_DEVICE_MCP25XXFD_CONFIG_H_
