#ifndef CAN_CONTROLLER_CONFIGURATION_H_
#define CAN_CONTROLLER_CONFIGURATION_H_

// #define MCP2518FD
// #define MCP2517FD
// #define MCP2515

#if !defined(MCP2518FD) && !defined(MCP2517FD) && !defined(MCP2515)
#error "One of device types MCP2518FD, MCP2517FD, or MCP2515 must be defined"
#endif

#include "can-controller/bit-time.h"

/* Set default platform if not defined explicitly */
#if !defined(CONTROLLER_PLATFORM_RASPBERRY_PI) &&                              \
    !defined(CONTROLLER_PLATFORM_PSOC)
#define CONTROLLER_PLATFORM_PSOC
#endif

#if defined(MCP2518FD) || defined(MCP2517)
#include "can-controller/device/mcp25xxfd_config.h"
#define DEVICE_CAN_CONFIG_T mcp25xxfd_can_config_t
#define DEVICE_SYSTEM_CONFIG_T mcp25xxfd_system_config_t
#endif
#if defined(MCP2515)
#include "can-controller/device/mcp2515_config.h"
#define DEVICE_CAN_CONFIG_T mcp2515_can_config_t
#define DEVICE_SYSTEM_CONFIG_T mcp2515_system_config_t
#endif

typedef struct system_config {
  uint32_t osc_clock_hz;
  uint32_t spi_speed_hz;

  DEVICE_SYSTEM_CONFIG_T device;
} can_system_config_t;

typedef struct can_config {
  can_system_config_t system;

  can_bt_config_t bit_time;

  DEVICE_CAN_CONFIG_T device;
} can_config_t;

#endif // CAN_CONTROLLER_CONFIGURATION_H_