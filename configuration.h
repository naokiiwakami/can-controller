#ifndef CAN_CONTROLLER_CONFIGURATION_H_
#define CAN_CONTROLLER_CONFIGURATION_H_

/* Uncomment one of the supported CAN controller device */
#define CONTROLLER_DEVICE_MCP2515

/* Set default platform if not defined explicitly */
#if !defined(CONTROLLER_PLATFORM_RASPBERRY_PI) && \
    !defined(CONTROLLER_PLATFORM_PSOC)
#define CONTROLLER_PLATFORM_PSOC
#endif

#endif  // CAN_CONTROLLER_CONFIGURATION_H_