#ifndef CAN_CONTROLLER_CONFIGURATION_H_
#define CAN_CONTROLLER_CONFIGURATION_H_

#define CONTROLLER_DEVICE mcp2515
#define CONTROLLER_PLATFORM raspberry_pi

#if CONTROLLER_DEVICE == mcp2525
#define can_init mcp2515_init
#define can_send_message mcp2515_send_message
#endif

#endif  // CAN_CONTROLLER_CONFIGURATION_H_