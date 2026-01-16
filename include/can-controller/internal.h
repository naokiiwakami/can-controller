#ifndef CAN_CONTROLLER_INTERNAL_H_
#define CAN_CONTROLLER_INTERNAL_H_

#include "can-controller/configuration.h"

extern void initialize_api();

// Each device must have implementations of following methods
extern void handle_rx();
extern uint8_t device_init(const can_config_t *config);

#endif // CAN_CONTROLLER_INTERNAL_H_
