#ifndef CAN_CONTROLLER_API_H_
#define CAN_CONTROLLER_API_H_

#include <stdint.h>

#include "can-controller/can_message.h"
#include "can-controller/configuration.h"

/*
 * Set of functions to use CAN controller.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the can controller.
 *
 * @returns 0 on successful initialization. 1 on error.
 */
extern uint8_t can_init();

extern can_message_t *can_create_message();
extern void can_free_message(can_message_t *message);

extern void can_send_message(can_message_t *message);

/**
 * Application specific callback function to handle a received message.
 *
 * The application must implement this method. The application also is
 * responsible to free the message by calling can_free_message().
 */
extern void can_consume_rx_message(can_message_t *message);

#ifdef __cplusplus
}
#endif
#endif  // CAN_CONTROLLER_API_H_
