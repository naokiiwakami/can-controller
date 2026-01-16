#ifndef CAN_CONTROLLER_CAN_MESSAGE_H_
#define CAN_CONTROLLER_CAN_MESSAGE_H_

#include <stdint.h>

/**
 * Struct that carries a CAN message.
 */
typedef struct can_message {
  /**
   * Standard or extended ID
   */
  uint32_t id;
  /**
   * Boolean to indicate whether the ID is extended
   */
  uint8_t is_extended;
  /**
   * Boolean to indicate whether the message is remote.
   * The flag is ignored when sending as an FD frame.
   */
  uint8_t is_remote;
  /**
   * Data length a.k.a. DLC
   */
  uint8_t data_length;
  /**
   * Boolean to indicate whether the frame is an FDF.
   */
  uint8_t is_fd;
  /**
   * Boolean to indicate whehter the data rate is/was switched.
   */
  uint8_t brs;
  /**
   * Data bytes
   */
  uint8_t data[8];
} can_message_t;

#endif // CAN_CONTROLLER_CAN_MESSAGE_H_
