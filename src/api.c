#include "can-controller/api.h"

#if CONTROLLER_PLATFORM == raspberry_pi
#define __message_create_by_malloc
#endif

#ifdef __message_create_by_malloc
#include <malloc.h>
#include <stddef.h>
#endif

can_message_t *can_create_message() {
  can_message_t *message = NULL;
#ifdef __message_create_by_malloc
  // The message may be reused for sending and receiving data to/from the
  // controller device via SPI interface. We allocate 8 byte header field before
  // the 8 byte data in order to allow putting some preceding data, such as SPI
  // command and CAN header fields. This sacrifices security as the message data
  // may be overwritten but gives better performance and smaller memory
  // footprints.
  void *buffer = malloc(16);
  message = (can_message_t *)(buffer + 8 - offsetof(can_message_t, data));
#endif
  return message;
}

void can_free_message(can_message_t *message) {
#ifdef __message_create_by_malloc
  void *buffer = message + offsetof(can_message_t, data) - 8;
#endif
}
