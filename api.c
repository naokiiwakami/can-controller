#include "can-controller/api.h"

#if defined(CONTROLLER_PLATFORM_RASPBERRY_PI) || \
    defined(CONTROLLER_PLATFORM_PSOC)
#define __message_create_by_malloc
#endif

#ifdef __message_create_by_malloc
#include <malloc.h>
#include <stddef.h>
#include <string.h>
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
  memset(buffer, 0, 16);
  message = (can_message_t *)(buffer + 16 - sizeof(can_message_t));
#endif
  return message;
}

void can_free_message(can_message_t *message) {
#ifdef __message_create_by_malloc
  void *buffer = message + sizeof(can_message_t) - 16;
  free(buffer);
#endif
}

#ifdef SUPPORT_CALLBACK_INJECTION
static void (*consume_rx_message)(can_message_t *) = NULL;

void can_consume_rx_message(can_message_t *message) {
  if (consume_rx_message != NULL) {
    consume_rx_message(message);
  }
}

void can_set_rx_message_consumer(void (*callback)(can_message_t *)) {
  consume_rx_message = callback;
}
#endif
