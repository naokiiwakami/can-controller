#include "can-controller/api.h"

#if CONTROLLER_PLATFORM == raspberry_pi
#define __message_create_by_malloc
#endif

#ifdef __message_create_by_malloc
#include <malloc.h>
#endif

can_message_t *can_create_message() {
  can_message_t *message = NULL;
#ifdef __message_create_by_malloc
  message = (can_message_t *)malloc(sizeof(can_message_t));
#endif
  return message;
}

void can_free_message(can_message_t *message) {
#ifdef __message_create_by_malloc
  free(message);
#endif
}
