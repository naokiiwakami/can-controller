#include "can-controller/api.h"

#if defined(CONTROLLER_PLATFORM_RASPBERRY_PI)
#define __message_create_by_malloc
#endif

#if defined(CONTROLLER_PLATFORM_PSOC)
#define __message_create_by_mempool
#endif

#include <stddef.h>

#ifdef __message_create_by_malloc
#include <malloc.h>
#include <string.h>
#endif  // __message_create_by_malloc

#include "can-controller/internal.h"

#ifdef __message_create_by_mempool
#define MAX_NUM_MESSAGES 32
typedef struct message_tray {
    can_message_t message;
    struct message_tray *prev;
    struct message_tray *next;
} message_tray_t;
static message_tray_t __tray_head;
static message_tray_t __tray_tail;
static message_tray_t __trays[MAX_NUM_MESSAGES];

static void init_message_pool()
{
    __tray_head.prev = NULL;
    __tray_tail.next = NULL;
    message_tray_t *prev = &__tray_head;
    for (int i = 0; i < MAX_NUM_MESSAGES; ++i) {
        message_tray_t *current = &__trays[i];
        current->prev = prev;
        prev->next = current;
        prev = current;
    }
    prev->next = &__tray_tail;
    __tray_tail.prev = prev;
}
#endif  // __message_create_by_mempool

void initialize_api()
{
#ifdef __message_create_by_mempool
  init_message_pool();
#endif
}

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
#endif  // __message_create_by_malloc

#ifdef __message_create_by_mempool
    message_tray_t *tray = __tray_head.next;
    if (tray == &__tray_tail) {
        return NULL;
    }
    __tray_head.next = tray->next;
    tray->next->prev = &__tray_head;
    message = (can_message_t *)tray;
#endif  // __message_create_by_mempool
  return message;
}

void can_free_message(can_message_t *message) {
#ifdef __message_create_by_malloc
  void *buffer = message + sizeof(can_message_t) - 16;
  free(buffer);
#endif
#ifdef __message_create_by_mempool
    message_tray_t *tray = (message_tray_t *) message;
    tray->next = &__tray_tail;
    tray->prev = __tray_tail.prev;
    tray->prev->next = tray;
    tray->next->prev = tray;
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
