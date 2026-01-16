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
#endif // __message_create_by_malloc

#include "can-controller/internal.h"
#include "can-controller/lib.h"

// Bytes before data array in a message object may be used as an SPI header.
// Enough space is necessary.
#define __CAN_MESSAGE_OFFSET 8
#define __CAN_MESSAGE_ALLOC_SIZE (sizeof(can_message_t) + __CAN_MESSAGE_OFFSET)

#ifdef __message_create_by_mempool
#define MAX_NUM_MESSAGES 32
typedef struct message_tray {
  uint8_t padding[__CAN_MESSAGE_OFFSET] can_message_t message;
  struct message_tray *prev;
  struct message_tray *next;
} message_tray_t;
static message_tray_t __tray_head;
static message_tray_t __tray_tail;
static message_tray_t __trays[MAX_NUM_MESSAGES];

static void init_message_pool() {
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
#endif // __message_create_by_mempool

void initialize_api() {
#ifdef __message_create_by_mempool
  init_message_pool();
#endif
}

uint8_t can_init(const can_config_t *config) {
  initialize_api();
  if (platform_init_spi(config)) {
    return 1;
  }

  if (device_init(config)) {
    return 1;
  }

  // The interrupts are triggered by the RX0BF (11) pin on the MCP2515 chip.
  if (platform_init_rx_interrupt()) {
    return 1;
  }

  return 0;
}

can_message_t *can_create_message() {
  can_message_t *message = NULL;
#ifdef __message_create_by_malloc
  // The message may be reused for sending and receiving data to/from the
  // controller device via SPI interface. We allocate 16 byte header field
  // before the 8 byte data in order to allow putting some preceding data, such
  // as SPI command and CAN header fields. This sacrifices security as the
  // message data may be overwritten but gives better performance and smaller
  // memory footprints.
  uint8_t *buffer = (uint8_t *)malloc(__CAN_MESSAGE_ALLOC_SIZE);
  memset(buffer, 0, __CAN_MESSAGE_ALLOC_SIZE);
  message = (can_message_t *)(buffer + __CAN_MESSAGE_OFFSET);
#endif // __message_create_by_malloc

#ifdef __message_create_by_mempool
  message_tray_t *tray = __tray_head.next;
  if (tray == &__tray_tail) {
    return NULL;
  }
  __tray_head.next = tray->next;
  tray->next->prev = &__tray_head;
  message = (can_message_t *)tray;
#endif // __message_create_by_mempool
  return message;
}

void can_free_message(can_message_t *message) {
#ifdef __message_create_by_malloc
  uint8_t *buffer = (uint8_t *)message - __CAN_MESSAGE_OFFSET;
  free(buffer);
#endif
#ifdef __message_create_by_mempool
  message_tray_t *tray = (message_tray_t *)message;
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
