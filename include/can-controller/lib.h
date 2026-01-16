#ifndef CAN_CONTROLLER_LIB_H_
#define CAN_CONTROLLER_LIB_H_

/**
 * platform specific implementations
 */

#include <stddef.h>
#include <stdint.h>

extern void platform_sleep_ms(uint32_t milliseconds);
extern void platform_write_spi(uint8_t *buffer, size_t length);
extern int platform_init_spi();
extern int platform_init_rx_interrupt();


#endif  // CAN_CONTROLLER_LIB_H_
