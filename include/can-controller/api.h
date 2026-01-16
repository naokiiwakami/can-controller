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

#define PAYLOAD_SIZE 8

/**
 * Sends a CAN message.
 *
 * @param message Message to send. The passed message must be created by
 * can_create_message(). Also note that the message content is overwritten for
 * performance in this method. If you need the values after calling this
 * function, save them before calling.
 */
extern void can_send_message(can_message_t *message);

////////////////////////////////////////////////////////////////

/**
 * Create default configuration.
 *
 * @returns default configuration
 */
extern can_config_t can_make_default_config();

/**
 * Sets bitrate.
 *
 * @param config The configuration object
 * @param bitrate Bitrate in bps
 * @returns 0 on successful configuration, 1 on erro
 */
extern int can_set_bitrate(can_config_t *config, uint32_t bitrate);

/**
 * Sets FD data bitrate.
 *
 * Calling this method also enables FD data transfer and bit rate switching
 * (BRS).
 *
 * @param config The configuration object
 * @param bitrate Bitrate in bps
 * @returns 0 on successful configuration, 1 on erro
 */
extern int can_set_fd_data_bitrate(can_config_t *config, uint32_t bitrate);

/**
 * Initializes the can controller.
 *
 * @returns 0 on successful initialization. 1 on error.
 */
extern uint8_t can_init(const can_config_t *config);

/**
 * Start configuring ID filters.
 *
 * @returns Filter configuration handle. NULL if the device does not support
 * filter configuration from users.
 */
extern void *can_filter_start_config();

extern int can_filter_clear(void *handle);

/**
 * Set filter to accept IDs greater than or equal to a lower boundary.
 *
 * @param handle - Filter configuration handle
 * @param lower_boundary - The lower boundary to accept IDs
 *
 * @returns 0 on successful configuration, -1 when the input is invalid, -2 when
 * too many filters are needed.
 */
extern int can_filter_add_std_id_gte(void *handle, uint16_t lower_boundary);

extern int can_filter_add_ext_id_all(void *handle);

/**
 * Apply the filter configuration.
 *
 * @param handle - Filter configuration handle.
 */
extern int can_filter_apply_config(void *handle);

/**
 * Start the can controller.
 *
 * @returns 0 on successful initialization. 1 on error.
 */
extern uint8_t can_start();

extern can_message_t *can_create_message();
extern void can_free_message(can_message_t *message);

/**
 * Application specific callback function to handle a received message.
 *
 * The application must implement this method. The application also is
 * responsible to free the message by calling can_free_message().
 */
extern void can_consume_rx_message(can_message_t *message);

#ifdef SUPPORT_CALLBACK_INJECTION
extern void can_set_rx_message_consumer(void (*callback)(can_message_t *));
#endif

#ifdef __cplusplus
}
#endif
#endif // CAN_CONTROLLER_API_H_
