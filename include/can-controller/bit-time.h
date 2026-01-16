#pragma once

#include <stdint.h>

// constants
#define kNsync 1            /* it's always 1 */
#define kBusDelayPerMeter 5 /* based on 2/3 of speed of light */

/**
 * bit time calculation source.
 */
typedef struct can_bit_time_source {
  uint32_t clock;       // in Hz
  uint32_t bit_rate;    // target bit rate in bps
  uint32_t split_point; // in percent
} can_bt_source_t;

/**
 * bit time calculation results.
 */
typedef struct can_bit_time_params {
  uint32_t tq;     // time quanta in ns
  uint32_t num_tq; // number of TQ
  uint32_t prseg;  // propergation segment
  uint32_t tseg1;  // phase segment 1
  uint32_t tseg2;  // phase segment 2
  uint32_t sjw;    // synchronization jump width
} can_bt_params_t;

/**
 * bit time configuration parameters
 */
typedef struct can_bit_time_config {
  uint32_t bus_length;            // physical length in meters
  uint32_t tx_propergation_delay; // tranceiver tx delay in ns
  uint32_t rx_propergation_delay; // tranceiver rx delay in ns

  can_bt_source_t nominal_bt_src;
  can_bt_params_t nominal_bt_params;

  can_bt_source_t data_bt_src;
  can_bt_params_t data_bt_params;

} can_bt_config_t;

extern uint32_t can_bt_get_propergation_delay(const can_bt_config_t *config);
extern int32_t can_bt_calculate_bit_time_params(can_bt_params_t *params,
                                                const can_bt_source_t *src,
                                                uint32_t propergation_delay);
/*
extern int32_t can_bt_set_bitrate(can_config_t *config, uint32_t bitrate);
extern int32_t can_bt_set_fd_data_bitrate(can_config_t *config,
uint32_t bitrate);
*/
