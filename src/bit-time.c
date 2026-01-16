#include "can-controller/bit-time.h"

#define kTrilion 1000000000
#define kMillion 1000000
#define MIN(x, y) ((x) < (y) ? (x) : (y))

uint32_t can_bt_get_propergation_delay(const can_bt_config_t *config) {
  return 2 * (kBusDelayPerMeter * config->bus_length +
              config->tx_propergation_delay + config->rx_propergation_delay);
}

int32_t can_bt_calculate_bit_time_params(can_bt_params_t *params,
                                         const can_bt_source_t *src,
                                         uint32_t propergation_delay) {
  uint32_t tq = kTrilion / src->clock;    // time quanta in nano sec
  uint32_t bt = kTrilion / src->bit_rate; // ns
  params->num_tq = bt / tq;
  params->prseg = (propergation_delay + tq - 1) / tq;
  params->tseg1 = src->split_point * params->num_tq / 100 - 1;
  params->tseg2 = params->num_tq - kNsync - params->tseg1;
  if (params->tseg1 > params->prseg) {
    uint32_t phseg1 = params->tseg1 - params->prseg;
    params->sjw = MIN(phseg1, params->tseg2);
  } else {
    params->sjw = MIN(params->tseg1, params->tseg2);
  }
  params->tq = tq;
  return 0;
}