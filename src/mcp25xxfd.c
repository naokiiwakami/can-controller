#include <stdio.h>
#include <string.h>

#include "can-controller/api.h"
#include "can-controller/device/mcp25xxfd_defines.h"
#include "can-controller/device/mcp25xxfd_register.h"
#include "can-controller/lib.h"

// MCP25xxFD SPI Instructions ///////////////////////////////
#define MCP25xxFD_RESET 0x00      // C = 0b0000; A = 0x000
#define MCP25xxFD_READ 0x30       // C = 0b0011; A; D = SDO
#define MCP25xxFD_WRITE 0x20      // C = 0b0010; A; D = SDI
#define MCP25xxFD_READ_CRC 0xb0   // C = 0b1011; A; N; D = SDO; CRC = SDO
#define MCP25xxFD_WRITE_CRC 0xa0  // C = 0b1010; A; N; D = SDI; CRC = SDI
#define MCP25xxFD_WRITE_SAFE 0xc0 // C = 0b1100; A; D = SDI; CRC = SDI

// Private method declarations /////////////////////////////
union RegisterSpiObject {
  struct __attribute__((__packed__)) RegisterData {
    uint8_t header[2];
    union RegisterBody {
      uint8_t byte[4];
      uint32_t word;
    } body;
  } data;
  uint8_t buffer[6];
};

union TxMessageSpiObject {
  struct __attribute__((__packed__)) TxMessageData {
    uint8_t header[2];
    struct __attribute__((__packed__)) TxMessage {
      CAN_MSGOBJ_ID id;
      CAN_TX_MSGOBJ_CTRL ctrl;
      uint8_t payload[PAYLOAD_SIZE];
    } message;
  } data;
  uint8_t buffer[PAYLOAD_SIZE + 10];
};

union RxMessageSpiObject {
  struct __attribute__((__packed__)) RxMessageData {
    uint8_t header[2];
    struct __attribute__((__packed__)) RxMessage {
      CAN_MSGOBJ_ID id;
      CAN_RX_MSGOBJ_CTRL ctrl;
      uint8_t payload[PAYLOAD_SIZE];
    } message;
  } data;
  uint8_t buffer[PAYLOAD_SIZE + 10];
};

typedef struct __CanFilterConfig {
  uint32_t modified;
  REG_CiFLTCON_BYTE con;
  REG_CiFLTOBJ obj;
  REG_CiMASK mask;
} can_filter_config_t;

typedef struct __CanFilterAllConfigs {
  uint32_t num_filters;
  can_filter_config_t filters[CAN_FILTER_TOTAL];
} can_filter_all_configs_t;

static void mcp25xxfd_read_register(uint16_t address,
                                    union RegisterSpiObject *info,
                                    size_t length);
static void mcp25xxfd_write_register(uint16_t address,
                                     union RegisterSpiObject *info,
                                     size_t length);

static union TxMessageSpiObject *make_tx_message(can_message_t *message);
static union RxMessageSpiObject *make_rx_message(can_message_t *message);

static void mcp25xxfd_reset();
static uint8_t mcp25xxfd_config_osc(can_config_t *config);
static uint8_t mcp25xxfd_config_io();
static uint8_t mcp25xxfd_config_bit_time(const can_config_t *config);
static uint8_t mcp25xxfd_config_interrupt();
static uint8_t mcp25xxfd_config_can_control(const can_config_t *config);
static uint8_t mcp25xxfd_config_txfifo();
static uint8_t mcp25xxfd_config_rxfifo();
static uint8_t mcp25xxfd_config_default_filter();
static uint8_t mcp25xxfd_change_mode(uint8_t mode);

static can_message_t *mcp25xxfd_get_rx_message();

#ifdef DEBUG
static void dump_filter(CAN_FILTER channel);
#endif

// Implementation of API methods ///////////////////////////////////

can_config_t can_make_default_config() {
  can_config_t config = {0};
  // 16MHz, The device accepts up to 20MHz but frequent communication errors
  // occur with the rate.
  config.spi_speed_hz = 16000000;

  // CAN controller
  config.device.fd_mode = 1;
  config.device.brs_enabled = 1;

  // oscillator config
  config.device.osc_clock_hz = 20000000;
  config.device.pll_enabled = 0;

  // bit time config sources
  config.bt_config.bus_length = 10;
  config.bt_config.tx_propergation_delay = 100; // from TCAN3413 datasheet
  config.bt_config.rx_propergation_delay = 90;  // from TCAN3413 datasheet

  config.device.nominal_baud_rate_prescale = 1;
  config.device.data_baud_rate_prescale = 1;

  uint32_t sys_clock =
      config.device.osc_clock_hz * (config.device.pll_enabled ? 10 : 1);
  uint32_t propergation_delay =
      can_bt_get_propergation_delay(&config.bt_config);

  config.bt_config.nominal_bt_src.bit_rate = 1000000;
  config.bt_config.nominal_bt_src.split_point = 80;
  config.bt_config.nominal_bt_src.clock =
      sys_clock / config.device.nominal_baud_rate_prescale;
  can_bt_calculate_bit_time_params(&config.bt_config.nominal_bt_params,
                                   &config.bt_config.nominal_bt_src,
                                   propergation_delay);

  config.bt_config.data_bt_src.bit_rate = 4000000;
  config.bt_config.data_bt_src.split_point = 80;
  config.bt_config.data_bt_src.clock =
      sys_clock / config.device.data_baud_rate_prescale;
  can_bt_calculate_bit_time_params(&config.bt_config.data_bt_params,
                                   &config.bt_config.data_bt_src,
                                   propergation_delay);

  return config;
}

int can_set_bitrate(can_config_t *config, uint32_t bitrate) {
  config->bt_config.nominal_bt_src.bit_rate = bitrate;

  uint32_t propergation_delay =
      can_bt_get_propergation_delay(&config->bt_config);
  can_bt_calculate_bit_time_params(&config->bt_config.nominal_bt_params,
                                   &config->bt_config.nominal_bt_src,
                                   propergation_delay);
  return 0;
}

int can_set_fd_data_bitrate(can_config_t *config, uint32_t bitrate) {
  config->bt_config.data_bt_src.bit_rate = bitrate;
  config->device.fd_mode = 1;
  config->device.brs_enabled = 1;

  uint32_t propergation_delay =
      can_bt_get_propergation_delay(&config->bt_config);
  can_bt_calculate_bit_time_params(&config->bt_config.data_bt_params,
                                   &config->bt_config.data_bt_src,
                                   propergation_delay);
  return 0;
}

uint8_t device_init(can_config_t *config) {
  mcp25xxfd_reset();
  if (mcp25xxfd_config_osc(config)) {
    return 1;
  }
  if (mcp25xxfd_config_io()) {
    return 1;
  }
  if (mcp25xxfd_config_bit_time(config)) {
    return 1;
  }
  if (mcp25xxfd_config_interrupt()) {
    return 1;
  }
  if (mcp25xxfd_config_can_control(config)) {
    return 1;
  }
  if (mcp25xxfd_config_txfifo()) {
    return 1;
  }
  if (mcp25xxfd_config_rxfifo()) {
    return 1;
  }
  if (mcp25xxfd_config_default_filter()) {
    return 1;
  }
  return 0;
}

static void load_filters(can_filter_all_configs_t *config) {
  config->num_filters = 0;
  union RegisterSpiObject con_info = {0};
  // read filter
  for (CAN_FILTER channel = CAN_FILTER0; channel < CAN_FILTER_TOTAL;
       ++channel) {
    can_filter_config_t *filter = &config->filters[channel];
    filter->modified = 0;
    uint32_t offset = channel % 4;
    uint16_t address;
    if (offset == 0) {
      address = cREGADDR_CiFLTCON + channel;
      mcp25xxfd_read_register(address, &con_info, 4);
    }
    filter->con.byte = con_info.data.body.byte[offset];
    if (filter->con.bF.Enable) {
      // assuming filters are contiguous
      ++config->num_filters;
    } else {
      filter->obj.word = 0;
      filter->mask.word = 0;
      continue;
    }

    union RegisterSpiObject info = {0};
    address = cREGADDR_CiFLTOBJ + (channel * CiFILTER_OFFSET);
    mcp25xxfd_read_register(address, &info, 4);
    filter->obj.word = info.data.body.word;
    address = cREGADDR_CiMASK + (channel * CiFILTER_OFFSET);
    mcp25xxfd_read_register(address, &info, 4);
    filter->mask.word = info.data.body.word;
  }
}

void *can_filter_start_config() {
  can_filter_all_configs_t *config =
      (can_filter_all_configs_t *)malloc(sizeof(can_filter_all_configs_t));
  load_filters(config);
  return config;
}

int can_filter_clear(void *handle) {
  can_filter_all_configs_t *config = (can_filter_all_configs_t *)handle;
  uint8_t modified = 0;
  union RegisterSpiObject con_info = {0};
  for (CAN_FILTER channel = CAN_FILTER0; channel < CAN_FILTER_TOTAL;
       ++channel) {
    can_filter_config_t *filter = &config->filters[channel];
    uint32_t offset = channel % 4;
    if (filter->con.bF.Enable) {
      filter->con.bF.Enable = 0;
      modified = 1;
    }
    con_info.data.body.byte[offset] = filter->con.byte;
    if (offset == 3) {
      if (modified) {
        uint16_t address = cREGADDR_CiFLTCON + channel - offset;
        mcp25xxfd_write_register(address, &con_info, 4);
      }
      modified = 0;
    }
  }
  load_filters(config);
  if (config->num_filters > 0) {
    fprintf(stderr, "Failed to clear filters; %u remaining\n",
            config->num_filters);
    return -1;
  }
  return 0;
}

#define MAX_SID 0x7FF

int can_filter_add_std_id_gte(void *handle, uint16_t lower_boundary) {
  can_filter_all_configs_t *config = (can_filter_all_configs_t *)handle;
  if (lower_boundary > MAX_SID) {
    fprintf(stderr, "lower_boundary exceeds CAN STD ID range: %04x",
            lower_boundary);
    return -1; // Invalid input
  }

  uint16_t start = lower_boundary;

  while (start <= MAX_SID) {
    uint16_t mask = 0x7FF; // Start with most specific mask
    uint16_t step = 1;

    // Try to find the largest block starting at 'start' that fits within the
    // allowed range
    for (int bit = 0; bit <= 10; bit++) {
      uint16_t block_size = 1 << bit;
      uint16_t aligned_start = start & ~(block_size - 1);

      if (aligned_start != start) {
        break; // Not aligned for this block size
      }

      if (start + block_size - 1 > MAX_SID) {
        break; // Overflows SID range
      }

      mask = ~(block_size - 1) & 0x7FF;
      step = block_size;
    }

    CAN_FILTER channel = config->num_filters;
    if (channel >= CAN_FILTER_TOTAL) {
      fprintf(stderr, "No more filter available");
      return -2; // Too many filters needed
    }

    can_filter_config_t *filter = &config->filters[channel];
    filter->obj.bF.EXIDE = 0;
    filter->obj.bF.SID11 = 0;
    filter->obj.bF.EID = 0;
    filter->obj.bF.SID = start;
    filter->mask.bF.MIDE = 1;
    filter->mask.bF.MSID11 = 0;
    filter->mask.bF.MEID = 0;
    filter->mask.bF.MSID = mask;
    filter->con.bF.Enable = 1;
    filter->con.bF.BufferPointer = CAN_FIFO_CH2;
    filter->modified = 1;

    ++config->num_filters;
    start += step;
  }

  return 0;
}

int can_filter_add_ext_id_all(void *handle) {
  can_filter_all_configs_t *config = (can_filter_all_configs_t *)handle;
  if (config->num_filters >= CAN_FILTER_TOTAL) {
    fprintf(stderr, "No more filter available");
    return -2;
  }
  CAN_FILTER channel = config->num_filters++;
  can_filter_config_t *filter = &config->filters[channel];
  filter->obj.bF.EXIDE = 1;
  filter->obj.bF.SID = 0;
  filter->obj.bF.EID = 0;
  filter->mask.bF.MIDE = 1;
  filter->mask.bF.MEID = 0;
  filter->mask.bF.MSID = 0;
  filter->con.bF.Enable = 1;
  filter->con.bF.BufferPointer = CAN_FIFO_CH2;
  filter->modified = 1;
  return 0;
}

int can_filter_apply_config(void *handle) {
  can_filter_all_configs_t *config = (can_filter_all_configs_t *)handle;

  union RegisterSpiObject con_info = {0};
  uint8_t modified = 0;
  for (CAN_FILTER channel = CAN_FILTER0; channel < CAN_FILTER_TOTAL;
       ++channel) {
    uint16_t address;
    can_filter_config_t *filter = &config->filters[channel];
    if (filter->modified) {
      modified = true;

      union RegisterSpiObject info = {0};
      address = cREGADDR_CiFLTOBJ + (channel * CiFILTER_OFFSET);
      info.data.body.word = filter->obj.word;
      mcp25xxfd_write_register(address, &info, 4);

      address = cREGADDR_CiMASK + (channel * CiFILTER_OFFSET);
      info.data.body.word = filter->mask.word;
      mcp25xxfd_write_register(address, &info, 4);
    }

    uint32_t offset = channel % 4;
    con_info.data.body.byte[offset] = filter->con.byte;
    if (offset == 3) {
      if (modified) {
        address = cREGADDR_CiFLTCON + channel - offset;
        mcp25xxfd_write_register(address, &con_info, 4);
      }
      modified = 0;
    }
  }

  free(config);
  return 0;
}

uint8_t can_start() {
  // Change the controller to classic CAN mode
  if (mcp25xxfd_change_mode(CAN_NORMAL_MODE)) {
    return 1;
  }

  // Reset GPIO0 pin to enable the CAN tranceiver
  union RegisterSpiObject info = {0};
  mcp25xxfd_read_register(cREGADDR_IOCON, &info, 4);
  REG_IOCON io_con;
  io_con.word = info.data.body.word;
  io_con.bF.LAT0 = 0;
  io_con.bF.TRIS0 = 0;
  info.data.body.word = io_con.word;
  mcp25xxfd_write_register(cREGADDR_IOCON, &info, 4);
  return 0;
}

void handle_rx() {
  for (;;) {
    can_message_t *message = mcp25xxfd_get_rx_message();
    if (message == NULL) {
      break;
    }
    can_consume_rx_message(message);
  }
}

void can_send_message(can_message_t *message) {
  uint8_t dlc = message->data_length;
  union TxMessageSpiObject *tx = make_tx_message(message);

  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH1;

  union RegisterSpiObject reg;
  uint16_t register_address = cREGADDR_CiFIFOUA + channel * CiFIFO_OFFSET;
  mcp25xxfd_read_register(register_address, &reg, 2);
  uint16_t fifo_address = reg.data.body.word + cRAMADDR_START;
  size_t bytes_to_send = 10 + (dlc + 3) / 4 * 4;
  tx->data.header[0] = MCP25xxFD_WRITE | (fifo_address >> 8);
  tx->data.header[1] = fifo_address & 0xff;
  platform_write_spi(tx->buffer, bytes_to_send);

  // flush
  register_address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
  reg.data.body.byte[0] = 0x3; // UINC + TXREQ
  mcp25xxfd_write_register(register_address, &reg, 1);

  can_free_message(message);
}

////////////////////////////////////////////////////////////

union TxMessageSpiObject *make_tx_message(can_message_t *message) {
  union TxMessageSpiObject *tx =
      (union TxMessageSpiObject *)(message->data - 10);
  uint32_t id = message->id;
  uint8_t dlc = message->data_length & 0xf;
  uint8_t is_remote = message->is_remote;
  uint8_t is_extended = message->is_extended;
  uint8_t is_fd = message->is_fd;
  uint8_t brs = message->brs;
  memset(&tx->data.message.id, 0, 8); // CAN_MSGOBJ_ID + CAN_TX_MSGOBJ_CTRL
  if (is_extended) {
    tx->data.message.ctrl.IDE = 1;
    tx->data.message.id.EID = id & 0x3ffff;
    tx->data.message.id.SID = id >> 18;
  } else {
    tx->data.message.id.SID = id & 0x7ff;
  }
  tx->data.message.ctrl.DLC = dlc;
  tx->data.message.ctrl.FDF = is_fd;
  tx->data.message.ctrl.RTR = is_fd ? 0 : is_remote;
  tx->data.message.ctrl.BRS = brs;
  return tx;
}

union RxMessageSpiObject *make_rx_message(can_message_t *message) {
  union RxMessageSpiObject *rx =
      (union RxMessageSpiObject *)(message->data - 10);
  return rx;
}

// MCP25xxFD SPI operations ///////////////////////////////////////////
void mcp25xxfd_read_register(uint16_t address, union RegisterSpiObject *info,
                             size_t length) {
  info->data.header[0] = MCP25xxFD_READ | (address >> 8);
  info->data.header[1] = address & 0xff;
  platform_write_spi(info->buffer, length + 2);
}

void mcp25xxfd_write_register(uint16_t address, union RegisterSpiObject *info,
                              size_t length) {
  info->data.header[0] = MCP25xxFD_WRITE | (address >> 8);
  info->data.header[1] = address & 0xff;
  platform_write_spi(info->buffer, length + 2);
}

void mcp25xxfd_reset() {
  uint8_t buf[2] = {MCP25xxFD_RESET, 0};
  platform_write_spi(buf, 2);
  platform_sleep_ms(10);
}

// MCP25xxFD methods //////////////////////////////////////////////////

uint8_t mcp25xxfd_config_osc(can_config_t *config) {
  // Set OSC
  REG_OSC osc = {0};
  osc.bF.PllEnable =
      config->device.pll_enabled; // 0: no PLL, 1: sysclk from 10x PLL
  osc.bF.OscDisable = 0;          // 0: Enable clock, 1: Clock disabled
  osc.bF.LowPowerModeEnable = 0;
  osc.bF.SCLKDIV = 0; // 0: divided by 1, 1: divided by 2
  osc.bF.CLKODIV = OSC_CLKO_DIV1;
  union RegisterSpiObject info = {0};
  info.data.body.word = osc.word;
  mcp25xxfd_write_register(cREGADDR_OSC, &info, 1);
  // check OSC status
  mcp25xxfd_read_register(cREGADDR_OSC, &info, 2);
  if ((info.data.body.byte[1] & 0x4) == 0) {
    fprintf(stderr, "Clock is not ready\n");
    return 1;
  }
  if ((info.data.body.byte[0] & 0x10)) {
    fprintf(stderr, "System clock divisor is ON wrongfully\n");
    return 1;
  }
  if (info.data.body.byte[1] & 0x1) {
    fprintf(stderr, "PLL is ON wrongfully\n");
    return 1;
  }
  return 0;
}

uint8_t mcp25xxfd_config_io() {
  union RegisterSpiObject info = {0};
  mcp25xxfd_read_register(cREGADDR_IOCON, &info, 4);
  REG_IOCON io_con;
  io_con.word = info.data.body.word;
  // Use this pin for RX interrupt
  io_con.bF.PinMode1 = 0;
  // Use this pin for GPIO to control tranceiver stand-by
  io_con.bF.PinMode0 = 1;
  io_con.bF.LAT0 = 1;
  io_con.bF.GPIO0 = 1;
  io_con.bF.TRIS0 = 1;

  info.data.body.word = io_con.word;
  mcp25xxfd_write_register(cREGADDR_IOCON, &info, 4);
  mcp25xxfd_read_register(cREGADDR_IOCON, &info, 4);
  if ((info.data.body.byte[3] & 0x2) != 0) {
    fprintf(stderr, "Failed to set up interrupt pings: %02x\n",
            info.data.body.byte[3]);
    return 1;
  }
  return 0;
}

uint8_t mcp25xxfd_config_bit_time(const can_config_t *config) {
  union RegisterSpiObject info = {0};

  uint32_t tdco = config->device.data_baud_rate_prescale *
                  config->bt_config.data_bt_params.tseg1;

  // uint32_t nominal_time_quanta = nBrp
  REG_CiNBTCFG nbt_config = {0};
  nbt_config.bF.BRP = config->device.nominal_baud_rate_prescale - 1;
  nbt_config.bF.TSEG1 = config->bt_config.nominal_bt_params.tseg1 - 1;
  nbt_config.bF.TSEG2 = config->bt_config.nominal_bt_params.tseg2 - 1;
  nbt_config.bF.SJW = config->bt_config.nominal_bt_params.sjw - 1;

  info.data.body.word = nbt_config.word;
  mcp25xxfd_write_register(cREGADDR_CiNBTCFG, &info, 4);

  REG_CiDBTCFG dbt_config = {0};
  REG_CiTDC tdc_config = {0};
  dbt_config.bF.BRP = config->device.data_baud_rate_prescale - 1;
  dbt_config.bF.TSEG1 = config->bt_config.data_bt_params.tseg1 - 1;
  dbt_config.bF.TSEG2 = config->bt_config.data_bt_params.tseg2 - 1;
  dbt_config.bF.SJW = config->bt_config.data_bt_params.sjw - 1;

  info.data.body.word = dbt_config.word;
  mcp25xxfd_write_register(cREGADDR_CiDBTCFG, &info, 4);

  tdc_config.bF.TDCMode = CAN_SSP_MODE_AUTO;
  tdc_config.bF.TDCOffset = tdco;
  tdc_config.bF.TDCValue = 0;

  info.data.body.word = tdc_config.word;
  mcp25xxfd_write_register(cREGADDR_CiTDC, &info, 4);

  return 0;
}

uint8_t mcp25xxfd_config_interrupt() {
  // enable receive FIFO interrupt
  REG_CiINTENABLE enables;
  enables.word = 0;
  enables.IE.RXIE = 1;
  union RegisterSpiObject info = {0};
  info.data.body.word = enables.word;
  mcp25xxfd_write_register(cREGADDR_CiINTENABLE, &info, 1);
  return 0;
}

uint8_t mcp25xxfd_config_can_control(const can_config_t *config) {
  REG_CiCON can_ctl;
  can_ctl.word = canControlResetValues[cREGADDR_CiCON / 4];
  can_ctl.bF.TxBandWidthSharing = CAN_TXBWS_NO_DELAY;
  can_ctl.bF.TXQEnable = 0;
  can_ctl.bF.StoreInTEF = 0;
  can_ctl.bF.SystemErrorToListenOnly = 0;
  can_ctl.bF.EsiInGatewayMode = 0;
  can_ctl.bF.RestrictReTxAttempts = 0;
  can_ctl.bF.BitRateSwitchDisable = config->device.brs_enabled ? 0 : 1;

  union RegisterSpiObject info = {0};
  info.data.body.word = can_ctl.word;
  mcp25xxfd_write_register(cREGADDR_CiCON + 2, &info, 1);
  return 0;
}

uint8_t mcp25xxfd_config_txfifo() {
  // Use FIFO1 for TX
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH1;

  union RegisterSpiObject info = {0};

  REG_CiFIFOCON fifo_con;
  fifo_con.word = canFifoResetValues[0];
  fifo_con.txBF.PayLoadSize = CAN_PLSIZE_8;
  fifo_con.txBF.FifoSize = 0x4;    // 5 messages
  fifo_con.txBF.TxAttempts = 0x3;  // unlimited
  fifo_con.txBF.TxPriority = 0x10; // setting medium
  fifo_con.txBF.TxEnable = 1;

  info.data.body.word = fifo_con.word;

  uint16_t address = cREGADDR_CiFIFOCON + channel * CiFIFO_OFFSET;
  mcp25xxfd_write_register(address, &info, 4);
  return 0;
}

uint8_t mcp25xxfd_config_rxfifo() {
  // Use FIFO2 for RX
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH2;

  REG_CiFIFOCON fifo_con;
  fifo_con.word = canFifoResetValues[0];
  fifo_con.rxBF.FifoSize = 0x1f;
  fifo_con.rxBF.PayLoadSize = CAN_PLSIZE_8;
  fifo_con.rxBF.RxNotEmptyIE = 1;

  union RegisterSpiObject info = {0};
  info.data.body.word = fifo_con.word;
  uint16_t address = cREGADDR_CiFIFOCON + channel * CiFIFO_OFFSET;
  mcp25xxfd_write_register(address, &info, 4);

  info.data.body.word = 0;
  mcp25xxfd_read_register(cREGADDR_CiINTENABLE, &info, 2);
  info.data.body.word |= CAN_RX_EVENT;
  mcp25xxfd_write_register(cREGADDR_CiINTENABLE, &info, 2);
  return 0;
}

static uint8_t mcp25xxfd_disable_filter(CAN_FILTER filter) {
  union RegisterSpiObject info = {0};
  uint16_t address = cREGADDR_CiFLTCON + filter;
  mcp25xxfd_read_register(address, &info, 1);
  REG_CiFLTCON_BYTE filter_ctl;
  filter_ctl.byte = info.data.body.byte[0];
  filter_ctl.bF.Enable = 0;
  info.data.body.byte[0] = filter_ctl.byte;
  mcp25xxfd_write_register(address, &info, 1);
  return 0;
}

static uint8_t mcp25xxfd_link_filter_to_fifo(CAN_FILTER filter,
                                             CAN_FIFO_CHANNEL channel,
                                             uint8_t enable) {
  REG_CiFLTCON_BYTE filter_ctl;
  filter_ctl.byte = 0;
  filter_ctl.bF.Enable = enable ? 1 : 0;
  filter_ctl.bF.BufferPointer = channel;
  uint16_t address = cREGADDR_CiFLTCON + filter;
  union RegisterSpiObject info = {0};
  info.data.body.byte[0] = filter_ctl.byte;
  mcp25xxfd_write_register(address, &info, 1);

  return 0;
}

uint8_t mcp25xxfd_config_default_filter() {
  uint16_t address;
  // disable filter 0
  mcp25xxfd_disable_filter(CAN_FILTER0);

  // Configure filter 0 object and mask to match all
  CAN_FILTER filter = CAN_FILTER0;
  union RegisterSpiObject info = {0};
  info.data.body.word = 0;
  address = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);
  mcp25xxfd_write_register(address, &info, 4);
  info.data.body.word = 0;
  address = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);
  mcp25xxfd_write_register(address, &info, 4);

  // link filter to the RX fifo, then enable the filter
  mcp25xxfd_link_filter_to_fifo(CAN_FILTER0, CAN_FIFO_CH2, 1);

  return 0;
}

uint8_t mcp25xxfd_change_mode(uint8_t mode) {
  // Set REQOP
  union RegisterSpiObject info = {0};
  mcp25xxfd_read_register(cREGADDR_CiCON + 3, &info, 1);
  info.data.body.byte[0] &= 0xf8;
  info.data.body.byte[0] |= mode & 0x7;
  mcp25xxfd_write_register(cREGADDR_CiCON + 3, &info, 1);

  // Verify OPMOD
  uint8_t current;
  for (int i = 0; i < 100; ++i) {
    mcp25xxfd_read_register(cREGADDR_CiCON + 2, &info, 1);
    current = info.data.body.byte[0] >> 5;
    if (current == mode) {
      return 0;
    }
  }
  fprintf(stderr, "Setting mode to %x failed; current=%x\n", mode, current);
  return 1;
}

can_message_t *mcp25xxfd_get_rx_message() {
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH2;

  union RegisterSpiObject reg;
  // read interrupt flags
  uint16_t address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
  mcp25xxfd_read_register(address, &reg, 1);
  CAN_RX_FIFO_EVENT flags = reg.data.body.byte[0] & CAN_RX_FIFO_ALL_EVENTS;

  if ((flags & CAN_RX_FIFO_NOT_EMPTY_EVENT) == 0) {
    // no more messages
    return NULL;
  }

  // Resolve the FIFO address
  uint16_t register_address = cREGADDR_CiFIFOUA + channel * CiFIFO_OFFSET;
  mcp25xxfd_read_register(register_address, &reg, 2);
  uint16_t fifo_address = reg.data.body.word + cRAMADDR_START;

  // Fetch the FIFO data
  can_message_t *message = can_create_message();
  union RxMessageSpiObject *rx = make_rx_message(message);
  uint8_t bytes_to_read = PAYLOAD_SIZE + 10; // 10: header bytes
  rx->data.header[0] = MCP25xxFD_READ | (fifo_address >> 8);
  rx->data.header[1] = fifo_address & 0xff;
  platform_write_spi(rx->buffer, bytes_to_read);

  // UINC channel
  register_address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
  reg.data.body.byte[0] = 0x1; // UINC
  mcp25xxfd_write_register(register_address, &reg, 1);

  // Build the message
  // The header part will be overwritten, make copies first
  CAN_MSGOBJ_ID id = rx->data.message.id;
  CAN_RX_MSGOBJ_CTRL ctrl = rx->data.message.ctrl;
  message->data_length = ctrl.DLC;
  message->is_extended = ctrl.IDE;
  message->is_remote = ctrl.RTR;
  if (message->is_extended) {
    message->id = id.EID | id.SID << 18;
  } else {
    message->id = id.SID;
  }

  return message;
}

#ifdef DEBUG
/**
 * Dump a filter config.
 */
void dump_filter(CAN_FILTER channel) {
  union SpiRegister info = {0};
  can_filter_config_t filter = {0};
  uint16_t address = cREGADDR_CiFLTCON + channel;
  mcp25xxfd_read_register(address, &info, 1);
  filter.con.byte = info.data.body.byte[0];
  address = cREGADDR_CiFLTOBJ + (channel * CiFILTER_OFFSET);
  mcp25xxfd_read_register(address, &info, 4);
  filter.obj.word = info.data.body.word;
  address = cREGADDR_CiMASK + (channel * CiFILTER_OFFSET);
  mcp25xxfd_read_register(address, &info, 4);
  filter.mask.word = info.data.body.word;
  fprintf(stderr, "filter=%02x:\n", channel);
  fprintf(stderr, "  enabled: %u\n", filter.con.bF.Enable);
  fprintf(stderr, "  fifo: %02x\n", filter.con.bF.BufferPointer);
  fprintf(stderr, "  EXIDE: %u\n", filter.obj.bF.EXIDE);
  fprintf(stderr, "  EID: %03x\n", filter.obj.bF.EID);
  fprintf(stderr, "  EID: %02x\n", filter.obj.bF.SID);
  fprintf(stderr, "  MIDE: %u\n", filter.mask.bF.MIDE);
  fprintf(stderr, "  MEID: %03x\n", filter.mask.bF.MEID);
  fprintf(stderr, "  MSID: %03x\n", filter.mask.bF.MSID);
}
#endif
