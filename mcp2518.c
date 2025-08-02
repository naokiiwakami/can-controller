#include <stdio.h>
#include <string.h>

#include "can-controller/api.h"
#include "can-controller/device/mcp2518.h"
#include "can-controller/device/mcp2518_defines.h"
#include "can-controller/device/mcp2518_register.h"
#include "can-controller/lib.h"

// Private method declarations /////////////////////////////
union SpiRegister {
  struct __attribute__((__packed__)) RegisterData {
    uint8_t header[2];
    union RegisterBody {
      uint8_t byte[4];
      uint32_t word;
    } body;
  } data;
  uint8_t buffer[6];
};

union SpiTxMessage {
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

union SpiRxMessage {
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

static void mcp2518_read_register(uint16_t address, union SpiRegister *info,
                                  size_t length);
static void mcp2518_write_register(uint16_t address, union SpiRegister *info,
                                   size_t length);

static void mcp2518_reset();
static uint8_t mcp2518_config_osc();
static uint8_t mcp2518_config_io();
static uint8_t mcp2518_config_bit_time();
static uint8_t mcp2518_config_interrupt();
static uint8_t mcp2518_config_can_control();
static uint8_t mcp2518_config_txfifo();
static uint8_t mcp2518_config_rxfifo();
static uint8_t mcp2518_config_filter();
static uint8_t mcp2518_change_mode(uint8_t mode);

static can_message_t *mcp2518_get_rx_message();

////////////////////////////////////////////////////////////

uint8_t device_init() {
  mcp2518_reset();
  if (mcp2518_config_osc()) {
    return 1;
  }
  if (mcp2518_config_io()) {
    return 1;
  }
  if (mcp2518_config_bit_time()) {
    return 1;
  }
  if (mcp2518_config_interrupt()) {
    return 1;
  }
  if (mcp2518_config_can_control()) {
    return 1;
  }
  if (mcp2518_config_txfifo()) {
    return 1;
  }
  if (mcp2518_config_rxfifo()) {
    return 1;
  }
  if (mcp2518_config_filter()) {
    return 1;
  }
  return mcp2518_change_mode(CAN_CLASSIC_MODE);
}

uint8_t device_start_can() { return 0; }

void handle_rx() {
  for (;;) {
    can_message_t *message = mcp2518_get_rx_message();
    if (message == NULL) {
      break;
    }
    can_consume_rx_message(message);
  }
}

static union SpiTxMessage *make_tx_message(can_message_t *message) {
  union SpiTxMessage *tx = (union SpiTxMessage *)(message->data - 10);
  uint32_t id = message->id;
  uint8_t dlc = message->data_length & 0xf;
  uint8_t is_remote = message->is_remote;
  uint8_t is_extended = message->is_extended;
  memset(&tx->data.message.id, 0, 8);
  if (is_extended) {
    tx->data.message.ctrl.IDE = 1;
    tx->data.message.id.EID = id & 0x3ffff;
    tx->data.message.id.SID = id >> 18;
  } else {
    tx->data.message.id.SID = id & 0x7ff;
  }
  tx->data.message.ctrl.RTR = is_remote;
  tx->data.message.ctrl.DLC = dlc;
  return tx;
}

void can_send_message(can_message_t *message) {
  uint8_t dlc = message->data_length;
  union SpiTxMessage *tx = make_tx_message(message);

  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH1;

  union SpiRegister reg;
  uint16_t register_address = cREGADDR_CiFIFOUA + channel * CiFIFO_OFFSET;
  mcp2518_read_register(register_address, &reg, 2);
  uint16_t fifo_address = reg.data.body.word + cRAMADDR_START;
  size_t bytes_to_send = 10 + (dlc + 3) / 4 * 4;
  tx->data.header[0] = MCP2518_WRITE | (fifo_address >> 8);
  tx->data.header[1] = fifo_address & 0xff;
  platform_write_spi(tx->buffer, bytes_to_send);

  // flush
  register_address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
  reg.data.body.byte[0] = 0x3; // UINC + TXREQ
  mcp2518_write_register(register_address, &reg, 1);

  can_free_message(message);
}

////////////////////////////////////////////////////////////

// MCP2518 SPI operations ///////////////////////////////////////////
void mcp2518_read_register(uint16_t address, union SpiRegister *info,
                           size_t length) {
  info->data.header[0] = MCP2518_READ | (address >> 8);
  info->data.header[1] = address & 0xff;
  platform_write_spi(info->buffer, length + 2);
}

void mcp2518_write_register(uint16_t address, union SpiRegister *info,
                            size_t length) {
  info->data.header[0] = MCP2518_WRITE | (address >> 8);
  info->data.header[1] = address & 0xff;
  platform_write_spi(info->buffer, length + 2);
}

void mcp2518_reset() {
  uint8_t buf[2] = {MCP2518_RESET, 0};
  platform_write_spi(buf, 2);
  platform_sleep_ms(10);
}

uint8_t mcp2518_config_osc() {
  union SpiRegister info = {0};
  // Set OSC
  info.data.body.byte[0] = 0x0; // no PLL, no LP, no clock divisers
  mcp2518_write_register(cREGADDR_OSC, &info, 1);
  // check OSC status
  mcp2518_read_register(cREGADDR_OSC, &info, 2);
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

uint8_t mcp2518_config_io() {
  union SpiRegister info = {0};
  mcp2518_read_register(cREGADDR_IOCON + 3, &info, 1);
  info.data.body.byte[0] &=
      0xfc; // PM1 (rx) and PM0 (tx) are used for interrupt pints
  mcp2518_write_register(cREGADDR_IOCON + 3, &info, 1);
  mcp2518_read_register(cREGADDR_IOCON, &info, 4);
  if (info.data.body.byte[3] != 0) {
    fprintf(stderr, "Failed to set up interrupt pings: %02x\n",
            info.data.body.byte[3]);
    return 1;
  }
  return 0;
}

uint8_t mcp2518_config_bit_time() {
  union SpiRegister info = {0};
  // Bit timing for NBR 1Mbps, DBR 4Mbps, Fsys=20MHz
  REG_CiNBTCFG nbt_config = {0};
  nbt_config.bF.BRP = 0;
  nbt_config.bF.TSEG1 = 14;
  nbt_config.bF.TSEG2 = 3;
  nbt_config.bF.SJW = 3;
  info.data.body.word = nbt_config.word;
  mcp2518_write_register(cREGADDR_CiNBTCFG, &info, 4);

  REG_CiDBTCFG dbt_config = {0};
  REG_CiTDC tdc_config = {0};
  dbt_config.bF.BRP = 0;
  dbt_config.bF.TSEG1 = 2;
  dbt_config.bF.TSEG2 = 0;
  dbt_config.bF.SJW = 0;
  tdc_config.bF.TDCMode = CAN_SSP_MODE_AUTO;
  tdc_config.bF.TDCOffset = 3;
  tdc_config.bF.TDCValue = 0;

  info.data.body.word = dbt_config.word;
  mcp2518_write_register(cREGADDR_CiDBTCFG, &info, 4);

  info.data.body.word = tdc_config.word;
  mcp2518_write_register(cREGADDR_CiTDC, &info, 4);

  return 0;
}

uint8_t mcp2518_config_interrupt() {
  // enable receive FIFO interrupt
  REG_CiINTENABLE enables;
  enables.word = 0;
  enables.IE.RXIE = 1;
  union SpiRegister info = {0};
  info.data.body.word = enables.word;
  mcp2518_write_register(cREGADDR_CiINTENABLE, &info, 1);
  return 0;
}

uint8_t mcp2518_config_can_control() {
  REG_CiCON can_ctl;
  can_ctl.word = canControlResetValues[cREGADDR_CiCON / 4];
  can_ctl.bF.TxBandWidthSharing = 0;
  can_ctl.bF.TXQEnable = 0;
  can_ctl.bF.StoreInTEF = 0;
  can_ctl.bF.SystemErrorToListenOnly = 0;
  can_ctl.bF.EsiInGatewayMode = 0;
  can_ctl.bF.RestrictReTxAttempts = 0;

  union SpiRegister info = {0};
  info.data.body.word = can_ctl.word;
  mcp2518_write_register(cREGADDR_CiCON + 2, &info, 1);
  return 0;
}

uint8_t mcp2518_config_txfifo() {
  // Use FIFO1 for TX
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH1;

  union SpiRegister info = {0};

  REG_CiFIFOCON fifo_con;
  fifo_con.word = canFifoResetValues[0];
  fifo_con.txBF.PayLoadSize = CAN_PLSIZE_8;
  fifo_con.txBF.FifoSize = 0x4;    // 5 messages
  fifo_con.txBF.TxAttempts = 0x3;  // unlimited
  fifo_con.txBF.TxPriority = 0x10; // setting medium
  fifo_con.txBF.TxEnable = 1;

  info.data.body.word = fifo_con.word;

  uint16_t address = cREGADDR_CiFIFOCON + channel * CiFIFO_OFFSET;
  mcp2518_write_register(address, &info, 4);
  return 0;
}

uint8_t mcp2518_config_rxfifo() {
  // Use FIFO2 for RX
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH2;

  REG_CiFIFOCON fifo_con;
  fifo_con.word = canFifoResetValues[0];
  fifo_con.rxBF.FifoSize = 0x1f;
  fifo_con.rxBF.PayLoadSize = CAN_PLSIZE_8;
  fifo_con.rxBF.RxNotEmptyIE = 1;

  union SpiRegister info = {0};
  info.data.body.word = fifo_con.word;
  uint16_t address = cREGADDR_CiFIFOCON + channel * CiFIFO_OFFSET;
  mcp2518_write_register(address, &info, 4);

  info.data.body.word = 0;
  mcp2518_read_register(cREGADDR_CiINTENABLE, &info, 2);
  info.data.body.word |= CAN_RX_EVENT;
  mcp2518_write_register(cREGADDR_CiINTENABLE, &info, 2);
  return 0;
}

static uint8_t mcp2518_disable_filter(CAN_FILTER filter) {
  union SpiRegister info = {0};
  uint16_t address = cREGADDR_CiFLTCON + filter;
  mcp2518_read_register(address, &info, 1);
  REG_CiFLTCON_BYTE filter_ctl;
  filter_ctl.byte = info.data.body.byte[0];
  filter_ctl.bF.Enable = 0;
  info.data.body.byte[0] = filter_ctl.byte;
  mcp2518_write_register(address, &info, 1);
  return 0;
}

static uint8_t mcp2518_link_filter_to_fifo(CAN_FILTER filter,
                                           CAN_FIFO_CHANNEL channel,
                                           uint8_t enable) {
  REG_CiFLTCON_BYTE filter_ctl;
  filter_ctl.byte = 0;
  filter_ctl.bF.Enable = enable ? 1 : 0;
  filter_ctl.bF.BufferPointer = channel;
  uint16_t address = cREGADDR_CiFLTCON + filter;
  union SpiRegister info = {0};
  info.data.body.byte[0] = filter_ctl.byte;
  mcp2518_write_register(address, &info, 1);

  return 0;
}

uint8_t mcp2518_config_filter() {
  uint16_t address;
  // disable filter 0
  mcp2518_disable_filter(CAN_FILTER0);

  // Configure filter 0 object and mask to match all
  CAN_FILTER filter = CAN_FILTER0;
  union SpiRegister info = {0};
  info.data.body.word = 0;
  address = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);
  mcp2518_write_register(address, &info, 4);
  info.data.body.word = 0;
  address = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);
  mcp2518_write_register(address, &info, 4);

  // link filter to the RX fifo, then enable the filter
  mcp2518_link_filter_to_fifo(CAN_FILTER0, CAN_FIFO_CH2, 1);

  return 0;
}

uint8_t mcp2518_change_mode(uint8_t mode) {
  // Set REQOP
  union SpiRegister info = {0};
  mcp2518_read_register(cREGADDR_CiCON + 3, &info, 1);
  info.data.body.byte[0] &= 0xf8;
  info.data.body.byte[0] |= mode & 0x7;
  mcp2518_write_register(cREGADDR_CiCON + 3, &info, 1);

  // Verify OPMOD
  uint8_t current;
  for (int i = 0; i < 100; ++i) {
    mcp2518_read_register(cREGADDR_CiCON + 2, &info, 1);
    current = info.data.body.byte[0] >> 5;
    if (current == mode) {
      return 0;
    }
  }
  fprintf(stderr, "Setting mode to %x failed; current=%x\n", mode, current);
  return 1;
}

static union SpiRxMessage *make_rx_message(can_message_t *message) {
  union SpiRxMessage *rx = (union SpiRxMessage *)(message->data - 10);
  return rx;
}

can_message_t *mcp2518_get_rx_message() {
  CAN_FIFO_CHANNEL channel = CAN_FIFO_CH2;

  union SpiRegister reg;
  // read interrupt flags
  uint16_t address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
  mcp2518_read_register(address, &reg, 1);
  CAN_RX_FIFO_EVENT flags = reg.data.body.byte[0] & CAN_RX_FIFO_ALL_EVENTS;

  if ((flags & CAN_RX_FIFO_NOT_EMPTY_EVENT) == 0) {
    // no more messages
    return NULL;
  }

  // Resolve the FIFO address
  uint16_t register_address = cREGADDR_CiFIFOUA + channel * CiFIFO_OFFSET;
  mcp2518_read_register(register_address, &reg, 2);
  uint16_t fifo_address = reg.data.body.word + cRAMADDR_START;

  // Fetch the FIFO data
  can_message_t *message = can_create_message();
  union SpiRxMessage *rx = make_rx_message(message);
  uint8_t bytes_to_read = PAYLOAD_SIZE + 10; // 10: header bytes
  rx->data.header[0] = MCP2518_READ | (fifo_address >> 8);
  rx->data.header[1] = fifo_address & 0xff;
  platform_write_spi(rx->buffer, bytes_to_read);

  // UINC channel
  register_address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
  reg.data.body.byte[0] = 0x1; // UINC
  mcp2518_write_register(register_address, &reg, 1);

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