#include "can-controller/device/mcp2515.h"

#include <alloca.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "can-controller/api.h"
#include "can-controller/can_message.h"

inline static void platform_write_spi(uint8_t *buffer, size_t length);
static int platform_init_mcp2515_spi();
static int platform_init_mcp2515_interrupt();

static void mcp2515_configure_1meg_bps();
static void mcp2515_configure_receive_buffer_0();
static void mcp2515_configure_receive_buffer_1();
static void mcp2515_configure_RXnBF_pins();
static void mcp2515_handle_rx();

uint8_t mcp2515_init() {
  if (platform_init_mcp2515_spi()) {
    return 1;
  }

  mcp2515_reset();
  // Set MCP2515 to Configuration mode
  mcp2515_write_register(CANCTRL, 0x87);

  mcp2515_configure_1meg_bps();
  mcp2515_configure_receive_buffer_0();
  mcp2515_configure_receive_buffer_1();
  mcp2515_configure_RXnBF_pins();

  // The interrupts are triggered by the RX0BF (11) pin on the MCP2515 chip.
  if (platform_init_mcp2515_interrupt()) {
    return 1;
  }

  // set to normal mode
  mcp2515_bit_modify(CANCTRL, OP_MODE_MASK, OP_MODE_NORMAL);

  return 0;
}

void mcp2515_reset() {
  uint8_t buf[1] = {MCP_RESET};
  platform_write_spi(buf, 1);
  usleep(10000);  // Wait 10ms for reset
}

void mcp2515_configure_1meg_bps() {
  // Bit timing for 1Mbps, 20MHz, SAM=1
  // CNF1: SJW=1, BRP=0 (0x00)
  // CNF2: BTLMODE=1, SAM=1, PHSEG1=3, PRSEG=2 (0xD1)
  // CNF3: PHSEG2=3 (0x03)
  mcp2515_write_register(CNF1, 0x00);
  mcp2515_write_register(CNF2, 0xD1);
  mcp2515_write_register(CNF3, 0x03);
}

void mcp2515_configure_receive_buffer_0() {
#define RXB0CTRL_RXM 0b11  // catch 'em all
#define RXB0CTRL_RXM_BIT \
  5  // bit 6-5 RXM<1:0>: R/W-00: Receive Buffer Operating mode bits
     //         11 = Turn mask/filters off; receive any message
     //         10 = Receive only valid messages with extended identifiers that
     //         meet filter criteria 01 = Receive only valid messages with
     //         standard identifiers that meet filter criteria. Extended ID
     //         filter
     //           registers RXFnEID8:RXFnEID0 are ignored for the messages with
     //           standard IDs.
     //         00 = Receive all valid messages using either standard or
     //         extended identifiers that meet filter criteria.
     //           Extended ID filter registers RXFnEID8:RXFnEID0 are applied to
     //           first two bytes of data in the messages with standard IDs.
#define RXB0CTRL_RXRTR_BIT \
  3  // bit 3   RXRTR: R-0: Received Remote Transfer Request bit
     //         1 = Remote Transfer Request Received
     //         0 = No Remote Transfer Request Received
#define RXB0CTRL_BUKT 0
#define RXB0CTRL_BUKT_BIT \
  2  // bit 2   BUKT: R/W-0: Rollover Enable bit
     //         1 = RXB0 message will rollover and be written to RXB1 if RXB0 is
     //         full 0 = Rollover disabled
#define RXB0CTRL_BUKT1_BIT \
  1  // bit 1   BUKT1: R-0: Read-only Copy of BUKT bit (used internally bu the
     // MCP2515)
#define RXB0CTRL_FILHIT0_BIT \
  0  // bit 0   FILHIT0: R-0: Filter Hit bit – indicates which acceptance filter
     // enabled reception of message
     //         1 = Acceptance Filter 1 (RXF1)
     //         0 = Acceptance Filter 0 (RXF0)
  uint8_t value =
      (RXB0CTRL_RXM << RXB0CTRL_RXM_BIT) + (RXB0CTRL_BUKT << RXB0CTRL_BUKT_BIT);
  mcp2515_write_register(RXB0CTRL, value);
}

void mcp2515_configure_receive_buffer_1() {
#define RXB1CTRL_RXM 0b10
#define RXB1CTRL_RXM_BIT \
  5  // bit 6-5 RXM<1:0>: R/W-00: Receive Buffer Operating mode bits
     //         11 = Turn mask/filters off; receive any message
     //         10 = Receive only valid messages with extended identifiers that
     //         meet filter criteria 01 = Receive only valid messages with
     //         standard identifiers that meet filter criteria 00 = Receive all
     //         valid messages using either standard or extended identifiers
     //         that meet filter criteria
#define RXB1CTRL_RXRTR_BIT \
  3  // bit 3   RXRTR: R-0: Received Remote Transfer Request bit
     //         1 = Remote Transfer Request Received
     //         0 = No Remote Transfer Request Received
#define RXB1CTRL_FILHIT_BIT \
  0  // bit 2-0 FILHIT<2:0>: R-0: Filter Hit bits - indicates which acceptance
     // filter enabled reception of message
     //         101 = Acceptance Filter 5 (RXF5)
     //         100 = Acceptance Filter 4 (RXF4)
     //         011 = Acceptance Filter 3 (RXF3)
     //         010 = Acceptance Filter 2 (RXF2)
     //         001 = Acceptance Filter 1 (RXF1) (Only if BUKT bit set in
     //         RXB0CTRL) 000 = Acceptance Filter 0 (RXF0) (Only if BUKT bit set
     //         in RXB0CTRL)
  uint8_t value = (RXB1CTRL_RXM << RXB1CTRL_RXM_BIT);
  mcp2515_write_register(RXB1CTRL, value);
}

void mcp2515_configure_RXnBF_pins() {
#define BFPCTRL_B1BFS 0
#define BFPCTRL_B1BFS_BIT \
  5  // bit 5 B1BFS: R/W-0: RX1BF Pin State bit (Digital Output mode only)
     //       - Reads as ‘0’ when RX1BF is configured as interrupt pin
#define BFPCTRL_B0BFS 0
#define BFPCTRL_B0BFS_BIT \
  4  // bit 4 B0BFS: R/W-0: RX0BF Pin State bit (Digital Output mode only)
     //       - Reads as ‘0’ when RX0BF is configured as interrupt pin
#define BFPCTRL_B1BFE 0
#define BFPCTRL_B1BFE_BIT \
  3  // bit 3 B1BFE: R/W-0: RX1BF Pin Function Enable bit
     //       1 = Pin function enabled, operation mode determined by B1BFM bit
     //       0 = Pin function disabled, pin goes to high-impedance state
#define BFPCTRL_B0BFE 1
#define BFPCTRL_B0BFE_BIT \
  2  // bit 2 B0BFE: R/W-0: RX0BF Pin Function Enable bit
     //       1 = Pin function enabled, operation mode determined by B0BFM bit
     //       0 = Pin function disabled, pin goes to high-impedance state
#define BFPCTRL_B1BFM 0
#define BFPCTRL_B1BFM_BIT \
  1  // bit 1 B1BFM: R/W-0: RX1BF Pin Operation mode bit
     //       1 = Pin is used as interrupt when valid message loaded into RXB1
     //       0 = Digital Output mode
#define BFPCTRL_B0BFM 1
#define BFPCTRL_B0BFM_BIT \
  0  // bit 0 B0BFM: R/W-0: RX0BF Pin Operation mode bit
     //       1 = Pin is used as interrupt when valid message loaded into RXB0
     //       0 = Digital Output mode
  uint8_t value = (BFPCTRL_B1BFS << BFPCTRL_B1BFS_BIT) +
                  (BFPCTRL_B0BFS << BFPCTRL_B0BFS_BIT) +
                  (BFPCTRL_B1BFE << BFPCTRL_B1BFE_BIT) +
                  (BFPCTRL_B0BFE << BFPCTRL_B0BFE_BIT) +
                  (BFPCTRL_B1BFM << BFPCTRL_B1BFM_BIT) +
                  (BFPCTRL_B0BFM << BFPCTRL_B0BFM_BIT);
  mcp2515_write_register(BFPCTRL, value);
}

uint8_t *mcp2515_read(uint8_t address, uint8_t *buffer, size_t length) {
  buffer[0] = MCP_READ;
  buffer[1] = address;
  platform_write_spi(buffer, length + 2);
  return buffer + 2;
}

void mcp2515_write_register(uint8_t address, uint8_t value) {
  uint8_t buf[3] = {MCP_WRITE, address, value};
  platform_write_spi(buf, 3);
}

uint8_t mcp2515_read_register(uint8_t address) {
  uint8_t buf[3] = {MCP_READ, address, 0x00};
  return mcp2515_read(address, buf, 1)[0];
}

void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data) {
  uint8_t buf[4] = {MCP_BIT_MODIFY, address, mask, data};
  platform_write_spi(buf, 4);
}

static void mcp2515_parse_rx_message() {
  // auto *message = queue_reserve_item();
  can_message_t *message = can_create_message();
  uint8_t buffer[16];
  uint8_t *out_buffer = mcp2515_read(RXB0SIDH, buffer, 13);
  uint16_t sid =
      ((uint16_t)out_buffer[RXBnSIDH]) << 3 | out_buffer[RXBnSIDL] >> 5;
  message->is_extended = out_buffer[RXBnSIDL] & (1 << RXBnSIDL_IDE_BIT);
  if (message->is_extended) {
    uint32_t eid = out_buffer[RXBnSIDL] & 0x3;
    eid <<= 8;
    eid |= out_buffer[RXBnEID8];
    eid <<= 8;
    eid |= out_buffer[RXBnEID0];
    eid |= sid << 18;
    message->id = eid;
    message->is_remote = out_buffer[RXBnDLC] & (1 << RXBnDLC_RTR_BIT);
  } else {
    message->id = sid;
    message->is_remote = out_buffer[RXBnSIDL] & (1 << RXBnSIDL_SRR_BIT);
  }
  message->data_length = out_buffer[RXBnDLC] & RXBnDLC_DLC_MASK;
  if (!message->is_remote && message->data_length > 0) {
    // TODO: Eliminate this copy
    memcpy(message->data, out_buffer + RXBnD0, message->data_length);
  }
  can_consume_rx_message(message);
}

void mcp2515_handle_rx() {
  mcp2515_bit_modify(CANINTF, (1 << CANINTF_RX0IF_BIT), 0);
  mcp2515_parse_rx_message();
}

int mcp2515_set_can_id_std(uint8_t *buffer, uint16_t id, uint8_t data_length) {
  int index = 2;  // skip instruction and address
  //               7     6     5     4     3     2     1     0
  // TXBnSIDH: SID10  SID9  SID8  SID7  SID6  SID5  SID4  SID3
  // TXBnSIDL:  SID2  SID1  SID0   -   EXIDE   -   EID17 EID16
  id <<= 5;
  buffer[index + 1] = (uint8_t)id;  // TXBnSIDH
  id >>= 8;
  buffer[index] = (uint8_t)id;  // TXBnSIDL

  index += 2;
  buffer[index++] = 0;  // TXBnEID8
  buffer[index++] = 0;  // TXBnEID0

  // TXBnDLC :   -    RTR     -    -   DLC3  DLC2  DLC1  DLC0
  buffer[index++] = data_length & 0xf;  // TXBnDLC

  return index;
}

void mcp2515_message_request_to_send_txb0(uint8_t *buffer,
                                          size_t buffer_length) {
  // put the instruction and the address
  buffer[0] = MCP_WRITE;
  buffer[1] = TXB0SIDH;
  platform_write_spi(buffer, buffer_length);
  uint8_t internal_buf[1] = {MCP_RTS_TXB0};
  platform_write_spi(internal_buf, 1);
}

void mcp2515_send_message(can_message_t *message) {
  uint8_t local_buf[16];
  int index =
      mcp2515_set_can_id_std(local_buf, message->id, message->data_length);
  memcpy(local_buf + index, message->data, message->data_length);
  mcp2515_message_request_to_send_txb0(local_buf, index);
  can_free_message(message);
}

///////////////////////////////////////////////
// Platform Specific Implementations
///////////////////////////////////////////////

// Raspberry Pi
//
#if CONTROLLER_PLATFORM == raspberry_pi
#include <wiringPi.h>
#include <wiringPiSPI.h>

// SPI channel (0 for /dev/spidev0.0)
#define SPI_CHANNEL 0

#define SPI_SPEED 10000000  // 10MHz

// Use GPIO 1 pin (physical 28 pin) on the 40-pin Raspberry Pi.
// See
// https://github.com/WiringPi/WiringPi/blob/master/documentation/english/functions.md
// for Wiring Pi pin number configuration.
#define INTERRUPT_PIN 31

int platform_init_mcp2515_spi() {
  if (wiringPiSetup() == -1) {
    fprintf(stderr, "WiringPi setup failed\n");
    return 1;
  }
  if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
    fprintf(stderr, "SPI setup failed\n");
    return 1;
  }
  return 0;
}

int platform_init_mcp2515_interrupt() {
  // Enable pin down interrupt. Connect the RX0BF (11) pin on the MCP2515 chip
  // to the GPIO 1 pin (physical 28 pin) on the 40-pin Raspberry Pi.
  if (wiringPiISR(INTERRUPT_PIN, INT_EDGE_FALLING, mcp2515_handle_rx)) {
    fprintf(stderr, "ISR setup failed\n");
    return 1;
  }
  return 0;
}

inline void platform_write_spi(uint8_t *buffer, size_t length) {
  wiringPiSPIDataRW(SPI_CHANNEL, buffer, length);
}

#endif  // CONTROLLER_PLATFORM == raspberry_pi
