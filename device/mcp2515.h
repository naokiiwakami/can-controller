#pragma once

#include <stddef.h>
#include <stdint.h>

#include "can-controller/can_message.h"

// MCP2515 SPI Commands
#define MCP2515_RESET 0xc0           // 1100 0000
#define MCP2515_READ 0x03            // 0000 0011
#define MCP2515_READ_RX_BUFFER 0x90  // 1001 0nm0
#define MCP2515_WRITE 0x02           // 0000 0010
#define MCP2515_LOAD_TX_BUFFER 0x40  // 0100 0abc
#define MCP2515_RTS_TXB0 0x81        // 1000 0001
#define MCP2515_RTS_TXB1 0x82        // 1000 0010
#define MCP2515_RTS_TXB2 0x84        // 1000 0100
#define MCP2515_READ_STATUS 0xa0     // 1010 0000
#define MCP2515_RX_STATUS 0xb0       // 1011 0000
#define MCP2515_BIT_MODIFY 0x05      // 0000 0101

// MCP2515 Registers /////////////////////////////////////////

// higher-order address = 0
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0a
#define RXF2EID0 0x0b
#define BFPCTRL 0x0c
#define TXRTSCTRL 0x0d
#define CANSTAT 0x0e
#define CANCTRL 0x0f

// higher-order address = 1
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1a
#define RXF5EID0 0x1b
#define TEC 0x1c
#define REC 0x1d
#define CANSTAT1 0x1e
#define CANCTRL1 0x1f

// higher-order address = 2
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2a
#define CANINTE 0x2b
#define CANINTF 0x2c
#define EFLG 0x2d
#define CANSTAT2 0x2e
#define CANCTRL2 0x2f

// register addresses (TXB0)
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
#define TXB0D0 0x36
#define TXB0D1 0x37
#define TXB0D2 0x38
#define TXB0D3 0x39
#define TXB0D4 0x3a
#define TXB0D5 0x3b
#define TXB0D6 0x3c
#define TXB0D7 0x3d
#define CANSTAT3 0x3e
#define CANCTRL3 0x3f

// register addresses (TXB1)
#define TXB1CTRL 0x40
#define TXB1SIDH 0x41
#define TXB1SIDL 0x42
#define TXB1EID8 0x43
#define TXB1EID0 0x44
#define TXB1DLC 0x45
#define TXB1D0 0x46
#define TXB1D1 0x47
#define TXB1D2 0x48
#define TXB1D3 0x49
#define TXB1D4 0x4a
#define TXB1D5 0x4b
#define TXB1D6 0x4c
#define TXB1D7 0x4d
#define CANSTAT4 0x4e
#define CANCTRL4 0x4f

// register addresses (TXB2)
#define TXB2CTRL 0x50
#define TXB2SIDH 0x51
#define TXB2SIDL 0x52
#define TXB2EID8 0x53
#define TXB2EID0 0x54
#define TXB2DLC 0x55
#define TXB2D0 0x56
#define TXB2D1 0x57
#define TXB2D2 0x58
#define TXB2D3 0x59
#define TXB2D4 0x5a
#define TXB2D5 0x5b
#define TXB2D6 0x5c
#define TXB2D7 0x5d
#define CANSTAT5 0x5e
#define CANCTRL5 0x5f

// register addresses (RXB0)
#define RXB0CTRL 0x60
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66
#define RXB0D1 0x67
#define RXB0D2 0x68
#define RXB0D3 0x69
#define RXB0D4 0x6a
#define RXB0D5 0x6b
#define RXB0D6 0x6c
#define RXB0D7 0x6d
#define CANSTAT6 0x6e
#define CANCTRL6 0x6f

// register addresses (RXB1)
#define RXB1CTRL 0x70
#define RXB1SIDH 0x71
#define RXB1SIDL 0x72
#define RXB1EID8 0x73
#define RXB1EID0 0x74
#define RXB1DLC 0x75
#define RXB1D0 0x76
#define RXB1D1 0x77
#define RXB1D2 0x78
#define RXB1D3 0x79
#define RXB1D4 0x7a
#define RXB1D5 0x7b
#define RXB1D6 0x7c
#define RXB1D7 0x7d
#define CANSTAT7 0x7e
#define CANCTRL7 0x7f

// RXBn register offsets starting from SIDH
#define RXBnSIDH 0
#define RXBnSIDL 1
#define RXBnEID8 2
#define RXBnEID0 3
#define RXBnDLC 4
#define RXBnD0 5
#define RXBnD1 6
#define RXBnD2 7
#define RXBnD3 8
#define RXBnD4 9
#define RXBnD5 10
#define RXBnD6 11
#define RXBnD7 12

// RXBn offset bits
#define RXBnSIDL_SRR_BIT 4
#define RXBnSIDL_IDE_BIT 3
#define RXBnDLC_RTR_BIT 6
#define RXBnDLC_DLC_MASK 0x0f

#define CANINTF_RX0IF_BIT 0
#define CANINTF_RX1IF_BIT 1
#define CANINTF_TX0IF_BIT 2
#define CANINTF_TX1IF_BIT 3
#define CANINTF_TX2IF_BIT 4
#define CANINTF_ERRIF_BIT 5
#define CANINTF_WAKIF_BIT 6
#define CANINTF_MERRF_BIT 7

// Operation modes
#define OP_MODE_NORMAL (0b000 << 5)
#define OP_MODE_SLEEP (0b001 << 5)
#define OP_MODE_LOOPBACK (0b010 << 5)
#define OP_MODE_LISTEN_ONLY (0b011 << 5)
#define OP_MODE_CONFIGURATION (0b100 << 5)
#define OP_MODE_MASK 0xe0
