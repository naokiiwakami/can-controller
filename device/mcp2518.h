#pragma once

#include <stddef.h>
#include <stdint.h>

#include "can-controller/can_message.h"

// MCP2518 SPI Instructions
#define MCP2518_RESET 0x00      // C = 0b0000; A = 0x000
#define MCP2518_READ 0x30       // C = 0b0011; A; D = SDO
#define MCP2518_WRITE 0x20      // C = 0b0010; A; D = SDI
#define MCP2518_READ_CRC 0xb0   // C = 0b1011; A; N; D = SDO; CRC = SDO
#define MCP2518_WRITE_CRC 0xa0  // C = 0b1010; A; N; D = SDI; CRC = SDI
#define MCP2518_WRITE_SAFE 0xc0 // C = 0b1100; A; D = SDI; CRC = SDI

// MCP2518 Registers /////////////////////////////////////////
// LSB first

// MCP2518FD registers
#define OSC 0xe00
#define IOCON 0xe04
/*
#define CRC 0xe08
#define ECCCON 0xe0c
#define ECCSTAT 0xe10
#define DEVID 0xe14
*/

// CAN FD controller module registers
#define C1CON 0x000
#define C1NBTCFG 0x004
#define C1DBTCFG 0x008
#define C1TDC 0x00c
#define C1TBC 0x010
#define C1TSCON 0x014
#define C1VEC 0x018
#define C1INT 0x01c
#define C1RXIF 0x020
#define C1TXIF 0x024
#define C1RXOVIF 0x028
#define C1TXATIF 0x02c
#define C1TXREQ 0x030
#define C1TREC 0x034
#define C1BDIAG0 0x038
#define C1BDIAG1 0x03c
#define C1TEFCON 0x040
#define C1TEFSTA 0x044
#define C1TEFUA 0x048
// 0x04c reserved
#define C1TXQCON 0x050
#define C1TXQSTA 0x054
#define C1TXQUA 0x058

// FIFO control
#define C1FIFOCON1 0x05c
#define C1FIFOSTA1 0x060
#define C1FIFOUA1 0x064

#define C1FIFOCON2 0x068
#define C1FIFOSTA2 0x06c
#define C1FIFOUA2 0x070

// filter control
#define C1FLTCON0 0x1d0

// filter object
#define C1FLTOBJ0 0x1f0
#define C1MASK0 0x1f4

// Configuration values //////////////////////////////////////////////
