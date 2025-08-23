///////////////////////////////////////////////
// Platform Specific Implementations
///////////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

#include "can-controller/api.h"

// Raspberry Pi
//
#ifdef CONTROLLER_PLATFORM_RASPBERRY_PI
#include <semaphore.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

void platform_sleep_ms(uint32_t milliseconds) { usleep(milliseconds * 1000); }

// SPI channel (0 for /dev/spidev0.0)
#define SPI_CHANNEL 0

#if defined(MCP2518FD) || defined(MCP2517FD)
// 16MHz, The device accepts up to 20MHz but frequent communication errors
// occur with the rate.
#define SPI_SPEED 16000000
#else
// 10MHz
#define SPI_SPEED 10000000
#endif

// Use GPIO 1 pin (physical 28 pin) on the 40-pin Raspberry Pi.
// See
// https://github.com/WiringPi/WiringPi/blob/master/documentation/english/functions.md
// for Wiring Pi pin number configuration.
#define INTERRUPT_PIN 31

int platform_init_spi() {
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

int platform_init_rx_interrupt() {
  // Enable pin down interrupt. Connect the RX0BF (11) pin on the MCP2515 chip
  // to the GPIO 1 pin (physical 28 pin) on the 40-pin Raspberry Pi.
  if (wiringPiISR(INTERRUPT_PIN, INT_EDGE_FALLING, handle_rx)) {
    fprintf(stderr, "ISR setup failed\n");
    return 1;
  }
  return 0;
}

void platform_write_spi(uint8_t *buffer, size_t length) {
  wiringPiSPIDataRW(SPI_CHANNEL, buffer, length);
}

#endif // CONTROLLER_PLATFORM == raspberry_pi

#ifdef CONTROLLER_PLATFORM_PSOC

#include "project.h"

void platform_sleep_ms(uint32_t milliseconds) { CyDelay(milliseconds); }

int platform_init_spi() {
  SPIM_CAN_Start();
  return 0;
}

CY_ISR(ISR_RX0BF) { handle_rx(); }

int platform_init_rx_interrupt() {
  // Enable interrupt to detect message received. Connect the RX0BF (11) pin
  // on the MCP2515 chip to a pin on PSoC that is connected to an ISR module
  // through an inverter. The name must be isr_RX0BF with type rising edge.
  isr_RX0BF_ClearPending();
  isr_RX0BF_StartEx(ISR_RX0BF);
  return 0;
}

void platform_write_spi(uint8_t *buffer, size_t length) {
  while (SPIM_CAN_GetRxBufferSize()) {
    SPIM_CAN_ReadRxData();
  }
  while (0u == (SPIM_CAN_TX_STATUS_REG & SPIM_CAN_STS_TX_FIFO_EMPTY)) {
  }

  /* Put data elements into the TX FIFO and get data elements from the RX fifo
   */
  size_t index_write = 0;
  size_t index_read = 0;
  while (index_read < length) {
    if (index_write < length &&
        (SPIM_CAN_TX_STATUS_REG & SPIM_CAN_STS_TX_FIFO_NOT_FULL)) {
      CY_SET_REG8(SPIM_CAN_TXDATA_PTR, buffer[index_write++]);
    }
    if (SPIM_CAN_RX_STATUS_REG & SPIM_CAN_STS_RX_FIFO_NOT_EMPTY) {
      buffer[index_read++] = CY_GET_REG8(SPIM_CAN_RXDATA_PTR);
    }
  }
}

#endif // CONTROLLER_PLATFORM == raspberry_pi
