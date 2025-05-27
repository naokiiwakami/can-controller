# can-controller
Interface to CAN controllers

This library provides access to CAN drivers from various platforms.

Coverages of support are currently:

|              | mcp2515 |
| ------------ | :-----: |
| PSoC         |    v    |
| Raspberry Pi |    v    |

## Getting Started
The CAN bus speed is configured to 1 Mbps in this library.

### Hardware
#### CAN Controller
Set up the CAN controller. Following is the schematic of the MCP2515 controller.

![mcp2515 controller](doc/images/mcp2515.svg)

Connect this controller to a CAN bus.

#### Connection to the Processor
Next, connect to the processor. The processor should have a set of SPI pins and a GPIO pin where pin down interrupt is available.

##### Connection to PSoC
For PSoC, create components SPI Master component and ISR. The ISR should be connected to an input pin through an inverter. The names must be `SPIM_CAN` and `isr_RX0BF`. The controller is tested with clocks at 8 MHz and 12 MHz (they make the SPI clocks 4 MHz and 6 MHz respectively). The MCP2515 controller accepts SPI clock up to 10 MHz, so you may be able to increase the clock frequency if the processor can make it (untested though).

![mcp2515 controller](doc/images/psoc-components.png)

No special configuration is necessary for `SPIM_CAN`, but change ISR interrupt type to `RISING_EDGE` (which makes falling edge by combination with the inverter). The component does not support `FALLING_EDGE` interrupt, so put an inverter between the ISR and the pin.

![mcp2515 controller](doc/images/psoc-isr-config.png)

##### Connection to Raspberry Pi
For Raspberry PI, use the primary SPI ports and GPIO 1 for receiving the interrupt signal.  Use CE0 for the chip select.

![mcp2515 controller](doc/images/rpi-40pins.png)

### Software
#### API Setup
Call API function `can_init()` at the beggining of the application in order to enable this library.

```c
#include "can-controller/api.h"

int main() {
  if (can_init()) {
    return -1;
  }

  // put your application code here

  return 0;
}
```

It is also necessary to implement the callback function that consumes received messages.
This callback function is invoked whenever the controller receives a message.
The following is an example "hello world" implementation of the consumer:

```c
void can_consume_rx_message(can_message_t *message) {
  printf("message received: ID=%d\n", message->id);
  can_free_message(message);
}
```

Note that this function is called by an interrupt handler, so the execution of the consumer should be quick.
Also, the application is responsible to invalidate the `message` after it is utilized.
Use API function `can_free_message()`.

There's an option to inject the callback function instead of implementing the function `can_consume_rx_message()`.
Compile the library with flag `-DSUPPORT_CALLBACK_INJECTION`, then the library
provides the following API function instead of `can_consumer_rx_message` callback.

```c
void can_set_rx_message_consumer(void (*consumer)(can_message_t *));
```

Call this function before `can_init()` to inject the incoming message consumer, for example:

```c
#include "can-controller/api.h"

static void notify(can_message_t *message) {
  printf("message received: ID=%08x\n", message->id);
  can_free_message(message);
}

int main() {
  can_set_rx_message_consumer(notify);
  if (can_init()) {
    return -1;
  }

  // put your application code here

  return 0;
}
```

In order to send a CAN message, use API function `can_send_message()` such as:

```c
can_message_t *message = can_create_message();
message->id = 0x123;
message->is_extended = 0;
message->is_remote = 0;
message->data_length = 5;
memcpy(message->data, "hello", 5);
can_send_message(response);
```

Note that the message object must be created by API function `can_create_message()`.

#### Data Types
Type `can_message_t` carries a CAN message. It is a structure defined a the following:

```c
typedef struct can_message {
  uint32_t id;
  uint8_t is_extended;
  uint8_t is_remote;
  uint8_t data_length;
  uint8_t data[8];
} can_message_t;
```

Since this library is made with efficiency for embedded system in mind, there is a restriction
in using this type.
When you exchange a message by this data type via the API functions, always create or free
it using the API functions `can_create_message()` and `can_free_message()`.  The system `malloc()`
or `free()` must not be used.  An object as a local variable must not be used, too.

#### Build Using CMake

The library provides `CMakeLists.txt` that makes a static link library.
The cmake takes the following variables:

| Variable Name              | Possible Values      | Required | Description                                     |
| -------------------------- | -------------------- | -------- | ----------------------------------------------- |
| PLATFORM                   | raspberry-pi or psoc | yes      | Specifies the target platform                   |
| SUPPORT_CALLBACK_INGESTION | true or false        |          | Provides RX callback injection function if true |

Example of the cmake usage:

```
cmake -DPLATFORM=raspberry-pi -DSUPPORT_CALLBACK_INJECTION=true \
    -B build .
cmake --build build
```
