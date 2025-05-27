# can-controller
Interface to CAN controllers

This library implements access to CAN drivers on different platforms.

Coverages of support are currently:

|              | mcp2515 |
| ------------ | :-----: |
| PSoC         |    v    |
| Raspberry Pi |    v    |

## Getting Started
The CAN bus configuration is fixed to 1 Mbps communication in this library.

### Hardware
#### CAN Controller
Set up the CAN controller. Following is the schematic of MCP2515 controller.

![mcp2515 controller](doc/images/mcp2515.svg)

Connect this controller to a CAN bus.

#### Connection to the Processor
Next, connect to the processor. The processor should have a set of SPI pins and a GPIO pin where pin down interrupt can be enabled.

##### Connection to PSoC
For PSoC, create a SPI Master component and an ISR that is connected by an input pin through an inverter. The names must be `SPIM_CAN` and `isr_RX0BF`. The controller is tested with clocks 8 MHz and 12 MHz (SPI clock 4 MHz and 6 MHz respectively). But the MCP2515 controller accepts SPI clock up to 10 MHz.

![mcp2515 controller](doc/images/psoc-components.png)

No special configuration is necessary for `SPIM_CAN`, but change ISR interrupt type to `RISING_EDGE` (which makes falling edge by combination with the inverter).

![mcp2515 controller](doc/images/psoc-isr-config.png)

##### Connection to Raspberry Pi
For Raspberry PI, use the primary SPI ports and GPIO 1 for receiving the interrupt signal.

![mcp2515 controller](doc/images/rpi-40pins.png)

### Software
#### API Setup
In the application code, call API method `can_init()` at the beginning of the program.

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

And implement the callback function that consumers received messages.
This callback method is invoked whenever the controller receives a message.
The following is an example "hello world" implementation of the consumer:

```c
void can_consume_rx_message(can_message_t *message) {
  printf("message received: ID=%d\n", message->id);
  can_free_message(message);
}
```

Note that this method is called by an interrupt handler, so the execution of the consumer should be quick.
Also, the application is responsible to release the `message`.  Use API method `can_free_message()` to release the memory.

There's an option to inject the callback function instead of implement one directly.
Compile the library with flag `-DSUPPORT_CALLBACK_INJECTION`, then the library
provides the following API function instead of `can_consumer_rx_message` callback.

```c
void can_set_rx_message_consumer(void (*consumer)(can_message_t *));
```

Call this method before `can_init()` to inject the incoming message consumer, for example:

```c
#include "can-controller/api.h"

static void notify(can_message_t *message) {
  printf("message received: ID=%d\n", message->id);
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

In order to send a CAN message, use API method `can_send_message()` such as:

```c
can_message_t *message = can_create_message();
message->id = 0x123;
message->is_extended = 0;
message->is_remote = 0;
message->data_length = 5;
memcpy(message->data, "hello", 5);
can_send_message(response);
```

Note that the message object must be created by API method `can_create_message()`.

#### Build Using CMake

The library provides `CMakeLists.txt` that makes a static link library.
The cmake takes the following variables:

| Variable Name              | Possible Values      | Required | Description                                   |
| -------------------------- | -------------------- | -------- | --------------------------------------------- |
| PLATFORM                   | raspberry-pi or psoc | yes      | Specifies the target platform                 |
| SUPPORT_CALLBACK_INGESTION | true or false        |          | Provides RX callback injection method if true |

Example of the cmake usage:

```
cmake -DPLATFORM=raspberry-pi -DSUPPORT_CALLBACK_INJECTION=true \
    -B build .
cmake --build build
```
