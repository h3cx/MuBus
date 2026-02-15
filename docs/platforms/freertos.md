# FreeRTOS Runtime Guide

**Runtime prerequisite:** define `MUBUS_RUNTIME_FREERTOS` before including `MuBus.h`.

Related references:
- [Runtime selection](../runtime-selection.md)
- [`MuBusNode` API](../api/mubus-node.md)
- [Parser modes](../api/parser-modes.md)
- [Config reference](../api/config-types.md)
- [`MuTransport` API](../api/mu-transport.md)

## What is available in this runtime

| API surface | Available on FreeRTOS runtime? | Notes |
|---|---|---|
| `MuBusNode(MuTransport *transport, ...)` constructors | Yes | Primary integration path. |
| `begin(MuTransport *transport, ...)` overloads | Yes | Rebinds caller-provided transport pointer. |
| `MuBusNode(HardwareSerial*)` / `MuBusNode(mbed::BufferedSerial*)` | No | Runtime-specific serial overloads are not compiled in. |
| `startParserThread(...)` | Yes by default | Uses FreeRTOS task/event/semaphore APIs. |
| `poll()` / `tick()` | Yes | Cooperative parser mode when thread disabled/not used. |

## Transport setup (FreeRTOS runtime)

This runtime includes an optional ESP-IDF UART transport adapter for ESP32 targets (`src/transport/espidf_uart_transport.{h,cpp}`).

For other platforms, implement `MuTransport`:

```cpp
class UartTransport : public MuBus::MuTransport {
public:
  bool write(const uint8_t *data, size_t len) override;
  bool readByte(uint8_t &byte) override;
};

UartTransport uart;
MuBus::MuBusNode node(&uart, 0x01);
```


## ESP32 + ESP-IDF transport adapter

On ESP32 FreeRTOS/ESP-IDF builds, you can use `MuBus::EspIdfUartTransport`:

```cpp
#include <MuBus.h>
#include "transport/espidf_uart_transport.h"

MuBus::EspIdfUartTransportConfig cfg;
cfg.uart_num = UART_NUM_1;
cfg.tx_pin = GPIO_NUM_4;
cfg.rx_pin = GPIO_NUM_5;

MuBus::EspIdfUartTransport transport(cfg);
if (transport.begin()) {
  MuBus::MuBusNode node(&transport, 0x01);
  node.startParserThread(1, 200);
}
```

`readByte()` in this adapter is non-blocking via `uart_read_bytes(..., 1, 0)`, matching MuBus parser expectations.

## Minimal sender example

```cpp
#include <MuBus.h>

class UartTransport : public MuBus::MuTransport {
public:
  bool write(const uint8_t *data, size_t len) override { return uart_write(data, len); }
  bool readByte(uint8_t &byte) override { return uart_read_nonblocking(&byte, 1); }
};

UartTransport uart;
MuBus::MuBusNode node(&uart, 0x40);

void senderTask(void *) {
  static const uint8_t msg[] = {'p','i','n','g'};
  for (;;) {
    node.send(0x41, msg, sizeof(msg));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
```

## Minimal receiver example

```cpp
#include <MuBus.h>

extern UartTransport uart;
MuBus::MuBusNode node(&uart, 0x41);

void receiverTask(void *) {
  MuBus::MuBusNode::Frame frame;
  for (;;) {
    node.tick();
    if (node.receive(frame)) {
      // use frame
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
```

## Callback + threaded example

```cpp
#include <MuBus.h>

extern UartTransport uart;
MuBus::MuBusNode node(&uart, 0x42);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
  // callback from MuBus parser task context
  (void)frame;
}

void appTask(void *) {
  node.onFrame(onFrame);
  if (!node.startParserThread(1, 200)) {
    // fallback to polling if task create fails
    for (;;) {
      node.tick();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
```

## Polling example (no parser thread)

```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 4;

UartTransport uart;
MuBus::MuBusNode node;

void appTask(void *) {
  node.begin(&uart, 0x43, cfg);

  for (;;) {
    node.poll();
    MuBus::MuBusNode::Frame frame;
    while (node.receive(frame)) {
      // process frame
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
```

## Compile-time define in your main file

```cpp
#define MUBUS_RUNTIME_FREERTOS
#include <MuBus.h>
```

## Why an overload is missing

If serial constructors are missing on FreeRTOS builds, that is expected:
- `HardwareSerial*` overloads are Arduino-only.
- `mbed::BufferedSerial*` overloads are mbed-only.
- Use `MuTransport*` overloads and provide a transport wrapper.

## Copy-paste quickstart (FreeRTOS)

**Prerequisites:** working non-blocking UART read/write functions and FreeRTOS task scheduler running.

```cpp
#include <MuBus.h>

class UartTransport : public MuBus::MuTransport {
public:
  bool write(const uint8_t *data, size_t len) override { return uart_write(data, len); }
  bool readByte(uint8_t &byte) override { return uart_read_nonblocking(&byte, 1); }
};

UartTransport uart;
MuBus::MuBusNode node(&uart, 0x03);

extern "C" void app_main() {
  node.startParserThread(1, 200);

  static const uint8_t hello[] = "hello";
  for (;;) {
    node.send(0x04, hello, sizeof(hello) - 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
```

**Expected behavior:** sends "hello" every second to destination `0x04`; parser task continuously consumes incoming bytes.
