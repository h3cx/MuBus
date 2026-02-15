# mbed Runtime Guide

**Runtime prerequisite:** define `MUBUS_RUNTIME_MBED` before including `MuBus.h`.

Related references:
- [Runtime selection](../runtime-selection.md)
- [`MuBusNode` API](../api/mubus-node.md)
- [Parser modes](../api/parser-modes.md)
- [Config reference](../api/config-types.md)

## What is available in this runtime

| API surface | Available on mbed runtime? | Notes |
|---|---|---|
| `MuBusNode(mbed::BufferedSerial *port, ...)` constructors | Yes | Uses internal `MbedBufferedSerialTransport`. |
| `begin(mbed::BufferedSerial *port, ...)` overloads | Yes | Rebinds/owns new transport. |
| `MuBusNode(HardwareSerial*, ...)` | No | Arduino-only overloads. |
| `startParserThread(...)` | Yes by default | Requires `MUBUS_ENABLE_PARSER_THREAD`. |
| `poll()` / `tick()` | Yes | Works with or without parser thread. |

## Transport setup (mbed-only signatures)

```cpp
mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&port, 0x01);

bool ok = node.begin(&port, 0x01);
bool ok2 = node.begin(&port, 0x01, config);
```

## Minimal sender example

```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&port, 0x10);

int main() {
  while (true) {
    static const uint8_t msg[] = {'p','i','n','g'};
    (void)node.send(0x20, msg, sizeof(msg));
    ThisThread::sleep_for(500ms);
  }
}
```

## Minimal receiver example

```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&port, 0x20);

int main() {
  while (true) {
    node.tick();
    MuBus::MuBusNode::Frame frame;
    if (node.receive(frame)) {
      // inspect frame.src / frame.len / frame.payload
    }
    ThisThread::sleep_for(1ms);
  }
}
```

## Callback + threaded example

```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&port, 0x30);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
  // callback invoked from parser thread context
  (void)frame;
}

int main() {
  node.onFrame(onFrame);

  if (!node.startParserThread(1, 200)) {
    // fallback when thread start fails
    while (true) {
      node.tick();
      ThisThread::sleep_for(1ms);
    }
  }

  while (true) {
    ThisThread::sleep_for(100ms);
  }
}
```

## Polling example (thread disabled or explicit cooperative parsing)

```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 8;
cfg.tx_mode = MuBus::TxMode::Queue;
cfg.tx_queue_depth = 4;

mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node;

int main() {
  node.begin(&port, 0x31, cfg);

  while (true) {
    node.poll();
    MuBus::MuBusNode::Frame frame;
    while (node.receive(frame)) {
      // process queued frames
    }
    ThisThread::sleep_for(1ms);
  }
}
```

## Compile-time define in your main file

```cpp
#define MUBUS_RUNTIME_MBED
#include <MuBus.h>
```

## Why an overload is missing

If mbed overloads are not visible:
- Ensure only `MUBUS_RUNTIME_MBED` is defined.
- Confirm mbed headers are available in this target.
- If using a non-`BufferedSerial` driver, wrap it in a `MuTransport` implementation. See [MuTransport API](../api/mu-transport.md).

## Copy-paste quickstart (mbed)

**Prerequisites:** UART wiring to another MuBus node, RX/TX pins set correctly.

```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial bus(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&bus, 0x02);

int main() {
  node.startParserThread(1, 200);

  while (true) {
    static const uint8_t hello[] = "hello";
    node.send(0x01, hello, sizeof(hello) - 1);
    ThisThread::sleep_for(1s);
  }
}
```

**Expected behavior:** sends "hello" to node `0x01` every second while parser runs in background thread.
