# Arduino Runtime Guide

**Runtime prerequisite:** compile with `-DMUBUS_RUNTIME_ARDUINO`.

Related references:
- [Runtime selection](../runtime-selection.md)
- [`MuBusNode` API](../api/mubus-node.md)
- [Parser modes](../api/parser-modes.md)
- [Config reference](../api/config-types.md)

## What is available in this runtime

| API surface | Available on Arduino runtime? | Notes |
|---|---|---|
| `MuBusNode(HardwareSerial *port, ...)` constructors | Yes | Uses internal `ArduinoSerialTransport`. |
| `begin(HardwareSerial *port, ...)` overloads | Yes | Rebinds/owns new transport. |
| `MuBusNode(mbed::BufferedSerial*, ...)` | No | mbed-only overloads. |
| `startParserThread(...)` | Typically no | Returns `false` when parser thread feature is disabled (default on Arduino). |
| `poll()` / `tick()` | Yes | Primary parser mode for loop-based firmware. |

## Transport setup (Arduino-only signatures)

Use exactly these Arduino signatures:

```cpp
MuBus::MuBusNode node(&Serial1, 0x01);
bool ok = node.begin(&Serial1, 0x01);
bool ok2 = node.begin(&Serial1, 0x01, config);
```

`HardwareSerial` naming is used consistently in this runtime.

## Minimal sender example

```cpp
#include <Arduino.h>
#include <MuBus.h>

constexpr uint32_t kBaud = 115200;
MuBus::MuBusNode node(&Serial1, 0x11);

void setup() {
  Serial1.begin(kBaud);
}

void loop() {
  static const uint8_t msg[] = {'p','i','n','g'};
  (void)node.send(0x22, msg, sizeof(msg));
  delay(500);
}
```

## Minimal receiver example

```cpp
#include <Arduino.h>
#include <MuBus.h>

constexpr uint32_t kBaud = 115200;
MuBus::MuBusNode node(&Serial1, 0x22);

void setup() {
  Serial.begin(kBaud);
  Serial1.begin(kBaud);
}

void loop() {
  node.tick();
  MuBus::MuBusNode::Frame frame;
  if (node.receive(frame)) {
    Serial.print("from="); Serial.print(frame.src);
    Serial.print(" len="); Serial.println(frame.len);
  }
}
```

## Callback example (cooperative callback dispatch)

Arduino runtime is usually non-threaded. Callback dispatch still works when `tick()` parses frames.

```cpp
#include <Arduino.h>
#include <MuBus.h>

MuBus::MuBusNode node(&Serial1, 0x33);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
  Serial.print("callback bytes=");
  Serial.println(frame.len);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  node.onFrame(onFrame);
}

void loop() {
  node.tick();
}
```

## Polling example for non-threaded loops

```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 4;

MuBus::MuBusNode node;

void setup() {
  Serial1.begin(115200);
  node.begin(&Serial1, 0x44, cfg);
}

void loop() {
  node.poll();
  while (node.available()) {
    MuBus::MuBusNode::Frame frame;
    if (node.receive(frame)) {
      // handle frame
    }
  }
}
```

## Build snippet (PlatformIO)

```ini
[env:arduino_example]
platform = atmelavr
board = uno
framework = arduino
build_flags =
  -DMUBUS_RUNTIME_ARDUINO
  -DMUBUS_ENABLE_PARSER_THREAD=0
```

## Why an overload is missing

If you see missing constructor/begin overloads:
- Ensure only `MUBUS_RUNTIME_ARDUINO` is defined.
- Do not use `mbed::BufferedSerial` signatures in Arduino builds.
- If you need a custom bus backend, use `MuTransport*` APIs instead. See [`MuTransport` reference](../api/mu-transport.md).

## Copy-paste quickstart (Arduino baremetal)

**Prerequisites:** two MuBus nodes wired on UART, same baud, this node address `0x01`.

```cpp
#include <Arduino.h>
#include <MuBus.h>

MuBus::MuBusNode node(&Serial1, 0x01);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  node.tick();

  static uint32_t last = 0;
  if (millis() - last > 1000) {
    last = millis();
    static const uint8_t hello[] = "hello";
    node.send(0x02, hello, sizeof(hello) - 1);
  }

  MuBus::MuBusNode::Frame frame;
  if (node.receive(frame)) {
    Serial.println("received frame");
  }
}
```

**Expected behavior:** sends "hello" every second to `0x02`; prints when incoming frames arrive.
