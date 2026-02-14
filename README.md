<p align="center">
  <img src="https://raw.githubusercontent.com/h3cx/MuBus/main/images/mubus.png"
       width="30%" />
</p>
A high performance, low overhead serial communication protocol

> [!IMPORTANT]
> μBus currently supports Arduino-compatible targets with `HardwareSerial`, and mbed targets with `mbed::BufferedSerial`.

# Installation
μBus is currently aimed at PlatformIO-based workflows.

```sh
pio pkg install -l "https://github.com/h3cx/MuBus.git"
```

> [!IMPORTANT]
> Expect regular breaking changes during initial development.

# Quick start
## Arduino (cooperative parser via `tick()`)
```cpp
#include <Arduino.h>
#include <MuBus.h>

constexpr uint32_t kBaud = 115200;
constexpr uint8_t kNodeAddr = 0x01;

MuBus::MuBusNode node(&Serial1, kNodeAddr);

void setup() {
  Serial.begin(kBaud);
  Serial1.begin(kBaud);
}

void loop() {
  node.tick(); // parser work on non-RTOS targets

  MuBus::MuBusNode::Frame frame;
  if (node.receive(frame)) {
    // use frame.src, frame.dst, frame.len, frame.payload
  }
}
```

## mbed (worker thread parser)
```cpp
#include <mbed.h>
#include <MuBus.h>

constexpr uint8_t kNodeAddr = 0x02;

mbed::BufferedSerial bus_port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&bus_port, kNodeAddr);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
  // consume frame here
}

int main() {
  node.onFrame(onFrame);

  // parser runs in dedicated RTOS thread on mbed targets
  if (!node.startParserThread(1, 200)) {
    // fallback if needed
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

# Protocol specification
## Byte layout
All multibyte protocol fields use **little-endian** encoding.

| Offset | Size | Field | Notes |
|---|---:|---|---|
| 0 | 1 | `SYNC0` | fixed `0xD3` |
| 1 | 1 | `SYNC1` | fixed `0x91` |
| 2 | 1 | `SRC` | source node address |
| 3 | 1 | `DST` | destination address (`0x00` broadcast, `0xFF` reserved) |
| 4 | 1 | `LEN_L` | payload length low byte |
| 5 | 1 | `LEN_H` | payload length high byte |
| 6.. | `LEN` | `PAYLOAD` | raw payload bytes |
| tail | 0/1/2 | `CRC` | omitted for `CrcMode::None`, 1 byte for `Crc8`, 2 bytes for `Crc16` |

Length decode formula:

```text
LEN = LEN_L | (LEN_H << 8)
```

Maximum payload size is 506 bytes, keeping the base header+payload frame bounded to 512 bytes.

## Address semantics
- **Unicast:** `0x01` to `0xFE`
- **Broadcast:** `0x00`
- **Reserved:** `0xFF` (do not assign as a node address)

## Destination filter modes
Configure with `MuBusConfig::destination_filter_mode`:
- `AddressedOrBroadcast` (default)
- `AcceptBroadcastOnly`
- `AcceptUnicastOnly`
- `Promiscuous`

# Configuration matrix
Use `MuBusConfig` to pick queueing and parser execution model.

| Dimension | Option | Behavior | Typical use |
|---|---|---|---|
| RX mode | `RxMode::SingleSlot` | Only most-recent pending frame storage (`rx_queue_depth` forced to 1) | Lowest RAM on small MCUs |
| RX mode | `RxMode::Ring` | Queue of up to `rx_queue_depth` frames | Bursty traffic, slower consumers |
| TX mode | `TxMode::Direct` | Write immediately (`tx_queue_depth` forced to 1) | Lowest latency and RAM |
| TX mode | `TxMode::Queue` | Queue up to `tx_queue_depth` frames before flush | Producer/consumer decoupling |
| Parser execution | `startParserThread(...)` | Dedicated parser worker thread (mbed RTOS targets) | Background parsing with callback flow |
| Parser execution | `tick()` / `poll()` | Cooperative parsing in main loop | Arduino / no RTOS |

# RAM and CPU tradeoff notes
- **Single-slot + direct TX** minimizes memory use and is usually best for simple request/response loops.
- **Ring RX and/or queued TX** increases RAM roughly linearly with queue depth but smooths bursts and reduces producer blocking.
- Approximate slot footprint: `kHeaderSize + max_payload + metadata` = `6 + 506 + 4 = 516` bytes per queue entry at defaults.
- Increasing queue depths increases copy/queue bookkeeping work, but reduces dropped frames when producers outrun consumers.
- Threaded parser mode moves parse work off the main loop (mbed RTOS), while `tick()` keeps scheduling explicit and deterministic on bare-metal loops.

# Error handling and diagnostics
## Return-value checks
`send`, `broadcast`, `receive`, and parser-thread control APIs return `bool`; always check them.

```cpp
if (!node.send(0x02, payload, payload_len)) {
  // TX rejected (queue full, invalid args, transport not ready, etc.)
}
```

## Runtime counters
Use diagnostics for fault attribution and tuning:

```cpp
auto diag = node.getDiagnostics();
// diag.sync_errors
// diag.destination_mismatch
// diag.length_overflow
// diag.crc_fail
// diag.timeout_count
// diag.drop_count

node.resetDiagnostics();
```

Interpretation tips:
- `sync_errors`: UART stream noise/misalignment.
- `destination_mismatch`: frames observed but filtered by destination policy.
- `length_overflow`: sender exceeding configured/allowed payload size.
- `crc_fail`: corruption or CRC mode mismatch between peers.
- `timeout_count`: partial frame stalled beyond `parser_timeout_ms`.
- `drop_count`: queue pressure (RX/TX full) or rejected enqueue path.

# Migration notes (legacy API → current API)
Legacy wrappers are still present but deprecated:
- `parse()`
- `getPayload()`
- `getPayloadSize()`
- non-const `broadcast(uint8_t*, uint16_t)` and legacy `send(uint8_t*, uint16_t, uint8_t)` signatures

Preferred replacements:
- **Polling path:** `available()` + `receive(Frame&)`
- **Callback path:** `onFrame(...)` (+ `startParserThread` on mbed RTOS, or `tick()` elsewhere)
- **TX path:** `send(dst, const uint8_t*, len)` / `broadcast(const uint8_t*, len)`

Migration pattern:
```cpp
// Old (deprecated)
if (node.parse()) {
  uint8_t *payload = node.getPayload();
  uint16_t len = node.getPayloadSize();
  // ...
}

// New
MuBus::MuBusNode::Frame frame;
if (node.receive(frame)) {
  uint8_t *payload = frame.payload;
  uint16_t len = frame.len;
  // ...
}
```

# Common pitfalls
- Do **not** dual-read the UART stream (for example, `Serial.read()` while MuBus parsing is active). Let MuBus own RX bytes.
- Ensure both peers use the same `crc_mode` and payload size expectations.
- If you enable ring/queue modes, choose queue depths that fit your RAM budget.
- On cooperative targets, call `tick()` frequently enough to avoid parser backlog/timeouts.
- When using threaded mode, stop the parser thread cleanly before transport teardown.

# Example sketches
Sketches are available under `examples/`:

- `examples/sender_basic/sender_basic.ino`: simple periodic sender.
- `examples/receiver_callback_threaded/receiver_callback_threaded.ino`: callback receiver with parser worker thread on mbed RTOS targets (falls back to `tick()` elsewhere).
- `examples/receiver_polling/receiver_polling.ino`: polling receiver using `available()/receive()`.
- `examples/receiver_migration_parse_getpayload/receiver_migration_parse_getpayload.ino`: migration from deprecated `parse()+getPayload()` APIs to `receive(Frame&)`.
