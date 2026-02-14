# MuBus API & Integration Guide

## Versioning scope
This guide documents the current MuBus implementation in:

- `src/MuBus.h`
- `src/MuBus.cpp`
- `src/transport/arduino_transport.*`
- `src/transport/mbed_transport.*`

It reflects the API and runtime behavior in the current repository state, including legacy/deprecated wrappers still exposed for compatibility.

## Who this is for
- **Arduino users** integrating MuBus over `HardwareSerial` in a cooperative loop.
- **mbed users** integrating MuBus over `mbed::BufferedSerial` with optional RTOS parser thread.
- **Embedded integrators** implementing custom `MuTransport` backends, tuning memory/latency, and maintaining protocol compatibility.

---

## 1) Protocol fundamentals

### 1.1 Wire frame layout (byte-by-byte)
All frames use this layout:

| Byte offset | Field | Size (bytes) | Value / meaning |
|---:|---|---:|---|
| 0 | `SYNC0` | 1 | fixed `0xD3` |
| 1 | `SYNC1` | 1 | fixed `0x91` |
| 2 | `SRC` | 1 | source node address |
| 3 | `DST` | 1 | destination address |
| 4 | `LEN0` | 1 | payload length low byte |
| 5 | `LEN1` | 1 | payload length high byte |
| 6.. | `PAYLOAD` | `LEN` | payload bytes |
| tail | `CRC` | 0/1/2 | optional: none / CRC8 / CRC16 |

`LEN` is little-endian:

```text
LEN = LEN0 | (LEN1 << 8)
```

### 1.2 Address semantics
- **Unicast:** `0x01` to `0xFE`.
- **Broadcast:** `0x00` (`kBroadcastAddress`).
- **Reserved:** `0xFF` (`kReservedAddress`), should not be assigned as a node address.

Destination acceptance is further controlled by `MuBusConfig::destination_filter_mode`.

### 1.3 Worked frame example
Example frame with no CRC:

```text
D3 91 12 34 03 00 AA BB CC
```

Decoded:
- `D3 91`: sync bytes
- `SRC=0x12`
- `DST=0x34`
- `LEN0=0x03`, `LEN1=0x00` → `LEN=3`
- payload: `AA BB CC`
- CRC omitted (`CrcMode::None`)

---

## 2) Core architecture overview

### 2.1 Main roles
- **`MuBusNode`**: high-level API for configuring a node, sending/receiving frames, parser lifecycle, diagnostics.
- **`MuTransport`**: abstract transport interface (`write`, `readByte`) used by parser and TX path.
- **Transport backends (`src/transport/`)**:
  - `ArduinoSerialTransport`: wraps `arduino::HardwareSerial`.
  - `MbedBufferedSerialTransport`: wraps `mbed::BufferedSerial`.
- **Parser FSM + queueing**:
  - Incremental byte parser (`Sync0 → Sync1 → Src → Dst → Len0 → Len1 → Payload → Crc`).
  - RX storage can be single-slot or ring queue.
  - TX can be direct immediate write or queued then flushed.

### 2.2 RX/TX high-level flow

#### Polling/cooperative mode
1. Application calls `poll()` or `tick()` (and `available()/receive()` also trigger polling).
2. `MuBusNode` consumes bytes via `transport_->readByte`.
3. FSM assembles frames, validates destination/CRC/length constraints.
4. Accepted frames go to:
   - pending single frame (`RxMode::SingleSlot`), or
   - RX queue (`RxMode::Ring`).
5. If callback is installed via `onFrame`, callback is invoked for accepted frames.
6. If TX queue mode is enabled, `pollCore()` flushes pending TX entries.

#### Callback/thread mode (mbed-capable builds)
1. Register callback via `onFrame(...)`.
2. Call `startParserThread(...)`.
3. Internal RTOS thread calls `poll()` periodically.
4. Frames are parsed and callback-dispatched without explicit `tick()` from main loop.
5. Stop with `stopParserThread()` before teardown/reconfiguration.

---

## 3) Public API reference

> Namespace: `MuBus`

### 3.1 Public types and constants

#### Enums
- `enum class RxMode : uint8_t { SingleSlot, Ring };`
- `enum class TxMode : uint8_t { Direct, Queue };`
- `enum class CrcMode : uint8_t { None, Crc8, Crc16 };`
- `enum class DestinationFilterMode : uint8_t { AddressedOrBroadcast, AcceptBroadcastOnly, AcceptUnicastOnly, Promiscuous };`

#### `struct MuBusDiagnostics`
- `uint32_t sync_errors`
- `uint32_t destination_mismatch`
- `uint32_t length_overflow`
- `uint32_t crc_fail`
- `uint32_t timeout_count`
- `uint32_t drop_count`

#### `struct MuBusStatus`
- `bool rx_queue_full`
- `bool tx_queue_full`
- `uint32_t dropped_frame_count`

#### `struct MuBusConfig`
- `RxMode rx_mode = RxMode::SingleSlot`
- `TxMode tx_mode = TxMode::Direct`
- `uint8_t rx_queue_depth = 1`
- `uint8_t tx_queue_depth = 1`
- `uint16_t max_payload_size = kMaxPayload` (`506`)
- `CrcMode crc_mode = CrcMode::None`
- `uint32_t parser_timeout_ms = 0`
- `uint32_t parser_thread_stack_bytes = 4096`
- `DestinationFilterMode destination_filter_mode = DestinationFilterMode::AddressedOrBroadcast`

#### `class MuBusNode::Frame`
- `uint8_t src`
- `uint8_t dst`
- `uint16_t len`
- `uint8_t *payload`

### 3.2 Constructors / lifecycle

#### `MuBusNode();`
Creates node with default config and no bound transport.

#### `MuBusNode(uint8_t addr);`
Same as default plus source address binding.

#### `MuBusNode(uint8_t addr, const MuBusConfig &config);`
Address + custom config.

#### `MuBusNode(MuTransport *transport);`
Binds caller-owned transport pointer.

#### `MuBusNode(MuTransport *transport, uint8_t addr);`
Transport + address.

#### `MuBusNode(MuTransport *transport, uint8_t addr, const MuBusConfig &config);`
Transport + address + config.

#### `MuBusNode(arduino::HardwareSerial *port);`
#### `MuBusNode(arduino::HardwareSerial *port, uint8_t addr);`
#### `MuBusNode(arduino::HardwareSerial *port, uint8_t addr, const MuBusConfig &config);`
On supported builds, allocates internal `ArduinoSerialTransport` and takes ownership.

#### `MuBusNode(mbed::BufferedSerial *port);`
#### `MuBusNode(mbed::BufferedSerial *port, uint8_t addr);`
#### `MuBusNode(mbed::BufferedSerial *port, uint8_t addr, const MuBusConfig &config);`
On supported builds, allocates internal `MbedBufferedSerialTransport` and takes ownership.

#### `~MuBusNode();`
Stops parser thread, releases owned transport, deletes thread sync primitives (when enabled).

### 3.3 Bring-up / teardown

#### `bool begin(MuTransport *transport, uint8_t addr);`
#### `bool begin(MuTransport *transport, uint8_t addr, const MuBusConfig &config);`
Rebinds transport/address (and optional config) at runtime, then resets parser, status, diagnostics, queues.
- Not required for first use when the node was already constructed with a serial port constructor.
- Returns `false` if transport pointer is null.

#### `bool begin(arduino::HardwareSerial *port, uint8_t addr);`
#### `bool begin(arduino::HardwareSerial *port, uint8_t addr, const MuBusConfig &config);`
On Arduino-transport-enabled builds: creates owned serial transport and assigns.
On disabled builds: always returns `false`.

#### `bool begin(mbed::BufferedSerial *port, uint8_t addr);`
#### `bool begin(mbed::BufferedSerial *port, uint8_t addr, const MuBusConfig &config);`
On mbed-transport-enabled builds: creates owned serial transport and assigns.
On disabled builds: always returns `false`.

#### `void stop();`
Stops parser thread, releases owned transport, and clears callback/parser/queue/status/diagnostics state.

### 3.4 TX API

#### `void bindAddr(uint8_t addr);`
Updates node source address (`SRC` field used for outgoing frames).

#### `bool send(uint8_t dst, const uint8_t *data, uint16_t len);`
Sends to `dst`.
- Validates `len <= max_payload_size`.
- Rejects `len > 0 && data == nullptr`.
- In `TxMode::Queue`: enqueues, attempts flush, returns enqueue result.
- In `TxMode::Direct`: writes immediately.
- On queue full: sets `status.tx_queue_full`, increments dropped/diagnostic counters.

Common failure modes:
- no transport,
- invalid payload pointer/length,
- transport write failure,
- queue full.

#### `bool broadcast(const uint8_t *data, uint16_t len);`
Equivalent to `send(kBroadcastAddress, data, len)`.

### 3.5 RX API

#### `bool available();`
Calls `poll()`, then reports data presence:
- ring mode: `rx_count_ > 0`
- single-slot: pending frame flag.

#### `bool receive(Frame &frame);`
Calls `poll()` then retrieves one frame.
- ring mode: dequeues oldest frame.
- single-slot: returns pending frame and clears pending flag.
- Returns `false` when no frame available.

Side effect: may parse new bytes due to implicit `poll()`.

#### `void onFrame(FrameCallback callback);`
Registers callback (`void (*)(const Frame&)`) invoked when accepted frame is parsed.

### 3.6 Polling / parser execution

#### `void poll();`
Runs parser and queue flush under internal lock.

#### `void tick();`
Alias of `poll()`.

#### `bool startParserThread();`
Uses stored parser thread config (default poll interval `1 ms`, stop timeout `100 ms`).

#### `bool startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms);`
Attempts to start RTOS parser thread.
- Returns `false` when thread feature is compiled out.
- Returns `false` if transport is null.
- Returns `true` if already running.
- Allocates RTOS helpers lazily (`rtos::Mutex`, `rtos::EventFlags`, `rtos::Semaphore`, `rtos::Thread`).

#### `bool stopParserThread();`
Stops parser thread if running.
- In thread-enabled builds: signals stop, waits semaphore up to timeout, joins thread.
- Can return `false` if stop wait times out.
- In thread-disabled builds: returns `true` (no-op success).

### 3.7 Status / diagnostics

#### `NodeStatus getStatus() const;`
Returns current status snapshot.

#### `Diagnostics getDiagnostics() const;`
Returns diagnostic counters snapshot.

#### `void resetDiagnostics();`
Resets diagnostics counters to zero.

### 3.8 Deprecated API and migration map

#### Deprecated symbols
- `bool send(uint8_t *buf, uint16_t len, uint8_t recv_addr);`
- `bool broadcast(uint8_t *buf, uint16_t len);`
- `bool parse();`
- `uint8_t *getPayload();`
- `uint16_t getPayloadSize();`
- `String formatHeader();`

#### Migration mapping
- `send(buf, len, recv_addr)` → `send(recv_addr, buf, len)`
- `broadcast(uint8_t*, len)` → `broadcast(const uint8_t*, len)`
- `parse()+getPayload()+getPayloadSize()` → `receive(Frame&)`

---

## 4) Configuration deep dive (`MuBusConfig`)

| Field | Default | Valid range / options | Tradeoffs |
|---|---|---|---|
| `rx_mode` | `SingleSlot` | `SingleSlot` / `Ring` | `SingleSlot` minimum RAM, newest frame only; `Ring` buffers bursts with higher RAM. |
| `tx_mode` | `Direct` | `Direct` / `Queue` | `Direct` lowest latency/memory; `Queue` decouples producers but uses RAM and may drop when full. |
| `rx_queue_depth` | `1` | clamped to `1..8`; forced `1` in `SingleSlot` | Higher depth tolerates burst RX, linear RAM increase. |
| `tx_queue_depth` | `1` | clamped to `1..8`; forced `1` in `Direct` | Higher depth smooths TX bursts, linear RAM increase. |
| `max_payload_size` | `506` | clamped to `0..506` | Larger max allows bigger frames but parser/TX buffers remain sized for max compile-time payload. |
| `crc_mode` | `None` | `None`, `Crc8`, `Crc16` | CRC adds integrity checks and bytes-on-wire + compute cost. |
| `parser_timeout_ms` | `0` | any `uint32_t` (`0` disables timeout) | Non-zero drops stalled partial frames; too small may reject valid slow streams. |
| `parser_thread_stack_bytes` | `4096` | any `uint32_t`; very small values risk parser-thread instability | mbed parser-thread stack reservation. Use `>= 3072` for tiny callbacks; prefer `4096-8192` for callback-heavy/diagnostic workloads. |
| `destination_filter_mode` | `AddressedOrBroadcast` | enum options listed above | Tight filtering reduces app-level noise; promiscuous captures all. |

### Recommended presets

#### Minimal RAM
```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::SingleSlot;
cfg.tx_mode = MuBus::TxMode::Direct;
cfg.max_payload_size = 64;
cfg.crc_mode = MuBus::CrcMode::None;
```

#### Balanced
```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 3;
cfg.tx_mode = MuBus::TxMode::Direct;
cfg.max_payload_size = 128;
cfg.crc_mode = MuBus::CrcMode::Crc8;
cfg.parser_timeout_ms = 20;
```

#### High-throughput / burst-tolerant
```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 8;
cfg.tx_mode = MuBus::TxMode::Queue;
cfg.tx_queue_depth = 8;
cfg.max_payload_size = 506;
cfg.crc_mode = MuBus::CrcMode::Crc16;
cfg.parser_timeout_ms = 50;
cfg.destination_filter_mode = MuBus::DestinationFilterMode::Promiscuous;
```

---

## 5) Threading and execution model

### 5.1 Expected lifecycle / call order
Typical threaded flow:
1. Construct node with canonical constructor usage (`port, addr` for basic or `port, addr, config` for advanced).
2. Optionally call `onFrame(...)`.
3. Call `startParserThread(...)`.
4. Runtime: app can use callbacks and/or `available()/receive()`.
5. If constructed with `arduino::HardwareSerial*` or `mbed::BufferedSerial*`, no initial `begin(...)` call is required; use `begin(...)` only when rebinding transport/address/config at runtime.
6. Before teardown or rebind: call `stopParserThread()` then `stop()` / `begin(...)` new transport.

### 5.2 Behavior when thread mode is unavailable
If `MUBUS_ENABLE_PARSER_THREAD == 0`:
- `startParserThread(...)` returns `false`.
- `stopParserThread()` returns `true` as no-op.
- Use `poll()` / `tick()` frequently.

### 5.3 Concurrency safety notes
- Internal parser and queue paths use a mutex when parser-thread feature is enabled.
- `getStatus()` and `getDiagnostics()` return snapshots without internal locking; treat as eventually consistent while active concurrent parser work is running.
- Callback executes from polling context:
  - cooperative mode: caller thread invoking `poll/tick/receive/available`
  - threaded mode: parser thread context
- Keep callback fast and non-blocking; avoid heavy operations that may delay parsing.
- `Frame::payload` points to internal storage. If data must outlive callback/dequeue cycle, copy it.

### 5.4 Namespace convention
MuBus code uses `MuBus::` namespace and expects RTOS symbols in `rtos::` (e.g., `rtos::Thread`, `rtos::Mutex`, `rtos::EventFlags`, `rtos::Semaphore`, `rtos::ThisThread`). On mbed-enabled builds this maps to the expected mbed RTOS APIs.

---

## 6) Diagnostics and error handling

### 6.1 `MuBusDiagnostics` field-by-field
- `sync_errors`: increments when byte stream does not match sync-state expectations.
- `destination_mismatch`: increments for parsed frames rejected by destination filter.
- `length_overflow`: increments when parsed length exceeds configured max or parser bounds.
- `crc_fail`: increments when received CRC does not match computed CRC.
- `timeout_count`: increments when partial frame exceeds `parser_timeout_ms`.
- `drop_count`: aggregate drops (queue overflow, filter rejection, CRC fail, timeout, overflow conditions).

### 6.2 `MuBusStatus` field-by-field
- `rx_queue_full`: set when RX enqueue fails due to full ring; clears once queue occupancy falls below depth.
- `tx_queue_full`: set when TX enqueue fails due to full queue; clears after queue flush empties.
- `dropped_frame_count`: aggregate dropped frame counter across RX/TX/parser rejection paths.

### 6.3 Troubleshooting matrix

| Symptom | Likely cause | What to inspect | Corrective action |
|---|---|---|---|
| No frames received | `poll()`/`tick()` not called (non-threaded mode) | Main loop scheduling; `startParserThread` return value | Call `tick()` regularly or enable/verify parser thread start. |
| `crc_fail` rising | peers use different `crc_mode`, or line corruption | Both endpoints config, UART wiring/noise | Match CRC mode, improve signal integrity/baud settings. |
| `length_overflow` rising | sender length > receiver `max_payload_size` | Sender payload size, receiver config | Increase receiver max payload or limit sender frame size. |
| `destination_mismatch` rising | destination filter excludes observed frames | `destination_filter_mode`, node address | Adjust filter mode or destination addresses. |
| `drop_count` + `rx_queue_full` | consumer too slow for RX rate | RX queue depth, callback latency | Increase `rx_queue_depth`, reduce callback work, poll faster. |
| `drop_count` + `tx_queue_full` | TX producer outruns transport | TX mode/depth, transport throughput | Increase `tx_queue_depth`, reduce send burst, lower payload size. |
| `timeout_count` rising | parser timeout too short or stream stalls | `parser_timeout_ms`, poll cadence | Raise/disable timeout (`0`) or improve scheduling/transport readiness. |
| Hard fault / random resets in threaded callback mode | parser thread stack exhaustion | `parser_thread_stack_bytes`, callback depth, temporary buffers | Increase parser thread stack size; start near `4096` and scale toward `8192` when callbacks/diagnostics are heavy. |

### 6.4 Build/link integration troubleshooting
- **Undefined `rtos::...` symbols**: ensure mbed target/build flags align with `MUBUS_HAS_MBED` and `MUBUS_ENABLE_PARSER_THREAD` expectations.
- **Arduino/mbed transport begin returns false**: transport backend may be disabled via compile switches (`MUBUS_ENABLE_ARDUINO_TRANSPORT`, `MUBUS_ENABLE_MBED_TRANSPORT`).
- **Header include mismatch**: include top-level `MuBus.h` (which forwards to `src/MuBus.h`) in user sketches/apps.

---

## 7) Usage cookbook

### 7.1 Minimal sender
```cpp
#include <MuBus.h>

MuBus::MuBusNode node(&Serial1, 0x01);

void setup() {
  Serial1.begin(115200);
}

void loop() {
  const uint8_t msg[] = {0x10, 0x20, 0x30};
  (void)node.send(0x02, msg, sizeof(msg));
  delay(100);
}
```

### 7.2 Minimal receiver (polling)
```cpp
#include <MuBus.h>

MuBus::MuBusNode node(&Serial1, 0x02);

void setup() {
  Serial1.begin(115200);
}

void loop() {
  node.tick();

  MuBus::MuBusNode::Frame frame;
  if (node.receive(frame)) {
    // frame.src, frame.dst, frame.len, frame.payload
  }
}
```

### 7.3 Callback + parser thread usage (canonical mbed basic)
```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial port(PA_9, PA_10, 115200);
MuBus::MuBusNode node(&port, 0x20);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
  // keep callback short
}

int main() {
  node.onFrame(onFrame);

  if (!node.startParserThread(1, 200)) {
    while (true) {
      node.tick();
      rtos::ThisThread::sleep_for(1ms);
    }
  }

  while (true) {
    rtos::ThisThread::sleep_for(100ms);
  }
}
```

### 7.4 Canonical mbed advanced construction
```cpp
#include <mbed.h>
#include <MuBus.h>

mbed::BufferedSerial port(PA_9, PA_10, 115200);

MuBus::MuBusConfig cfg;
cfg.crc_mode = MuBus::CrcMode::Crc16;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 4;

MuBus::MuBusNode node(&port, 0x20, cfg);
```

Use `begin(...)` after construction only when you need to rebind transport and/or apply a new runtime configuration.

### 7.5 Ring-buffer RX configuration
```cpp
MuBus::MuBusConfig cfg;
cfg.rx_mode = MuBus::RxMode::Ring;
cfg.rx_queue_depth = 4;

MuBus::MuBusNode node;
node.begin(&Serial1, 0x11, cfg);
```

### 7.6 CRC-enabled example
```cpp
MuBus::MuBusConfig cfg;
cfg.crc_mode = MuBus::CrcMode::Crc16;

MuBus::MuBusNode node(&Serial1, 0x01, cfg);

const uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
(void)node.broadcast(data, sizeof(data));
```

### 7.7 Migration: `parse()+getPayload()` to `receive(Frame&)`
```cpp
// Deprecated style
if (node.parse()) {
  uint8_t *p = node.getPayload();
  uint16_t n = node.getPayloadSize();
  (void)p; (void)n;
}

// Current style
MuBus::MuBusNode::Frame f;
if (node.receive(f)) {
  uint8_t *p = f.payload;
  uint16_t n = f.len;
  (void)p; (void)n;
}
```

### 7.8 Cross-links to repository examples
- Sender: `examples/sender_basic/sender_basic.ino`
- Polling receiver: `examples/receiver_polling/receiver_polling.ino`
- Callback/threaded receiver: `examples/receiver_callback_threaded/receiver_callback_threaded.ino`
- Migration sample: `examples/receiver_migration_parse_getpayload/receiver_migration_parse_getpayload.ino`

---

## 8) Embedded constraints and tuning

### 8.1 RAM footprint guidance
- Per frame-slot footprint at compile-time max payload is approximately:
  - `6` (header bytes) + `506` (payload buffer) + metadata (`src`,`dst`,`len` = `4`) = **516 bytes/slot**.
- RX ring memory impact scales roughly with `rx_queue_depth` (up to 8 slots).
- TX queue memory impact scales roughly with `tx_queue_depth` (up to 8 slots).
- Single-slot/direct modes minimize extra buffering.
- Lowering `max_payload_size` reduces accepted payload length, but internal slot arrays are still compiled at max (`kMaxPayload`) in current implementation.

### 8.2 Timing/latency notes
- In cooperative mode, parser latency equals how often `tick()`/`poll()` is called.
- In threaded mode, latency is tied to parser thread poll interval.
- `parser_timeout_ms` should exceed worst-case inter-byte gap expected on your link.
- `parser_thread_stack_bytes` should be sized for callback complexity: `>= 3072` for tiny callbacks, `4096-8192` recommended when callbacks and diagnostics formatting are active.
  - Too low: false frame drops/timeouts.
  - Too high: slower recovery from broken/incomplete frames.

---

## 9) Maintainer notes appendix

### 9.1 Where to add new transport backends
- Define backend type implementing `MuTransport` in `src/transport/`.
- Add guarded constructor/begin overloads in `MuBusNode` if exposing convenience path.
- Keep compile-time feature macros consistent with existing backend pattern.

### 9.2 Where parser/state-machine changes live
- FSM and frame finalization: `MuBusNode::parseByte`, `finalizeParsedFrame`, `readFrame` in `src/MuBus.cpp`.
- Protocol constants and config/status/diagnostics shape: `src/MuBus.h`.

### 9.3 Protocol/API evolution compatibility rules
- Preserve sync bytes and base header layout unless introducing explicit protocol versioning strategy.
- Treat deprecated wrappers as compatibility layer; document migration before removing them.
- Keep CRC encoding/endianness consistent across write/read paths.
- If changing defaults, update both docs and examples to avoid silent behavior shifts.

### 9.4 Documentation sync checklist
When API/protocol changes:
1. Update `src/MuBus.h` declarations and comments (if present).
2. Update this `docs/README.md` signatures/defaults/behavior notes.
3. Update root `README.md` quick-start and feature tables.
4. Verify examples compile against updated signatures.
5. Re-check troubleshooting and migration sections for stale guidance.

