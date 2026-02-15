# `MuBus::MuBusNode`

Header: `src/MuBus.h`

## Constructor and `begin(...)` availability by runtime

| Signature | Arduino | mbed | FreeRTOS |
|---|---:|---:|---:|
| `MuBusNode()` | ✅ | ✅ | ✅ |
| `MuBusNode(uint8_t addr)` | ✅ | ✅ | ✅ |
| `MuBusNode(uint8_t addr, const MuBusConfig &config)` | ✅ | ✅ | ✅ |
| `MuBusNode(MuTransport *transport)` | ✅ | ✅ | ✅ |
| `MuBusNode(MuTransport *transport, uint8_t addr)` | ✅ | ✅ | ✅ |
| `MuBusNode(MuTransport *transport, uint8_t addr, const MuBusConfig &config)` | ✅ | ✅ | ✅ |
| `MuBusNode(HardwareSerial *port, ...)` | ✅ | ❌ | ❌ |
| `MuBusNode(mbed::BufferedSerial *port, ...)` | ❌ | ✅ | ❌ |
| `begin(MuTransport *transport, uint8_t addr[, config])` | ✅ | ✅ | ✅ |
| `begin(HardwareSerial *port, uint8_t addr[, config])` | ✅ | ❌ | ❌ |
| `begin(mbed::BufferedSerial *port, uint8_t addr[, config])` | ❌ | ✅ | ❌ |

## Core methods

- `void bindAddr(uint8_t addr)`
- `bool send(uint8_t dst, const uint8_t *data, uint16_t len)`
- `bool broadcast(const uint8_t *data, uint16_t len)`
- `bool available()`
- `bool receive(Frame &frame)`
- `void onFrame(FrameCallback callback)`
- `void poll()`
- `void tick()`
- `bool startParserThread()`
- `bool startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms)`
- `bool stopParserThread()`
- `NodeStatus getStatus() const`
- `Diagnostics getDiagnostics() const`
- `void resetDiagnostics()`
- `void stop()`

## Deprecated API mapping

Deprecated methods still present:
- `bool send(uint8_t *buf, uint16_t len, uint8_t recv_addr)` → `send(recv_addr, buf, len)`
- `bool broadcast(uint8_t *buf, uint16_t len)` → `broadcast(const uint8_t*, len)`
- `bool parse()` → `receive(Frame&)` or callback model
- `uint8_t *getPayload()`, `uint16_t getPayloadSize()` (legacy parser path)

See [migration guide](../migration.md).

## Why an overload is missing

Most missing-overload issues are runtime macro mismatches.

- Need `HardwareSerial*` overloads: compile with `MUBUS_RUNTIME_ARDUINO`.
- Need `mbed::BufferedSerial*` overloads: compile with `MUBUS_RUNTIME_MBED`.
- On FreeRTOS, use only `MuTransport*` overloads.

Also verify only one `MUBUS_RUNTIME_*` macro is defined. See [runtime selection](../runtime-selection.md).
