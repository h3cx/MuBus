# `MuBus::MuTransport`

Header: `src/MuBus.h`

`MuTransport` is the runtime-neutral I/O abstraction used by `MuBusNode`.

```cpp
class MuTransport {
public:
  virtual ~MuTransport() = default;
  virtual bool write(const uint8_t *data, size_t len) = 0;
  virtual bool readByte(uint8_t &byte) = 0;
};
```

## Contract

- `write(...)` should attempt to write the full buffer and return `true` only on full success.
- `readByte(...)` should be non-blocking and return `true` only when one byte was read.
- `readByte(...)` should return `false` when no byte is available.

## Built-in runtime adapters

- Arduino runtime: `ArduinoSerialTransport(HardwareSerial *port)`
- mbed runtime: `MbedBufferedSerialTransport(mbed::BufferedSerial *port)`
- FreeRTOS runtime: no built-in serial adapter; implement your own transport.

## Minimal custom transport skeleton

```cpp
class MyTransport : public MuBus::MuTransport {
public:
  bool write(const uint8_t *data, size_t len) override {
    return driver_write(data, len);
  }

  bool readByte(uint8_t &byte) override {
    return driver_read_one_nonblocking(byte);
  }
};
```

Usage:

```cpp
MyTransport transport;
MuBus::MuBusNode node(&transport, 0x01);
```
