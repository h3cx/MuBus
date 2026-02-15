# Migration Guide (Legacy APIs → Current APIs)

This page maps commonly used legacy calls to current MuBus APIs.

## 1) `parse()` + `getPayload()` flow → `receive(Frame&)`

### Before
```cpp
if (node.parse()) {
  uint8_t *payload = node.getPayload();
  uint16_t len = node.getPayloadSize();
  // process payload
}
```

### After
```cpp
MuBus::MuBusNode::Frame frame;
if (node.receive(frame)) {
  const uint8_t *payload = frame.payload;
  uint16_t len = frame.len;
  // process payload
}
```

## 2) Legacy `send(buf, len, dst)` argument order → `send(dst, data, len)`

### Before
```cpp
uint8_t msg[] = {1,2,3};
node.send(msg, sizeof(msg), 0x12);
```

### After
```cpp
const uint8_t msg[] = {1,2,3};
node.send(0x12, msg, sizeof(msg));
```

## 3) Mutable `broadcast(uint8_t*)` → const-correct `broadcast(const uint8_t*)`

### Before
```cpp
uint8_t msg[] = {4,5,6};
node.broadcast(msg, sizeof(msg));
```

### After
```cpp
const uint8_t msg[] = {4,5,6};
node.broadcast(msg, sizeof(msg));
```

## 4) Implicit runtime assumptions → explicit runtime macro selection

### Before
- Build relied on environment-specific include behavior.

### After
- Define exactly one runtime macro:
  - `MUBUS_RUNTIME_ARDUINO`
  - `MUBUS_RUNTIME_MBED`
  - `MUBUS_RUNTIME_FREERTOS`

See [runtime selection](runtime-selection.md).

## 5) Parser execution assumptions

### Before
- `parse()` called directly in loop.

### After
- Use `tick()`/`poll()` for cooperative parsing.
- Or `startParserThread(...)` on RTOS-enabled runtimes.

## Migration checklist

- [ ] Select exactly one runtime macro.
- [ ] Replace deprecated send/broadcast signatures.
- [ ] Replace `parse()`/payload getters with `receive(Frame&)` or callback.
- [ ] Verify constructor and `begin(...)` overloads match runtime.
- [ ] Align CRC mode across all nodes.
