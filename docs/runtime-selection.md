# Runtime Selection

MuBus requires a compile-time runtime identity in `src/MuBus.h`.

## Required macro
Define **exactly one** of:

- `MUBUS_RUNTIME_ARDUINO`
- `MUBUS_RUNTIME_MBED`
- `MUBUS_RUNTIME_FREERTOS`

If you define none, build fails with:

> `Select one runtime: define exactly one of MUBUS_RUNTIME_ARDUINO, MUBUS_RUNTIME_MBED, or MUBUS_RUNTIME_FREERTOS`

If you define more than one, build fails with:

> `Only one runtime may be selected: define exactly one MUBUS_RUNTIME_* macro`

## Preferred pattern: define in your main source file

Place one runtime define **before** `#include <MuBus.h>` in your main `.ino`/`.cpp`:

```cpp
#define MUBUS_RUNTIME_ARDUINO
// #define MUBUS_RUNTIME_MBED
// #define MUBUS_RUNTIME_FREERTOS
#include <MuBus.h>
```

Parser thread override (optional):

```cpp
#define MUBUS_DISABLE_PARSER_THREAD   // force cooperative parsing
// #define MUBUS_ENABLE_PARSER_THREAD // force threaded parsing when supported
#include <MuBus.h>
```

## What each runtime enables

| Runtime macro | Serial constructor overloads | RTOS parser thread support | Runtime transport helper |
|---|---|---|---|
| `MUBUS_RUNTIME_ARDUINO` | `MuBusNode(HardwareSerial*)` + matching `begin(...)` overloads | Disabled by default (`MUBUS_ENABLE_PARSER_THREAD` defaults false) | `ArduinoSerialTransport` |
| `MUBUS_RUNTIME_MBED` | `MuBusNode(mbed::BufferedSerial*)` + matching `begin(...)` overloads | Enabled by default | `MbedBufferedSerialTransport` |
| `MUBUS_RUNTIME_FREERTOS` | No runtime-specific serial overloads; use `MuTransport*` | Enabled by default | No built-in serial adapter; provide your own `MuTransport` |

## `MUBUS_ENABLE_PARSER_THREAD`
Default:
- Arduino runtime: `0`
- mbed/FreeRTOS runtime: `1`

You can force parser threading with `#define MUBUS_ENABLE_PARSER_THREAD` or disable it with `#define MUBUS_DISABLE_PARSER_THREAD` before including `MuBus.h`.

When disabled, `startParserThread(...)` returns `false`, and you must call `tick()`/`poll()` yourself.

## Common misconfiguration errors

### 1) Using `HardwareSerial` API in non-Arduino runtime
Symptom: missing type or no matching constructor.

Fix: switch runtime macro to `MUBUS_RUNTIME_ARDUINO`, or change code to `MuTransport*` / runtime-appropriate serial type.

### 2) Using `mbed::BufferedSerial` API outside mbed runtime
Symptom: `mbed` namespace or overload not found.

Fix: define `MUBUS_RUNTIME_MBED` before including `MuBus.h`, and ensure mbed headers are available.

### 3) Expecting built-in FreeRTOS serial wrapper
Symptom: no constructor overload for UART object.

Fix: implement a `MuTransport` subclass for your platform UART driver and call `begin(MuTransport*, addr, config)`.

### 4) Thread API returns `false`
Likely causes:
- `MUBUS_DISABLE_PARSER_THREAD`
- no transport bound yet
- RTOS object/task allocation failed

Fix: bind transport first and verify memory/RTOS resources.

Next: jump to your platform guide in [`docs/platforms/`](platforms/arduino.md).
