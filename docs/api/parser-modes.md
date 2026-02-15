# Parser Modes

`MuBusNode` supports cooperative polling and optional parser-thread execution.

## Cooperative mode

Always available:
- `poll()`
- `tick()` (alias of `poll()`)

Use this in baremetal loops or when thread feature is disabled.

## Parser thread mode

API:
- `startParserThread()`
- `startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms)`
- `stopParserThread()`

Behavior:
- Returns `false` if parser-thread feature is compiled out (`MUBUS_ENABLE_PARSER_THREAD=0`).
- Returns `false` if no transport is bound.
- Returns `true` immediately if thread already running.

Runtime notes:
- mbed: creates `rtos::Thread` and synchronization primitives.
- FreeRTOS: creates task + mutex/event group/semaphore.
- Arduino default: thread feature disabled.

## Choosing mode

- Prefer polling for deterministic main-loop ownership.
- Prefer parser thread for callback-driven designs on RTOS targets.
- Even in threaded mode, you can still call `receive(...)` to drain queued frames.

See runtime-specific recommendations:
- [Arduino guide](../platforms/arduino.md)
- [mbed guide](../platforms/mbed.md)
- [FreeRTOS guide](../platforms/freertos.md)
