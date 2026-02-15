# Diagnostics and Status

Header: `src/MuBus.h`

## `MuBusDiagnostics`

Counters:
- `sync_errors`
- `destination_mismatch`
- `length_overflow`
- `crc_fail`
- `timeout_count`
- `drop_count`

Read via:

```cpp
auto d = node.getDiagnostics();
```

Reset via:

```cpp
node.resetDiagnostics();
```

## `MuBusStatus`

Fields:
- `rx_queue_full`
- `tx_queue_full`
- `dropped_frame_count`

Read via:

```cpp
auto s = node.getStatus();
```

## Interpretation hints

- Rising `sync_errors`: framing mismatch or line noise.
- Rising `crc_fail`: sender/receiver CRC mismatch or corrupted bytes.
- Rising `destination_mismatch`: destination filtering is dropping packets.
- Rising `drop_count` + queue_full flags: increase queue depth or consume frames faster.
- Rising `timeout_count`: parser timeout too aggressive for stream gaps.

For parser behavior and threading, see [parser modes](parser-modes.md).
