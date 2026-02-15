# Config Enums and Structs

Header: `src/MuBus.h`

## Constants

- `kHeaderSize = 6`
- `kMaxPayload = 506`
- `kMaxRxQueueDepth = 8`
- `kMaxTxQueueDepth = 8`
- `kDefaultParserThreadStackBytes = 4096`
- `kBroadcastAddress = 0x00`
- `kReservedAddress = 0xFF`

## Enums

- `RxMode { SingleSlot, Ring }`
- `TxMode { Direct, Queue }`
- `CrcMode { None, Crc8, Crc16 }`
- `DestinationFilterMode { AddressedOrBroadcast, AcceptBroadcastOnly, AcceptUnicastOnly, Promiscuous }`

## `MuBusConfig` reference

| Field | Default | Valid range / effective range | Runtime-specific behavior |
|---|---|---|---|
| `rx_mode` | `RxMode::SingleSlot` | enum | If `SingleSlot`, `rx_queue_depth` forced to `1`. |
| `tx_mode` | `TxMode::Direct` | enum | If `Direct`, `tx_queue_depth` forced to `1`. |
| `rx_queue_depth` | `1` | input clamped to `1..kMaxRxQueueDepth` | Used only in `RxMode::Ring`. |
| `tx_queue_depth` | `1` | input clamped to `1..kMaxTxQueueDepth` | Used only in `TxMode::Queue`. |
| `max_payload_size` | `kMaxPayload` (`506`) | input clamped to `0..kMaxPayload` | Shared behavior across runtimes. |
| `crc_mode` | `CrcMode::None` | enum | Shared behavior across runtimes. |
| `parser_timeout_ms` | `0` | `>=0` | `0` disables parser timeout reset logic. |
| `parser_thread_stack_bytes` | `4096` | positive integer recommended | Used by parser-thread creation on mbed/FreeRTOS. |
| `destination_filter_mode` | `AddressedOrBroadcast` | enum | Shared behavior across runtimes. |

## Queueing/threading tradeoffs

- Lower memory profile: `RxMode::SingleSlot` + `TxMode::Direct`.
- Burst tolerance: `RxMode::Ring` and/or `TxMode::Queue` with larger depths.
- Thread parser (`startParserThread`) reduces main-loop parser work but requires RTOS resources.
- Polling (`tick`/`poll`) gives deterministic explicit scheduling.

### Memory footprint examples (default payload size)
A queue slot stores header + payload + metadata: `6 + 506 + 4 = 516 bytes`.

- RX ring depth 4: ~`4 * 516 = 2064 bytes`
- TX queue depth 4: ~`2064 bytes`
- Both depth 4: ~`4128 bytes` (plus object/stack overhead)

## CRC compatibility matrix

| Sender `crc_mode` | Receiver `crc_mode` | Result |
|---|---|---|
| `None` | `None` | Frame accepted (CRC field absent). |
| `None` | `Crc8`/`Crc16` | Rejected (`crc_fail`) because receiver expects CRC bytes. |
| `Crc8` | `Crc8` | Accepted when CRC matches. |
| `Crc8` | `None` | Not interoperable framing; receiver treats CRC byte as next frame data. |
| `Crc16` | `Crc16` | Accepted when CRC matches. |
| `Crc16` | `None` | Not interoperable framing; trailing CRC bytes desynchronize stream. |
| `Crc8` | `Crc16` (or inverse) | Rejected / desync risk due to different CRC field size expectation. |

Use same CRC mode on all communicating nodes.

## Destination filter behavior examples

Assume node source address is `0x22`:

- `AddressedOrBroadcast`: accepts `dst == 0x22` and `dst == 0x00`
- `AcceptBroadcastOnly`: accepts only `dst == 0x00`
- `AcceptUnicastOnly`: accepts only `dst == 0x22`
- `Promiscuous`: accepts any destination byte
