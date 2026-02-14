<p align="center">
  <img src="https://raw.githubusercontent.com/h3cx/MuBus/main/images/mubus.png"
       width="30%" />
</p>
A high performance, low overhead serial communication protocol

> [!IMPORTANT]
> μBus currently only support Arduino and other related boards that have a `HardwareSerial` object

# Installation
μBus is currently aimed at being used with PlatformIO, it is available on the PlatformIO registry but likely not up to date due to active development.
In order to install μBus run the following in your PlatformIO project root folder:
```
pio pkg install -l "https://github.com/h3cx/MuBus.git"
```
> [!IMPORTANT]
> Expect regular breaking changes during initial development.

# Usage
## Constructor
The basic constructor follows the following pattern:
```cpp
MuBusNode(HardwareSerial& port, uint8_t addr)
```
For example, on an Arduino Mega using `Serial1` and with a device address of `0x01` this would give:
```cpp
MuBusNode node(&Serial1, 0x01);
```
You will still be required to call `Serial1.begin()` seperately as μBus doesn't handle this yet

## Sending Data
Once you have set up your node, you can now send data from it. Use `send(dst, data, len)` for unicast and `broadcast(...)` for broadcast (`dest_addr = 0x00`).
To broadcast you use the following function:
```cpp
bool broadcast(uint8_t *buf, uint16_t len)
```
The maximum `len` is 506 bytes, this is a limitation of μBus which has a maximum dataframe of 512 bytes.
An example usage sending a custom data struct:
```cpp
data_struct_t some_data;
node.broadcast(&some_data, sizeof(some_data));
```

## Receiving Data
In order to receive data, the parse function can be used:
```cpp
bool parse()
```
This function will search for header bytes in the serial interface, and only once it finds a matching packet will it fully read it. If it reads a valid data frame, it will return true. Destination filtering is applied as an explicit step after full header decode (after `SRC`, `DST`, and `LEN` are available).
This function is called in a loop to form the parser:
```cpp
while(true) {
  if (node.parse()) {
    //do something
  }
}
```
In order to access the payload the following functions can be used:
```cpp
uint8_t *getPayload();
uint16_t getPayloadSize();
```




## Example sketches
Sketches are available under `examples/`:

- `examples/sender_basic/sender_basic.ino`: simple periodic sender.
- `examples/receiver_callback_threaded/receiver_callback_threaded.ino`: callback receiver with parser worker thread on mbed RTOS targets (falls back to `tick()` elsewhere).
- `examples/receiver_polling/receiver_polling.ino`: polling receiver using `available()/receive()`.
- `examples/receiver_migration_parse_getpayload/receiver_migration_parse_getpayload.ino`: migration from deprecated `parse()+getPayload()` APIs to `receive(Frame&)`.

> [!IMPORTANT]
> When the MuBus parser is active, do not read from the UART directly (`Serial.read()`, `Serial.peek()`, etc.).
> Read incoming frames only through MuBus internals (`onFrame`, `available()/receive()`, or deprecated wrappers).

## Address Semantics
Address fields are one byte (`uint8_t`) and use the following meanings:

- **Unicast:** any non-zero, non-reserved destination (`0x01` to `0xFE`) is treated as a unicast destination address.
- **Broadcast:** destination `0x00` targets every node that is configured to accept broadcasts.
- **Reserved:** `0xFF` is reserved and should not be assigned as a normal node address.

### Parser destination filtering modes
Destination filtering is configurable through `MuBusConfig::destination_filter_mode`:

- `DestinationFilterMode::AddressedOrBroadcast` (default): accept frames addressed to this node or broadcast (`0x00`).
- `DestinationFilterMode::AcceptBroadcastOnly`: accept only broadcast frames.
- `DestinationFilterMode::AcceptUnicastOnly`: accept only unicast frames addressed to this node.
- `DestinationFilterMode::Promiscuous`: accept all destinations (debug/sniffer mode).

Example:
```cpp
MuBus::MuBusConfig config;
config.destination_filter_mode = MuBus::DestinationFilterMode::Promiscuous;
MuBus::MuBusNode node(&Serial1, 0x01, config);
```

## Optional parser worker mode
`MuBusNode` can now run parsing in either:
- a dedicated worker thread (mbed targets with RTOS), or
- a cooperative single-threaded path via `tick()` (Arduino and minimal targets).

### Frame callback
Register a callback that fires whenever a complete frame is accepted:
```cpp
void onFrameReady(const MuBus::MuBusNode::Frame &frame) {
  // frame.payload points to valid bytes for frame.len
}

node.onFrame(onFrameReady);
```

### mbed RTOS worker thread
```cpp
// uses previously configured values
node.startParserThread();

// or configure poll interval + stop timeout (ms)
node.startParserThread(2, 200);

// stop and join worker
node.stopParserThread();
```

### Arduino / non-RTOS cooperative parser
Call `tick()` from your main loop to use the same parser core without threading:
```cpp
void loop() {
  node.tick();
  // application work
}
```

## Memory Footprint
Packet RX/TX storage is now embedded in `MuBusNode` as fixed-size arrays (no per-frame dynamic allocation in the hot path).

Let:
- `H = 6` (`kHeaderSize`)
- `P = max_payload_size` (bounded by `kMaxPayload`, default 506)
- `M = metadata per queued entry` (`src[1] + dst[1] + len[2] = 4` bytes)

Then:

- **Single-slot mode footprint** (RX single-slot + TX direct):
  - Parser scratch buffer: `P`
  - Pending RX frame storage: `H + P + M`
  - Total hot-path buffer footprint: `2P + H + M`

- **Ring-buffer mode footprint per entry**:
  - `H + P + M`
  - With defaults: `6 + 506 + 4 = 516` bytes per entry.

Total queue storage in ring/queued mode is:
- RX: `rx_queue_depth * (H + P + M)`
- TX: `tx_queue_depth * (H + P + M)`

## Protocol Spec
Frame layout is defined byte-for-byte as:

- `SYNC[2]`: `0xD3`, `0x91`
- `SRC[1]`: source node address
- `DST[1]`: destination node address (`0x00` for broadcast, `0xFF` reserved)
- `LEN[2]`: payload length in **little-endian** byte order (`LEN[0]` = low byte, `LEN[1]` = high byte)
- `PAYLOAD[LEN]`: payload bytes
- optional `CRC[1|2]`: present only when `MuBusConfig::crc_mode` is `Crc8` or `Crc16`

Total frame size is `6 + LEN + crc_bytes`, where `crc_bytes` is:
- `0` for `CrcMode::None`
- `1` for `CrcMode::Crc8`
- `2` for `CrcMode::Crc16`

## Diagnostics
Use parser diagnostics for runtime telemetry and debugging:

```cpp
MuBus::MuBusConfig config;
config.crc_mode = MuBus::CrcMode::Crc16;
config.parser_timeout_ms = 20;

MuBus::MuBusNode node(&Serial1, 0x01, config);
auto diag = node.getDiagnostics();
// diag.sync_errors
// diag.destination_mismatch
// diag.length_overflow
// diag.crc_fail
// diag.timeout_count
// diag.drop_count

node.resetDiagnostics();
```

RX frames now have CRC verification before entering the queue/callback path when CRC is configured.
