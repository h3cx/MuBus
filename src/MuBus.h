#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <string>
using String = std::string;
#endif

class HardwareSerial;
namespace mbed {
class BufferedSerial;
}

namespace MuBus {

static constexpr uint8_t kSync0 = 0xD3;
static constexpr uint8_t kSync1 = 0x91;
static constexpr uint8_t kHeaderSize = 6;
static constexpr uint16_t kMaxPayload = 506;
static constexpr uint8_t kMaxRxQueueDepth = 8;
static constexpr uint8_t kMaxTxQueueDepth = 8;

enum class RxMode : uint8_t { SingleSlot, Ring };
enum class TxMode : uint8_t { Direct, Queue };

struct MuBusConfig {
  RxMode rx_mode = RxMode::SingleSlot;
  TxMode tx_mode = TxMode::Direct;
  uint8_t rx_queue_depth = 1;
  uint8_t tx_queue_depth = 1;
  uint16_t max_payload_size = kMaxPayload;
  bool crc_enabled = false;
};

struct MuBusStatus {
  bool rx_queue_full = false;
  bool tx_queue_full = false;
  uint32_t dropped_frame_count = 0;
};

class MuPacketHeader;
class MuTransport;

void encodeHeaderBytewise(uint8_t *out, uint8_t src, uint8_t dst, uint16_t len);
bool decodeHeaderBytewise(const uint8_t *in, MuPacketHeader &header);

class MuTransport {
public:
  virtual ~MuTransport() = default;
  virtual bool write(const uint8_t *data, size_t len) = 0;
  virtual bool readByte(uint8_t &byte) = 0;
};

class MuPacketHeader {
private:
  uint8_t source_addr_ = 0x00;
  uint8_t dest_addr_ = 0x00;
  uint16_t body_size_ = 0x0000;
  uint8_t s_head_[kHeaderSize];

public:
  MuPacketHeader() = default;
  MuPacketHeader(uint8_t source_addr);
  MuPacketHeader(uint8_t source_addr, uint8_t dest_addr);
  void bindSource(uint8_t addr);
  uint8_t getSource();
  void bindDest(uint8_t addr);
  uint8_t getDest();
  void setSize(uint16_t size);
  uint16_t getSize();
  uint8_t *serialize();
};

class MuBusNode {
private:
  enum class ParserState : uint8_t {
    Sync0,
    Sync1,
    Src,
    Dst,
    Len0,
    Len1,
    Payload,
    Crc,
  };

  struct ParserContext {
    ParserState state = ParserState::Sync0;
    uint8_t src = 0x00;
    uint8_t dst = 0x00;
    uint16_t len = 0x0000;
    uint16_t payload_index = 0x0000;
  };

  struct FrameSlot {
    uint8_t src = 0x00;
    uint8_t dst = 0x00;
    uint16_t len = 0x0000;
    uint8_t payload[kMaxPayload] = {0};
  };

  MuTransport *transport_ = nullptr;
  bool owns_transport_ = false;
  MuBusConfig config_{};
  MuBusStatus status_{};
  MuPacketHeader *out_packet_;
  MuPacketHeader *in_packet_ = new MuPacketHeader();
  uint8_t parser_payload_[kMaxPayload] = {0};
  bool has_pending_frame_ = false;
  ParserContext parser_{};
  FrameSlot rx_slots_[kMaxRxQueueDepth] = {};
  uint8_t rx_head_ = 0;
  uint8_t rx_tail_ = 0;
  uint8_t rx_count_ = 0;

  FrameSlot tx_slots_[kMaxTxQueueDepth] = {};
  uint8_t tx_head_ = 0;
  uint8_t tx_tail_ = 0;
  uint8_t tx_count_ = 0;

public:
  struct Frame {
    uint8_t src = 0x00;
    uint8_t dst = 0x00;
    uint16_t len = 0x0000;
    uint8_t *payload = nullptr;
  };

  using NodeStatus = MuBusStatus;
  using FrameCallback = void (*)(const Frame &frame);

private:
  Frame pending_frame_{};
  FrameCallback frame_callback_ = nullptr;
  bool readFrame(Frame &frame);
  bool enqueueRxFrame(const Frame &frame);
  bool dequeueRxFrame(Frame &frame);
  bool enqueueTxFrame(uint8_t dst, const uint8_t *data, uint16_t len);
  bool flushTxQueue();
  bool writeFrameNow(uint8_t dst, const uint8_t *data, uint16_t len);
  uint8_t computeCrc(uint8_t src, uint8_t dst, uint16_t len,
                     const uint8_t *payload) const;
  bool finalizeParsedFrame(uint8_t crc_byte, Frame &frame);
  void resetParser();
  bool readTransportByte(uint8_t &byte);
  bool parseByte(uint8_t byte, Frame &frame);
  bool assignTransport(MuTransport *transport, bool take_ownership, uint8_t addr,
                       const MuBusConfig &config);
  void applyConfig(const MuBusConfig &config);
  bool rxQueueEnabled() const;
  bool txQueueEnabled() const;

public:
  MuBusNode();
  MuBusNode(uint8_t addr);
  MuBusNode(uint8_t addr, const MuBusConfig &config);
  MuBusNode(MuTransport *transport);
  MuBusNode(MuTransport *transport, uint8_t addr);
  MuBusNode(MuTransport *transport, uint8_t addr, const MuBusConfig &config);
  MuBusNode(HardwareSerial *port);
  MuBusNode(HardwareSerial *port, uint8_t addr);
  MuBusNode(mbed::BufferedSerial *port);
  MuBusNode(mbed::BufferedSerial *port, uint8_t addr);
  ~MuBusNode();

  bool begin(MuTransport *transport, uint8_t addr);
  bool begin(MuTransport *transport, uint8_t addr, const MuBusConfig &config);
  bool begin(HardwareSerial *port, uint8_t addr);
  bool begin(HardwareSerial *port, uint8_t addr, const MuBusConfig &config);
  bool begin(mbed::BufferedSerial *port, uint8_t addr);
  bool begin(mbed::BufferedSerial *port, uint8_t addr, const MuBusConfig &config);
  void stop();

  void bindAddr(uint8_t addr);
  bool send(uint8_t dst, const uint8_t *data, uint16_t len);
  bool broadcast(const uint8_t *data, uint16_t len);

  bool available();
  bool receive(Frame &frame);
  void onFrame(FrameCallback callback);
  void poll();
  NodeStatus getStatus() const;

  // Deprecated: use send(dst, data, len) instead.
  bool send(uint8_t *buf, uint16_t len, uint8_t recv_addr);
  // Deprecated: use broadcast(data, len) with const payload.
  bool broadcast(uint8_t *buf, uint16_t len);
  // Deprecated: use receive(Frame&) or onFrame(...) instead.
  bool parse();
  // Deprecated: use receive(Frame&) and Frame::payload instead.
  uint8_t *getPayload();
  uint16_t getPayloadSize();
  String formatHeader();
};

} // namespace MuBus
