#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define MUBUS_RUNTIME_SELECTION_COUNT                                          \
  (defined(MUBUS_RUNTIME_ARDUINO) + defined(MUBUS_RUNTIME_MBED) +             \
   defined(MUBUS_RUNTIME_FREERTOS))

#if MUBUS_RUNTIME_SELECTION_COUNT == 0
#error "Select one runtime: define exactly one of MUBUS_RUNTIME_ARDUINO, MUBUS_RUNTIME_MBED, or MUBUS_RUNTIME_FREERTOS"
#elif MUBUS_RUNTIME_SELECTION_COUNT > 1
#error "Only one runtime may be selected: define exactly one MUBUS_RUNTIME_* macro"
#endif

#if defined(MUBUS_ENABLE_PARSER_THREAD) && defined(MUBUS_DISABLE_PARSER_THREAD)
#error "Choose only one parser thread control macro: define MUBUS_ENABLE_PARSER_THREAD or MUBUS_DISABLE_PARSER_THREAD"
#endif

#if !defined(MUBUS_ENABLE_PARSER_THREAD) && !defined(MUBUS_DISABLE_PARSER_THREAD)
#if defined(MUBUS_RUNTIME_MBED) || defined(MUBUS_RUNTIME_FREERTOS)
#define MUBUS_ENABLE_PARSER_THREAD
#endif
#endif

#ifdef MUBUS_RUNTIME_ARDUINO
#include <Arduino.h>
#endif


#ifdef MUBUS_RUNTIME_ARDUINO
class HardwareSerial;
#endif

#ifdef MUBUS_RUNTIME_MBED
namespace mbed {
class BufferedSerial;
}
namespace rtos {
class Thread;
class Mutex;
class EventFlags;
class Semaphore;
} // namespace rtos
#endif

#ifdef MUBUS_RUNTIME_FREERTOS
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

namespace MuBus {

static constexpr uint8_t kSync0 = 0xD3;
static constexpr uint8_t kSync1 = 0x91;
static constexpr uint8_t kBroadcastAddress = 0x00;
static constexpr uint8_t kReservedAddress = 0xFF;
static constexpr uint8_t kHeaderSize = 6;
static constexpr uint16_t kMaxPayload = 506;
static constexpr uint8_t kMaxRxQueueDepth = 8;
static constexpr uint8_t kMaxTxQueueDepth = 8;
static constexpr uint32_t kDefaultParserThreadStackBytes = 4096;

static_assert(kHeaderSize == 6, "MuBus header must be exactly 6 bytes");
static_assert(kHeaderSize + kMaxPayload == 512,
              "MuBus max frame size must remain 512 bytes");
static_assert(kMaxRxQueueDepth > 0,
              "kMaxRxQueueDepth must be at least 1 for single-slot fallback");
static_assert(kMaxTxQueueDepth > 0,
              "kMaxTxQueueDepth must be at least 1 for direct TX fallback");
static constexpr size_t kFrameSlotMetadataSize =
    sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t);
static constexpr size_t kFrameSlotFootprint =
    kHeaderSize + kMaxPayload + kFrameSlotMetadataSize;

static_assert(kMaxPayload <= UINT16_MAX,
              "kMaxPayload must fit in packet length field");
static_assert(kMaxRxQueueDepth <= UINT8_MAX,
              "kMaxRxQueueDepth must fit queue index counters");
static_assert(kMaxTxQueueDepth <= UINT8_MAX,
              "kMaxTxQueueDepth must fit queue index counters");

enum class RxMode : uint8_t { SingleSlot, Ring };
enum class TxMode : uint8_t { Direct, Queue };
enum class CrcMode : uint8_t { None, Crc8, Crc16 };
enum class DestinationFilterMode : uint8_t {
  AddressedOrBroadcast,
  AcceptBroadcastOnly,
  AcceptUnicastOnly,
  Promiscuous,
};

struct MuBusDiagnostics {
  uint32_t sync_errors = 0;
  uint32_t destination_mismatch = 0;
  uint32_t length_overflow = 0;
  uint32_t crc_fail = 0;
  uint32_t timeout_count = 0;
  uint32_t drop_count = 0;
};

struct MuBusConfig {
  RxMode rx_mode = RxMode::SingleSlot;
  TxMode tx_mode = TxMode::Direct;
  uint8_t rx_queue_depth = 1;
  uint8_t tx_queue_depth = 1;
  uint16_t max_payload_size = kMaxPayload;
  CrcMode crc_mode = CrcMode::None;
  uint32_t parser_timeout_ms = 0;
  uint32_t parser_thread_stack_bytes = kDefaultParserThreadStackBytes;
  DestinationFilterMode destination_filter_mode =
      DestinationFilterMode::AddressedOrBroadcast;
};

static_assert(static_cast<uint8_t>(RxMode::SingleSlot) !=
                  static_cast<uint8_t>(RxMode::Ring),
              "RxMode enum values must stay distinct");
static_assert(static_cast<uint8_t>(TxMode::Direct) !=
                  static_cast<uint8_t>(TxMode::Queue),
              "TxMode enum values must stay distinct");

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
  uint8_t getSource() const;
  void bindDest(uint8_t addr);
  uint8_t getDest() const;
  void setSize(uint16_t size);
  uint16_t getSize() const;
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
    uint16_t rx_crc = 0x0000;
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
  MuBusDiagnostics diagnostics_{};
  MuPacketHeader out_packet_{};
  MuPacketHeader in_packet_{};
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

  struct ParserThreadConfig {
    uint32_t poll_interval_ms = 1;
    uint32_t stop_timeout_ms = 100;
  };

  ParserThreadConfig parser_thread_config_{};
  bool parser_thread_running_ = false;
  uint32_t parser_last_byte_ms_ = 0;

#ifdef MUBUS_ENABLE_PARSER_THREAD
#ifdef MUBUS_RUNTIME_MBED
  rtos::Thread *parser_thread_ = nullptr;
  rtos::Mutex *state_mutex_ = nullptr;
  rtos::EventFlags *parser_thread_flags_ = nullptr;
  rtos::Semaphore *parser_thread_stopped_ = nullptr;
#elif defined(MUBUS_RUNTIME_FREERTOS)
  TaskHandle_t parser_thread_ = nullptr;
  SemaphoreHandle_t state_mutex_ = nullptr;
  EventGroupHandle_t parser_thread_flags_ = nullptr;
  SemaphoreHandle_t parser_thread_stopped_ = nullptr;
#endif
#endif

public:
  struct Frame {
    uint8_t src = 0x00;
    uint8_t dst = 0x00;
    uint16_t len = 0x0000;
    uint8_t *payload = nullptr;
  };

  using NodeStatus = MuBusStatus;
  using Diagnostics = MuBusDiagnostics;
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
  uint8_t computeCrc8(uint8_t src, uint8_t dst, uint16_t len,
                      const uint8_t *payload) const;
  uint16_t computeCrc16(uint8_t src, uint8_t dst, uint16_t len,
                        const uint8_t *payload) const;
  bool finalizeParsedFrame(Frame &frame);
  bool shouldAcceptDestination(uint8_t dst) const;
  void resetParser();
  void updateParserTimeout();
  uint8_t crcFieldSize() const;
  bool readTransportByte(uint8_t &byte);
  bool parseByte(uint8_t byte, Frame &frame);
  bool assignTransport(MuTransport *transport, bool take_ownership, uint8_t addr,
                       const MuBusConfig &config);
  void applyConfig(const MuBusConfig &config);
  bool rxQueueEnabled() const;
  bool txQueueEnabled() const;
  void pollCore();
  void lockState();
  void unlockState();
#ifdef MUBUS_ENABLE_PARSER_THREAD
  void parserThreadLoop();
#ifdef MUBUS_RUNTIME_FREERTOS
  static void parserThreadEntry(void *arg);
#endif
#endif

public:
  MuBusNode();
  MuBusNode(uint8_t addr);
  MuBusNode(uint8_t addr, const MuBusConfig &config);
  MuBusNode(MuTransport *transport);
  MuBusNode(MuTransport *transport, uint8_t addr);
  MuBusNode(MuTransport *transport, uint8_t addr, const MuBusConfig &config);
#ifdef MUBUS_RUNTIME_ARDUINO
  MuBusNode(HardwareSerial *port);
  MuBusNode(HardwareSerial *port, uint8_t addr);
  MuBusNode(HardwareSerial *port, uint8_t addr,
            const MuBusConfig &config);
#endif
#ifdef MUBUS_RUNTIME_MBED
  MuBusNode(mbed::BufferedSerial *port);
  MuBusNode(mbed::BufferedSerial *port, uint8_t addr);
  MuBusNode(mbed::BufferedSerial *port, uint8_t addr,
            const MuBusConfig &config);
#endif
  ~MuBusNode();

  bool begin(MuTransport *transport, uint8_t addr);
  bool begin(MuTransport *transport, uint8_t addr, const MuBusConfig &config);
#ifdef MUBUS_RUNTIME_ARDUINO
  bool begin(HardwareSerial *port, uint8_t addr);
  bool begin(HardwareSerial *port, uint8_t addr,
             const MuBusConfig &config);
#endif
#ifdef MUBUS_RUNTIME_MBED
  bool begin(mbed::BufferedSerial *port, uint8_t addr);
  bool begin(mbed::BufferedSerial *port, uint8_t addr, const MuBusConfig &config);
#endif
  void stop();

  void bindAddr(uint8_t addr);
  bool send(uint8_t dst, const uint8_t *data, uint16_t len);
  bool broadcast(const uint8_t *data, uint16_t len);

  bool available();
  bool receive(Frame &frame);
  void onFrame(FrameCallback callback);
  void poll();
  void tick();
  bool startParserThread();
  bool startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms);
  bool stopParserThread();
  NodeStatus getStatus() const;
  Diagnostics getDiagnostics() const;
  void resetDiagnostics();

  // Deprecated: use send(dst, data, len) instead.
  bool send(uint8_t *buf, uint16_t len, uint8_t recv_addr);
  // Deprecated: use broadcast(data, len) with const payload.
  bool broadcast(uint8_t *buf, uint16_t len);
  // Deprecated: use receive(Frame&) or onFrame(...) instead.
  bool parse();
  // Deprecated: use receive(Frame&) and Frame::payload instead.
  uint8_t *getPayload();
  uint16_t getPayloadSize();
  const char *formatHeader();

private:
  char formatted_header_[64] = {0};
};

} // namespace MuBus
