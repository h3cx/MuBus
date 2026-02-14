#define MUBUS_MBED
#include "Arduino.h"
#include "HardwareSerial.h"
#ifdef MUBUS_MBED
#include "mbed.h"
#endif // MUBUS_MBED
namespace MuBus {

static constexpr uint8_t kSync0 = 0xD3;
static constexpr uint8_t kSync1 = 0x91;
static constexpr uint8_t kHeaderSize = 6;
static constexpr uint16_t kMaxPayload = 506;

class MuPacketHeader;

void encodeHeaderBytewise(uint8_t *out, uint8_t src, uint8_t dst, uint16_t len);
bool decodeHeaderBytewise(const uint8_t *in, MuPacketHeader &header);

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
  static constexpr uint32_t kTransportReadTimeoutMs = 5;
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

  #ifdef MUBUS_MBED
  mbed::BufferedSerial *port_ = nullptr;
  #else
  HardwareSerial *port_ = nullptr;
  #endif
  MuPacketHeader *out_packet_;
  MuPacketHeader *in_packet_ = new MuPacketHeader();
  uint8_t *in_buf_ = (uint8_t *)malloc(kMaxPayload);
  bool has_pending_frame_ = false;
  ParserContext parser_{};

public:
  struct Frame {
    uint8_t src = 0x00;
    uint8_t dst = 0x00;
    uint16_t len = 0x0000;
    uint8_t *payload = nullptr;
  };

  using FrameCallback = void (*)(const Frame &frame);

private:
  Frame pending_frame_{};
  FrameCallback frame_callback_ = nullptr;
  bool readFrame(Frame &frame);
  void resetParser();
#ifdef MUBUS_MBED
  bool readTransportBytes(uint8_t *buffer, size_t len, uint32_t timeout_ms,
                          size_t &bytes_read);
#endif
  bool readTransportByte(uint8_t &byte);
  bool parseByte(uint8_t byte, Frame &frame);

public:
  MuBusNode();
  MuBusNode(uint8_t addr);
  #ifdef MUBUS_MBED
  MuBusNode(mbed::BufferedSerial *port);
  MuBusNode(mbed::BufferedSerial *port, uint8_t addr);
  #else
  MuBusNode(HardwareSerial *port);
  MuBusNode(HardwareSerial *port, uint8_t addr);
  #endif

  #ifdef MUBUS_MBED
  bool begin(mbed::BufferedSerial *port, uint8_t addr);
  #else
  bool begin(HardwareSerial *port, uint8_t addr);
  #endif
  void stop();

  void bindAddr(uint8_t addr);
  bool send(uint8_t dst, const uint8_t *data, uint16_t len);
  bool broadcast(const uint8_t *data, uint16_t len);

  bool available();
  bool receive(Frame &frame);
  void onFrame(FrameCallback callback);

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
