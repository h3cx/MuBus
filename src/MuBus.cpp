#include "MuBus.h"

namespace MuBus {

void encodeHeaderBytewise(uint8_t *out, uint8_t src, uint8_t dst, uint16_t len) {
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = src;
  out[3] = dst;
  out[4] = static_cast<uint8_t>(len & 0xFF);
  out[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
}

bool decodeHeaderBytewise(const uint8_t *in, MuPacketHeader &header) {
  if (in[0] != kSync0 || in[1] != kSync1) {
    return false;
  }
  header.bindSource(in[2]);
  header.bindDest(in[3]);
  const uint16_t len = static_cast<uint16_t>(in[4]) |
                       (static_cast<uint16_t>(in[5]) << 8);
  header.setSize(len);
  return true;
}

MuPacketHeader::MuPacketHeader(uint8_t source_addr)
    : source_addr_(source_addr) {}
MuPacketHeader::MuPacketHeader(uint8_t source_addr, uint8_t dest_addr)
    : source_addr_(source_addr), dest_addr_(dest_addr) {}

void MuPacketHeader::bindSource(uint8_t addr) { source_addr_ = addr; }
void MuPacketHeader::bindDest(uint8_t addr) { dest_addr_ = addr; }
uint8_t MuPacketHeader::getSource() { return source_addr_; }
uint8_t MuPacketHeader::getDest() { return dest_addr_; }
void MuPacketHeader::setSize(uint16_t size) { body_size_ = size; }
uint16_t MuPacketHeader::getSize() { return body_size_; }

uint8_t *MuPacketHeader::serialize() {
  encodeHeaderBytewise(s_head_, source_addr_, dest_addr_, body_size_);
  return s_head_;
}

MuBusNode::MuBusNode() : out_packet_(new MuPacketHeader()) {}
MuBusNode::MuBusNode(uint8_t addr) : out_packet_(new MuPacketHeader(addr)) {}
#ifdef MUBUS_MBED
MuBusNode::MuBusNode(mbed::BufferedSerial *port)
    : port_(port), out_packet_(new MuPacketHeader) {}
MuBusNode::MuBusNode(mbed::BufferedSerial *port, uint8_t addr)
    : port_(port), out_packet_(new MuPacketHeader(addr)) {}
#else
MuBusNode::MuBusNode(HardwareSerial *port)
    : port_(port), out_packet_(new MuPacketHeader) {}
MuBusNode::MuBusNode(HardwareSerial *port, uint8_t addr)
    : port_(port), out_packet_(new MuPacketHeader(addr)) {}
#endif

#ifdef MUBUS_MBED
bool MuBusNode::begin(mbed::BufferedSerial *port, uint8_t addr) {
  port_ = port;
  bindAddr(addr);
  resetParser();
  has_pending_frame_ = false;
  return port_ != nullptr;
}
#else
bool MuBusNode::begin(HardwareSerial *port, uint8_t addr) {
  port_ = port;
  bindAddr(addr);
  resetParser();
  has_pending_frame_ = false;
  return port_ != nullptr;
}
#endif

void MuBusNode::stop() {
  port_ = nullptr;
  resetParser();
  has_pending_frame_ = false;
  frame_callback_ = nullptr;
}

bool MuBusNode::send(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (port_ == nullptr) {
    return false;
  }
  if (len > kMaxPayload) {
    return false;
  }
  if (len > 0 && data == nullptr) {
    return false;
  }
  out_packet_->bindDest(dst);
  out_packet_->setSize(len);
  port_->write(out_packet_->serialize(), kHeaderSize);
  if (len > 0) {
    port_->write(data, len);
  }
  return true;
}

bool MuBusNode::broadcast(const uint8_t *data, uint16_t len) {
  return send(0x00, data, len);
}

void MuBusNode::bindAddr(uint8_t addr) { out_packet_->bindSource(addr); }
uint8_t *MuBusNode::getPayload() { return in_buf_; }
uint16_t MuBusNode::getPayloadSize() { return in_packet_->getSize(); }
String MuBusNode::formatHeader() {
  char msg_buf[64];
  snprintf(msg_buf, sizeof(msg_buf),
           "Source: 0x%02X\nDest: 0x%02X\nSize: 0x%04X", in_packet_->getSource(),
           in_packet_->getDest(), in_packet_->getSize());
  return String(msg_buf);
}

bool MuBusNode::available() {
  if (has_pending_frame_) {
    return true;
  }

  Frame frame;
  if (!readFrame(frame)) {
    return false;
  }

  pending_frame_ = frame;
  has_pending_frame_ = true;
  if (frame_callback_ != nullptr) {
    frame_callback_(pending_frame_);
  }
  return true;
}

bool MuBusNode::receive(Frame &frame) {
  if (!has_pending_frame_ && !available()) {
    return false;
  }

  frame = pending_frame_;
  has_pending_frame_ = false;
  return true;
}

void MuBusNode::onFrame(FrameCallback callback) { frame_callback_ = callback; }

bool MuBusNode::send(uint8_t *buf, uint16_t len, uint8_t recv_addr) {
  return send(recv_addr, buf, len);
}

bool MuBusNode::broadcast(uint8_t *buf, uint16_t len) {
  return broadcast(static_cast<const uint8_t *>(buf), len);
}

bool MuBusNode::parse() {
  Frame frame;
  if (!receive(frame)) {
    return false;
  }
  return true;
}

void MuBusNode::resetParser() {
  parser_ = ParserContext{};
}

#ifdef MUBUS_MBED
bool MuBusNode::readTransportBytes(uint8_t *buffer, size_t len,
                                  uint32_t timeout_ms,
                                  size_t &bytes_read) {
  bytes_read = 0;
  if (buffer == nullptr || len == 0 || port_ == nullptr) {
    return false;
  }

  const uint32_t start_ms = millis();
  while (bytes_read < len) {
    if (port_->readable()) {
      const ssize_t result = port_->read(buffer + bytes_read, len - bytes_read);
      if (result > 0) {
        bytes_read += static_cast<size_t>(result);
        continue;
      }
      if (result < 0) {
        return false;
      }
    }

    if (timeout_ms == 0) {
      break;
    }

    if (static_cast<uint32_t>(millis() - start_ms) >= timeout_ms) {
      break;
    }

    delay(1);
  }

  return bytes_read > 0;
}

#endif
bool MuBusNode::readTransportByte(uint8_t &byte) {
  if (port_ == nullptr) {
    return false;
  }

#ifdef MUBUS_MBED
  size_t bytes_read = 0;
  if (!readTransportBytes(&byte, 1, kTransportReadTimeoutMs, bytes_read)) {
    return false;
  }
  return bytes_read == 1;
#else
  if (!port_->available()) {
    return false;
  }

  const int value = port_->read();
  if (value < 0) {
    return false;
  }

  byte = static_cast<uint8_t>(value);
  return true;
#endif
}

bool MuBusNode::parseByte(uint8_t byte, Frame &frame) {
  switch (parser_.state) {
  case ParserState::Sync0:
    if (byte == kSync0) {
      parser_.state = ParserState::Sync1;
    }
    return false;

  case ParserState::Sync1:
    if (byte == kSync1) {
      parser_.state = ParserState::Src;
      return false;
    }

    parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
    return false;

  case ParserState::Src:
    parser_.src = byte;
    parser_.state = ParserState::Dst;
    return false;

  case ParserState::Dst:
    parser_.dst = byte;
    parser_.state = ParserState::Len0;
    return false;

  case ParserState::Len0:
    parser_.len = byte;
    parser_.state = ParserState::Len1;
    return false;

  case ParserState::Len1:
    parser_.len |= static_cast<uint16_t>(byte) << 8;
    parser_.payload_index = 0;

    if (parser_.len > kMaxPayload) {
      parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
      return false;
    }

    parser_.state = (parser_.len == 0) ? ParserState::Crc : ParserState::Payload;
    return false;

  case ParserState::Payload:
    in_buf_[parser_.payload_index++] = byte;
    if (parser_.payload_index < parser_.len) {
      return false;
    }

    parser_.state = ParserState::Crc;
    return false;

  case ParserState::Crc:
    in_packet_->bindSource(parser_.src);
    in_packet_->bindDest(parser_.dst);
    in_packet_->setSize(parser_.len);

    if (in_packet_->getDest() != out_packet_->getSource() &&
        in_packet_->getDest() != 0x00) {
      parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
      return false;
    }

    frame.src = in_packet_->getSource();
    frame.dst = in_packet_->getDest();
    frame.len = in_packet_->getSize();
    frame.payload = in_buf_;
    resetParser();
    return true;
  }

  resetParser();
  return false;
}

bool MuBusNode::readFrame(Frame &frame) {
  uint8_t byte = 0;
  while (readTransportByte(byte)) {
    if (parseByte(byte, frame)) {
      return true;
    }
  }

  return false;
}

} // namespace MuBus
