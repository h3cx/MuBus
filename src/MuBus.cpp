#include "MuBus.h"

#include <stdio.h>

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

MuBusNode::MuBusNode() : out_packet_(new MuPacketHeader()) { applyConfig(config_); }
MuBusNode::MuBusNode(uint8_t addr) : out_packet_(new MuPacketHeader(addr)) {
  applyConfig(config_);
}
MuBusNode::MuBusNode(uint8_t addr, const MuBusConfig &config)
    : config_(config), out_packet_(new MuPacketHeader(addr)) {
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport)
    : transport_(transport), out_packet_(new MuPacketHeader) {
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport, uint8_t addr)
    : transport_(transport), out_packet_(new MuPacketHeader(addr)) {
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport, uint8_t addr,
                     const MuBusConfig &config)
    : transport_(transport), config_(config), out_packet_(new MuPacketHeader(addr)) {
  applyConfig(config_);
}

MuBusNode::~MuBusNode() {
  if (owns_transport_) {
    delete transport_;
  }
  delete in_packet_;
  delete out_packet_;
}

void MuBusNode::applyConfig(const MuBusConfig &config) {
  config_ = config;
  if (config_.max_payload_size > kMaxPayload) {
    config_.max_payload_size = kMaxPayload;
  }
  if (config_.rx_queue_depth < 1) {
    config_.rx_queue_depth = 1;
  }
  if (config_.tx_queue_depth < 1) {
    config_.tx_queue_depth = 1;
  }
  if (config_.rx_queue_depth > kMaxRxQueueDepth) {
    config_.rx_queue_depth = kMaxRxQueueDepth;
  }
  if (config_.tx_queue_depth > kMaxTxQueueDepth) {
    config_.tx_queue_depth = kMaxTxQueueDepth;
  }
  if (config_.rx_mode == RxMode::SingleSlot) {
    config_.rx_queue_depth = 1;
  }
  if (config_.tx_mode == TxMode::Direct) {
    config_.tx_queue_depth = 1;
  }
}

bool MuBusNode::assignTransport(MuTransport *transport, bool take_ownership,
                                uint8_t addr, const MuBusConfig &config) {
  if (owns_transport_) {
    delete transport_;
  }
  transport_ = transport;
  owns_transport_ = take_ownership;
  applyConfig(config);
  bindAddr(addr);
  resetParser();
  has_pending_frame_ = false;
  rx_head_ = rx_tail_ = rx_count_ = 0;
  tx_head_ = tx_tail_ = tx_count_ = 0;
  status_ = NodeStatus{};
  return transport_ != nullptr;
}

bool MuBusNode::begin(MuTransport *transport, uint8_t addr) {
  return assignTransport(transport, false, addr, config_);
}

bool MuBusNode::begin(MuTransport *transport, uint8_t addr,
                      const MuBusConfig &config) {
  return assignTransport(transport, false, addr, config);
}

void MuBusNode::stop() {
  if (owns_transport_) {
    delete transport_;
  }
  transport_ = nullptr;
  owns_transport_ = false;
  resetParser();
  has_pending_frame_ = false;
  frame_callback_ = nullptr;
  rx_head_ = rx_tail_ = rx_count_ = 0;
  tx_head_ = tx_tail_ = tx_count_ = 0;
  status_ = NodeStatus{};
}

uint8_t MuBusNode::computeCrc(uint8_t src, uint8_t dst, uint16_t len,
                              const uint8_t *payload) const {
  uint8_t crc = 0;
  crc ^= src;
  crc ^= dst;
  crc ^= static_cast<uint8_t>(len & 0xFF);
  crc ^= static_cast<uint8_t>((len >> 8) & 0xFF);
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= payload[i];
  }
  return crc;
}

bool MuBusNode::writeFrameNow(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (transport_ == nullptr || len > config_.max_payload_size) {
    return false;
  }
  if (len > 0 && data == nullptr) {
    return false;
  }

  out_packet_->bindDest(dst);
  out_packet_->setSize(len);
  if (!transport_->write(out_packet_->serialize(), kHeaderSize)) {
    return false;
  }
  if (len > 0 && !transport_->write(data, len)) {
    return false;
  }

  if (config_.crc_enabled) {
    const uint8_t crc = computeCrc(out_packet_->getSource(), dst, len, data);
    return transport_->write(&crc, 1);
  }

  static const uint8_t zero_crc = 0;
  return transport_->write(&zero_crc, 1);
}

bool MuBusNode::enqueueTxFrame(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (tx_count_ >= config_.tx_queue_depth) {
    status_.tx_queue_full = true;
    status_.dropped_frame_count++;
    return false;
  }

  FrameSlot &slot = tx_slots_[tx_tail_];
  slot.src = out_packet_->getSource();
  slot.dst = dst;
  slot.len = len;
  for (uint16_t i = 0; i < len; ++i) {
    slot.payload[i] = data[i];
  }

  tx_tail_ = static_cast<uint8_t>((tx_tail_ + 1) % kMaxTxQueueDepth);
  tx_count_++;
  return true;
}

bool MuBusNode::flushTxQueue() {
  while (tx_count_ > 0) {
    FrameSlot &slot = tx_slots_[tx_head_];
    if (!writeFrameNow(slot.dst, slot.payload, slot.len)) {
      return false;
    }
    tx_head_ = static_cast<uint8_t>((tx_head_ + 1) % kMaxTxQueueDepth);
    tx_count_--;
  }
  status_.tx_queue_full = false;
  return true;
}

bool MuBusNode::send(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (len > config_.max_payload_size) {
    return false;
  }

  if (txQueueEnabled()) {
    const bool queued = enqueueTxFrame(dst, data, len);
    (void)flushTxQueue();
    return queued;
  }

  return writeFrameNow(dst, data, len);
}

bool MuBusNode::broadcast(const uint8_t *data, uint16_t len) {
  return send(0x00, data, len);
}

void MuBusNode::bindAddr(uint8_t addr) { out_packet_->bindSource(addr); }
uint8_t *MuBusNode::getPayload() { return pending_frame_.payload; }
uint16_t MuBusNode::getPayloadSize() { return pending_frame_.len; }
String MuBusNode::formatHeader() {
  char msg_buf[64];
  snprintf(msg_buf, sizeof(msg_buf), "Source: 0x%02X\nDest: 0x%02X\nSize: 0x%04X",
           pending_frame_.src, pending_frame_.dst, pending_frame_.len);
  return String(msg_buf);
}

bool MuBusNode::rxQueueEnabled() const { return config_.rx_mode == RxMode::Ring; }
bool MuBusNode::txQueueEnabled() const { return config_.tx_mode == TxMode::Queue; }

bool MuBusNode::enqueueRxFrame(const Frame &frame) {
  if (rx_count_ >= config_.rx_queue_depth) {
    status_.rx_queue_full = true;
    status_.dropped_frame_count++;
    return false;
  }

  FrameSlot &slot = rx_slots_[rx_tail_];
  slot.src = frame.src;
  slot.dst = frame.dst;
  slot.len = frame.len;
  for (uint16_t i = 0; i < frame.len; ++i) {
    slot.payload[i] = frame.payload[i];
  }

  rx_tail_ = static_cast<uint8_t>((rx_tail_ + 1) % kMaxRxQueueDepth);
  rx_count_++;
  return true;
}

bool MuBusNode::dequeueRxFrame(Frame &frame) {
  if (rx_count_ == 0) {
    return false;
  }

  FrameSlot &slot = rx_slots_[rx_head_];
  frame.src = slot.src;
  frame.dst = slot.dst;
  frame.len = slot.len;
  frame.payload = slot.payload;

  rx_head_ = static_cast<uint8_t>((rx_head_ + 1) % kMaxRxQueueDepth);
  rx_count_--;
  if (rx_count_ < config_.rx_queue_depth) {
    status_.rx_queue_full = false;
  }
  return true;
}

void MuBusNode::poll() {
  Frame frame;
  while (readFrame(frame)) {
    if (rxQueueEnabled()) {
      if (!enqueueRxFrame(frame)) {
        continue;
      }

      if (frame_callback_ != nullptr) {
        const uint8_t callback_index =
            static_cast<uint8_t>((rx_tail_ + kMaxRxQueueDepth - 1) % kMaxRxQueueDepth);
        Frame callback_frame;
        callback_frame.src = rx_slots_[callback_index].src;
        callback_frame.dst = rx_slots_[callback_index].dst;
        callback_frame.len = rx_slots_[callback_index].len;
        callback_frame.payload = rx_slots_[callback_index].payload;
        frame_callback_(callback_frame);
      }
    } else {
      pending_frame_ = frame;
      has_pending_frame_ = true;
      if (frame_callback_ != nullptr) {
        frame_callback_(pending_frame_);
      }
    }
  }

  if (txQueueEnabled()) {
    (void)flushTxQueue();
  }
}

bool MuBusNode::available() {
  poll();
  if (rxQueueEnabled()) {
    return rx_count_ > 0;
  }
  return has_pending_frame_;
}

bool MuBusNode::receive(Frame &frame) {
  poll();

  if (rxQueueEnabled()) {
    return dequeueRxFrame(frame);
  }

  if (!has_pending_frame_) {
    return false;
  }

  frame = pending_frame_;
  has_pending_frame_ = false;
  return true;
}

void MuBusNode::onFrame(FrameCallback callback) { frame_callback_ = callback; }

MuBusNode::NodeStatus MuBusNode::getStatus() const { return status_; }

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

void MuBusNode::resetParser() { parser_ = ParserContext{}; }

bool MuBusNode::readTransportByte(uint8_t &byte) {
  if (transport_ == nullptr) {
    return false;
  }
  return transport_->readByte(byte);
}

bool MuBusNode::finalizeParsedFrame(uint8_t crc_byte, Frame &frame) {
  in_packet_->bindSource(parser_.src);
  in_packet_->bindDest(parser_.dst);
  in_packet_->setSize(parser_.len);

  if (in_packet_->getDest() != out_packet_->getSource() &&
      in_packet_->getDest() != 0x00) {
    return false;
  }

  if (config_.crc_enabled) {
    const uint8_t expected_crc =
        computeCrc(parser_.src, parser_.dst, parser_.len, parser_payload_);
    if (crc_byte != expected_crc) {
      return false;
    }
  }

  frame.src = in_packet_->getSource();
  frame.dst = in_packet_->getDest();
  frame.len = in_packet_->getSize();
  frame.payload = parser_payload_;
  return true;
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

    if (parser_.len > config_.max_payload_size) {
      parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
      return false;
    }

    parser_.state = (parser_.len == 0) ? ParserState::Crc : ParserState::Payload;
    return false;

  case ParserState::Payload:
    parser_payload_[parser_.payload_index++] = byte;
    if (parser_.payload_index < parser_.len) {
      return false;
    }

    parser_.state = ParserState::Crc;
    return false;

  case ParserState::Crc: {
    const bool complete = finalizeParsedFrame(byte, frame);
    resetParser();
    return complete;
  }
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
