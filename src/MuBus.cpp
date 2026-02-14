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

bool MuBusNode::broadcast(uint8_t *buf, uint16_t len) {
  if (len > kMaxPayload) {
    return false;
  }
  if (len > 0 && buf == nullptr) {
    return false;
  }
  out_packet_->bindDest(0x00);
  out_packet_->setSize(len);
  port_->write(out_packet_->serialize(), kHeaderSize);
  if (len > 0) {
    port_->write(buf, len);
  }
  return true;
}

bool MuBusNode::send(uint8_t *buf, uint16_t len, uint8_t recv_addr) {
  if (len > kMaxPayload) {
    return false;
  }
  if (len > 0 && buf == nullptr) {
    return false;
  }
  out_packet_->bindDest(recv_addr);
  out_packet_->setSize(len);
  port_->write(out_packet_->serialize(), kHeaderSize);
  if (len > 0) {
    port_->write(buf, len);
  }
  return true;
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

#ifdef MUBUS_MBED
bool MuBusNode::parse() {
  if (port_->readable()) {
    uint8_t header[kHeaderSize];
    port_->read(header, 1);
    if (header[0] != kSync0) {
      return false;
    }

    while (!port_->readable()) {
      rtos::ThisThread::sleep_for(2);
    }
    port_->read(header + 1, 1);
    if (header[1] != kSync1) {
      return false;
    }

    while (port_->readable() < (kHeaderSize - 2)) {
      rtos::ThisThread::sleep_for(2);
    }
    port_->read(header + 2, kHeaderSize - 2);
    if (!decodeHeaderBytewise(header, *in_packet_)) {
      return false;
    }
    if (in_packet_->getDest() != out_packet_->getSource() &&
        in_packet_->getDest() != 0x00) {
      return false;
    }
    if (in_packet_->getSize() > kMaxPayload) {
      return false;
    }

    uint16_t accumulated = 0;
    while (accumulated < in_packet_->getSize()) {
      if (port_->readable()) {
        port_->read(in_buf_ + accumulated, 1);
        accumulated++;
      } else {
        rtos::ThisThread::sleep_for(2);
      }
    }
    return true;
  } else {
    rtos::ThisThread::sleep_for(5);
  }

  return false;
}
#else
bool MuBusNode::parse() {
  if (port_->available()) {
    uint8_t header[kHeaderSize];
    header[0] = port_->read();
    if (header[0] != kSync0) {
      return false;
    }
    while (!port_->available()) {
      delay(2);
    }
    header[1] = port_->read();
    if (header[1] != kSync1) {
      return false;
    }
    while (port_->available() < (kHeaderSize - 2)) {
      delay(2);
    }
    for (uint8_t i = 2; i < kHeaderSize; ++i) {
      header[i] = port_->read();
    }
    if (!decodeHeaderBytewise(header, *in_packet_)) {
      return false;
    }
    if (in_packet_->getDest() != out_packet_->getSource() &&
        in_packet_->getDest() != 0x00) {
      return false;
    }
    if (in_packet_->getSize() > kMaxPayload) {
      return false;
    }

    uint16_t accumulated = 0;
    while (accumulated < in_packet_->getSize()) {
      uint16_t remaining = in_packet_->getSize() - accumulated;
      uint16_t available = port_->available();
      if (available) {
        uint16_t read_max = min(remaining, available);
        port_->readBytes(in_buf_ + accumulated, read_max);
        accumulated += read_max;
      } else {
        delay(2);
      }
    }
    return true;
  }
  return false;
}
#endif
} // namespace MuBus
