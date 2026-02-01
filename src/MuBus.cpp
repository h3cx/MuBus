#include "MuBus.h"

namespace MuBus {
MuPacketHeader::MuPacketHeader(uint8_t source_addr)
    : source_addr_(source_addr) {}
MuPacketHeader::MuPacketHeader(uint8_t source_addr, uint8_t dest_addr)
    : source_addr_(source_addr), dest_addr_(dest_addr) {}

void MuPacketHeader::bindSource(uint8_t addr) { source_addr_ = addr; }
void MuPacketHeader::bindDest(uint8_t addr) { source_addr_ = addr; }
uint8_t MuPacketHeader::getSource() { return source_addr_; }
uint8_t MuPacketHeader::getDest() { return dest_addr_; }
void MuPacketHeader::setSize(uint16_t size) { body_size_ = size; };
uint16_t MuPacketHeader::getSize() { return body_size_; }

uint8_t *MuPacketHeader::serialize() {
  uint8_t *ptr = s_head_;
  memcpy(ptr, &header_, sizeof(header_));
  ptr += sizeof(header_);
  memcpy(ptr, &source_addr_, sizeof(source_addr_));
  ptr += sizeof(source_addr_);
  memcpy(ptr, &dest_addr_, sizeof(dest_addr_));
  ptr += sizeof(dest_addr_);
  memcpy(ptr, &body_size_, sizeof(body_size_));
  return (uint8_t *)s_head_;
};

MuBusNode::MuBusNode() : out_packet_(new MuPacketHeader()) {}
MuBusNode::MuBusNode(arduino::HardwareSerial *port)
    : port_(port), out_packet_(new MuPacketHeader) {}
MuBusNode::MuBusNode(uint8_t addr) : out_packet_(new MuPacketHeader(addr)) {}
MuBusNode::MuBusNode(arduino::HardwareSerial *port, uint8_t addr)
    : port_(port), out_packet_(new MuPacketHeader(addr)) {}

bool MuBusNode::broadcast(uint8_t *buf, uint16_t len) {
  if (len > 506) {
    return false;
  }
  if (len > 0 && buf == nullptr) {
    return false;
  }
  out_packet_->bindDest(0x00);
  port_->write(out_packet_->serialize(), 6);
  if (len > 0) {
    port_->write(buf, len);
  }
  return true;
}
bool MuBusNode::parse() {
  if (port_->available()) {
    uint8_t first_byte = port_->read();
    if (first_byte != 0xD3) {
      return false;
    }
    while (!port_->available()) {
      delay(2);
    }
    uint8_t second_byte = port_->read();
    if (second_byte != 0x91) {
      return false;
    }
    while (!port_->available()) {
      delay(2);
    }
    in_packet_->bindSource(port_->read());
    while (!port_->available()) {
      delay(2);
    }
    in_packet_->bindDest(port_->read());
    if (in_packet_->getDest() != out_packet_->getSource() &&
        in_packet_->getDest() != 0x00) {
      return false;
    }
    while (port_->available() < 2) {
      delay(2);
    }
    uint8_t b0 = port_->read();
    uint8_t b1 = port_->read();
    in_packet_->setSize(((uint16_t)b1 << 8) | b0);
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
  }
  return true;
}
void MuBusNode::bindAddr(uint8_t addr) { out_packet_->bindSource(addr); }
} // namespace MuBus
