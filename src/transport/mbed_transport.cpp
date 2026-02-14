#include "mbed_transport.h"

namespace MuBus {

#if __has_include(<mbed.h>)
MbedBufferedSerialTransport::MbedBufferedSerialTransport(
    mbed::BufferedSerial *port)
    : port_(port) {}

bool MbedBufferedSerialTransport::write(const uint8_t *data, size_t len) {
  if (port_ == nullptr) {
    return false;
  }
  return port_->write(data, len) == static_cast<ssize_t>(len);
}

bool MbedBufferedSerialTransport::readByte(uint8_t &byte) {
  if (port_ == nullptr || !port_->readable()) {
    return false;
  }

  return port_->read(&byte, 1) == 1;
}

MuBusNode::MuBusNode(mbed::BufferedSerial *port)
    : MuBusNode(new MbedBufferedSerialTransport(port)) {
  owns_transport_ = true;
}

MuBusNode::MuBusNode(mbed::BufferedSerial *port, uint8_t addr)
    : MuBusNode(new MbedBufferedSerialTransport(port), addr) {
  owns_transport_ = true;
}

bool MuBusNode::begin(mbed::BufferedSerial *port, uint8_t addr) {
  return assignTransport(new MbedBufferedSerialTransport(port), true, addr);
}
#else
MuBusNode::MuBusNode(mbed::BufferedSerial *port) : MuBusNode() { (void)port; }
MuBusNode::MuBusNode(mbed::BufferedSerial *port, uint8_t addr)
    : MuBusNode(addr) {
  (void)port;
}
bool MuBusNode::begin(mbed::BufferedSerial *port, uint8_t addr) {
  (void)port;
  (void)addr;
  return false;
}
#endif

} // namespace MuBus
