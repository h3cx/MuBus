#include "arduino_transport.h"

namespace MuBus {

#if defined(MUBUS_RUNTIME_ARDUINO)
ArduinoSerialTransport::ArduinoSerialTransport(HardwareSerial *port)
    : port_(port) {}

bool ArduinoSerialTransport::write(const uint8_t *data, size_t len) {
  if (port_ == nullptr) {
    return false;
  }
  return port_->write(data, len) == len;
}

bool ArduinoSerialTransport::readByte(uint8_t &byte) {
  if (port_ == nullptr || !port_->available()) {
    return false;
  }

  const int value = port_->read();
  if (value < 0) {
    return false;
  }

  byte = static_cast<uint8_t>(value);
  return true;
}

MuBusNode::MuBusNode(HardwareSerial *port)
    : MuBusNode(new ArduinoSerialTransport(port)) {
  owns_transport_ = true;
}

MuBusNode::MuBusNode(HardwareSerial *port, uint8_t addr)
    : MuBusNode(new ArduinoSerialTransport(port), addr) {
  owns_transport_ = true;
}

MuBusNode::MuBusNode(HardwareSerial *port, uint8_t addr,
                     const MuBusConfig &config)
    : MuBusNode(new ArduinoSerialTransport(port), addr, config) {
  owns_transport_ = true;
}

bool MuBusNode::begin(HardwareSerial *port, uint8_t addr) {
  return assignTransport(new ArduinoSerialTransport(port), true, addr, config_);
}

bool MuBusNode::begin(HardwareSerial *port, uint8_t addr,
                      const MuBusConfig &config) {
  return assignTransport(new ArduinoSerialTransport(port), true, addr, config);
}
#endif

} // namespace MuBus
