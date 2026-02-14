#pragma once

#include "../MuBus.h"

#if MUBUS_ENABLE_ARDUINO_TRANSPORT
#include <HardwareSerial.h>

namespace MuBus {

class ArduinoSerialTransport : public MuTransport {
public:
  explicit ArduinoSerialTransport(arduino::HardwareSerial *port);

  bool write(const uint8_t *data, size_t len) override;
  bool readByte(uint8_t &byte) override;

private:
  arduino::HardwareSerial *port_ = nullptr;
};

} // namespace MuBus
#endif
