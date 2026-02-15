#pragma once

#include "../MuBus.h"

#ifdef MUBUS_RUNTIME_ARDUINO
#include <HardwareSerial.h>

namespace MuBus {

class ArduinoSerialTransport : public MuTransport {
public:
  explicit ArduinoSerialTransport(HardwareSerial *port);

  bool write(const uint8_t *data, size_t len) override;
  bool readByte(uint8_t &byte) override;

private:
  HardwareSerial *port_ = nullptr;
};

} // namespace MuBus
#endif
