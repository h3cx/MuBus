#pragma once

#include "../MuBus.h"

#if __has_include(<HardwareSerial.h>)
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
