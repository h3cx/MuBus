#pragma once

#include "../MuBus.h"

#if MUBUS_RUNTIME_MBED
#include <mbed.h>

namespace MuBus {

class MbedBufferedSerialTransport : public MuTransport {
public:
  explicit MbedBufferedSerialTransport(mbed::BufferedSerial *port);

  bool write(const uint8_t *data, size_t len) override;
  bool readByte(uint8_t &byte) override;

private:
  mbed::BufferedSerial *port_ = nullptr;
};

} // namespace MuBus
#endif
