#define MUBUS_RUNTIME_ARDUINO
#include <MuBus.h>

namespace {
constexpr uint32_t kBaud = 115200;
constexpr uint8_t kNodeAddr = 0x02;

MuBus::MuBusNode node(&Serial1, kNodeAddr);
MuBus::MuBusNode::Frame frame;
}

void setup() {
  Serial.begin(kBaud);
  Serial1.begin(kBaud);
  while (!Serial) {
    ;
  }
  Serial.println("receiver polling example ready");
}

void loop() {
  if (!node.available()) {
    return;
  }

  if (!node.receive(frame)) {
    return;
  }

  Serial.print("frame src=");
  Serial.print(frame.src, HEX);
  Serial.print(" dst=");
  Serial.print(frame.dst, HEX);
  Serial.print(" len=");
  Serial.println(frame.len);

  for (uint16_t i = 0; i < frame.len; ++i) {
    Serial.print(frame.payload[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // IMPORTANT:
  // Do not call Serial1.read()/peek() while MuBus parser is active.
  // Read data only through MuBus APIs like available()/receive().
}
