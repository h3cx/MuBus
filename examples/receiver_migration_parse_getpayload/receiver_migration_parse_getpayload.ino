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
  Serial.println("migration example ready");
}

void loop() {
  // Old style (deprecated):
  // if (node.parse()) {
  //   uint8_t* payload = node.getPayload();
  //   uint16_t len = node.getPayloadSize();
  // }

  // New style:
  if (!node.receive(frame)) {
    return;
  }

  Serial.print("len=");
  Serial.println(frame.len);

  for (uint16_t i = 0; i < frame.len; ++i) {
    Serial.print(frame.payload[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // You may gate receive() with available():
  // if (node.available() && node.receive(frame)) { ... }
}
