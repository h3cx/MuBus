#include <MuBus.h>

namespace {
constexpr uint32_t kBaud = 115200;
constexpr uint8_t kNodeAddr = 0x01;
constexpr uint8_t kReceiverAddr = 0x02;
constexpr uint32_t kSendIntervalMs = 1000;

MuBus::MuBusNode node(&Serial1, kNodeAddr);
uint32_t last_send_ms = 0;
uint32_t counter = 0;
}

void setup() {
  Serial.begin(kBaud);
  Serial1.begin(kBaud);
  while (!Serial) {
    ;
  }
  Serial.println("MuBus sender example");
}

void loop() {
  const uint32_t now = millis();
  if (now - last_send_ms < kSendIntervalMs) {
    return;
  }
  last_send_ms = now;

  uint8_t payload[4];
  payload[0] = static_cast<uint8_t>(counter & 0xFF);
  payload[1] = static_cast<uint8_t>((counter >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>((counter >> 16) & 0xFF);
  payload[3] = static_cast<uint8_t>((counter >> 24) & 0xFF);

  if (node.send(kReceiverAddr, payload, sizeof(payload))) {
    Serial.print("sent counter=");
    Serial.println(counter);
  } else {
    Serial.println("send failed");
  }

  counter++;
}
