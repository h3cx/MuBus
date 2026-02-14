#include <MuBus.h>

namespace {
constexpr uint32_t kBaud = 115200;
constexpr uint8_t kNodeAddr = 0x02;

MuBus::MuBusNode node(&Serial1, kNodeAddr);

void onFrame(const MuBus::MuBusNode::Frame &frame) {
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
}
} // namespace

void setup() {
  Serial.begin(kBaud);
  Serial1.begin(kBaud);
  while (!Serial) {
    ;
  }

  node.onFrame(onFrame);

#if MUBUS_ENABLE_PARSER_THREAD
  // Threaded parser mode on mbed RTOS targets.
  if (!node.startParserThread(1, 200)) {
    Serial.println("failed to start parser thread");
  }
#else
  Serial.println("threaded parser unavailable on this target; using loop() + tick()");
#endif

  Serial.println("receiver callback example ready");
}

void loop() {
#if !MUBUS_ENABLE_PARSER_THREAD
  // Cooperative fallback for non-RTOS targets.
  node.tick();
#endif

  // IMPORTANT:
  // Do not read bytes from Serial1 directly while MuBus parser is active.
  // Keep all RX access inside MuBus internals (callback, available/receive, or parse wrappers).
}
