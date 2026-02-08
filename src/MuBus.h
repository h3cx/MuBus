#define MUBUS_MBED
#include "Arduino.h"
#include "HardwareSerial.h"
#ifdef MUBUS_MBED
#include "mbed.h"
#endif // MUBUS_MBED
namespace MuBus {

class MuPacketHeader {
private:
  uint16_t header_ = 0xD391;
  uint8_t source_addr_ = 0x00;
  uint8_t dest_addr_ = 0x00;
  uint16_t body_size_ = 0x0000;
  uint8_t s_head_[sizeof(header_) + sizeof(source_addr_) + sizeof(dest_addr_) +
                  sizeof(body_size_)];

public:
  MuPacketHeader() = default;
  MuPacketHeader(uint8_t source_addr);
  MuPacketHeader(uint8_t source_addr, uint8_t dest_addr);
  void bindSource(uint8_t addr);
  uint8_t getSource();
  void bindDest(uint8_t addr);
  uint8_t getDest();
  void setSize(uint16_t size);
  uint16_t getSize();
  uint8_t *serialize();
};

class MuBusNode {
private:
  #ifdef MUBUS_MBED
  mbed::BufferedSerial *port_ = nullptr;
  #else
  HardwareSerial *port_ = nullptr;
  #endif
  MuPacketHeader *out_packet_;
  MuPacketHeader *in_packet_ = new MuPacketHeader();
  uint8_t *in_buf_ = (uint8_t *)malloc(506);

public:
  MuBusNode();
  MuBusNode(uint8_t addr);
  #ifdef MUBUS_MBED
  MuBusNode(mbed::BufferedSerial *port);
  MuBusNode(mbed::BufferedSerial *port, uint8_t addr);
  #else
  MuBusNode(HardwareSerial *port);
  MuBusNode(HardwareSerial *port, uint8_t addr);
  #endif
  void bindAddr(uint8_t addr);
  bool broadcast(uint8_t *buf, uint16_t len);
  bool send(uint8_t *buf, uint16_t len, uint8_t recv_addr);
  bool parse();
  uint8_t *getPayload();
  uint16_t getPayloadSize();
  String formatHeader();
};

} // namespace MuBus
