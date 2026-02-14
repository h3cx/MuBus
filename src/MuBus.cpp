#include "MuBus.h"

#include <stdio.h>

#if !MUBUS_HAS_ARDUINO || MUBUS_ENABLE_PARSER_THREAD
#include <chrono>
#endif

#if MUBUS_HAS_MBED
#include <mbed.h>
#endif

#if MUBUS_HAS_FREERTOS
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

namespace MuBus {

namespace {
uint32_t nowMs() {
#if MUBUS_HAS_ARDUINO
  return millis();
#else
  using namespace std::chrono;
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
#endif
}
} // namespace

void encodeHeaderBytewise(uint8_t *out, uint8_t src, uint8_t dst, uint16_t len) {
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = src;
  out[3] = dst;
  out[4] = static_cast<uint8_t>(len & 0xFF);
  out[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
}

bool decodeHeaderBytewise(const uint8_t *in, MuPacketHeader &header) {
  if (in[0] != kSync0 || in[1] != kSync1) {
    return false;
  }
  header.bindSource(in[2]);
  header.bindDest(in[3]);
  const uint16_t len = static_cast<uint16_t>(in[4]) |
                       (static_cast<uint16_t>(in[5]) << 8);
  header.setSize(len);
  return true;
}

MuPacketHeader::MuPacketHeader(uint8_t source_addr)
    : source_addr_(source_addr) {}
MuPacketHeader::MuPacketHeader(uint8_t source_addr, uint8_t dest_addr)
    : source_addr_(source_addr), dest_addr_(dest_addr) {}

void MuPacketHeader::bindSource(uint8_t addr) { source_addr_ = addr; }
void MuPacketHeader::bindDest(uint8_t addr) { dest_addr_ = addr; }
uint8_t MuPacketHeader::getSource() const { return source_addr_; }
uint8_t MuPacketHeader::getDest() const { return dest_addr_; }
void MuPacketHeader::setSize(uint16_t size) { body_size_ = size; }
uint16_t MuPacketHeader::getSize() const { return body_size_; }

uint8_t *MuPacketHeader::serialize() {
  encodeHeaderBytewise(s_head_, source_addr_, dest_addr_, body_size_);
  return s_head_;
}

MuBusNode::MuBusNode() { applyConfig(config_); }
MuBusNode::MuBusNode(uint8_t addr) {
  out_packet_.bindSource(addr);
  applyConfig(config_);
}
MuBusNode::MuBusNode(uint8_t addr, const MuBusConfig &config)
    : config_(config) {
  out_packet_.bindSource(addr);
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport)
    : transport_(transport) {
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport, uint8_t addr)
    : transport_(transport) {
  out_packet_.bindSource(addr);
  applyConfig(config_);
}
MuBusNode::MuBusNode(MuTransport *transport, uint8_t addr,
                     const MuBusConfig &config)
    : transport_(transport), config_(config) {
  out_packet_.bindSource(addr);
  applyConfig(config_);
}

void MuBusNode::lockState() {
#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
  if (state_mutex_ != nullptr) {
    state_mutex_->lock();
  }
#elif MUBUS_HAS_FREERTOS
  if (state_mutex_ != nullptr) {
    (void)xSemaphoreTake(state_mutex_, portMAX_DELAY);
  }
#endif
#endif
}

void MuBusNode::unlockState() {
#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
  if (state_mutex_ != nullptr) {
    state_mutex_->unlock();
  }
#elif MUBUS_HAS_FREERTOS
  if (state_mutex_ != nullptr) {
    (void)xSemaphoreGive(state_mutex_);
  }
#endif
#endif
}

MuBusNode::~MuBusNode() {
  (void)stopParserThread();
  if (owns_transport_) {
    delete transport_;
  }
#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
  delete state_mutex_;
  delete parser_thread_flags_;
  delete parser_thread_stopped_;
#elif MUBUS_HAS_FREERTOS
  if (state_mutex_ != nullptr) {
    vSemaphoreDelete(state_mutex_);
    state_mutex_ = nullptr;
  }
  if (parser_thread_flags_ != nullptr) {
    vEventGroupDelete(parser_thread_flags_);
    parser_thread_flags_ = nullptr;
  }
  if (parser_thread_stopped_ != nullptr) {
    vSemaphoreDelete(parser_thread_stopped_);
    parser_thread_stopped_ = nullptr;
  }
#endif
#endif
}

void MuBusNode::applyConfig(const MuBusConfig &config) {
  config_ = config;
  if (config_.max_payload_size > kMaxPayload) {
    config_.max_payload_size = kMaxPayload;
  }
  if (config_.rx_queue_depth < 1) {
    config_.rx_queue_depth = 1;
  }
  if (config_.tx_queue_depth < 1) {
    config_.tx_queue_depth = 1;
  }
  if (config_.rx_queue_depth > kMaxRxQueueDepth) {
    config_.rx_queue_depth = kMaxRxQueueDepth;
  }
  if (config_.tx_queue_depth > kMaxTxQueueDepth) {
    config_.tx_queue_depth = kMaxTxQueueDepth;
  }
  if (config_.rx_mode == RxMode::SingleSlot) {
    config_.rx_queue_depth = 1;
  }
  if (config_.tx_mode == TxMode::Direct) {
    config_.tx_queue_depth = 1;
  }
}

bool MuBusNode::assignTransport(MuTransport *transport, bool take_ownership,
                                uint8_t addr, const MuBusConfig &config) {
  (void)stopParserThread();
  lockState();
  MuTransport *old = transport_;
  const bool old_owned = owns_transport_;
  transport_ = transport;
  owns_transport_ = take_ownership;
  if (old_owned && old != nullptr) {
    delete old;
  }
  applyConfig(config);
  bindAddr(addr);
  resetParser();
  has_pending_frame_ = false;
  rx_head_ = rx_tail_ = rx_count_ = 0;
  tx_head_ = tx_tail_ = tx_count_ = 0;
  status_ = NodeStatus{};
  diagnostics_ = Diagnostics{};
  const bool ok = transport_ != nullptr;
  unlockState();
  return ok;
}

bool MuBusNode::begin(MuTransport *transport, uint8_t addr) {
  return assignTransport(transport, false, addr, config_);
}

bool MuBusNode::begin(MuTransport *transport, uint8_t addr,
                      const MuBusConfig &config) {
  return assignTransport(transport, false, addr, config);
}

void MuBusNode::stop() {
  (void)stopParserThread();
  lockState();
  if (owns_transport_) {
    delete transport_;
  }
  transport_ = nullptr;
  owns_transport_ = false;
  resetParser();
  has_pending_frame_ = false;
  frame_callback_ = nullptr;
  rx_head_ = rx_tail_ = rx_count_ = 0;
  tx_head_ = tx_tail_ = tx_count_ = 0;
  status_ = NodeStatus{};
  diagnostics_ = Diagnostics{};
  unlockState();
}

uint8_t MuBusNode::computeCrc8(uint8_t src, uint8_t dst, uint16_t len,
                               const uint8_t *payload) const {
  uint8_t crc = 0;
  crc ^= src;
  crc ^= dst;
  crc ^= static_cast<uint8_t>(len & 0xFF);
  crc ^= static_cast<uint8_t>((len >> 8) & 0xFF);
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= payload[i];
  }
  return crc;
}

uint16_t MuBusNode::computeCrc16(uint8_t src, uint8_t dst, uint16_t len,
                                 const uint8_t *payload) const {
  uint16_t crc = 0xFFFF;
  auto step = [&crc](uint8_t b) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                : static_cast<uint16_t>(crc << 1);
    }
  };

  step(src);
  step(dst);
  step(static_cast<uint8_t>(len & 0xFF));
  step(static_cast<uint8_t>((len >> 8) & 0xFF));
  for (uint16_t i = 0; i < len; ++i) {
    step(payload[i]);
  }
  return crc;
}

uint8_t MuBusNode::crcFieldSize() const {
  switch (config_.crc_mode) {
  case CrcMode::Crc8:
    return 1;
  case CrcMode::Crc16:
    return 2;
  case CrcMode::None:
  default:
    return 0;
  }
}

bool MuBusNode::writeFrameNow(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (transport_ == nullptr || len > config_.max_payload_size) {
    return false;
  }
  if (len > 0 && data == nullptr) {
    return false;
  }

  out_packet_.bindDest(dst);
  out_packet_.setSize(len);
  if (!transport_->write(out_packet_.serialize(), kHeaderSize)) {
    return false;
  }
  if (len > 0 && !transport_->write(data, len)) {
    return false;
  }

  switch (config_.crc_mode) {
  case CrcMode::Crc8: {
    const uint8_t crc = computeCrc8(out_packet_.getSource(), dst, len, data);
    return transport_->write(&crc, 1);
  }
  case CrcMode::Crc16: {
    const uint16_t crc =
        computeCrc16(out_packet_.getSource(), dst, len, data);
    const uint8_t crc_buf[2] = {static_cast<uint8_t>(crc & 0xFF),
                                static_cast<uint8_t>((crc >> 8) & 0xFF)};
    return transport_->write(crc_buf, sizeof(crc_buf));
  }
  case CrcMode::None:
  default:
    return true;
  }
}

bool MuBusNode::enqueueTxFrame(uint8_t dst, const uint8_t *data, uint16_t len) {
  if (tx_count_ >= config_.tx_queue_depth) {
    status_.tx_queue_full = true;
    status_.dropped_frame_count++;
    diagnostics_.drop_count++;
    return false;
  }

  FrameSlot &slot = tx_slots_[tx_tail_];
  slot.src = out_packet_.getSource();
  slot.dst = dst;
  slot.len = len;
  for (uint16_t i = 0; i < len; ++i) {
    slot.payload[i] = data[i];
  }

  tx_tail_ = static_cast<uint8_t>((tx_tail_ + 1) % kMaxTxQueueDepth);
  tx_count_++;
  return true;
}

bool MuBusNode::flushTxQueue() {
  while (tx_count_ > 0) {
    FrameSlot &slot = tx_slots_[tx_head_];
    if (!writeFrameNow(slot.dst, slot.payload, slot.len)) {
      return false;
    }
    tx_head_ = static_cast<uint8_t>((tx_head_ + 1) % kMaxTxQueueDepth);
    tx_count_--;
  }
  status_.tx_queue_full = false;
  return true;
}

bool MuBusNode::send(uint8_t dst, const uint8_t *data, uint16_t len) {
  lockState();
  if (len > config_.max_payload_size) {
    unlockState();
    return false;
  }

  if (txQueueEnabled()) {
    const bool queued = enqueueTxFrame(dst, data, len);
    (void)flushTxQueue();
    unlockState();
    return queued;
  }

  const bool ok = writeFrameNow(dst, data, len);
  unlockState();
  return ok;
}

bool MuBusNode::broadcast(const uint8_t *data, uint16_t len) {
  return send(kBroadcastAddress, data, len);
}

void MuBusNode::bindAddr(uint8_t addr) { out_packet_.bindSource(addr); }
uint8_t *MuBusNode::getPayload() { return pending_frame_.payload; }
uint16_t MuBusNode::getPayloadSize() { return pending_frame_.len; }
String MuBusNode::formatHeader() {
  char msg_buf[64];
  snprintf(msg_buf, sizeof(msg_buf), "Source: 0x%02X\nDest: 0x%02X\nSize: 0x%04X",
           pending_frame_.src, pending_frame_.dst, pending_frame_.len);
  return String(msg_buf);
}

bool MuBusNode::rxQueueEnabled() const { return config_.rx_mode == RxMode::Ring; }
bool MuBusNode::txQueueEnabled() const { return config_.tx_mode == TxMode::Queue; }

bool MuBusNode::enqueueRxFrame(const Frame &frame) {
  if (rx_count_ >= config_.rx_queue_depth) {
    status_.rx_queue_full = true;
    status_.dropped_frame_count++;
    diagnostics_.drop_count++;
    return false;
  }

  FrameSlot &slot = rx_slots_[rx_tail_];
  slot.src = frame.src;
  slot.dst = frame.dst;
  slot.len = frame.len;
  for (uint16_t i = 0; i < frame.len; ++i) {
    slot.payload[i] = frame.payload[i];
  }

  rx_tail_ = static_cast<uint8_t>((rx_tail_ + 1) % kMaxRxQueueDepth);
  rx_count_++;
  return true;
}

bool MuBusNode::dequeueRxFrame(Frame &frame) {
  if (rx_count_ == 0) {
    return false;
  }

  FrameSlot &slot = rx_slots_[rx_head_];
  frame.src = slot.src;
  frame.dst = slot.dst;
  frame.len = slot.len;
  frame.payload = slot.payload;

  rx_head_ = static_cast<uint8_t>((rx_head_ + 1) % kMaxRxQueueDepth);
  rx_count_--;
  if (rx_count_ < config_.rx_queue_depth) {
    status_.rx_queue_full = false;
  }
  return true;
}

void MuBusNode::pollCore() {
  Frame frame;
  while (readFrame(frame)) {
    if (rxQueueEnabled()) {
      if (!enqueueRxFrame(frame)) {
        continue;
      }

      if (frame_callback_ != nullptr) {
        const uint8_t callback_index =
            static_cast<uint8_t>((rx_tail_ + kMaxRxQueueDepth - 1) %
                                 kMaxRxQueueDepth);
        Frame callback_frame;
        callback_frame.src = rx_slots_[callback_index].src;
        callback_frame.dst = rx_slots_[callback_index].dst;
        callback_frame.len = rx_slots_[callback_index].len;
        callback_frame.payload = rx_slots_[callback_index].payload;
        frame_callback_(callback_frame);
      }
    } else {
      pending_frame_ = frame;
      has_pending_frame_ = true;
      if (frame_callback_ != nullptr) {
        frame_callback_(pending_frame_);
      }
    }
  }

  if (txQueueEnabled()) {
    (void)flushTxQueue();
  }
}

void MuBusNode::poll() {
  lockState();
  pollCore();
  unlockState();
}

void MuBusNode::tick() { poll(); }

bool MuBusNode::startParserThread() {
  return startParserThread(parser_thread_config_.poll_interval_ms,
                           parser_thread_config_.stop_timeout_ms);
}

bool MuBusNode::startParserThread(uint32_t poll_interval_ms,
                                  uint32_t stop_timeout_ms) {
  parser_thread_config_.poll_interval_ms = poll_interval_ms;
  parser_thread_config_.stop_timeout_ms = stop_timeout_ms;
#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
  bool locked = false;
  if (state_mutex_ != nullptr) {
    state_mutex_->lock();
    locked = true;
  }

  if (parser_thread_running_) {
    if (locked) {
      state_mutex_->unlock();
    }
    return true;
  }
  if (transport_ == nullptr) {
    if (locked) {
      state_mutex_->unlock();
    }
    return false;
  }
  if (state_mutex_ == nullptr) {
    state_mutex_ = new rtos::Mutex();
    if (state_mutex_ != nullptr) {
      state_mutex_->lock();
      locked = true;
    }
  }
  if (parser_thread_flags_ == nullptr) {
    parser_thread_flags_ = new rtos::EventFlags();
  }
  if (parser_thread_stopped_ == nullptr) {
    parser_thread_stopped_ = new rtos::Semaphore(0, 1);
  }
  if (parser_thread_ == nullptr) {
    parser_thread_ = new rtos::Thread(
        osPriorityNormal, config_.parser_thread_stack_bytes, nullptr,
        "mubus_parser");
  }
  if (state_mutex_ == nullptr || parser_thread_flags_ == nullptr ||
      parser_thread_stopped_ == nullptr || parser_thread_ == nullptr) {
    if (locked) {
      state_mutex_->unlock();
    }
    return false;
  }

  parser_thread_running_ = true;
  parser_thread_->start(mbed::callback(this, &MuBusNode::parserThreadLoop));
  if (locked) {
    state_mutex_->unlock();
  }
  return true;
#elif MUBUS_HAS_FREERTOS
  bool locked = false;

  if (state_mutex_ == nullptr) {
    state_mutex_ = xSemaphoreCreateMutex();
  }
  if (state_mutex_ != nullptr) {
    locked = (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE);
  }

  if (parser_thread_running_) {
    if (locked) {
      (void)xSemaphoreGive(state_mutex_);
    }
    return true;
  }

  if (transport_ == nullptr) {
    if (locked) {
      (void)xSemaphoreGive(state_mutex_);
    }
    return false;
  }

  if (parser_thread_flags_ == nullptr) {
    parser_thread_flags_ = xEventGroupCreate();
  }
  if (parser_thread_stopped_ == nullptr) {
    parser_thread_stopped_ = xSemaphoreCreateBinary();
  }

  if (state_mutex_ == nullptr || parser_thread_flags_ == nullptr ||
      parser_thread_stopped_ == nullptr) {
    if (locked) {
      (void)xSemaphoreGive(state_mutex_);
    }
    return false;
  }

  parser_thread_running_ = true;
  const uint32_t stack_words =
      (config_.parser_thread_stack_bytes + sizeof(StackType_t) - 1) /
      sizeof(StackType_t);
  const BaseType_t created = xTaskCreate(
      &MuBusNode::parserThreadEntry, "mubus_parser",
      stack_words > 0 ? stack_words : 1024, this, tskIDLE_PRIORITY + 1,
      &parser_thread_);

  if (locked) {
    (void)xSemaphoreGive(state_mutex_);
  }

  if (created != pdPASS) {
    parser_thread_running_ = false;
    parser_thread_ = nullptr;
    return false;
  }

  return true;
#else
  (void)poll_interval_ms;
  (void)stop_timeout_ms;
  return false;
#endif
#else
  (void)poll_interval_ms;
  (void)stop_timeout_ms;
  return false;
#endif
}

bool MuBusNode::stopParserThread() {
#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
  rtos::Thread *thread = nullptr;
  bool locked = false;

  if (state_mutex_ != nullptr) {
    state_mutex_->lock();
    locked = true;
  }

  if (!parser_thread_running_) {
    thread = parser_thread_;
    parser_thread_ = nullptr;
    if (locked) {
      state_mutex_->unlock();
    }
    if (thread != nullptr) {
      thread->join();
      delete thread;
    }
    return true;
  }

  parser_thread_running_ = false;
  thread = parser_thread_;
  parser_thread_ = nullptr;

  if (locked) {
    state_mutex_->unlock();
  }

  if (parser_thread_flags_ != nullptr) {
    parser_thread_flags_->set(0x1);
  }
  if (parser_thread_stopped_ != nullptr &&
      !parser_thread_stopped_->try_acquire_for(
          std::chrono::milliseconds(parser_thread_config_.stop_timeout_ms))) {
    return false;
  }
  if (thread != nullptr) {
    thread->join();
    delete thread;
  }
  return true;
#elif MUBUS_HAS_FREERTOS
  bool locked = false;

  if (state_mutex_ != nullptr) {
    locked = (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE);
  }

  if (!parser_thread_running_) {
    if (locked) {
      (void)xSemaphoreGive(state_mutex_);
    }
    return true;
  }

  parser_thread_running_ = false;
  if (locked) {
    (void)xSemaphoreGive(state_mutex_);
  }

  if (parser_thread_flags_ != nullptr) {
    (void)xEventGroupSetBits(parser_thread_flags_, 0x1);
  }

  if (parser_thread_stopped_ != nullptr) {
    const TickType_t timeout_ticks =
        pdMS_TO_TICKS(parser_thread_config_.stop_timeout_ms);
    if (xSemaphoreTake(parser_thread_stopped_, timeout_ticks) != pdTRUE) {
      return false;
    }
  }

  parser_thread_ = nullptr;
  return true;
#else
  return true;
#endif
#else
  return true;
#endif
}

#if MUBUS_ENABLE_PARSER_THREAD
#if MUBUS_HAS_MBED
void MuBusNode::parserThreadLoop() {
  while (parser_thread_running_) {
    poll();
    if (parser_thread_flags_ == nullptr) {
      rtos::ThisThread::sleep_for(
          std::chrono::milliseconds(parser_thread_config_.poll_interval_ms));
      continue;
    }
    parser_thread_flags_->wait_any_for(
        0x1, std::chrono::milliseconds(parser_thread_config_.poll_interval_ms));
  }

  if (parser_thread_stopped_ != nullptr) {
    parser_thread_stopped_->release();
  }
}
#elif MUBUS_HAS_FREERTOS
void MuBusNode::parserThreadEntry(void *arg) {
  if (arg == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  static_cast<MuBusNode *>(arg)->parserThreadLoop();
}

void MuBusNode::parserThreadLoop() {
  while (parser_thread_running_) {
    poll();
    if (parser_thread_flags_ == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(parser_thread_config_.poll_interval_ms));
      continue;
    }

    (void)xEventGroupWaitBits(parser_thread_flags_, 0x1, pdTRUE, pdFALSE,
                              pdMS_TO_TICKS(parser_thread_config_.poll_interval_ms));
  }

  if (parser_thread_stopped_ != nullptr) {
    (void)xSemaphoreGive(parser_thread_stopped_);
  }

  parser_thread_ = nullptr;
  vTaskDelete(nullptr);
}
#endif
#endif

bool MuBusNode::available() {
  poll();
  lockState();
  const bool has_data = rxQueueEnabled() ? (rx_count_ > 0) : has_pending_frame_;
  unlockState();
  return has_data;
}

bool MuBusNode::receive(Frame &frame) {
  poll();
  lockState();

  if (rxQueueEnabled()) {
    const bool ok = dequeueRxFrame(frame);
    unlockState();
    return ok;
  }

  if (!has_pending_frame_) {
    unlockState();
    return false;
  }

  frame = pending_frame_;
  has_pending_frame_ = false;
  unlockState();
  return true;
}

void MuBusNode::onFrame(FrameCallback callback) {
  lockState();
  frame_callback_ = callback;
  unlockState();
}

MuBusNode::NodeStatus MuBusNode::getStatus() const { return status_; }

MuBusNode::Diagnostics MuBusNode::getDiagnostics() const {
  return diagnostics_;
}

void MuBusNode::resetDiagnostics() {
  lockState();
  diagnostics_ = Diagnostics{};
  unlockState();
}

bool MuBusNode::send(uint8_t *buf, uint16_t len, uint8_t recv_addr) {
  return send(recv_addr, buf, len);
}

bool MuBusNode::broadcast(uint8_t *buf, uint16_t len) {
  return broadcast(static_cast<const uint8_t *>(buf), len);
}

bool MuBusNode::parse() {
  Frame frame;
  if (!receive(frame)) {
    return false;
  }
  return true;
}

void MuBusNode::resetParser() {
  parser_ = ParserContext{};
  parser_last_byte_ms_ = 0;
}

void MuBusNode::updateParserTimeout() {
  if (config_.parser_timeout_ms == 0 || parser_.state == ParserState::Sync0 ||
      parser_last_byte_ms_ == 0) {
    return;
  }

  const uint32_t elapsed = nowMs() - parser_last_byte_ms_;
  if (elapsed > config_.parser_timeout_ms) {
    diagnostics_.timeout_count++;
    diagnostics_.drop_count++;
    status_.dropped_frame_count++;
    resetParser();
  }
}

bool MuBusNode::readTransportByte(uint8_t &byte) {
  if (transport_ == nullptr) {
    return false;
  }
  return transport_->readByte(byte);
}

bool MuBusNode::shouldAcceptDestination(uint8_t dst) const {
  switch (config_.destination_filter_mode) {
  case DestinationFilterMode::AddressedOrBroadcast:
    return dst == out_packet_.getSource() || dst == kBroadcastAddress;
  case DestinationFilterMode::AcceptBroadcastOnly:
    return dst == kBroadcastAddress;
  case DestinationFilterMode::AcceptUnicastOnly:
    return dst == out_packet_.getSource();
  case DestinationFilterMode::Promiscuous:
    return true;
  }

  return false;
}

bool MuBusNode::finalizeParsedFrame(Frame &frame) {
  in_packet_.bindSource(parser_.src);
  in_packet_.bindDest(parser_.dst);
  in_packet_.setSize(parser_.len);

  if (!shouldAcceptDestination(in_packet_.getDest())) {
    diagnostics_.destination_mismatch++;
    diagnostics_.drop_count++;
    status_.dropped_frame_count++;
    return false;
  }

  if (config_.crc_mode == CrcMode::Crc8) {
    const uint8_t expected_crc =
        computeCrc8(parser_.src, parser_.dst, parser_.len, parser_payload_);
    if (static_cast<uint8_t>(parser_.rx_crc & 0xFF) != expected_crc) {
      diagnostics_.crc_fail++;
      diagnostics_.drop_count++;
      status_.dropped_frame_count++;
      return false;
    }
  } else if (config_.crc_mode == CrcMode::Crc16) {
    const uint16_t expected_crc =
        computeCrc16(parser_.src, parser_.dst, parser_.len, parser_payload_);
    if (parser_.rx_crc != expected_crc) {
      diagnostics_.crc_fail++;
      diagnostics_.drop_count++;
      status_.dropped_frame_count++;
      return false;
    }
  }

  frame.src = in_packet_.getSource();
  frame.dst = in_packet_.getDest();
  frame.len = in_packet_.getSize();
  frame.payload = parser_payload_;
  return true;
}

bool MuBusNode::parseByte(uint8_t byte, Frame &frame) {
  switch (parser_.state) {
  case ParserState::Sync0:
    if (byte == kSync0) {
      parser_.state = ParserState::Sync1;
    } else {
      diagnostics_.sync_errors++;
    }
    return false;

  case ParserState::Sync1:
    if (byte == kSync1) {
      parser_.state = ParserState::Src;
      return false;
    }

    diagnostics_.sync_errors++;
    parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
    return false;

  case ParserState::Src:
    parser_.src = byte;
    parser_.state = ParserState::Dst;
    return false;

  case ParserState::Dst:
    parser_.dst = byte;
    parser_.state = ParserState::Len0;
    return false;

  case ParserState::Len0:
    parser_.len = byte;
    parser_.state = ParserState::Len1;
    return false;

  case ParserState::Len1:
    parser_.len |= static_cast<uint16_t>(byte) << 8;
    parser_.payload_index = 0;
    parser_.rx_crc = 0;

    if (parser_.len > config_.max_payload_size) {
      diagnostics_.length_overflow++;
      diagnostics_.drop_count++;
      status_.dropped_frame_count++;
      parser_.state = (byte == kSync0) ? ParserState::Sync1 : ParserState::Sync0;
      return false;
    }

    parser_.state = (parser_.len == 0) ? ParserState::Crc : ParserState::Payload;
    if (parser_.len == 0 && crcFieldSize() == 0) {
      const bool complete = finalizeParsedFrame(frame);
      resetParser();
      return complete;
    }
    return false;

  case ParserState::Payload:
    if (parser_.payload_index >= parser_.len ||
        parser_.payload_index >= config_.max_payload_size) {
      diagnostics_.length_overflow++;
      diagnostics_.drop_count++;
      status_.dropped_frame_count++;
      resetParser();
      return false;
    }

    parser_payload_[parser_.payload_index++] = byte;
    if (parser_.payload_index < parser_.len) {
      return false;
    }

    if (crcFieldSize() == 0) {
      const bool complete = finalizeParsedFrame(frame);
      resetParser();
      return complete;
    }

    parser_.state = ParserState::Crc;
    parser_.payload_index = 0;
    return false;

  case ParserState::Crc: {
    if (crcFieldSize() == 1) {
      parser_.rx_crc = byte;
      const bool complete = finalizeParsedFrame(frame);
      resetParser();
      return complete;
    }

    if (crcFieldSize() == 2) {
      if (parser_.payload_index == 0) {
        parser_.rx_crc = byte;
        parser_.payload_index = 1;
        return false;
      }
      parser_.rx_crc |= static_cast<uint16_t>(byte) << 8;
      const bool complete = finalizeParsedFrame(frame);
      resetParser();
      return complete;
    }

    const bool complete = finalizeParsedFrame(frame);
    resetParser();
    return complete;
  }
  }

  resetParser();
  return false;
}

bool MuBusNode::readFrame(Frame &frame) {
  updateParserTimeout();

  uint8_t byte = 0;
  while (readTransportByte(byte)) {
    parser_last_byte_ms_ = nowMs();
    if (parseByte(byte, frame)) {
      return true;
    }
  }

  updateParserTimeout();
  return false;
}

} // namespace MuBus
