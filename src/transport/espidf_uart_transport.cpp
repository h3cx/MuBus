#include "espidf_uart_transport.h"

#if MUBUS_RUNTIME_FREERTOS

namespace MuBus {

EspIdfUartTransport::EspIdfUartTransport(const EspIdfUartTransportConfig &cfg)
    : cfg_(cfg) {}

EspIdfUartTransport::~EspIdfUartTransport() {
  if (write_mutex_ != nullptr) {
    vSemaphoreDelete(write_mutex_);
    write_mutex_ = nullptr;
  }

  if (started_) {
    (void)uart_driver_delete(cfg_.uart_num);
    started_ = false;
  }
}

bool EspIdfUartTransport::begin() {
  if (started_) {
    return true;
  }

  if (cfg_.thread_safe_write && write_mutex_ == nullptr) {
    write_mutex_ = xSemaphoreCreateMutex();
    if (write_mutex_ == nullptr) {
      return false;
    }
  }

  uart_config_t uc = {};
  uc.baud_rate = cfg_.baud;
  uc.data_bits = cfg_.data_bits;
  uc.parity = cfg_.parity;
  uc.stop_bits = cfg_.stop_bits;
  uc.flow_ctrl = cfg_.flow_ctrl;
  uc.rx_flow_ctrl_thresh = cfg_.rx_flow_ctrl_thresh;
  uc.source_clk = UART_SCLK_DEFAULT;

  if (uart_param_config(cfg_.uart_num, &uc) != ESP_OK) {
    return false;
  }

  QueueHandle_t queue = nullptr;
  const int queue_len = cfg_.event_queue_len;
  if (uart_driver_install(cfg_.uart_num, cfg_.rx_buffer_bytes,
                          cfg_.tx_buffer_bytes, queue_len,
                          (queue_len > 0) ? &queue : nullptr,
                          0) != ESP_OK) {
    return false;
  }
  uart_queue_ = queue;

  const int tx =
      (cfg_.tx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.tx_pin;
  const int rx =
      (cfg_.rx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.rx_pin;
  const int rts =
      (cfg_.rts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.rts_pin;
  const int cts =
      (cfg_.cts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.cts_pin;

  if (uart_set_pin(cfg_.uart_num, tx, rx, rts, cts) != ESP_OK) {
    (void)uart_driver_delete(cfg_.uart_num);
    uart_queue_ = nullptr;
    return false;
  }

  started_ = true;
  return true;
}

bool EspIdfUartTransport::write(const uint8_t *data, size_t len) {
  if (!started_ || data == nullptr || len == 0) {
    return false;
  }

  if (write_mutex_ != nullptr) {
    (void)xSemaphoreTake(write_mutex_, portMAX_DELAY);
  }

  const int written =
      uart_write_bytes(cfg_.uart_num, reinterpret_cast<const char *>(data), len);

  if (cfg_.wait_tx_done) {
    (void)uart_wait_tx_done(cfg_.uart_num, cfg_.wait_tx_done_timeout);
  }

  if (write_mutex_ != nullptr) {
    (void)xSemaphoreGive(write_mutex_);
  }

  return written == static_cast<int>(len);
}

bool EspIdfUartTransport::readByte(uint8_t &byte) {
  if (!started_) {
    return false;
  }

  return uart_read_bytes(cfg_.uart_num, &byte, 1, 0) == 1;
}

void EspIdfUartTransport::flushInput() {
  if (!started_) {
    return;
  }
  (void)uart_flush_input(cfg_.uart_num);
}

int EspIdfUartTransport::bufferedRxBytes() const {
  if (!started_) {
    return 0;
  }

  size_t bytes = 0;
  if (uart_get_buffered_data_len(cfg_.uart_num, &bytes) != ESP_OK) {
    return 0;
  }

  return static_cast<int>(bytes);
}

} // namespace MuBus

#endif
