#include "espidf_uart_transport.h"

#ifdef MUBUS_RUNTIME_FREERTOS

namespace MuBus {

EspIdfUartTransport::EspIdfUartTransport(const EspIdfUartTransportConfig &cfg)
    : cfg_(cfg) {}

EspIdfUartTransport::~EspIdfUartTransport() {
  if (write_mutex_ != nullptr) {
    vSemaphoreDelete(write_mutex_);
    write_mutex_ = nullptr;
  }

  if (started_ && owns_driver_) {
    (void)uart_driver_delete(cfg_.uart_num);
  }

  started_ = false;
  owns_driver_ = false;
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

  last_err_ = ESP_OK;
  owns_driver_ = false;

  uart_config_t uc = {};
  uc.baud_rate = cfg_.baud;
  uc.data_bits = cfg_.data_bits;
  uc.parity = cfg_.parity;
  uc.stop_bits = cfg_.stop_bits;
  uc.flow_ctrl = cfg_.flow_ctrl;
  uc.rx_flow_ctrl_thresh = cfg_.rx_flow_ctrl_thresh;
  uc.source_clk = UART_SCLK_APB;

  last_err_ = uart_param_config(cfg_.uart_num, &uc);
  if (last_err_ != ESP_OK) {
    return false;
  }

  const int tx =
      (cfg_.tx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.tx_pin;
  const int rx =
      (cfg_.rx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.rx_pin;
  const int rts =
      (cfg_.rts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.rts_pin;
  const int cts =
      (cfg_.cts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : cfg_.cts_pin;

  last_err_ = uart_set_pin(cfg_.uart_num, tx, rx, rts, cts);
  if (last_err_ != ESP_OK) {
    return false;
  }

  QueueHandle_t queue = nullptr;
  const int queue_len = cfg_.event_queue_len;
  last_err_ = uart_driver_install(cfg_.uart_num, cfg_.rx_buffer_bytes,
                                  cfg_.tx_buffer_bytes, queue_len,
                                  (queue_len > 0) ? &queue : nullptr,
                                  0);

  if (last_err_ == ESP_OK) {
    owns_driver_ = true;
    uart_queue_ = queue;
  } else if (last_err_ == ESP_FAIL) {
    // ESP-IDF returns ESP_FAIL when a UART driver is already installed.
    // Reuse the existing driver instead of treating this as a fatal error.
    owns_driver_ = false;
    uart_queue_ = nullptr;
    last_err_ = ESP_OK;
  } else {
    return false;
  }

  last_err_ = uart_flush_input(cfg_.uart_num);
  if (last_err_ != ESP_OK) {
    if (owns_driver_) {
      (void)uart_driver_delete(cfg_.uart_num);
      owns_driver_ = false;
    }
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

  size_t total_written = 0;
  while (total_written < len) {
    const int n = uart_write_bytes(
        cfg_.uart_num,
        reinterpret_cast<const char *>(data) + total_written,
        len - total_written);
    if (n < 0) {
      if (write_mutex_ != nullptr) {
        (void)xSemaphoreGive(write_mutex_);
      }
      return false;
    }

    if (n == 0) {
      vTaskDelay(1);
      continue;
    }

    total_written += static_cast<size_t>(n);
  }

  if (cfg_.wait_tx_done) {
    (void)uart_wait_tx_done(cfg_.uart_num, cfg_.wait_tx_done_timeout);
  }

  if (write_mutex_ != nullptr) {
    (void)xSemaphoreGive(write_mutex_);
  }

  return true;
}

bool EspIdfUartTransport::readByte(uint8_t &byte) {
  if (!started_) {
    return false;
  }

  return uart_read_bytes(cfg_.uart_num, &byte, 1, cfg_.read_timeout) == 1;
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
