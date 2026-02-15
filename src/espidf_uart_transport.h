#pragma once

#include "MuBus.h"

#ifdef MUBUS_RUNTIME_FREERTOS

#include <cstddef>
#include <cstdint>

extern "C" {
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
}

namespace MuBus {

struct EspIdfUartTransportConfig {
  uart_port_t uart_num = UART_NUM_1;

  gpio_num_t tx_pin = GPIO_NUM_NC;
  gpio_num_t rx_pin = GPIO_NUM_NC;
  gpio_num_t rts_pin = GPIO_NUM_NC;
  gpio_num_t cts_pin = GPIO_NUM_NC;

  int baud = 115200;
  uart_word_length_t data_bits = UART_DATA_8_BITS;
  uart_parity_t parity = UART_PARITY_DISABLE;
  uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
  uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uint8_t rx_flow_ctrl_thresh = 0;

  int rx_buffer_bytes = 2048;
  int tx_buffer_bytes = 2048;
  int event_queue_len = 0;

  bool wait_tx_done = false;
  TickType_t wait_tx_done_timeout = pdMS_TO_TICKS(50);

  bool thread_safe_write = true;
};

class EspIdfUartTransport : public MuTransport {
public:
  explicit EspIdfUartTransport(const EspIdfUartTransportConfig &cfg);
  ~EspIdfUartTransport() override;

  bool begin();

  bool write(const uint8_t *data, size_t len) override;
  bool readByte(uint8_t &byte) override;

  void flushInput();
  int bufferedRxBytes() const;

  QueueHandle_t eventQueue() const { return uart_queue_; }

private:
  EspIdfUartTransportConfig cfg_;
  bool started_ = false;

  QueueHandle_t uart_queue_ = nullptr;
  SemaphoreHandle_t write_mutex_ = nullptr;
};

} // namespace MuBus

#endif
