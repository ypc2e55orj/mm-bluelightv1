#include "uart.h"

// ESP-IDF
#include <driver/uart.h>
#include <esp_intr_alloc.h>
#include <esp_vfs_dev.h>
#include <sdkconfig.h>

namespace driver::hardware {
/**
 * 参考:
 * https://github.com/espressif/esp-idf/blob/v5.1.1/examples/common_components/protocol_examples_common/stdin_out.c
 */
class Uart::UartImpl {
 public:
  explicit UartImpl() {
    // バッファを使用しない
    setvbuf(stdin, nullptr, _IONBF, 0);
    // UART(割り込みでの読み書き)を有効化
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0,
                                        nullptr, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    // 改行コードを設定
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CRLF);
  }
  ~UartImpl() {
    ESP_ERROR_CHECK(uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM));
  }
};

Uart::Uart() : impl_(new UartImpl()) {}
Uart::~Uart() = default;
}  // namespace driver::hardware