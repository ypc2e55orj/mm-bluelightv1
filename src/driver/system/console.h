#pragma once

// C++
#include <memory>

// ESP-IDF
#include <esp_console.h>

namespace driver::system {
class Console {
 private:
  class ConsoleImpl;
  std::unique_ptr<ConsoleImpl> impl_;

 public:
  explicit Console();
  ~Console();

  void start();
  void stop();
  void reg(esp_console_cmd_t *cmd);
};
}  // namespace driver::system