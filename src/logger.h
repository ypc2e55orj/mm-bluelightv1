#pragma once

// C++
#include <memory>

namespace logger {
class Logger {
 private:
  class LoggerImpl;
  std::unique_ptr<LoggerImpl> impl_;

 public:
  explicit Logger();
  ~Logger();
};
}  // namespace logger