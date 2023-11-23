#pragma once

// C++
#include <memory>

namespace log {
class Log {
 private:
  class LogImpl;
  std::unique_ptr<LogImpl> impl_;

 public:
  explicit Log();
  ~Log();
};
}  // namespace log