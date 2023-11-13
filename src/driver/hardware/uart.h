#pragma once

// C++
#include <memory>

namespace driver::hardware {
class Uart {
 private:
  class UartImpl;
  std::unique_ptr<UartImpl> impl_;

 public:
  explicit Uart();
  ~Uart();
};
}  // namespace driver::hardware