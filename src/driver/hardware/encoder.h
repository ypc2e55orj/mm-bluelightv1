#pragma once

// C++
#include <cstdint>
#include <memory>

// Project
#include "../peripherals/spi.h"
#include "base.h"

namespace driver::hardware {
class Encoder final : public DriverBase {
 private:
  class As5050aImpl;
  std::unique_ptr<As5050aImpl> impl_;

 public:
  // 分解能あたりの角度
  static constexpr uint16_t RESOLUTION = 1024 - 1;  // 10 bit

  explicit Encoder(peripherals::Spi &spi, gpio_num_t spics_io_num);
  ~Encoder();

  bool update() override;

  uint16_t raw();
  static constexpr uint16_t resolution() { return RESOLUTION; }
};
}  // namespace driver::hardware
