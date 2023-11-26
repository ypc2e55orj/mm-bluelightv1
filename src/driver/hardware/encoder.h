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
  explicit Encoder(peripherals::Spi &spi, gpio_num_t spics_io_num);
  ~Encoder();

  bool update() override;

  uint16_t raw();
  static uint16_t resolution();
  static float to_radian(uint16_t raw);
  static float to_degree(uint16_t raw);
};
}  // namespace driver::hardware
