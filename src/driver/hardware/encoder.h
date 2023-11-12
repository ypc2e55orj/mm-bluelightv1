#pragma once

// C++
#include <cstdint>
#include <memory>

// Project
#include "../peripherals/spi.h"
#include "base.h"

namespace driver::hardware {
class Encoder final : DriverBase {
 private:
  class AS5050AImpl;
  std::unique_ptr<AS5050AImpl> impl_;

 public:
  explicit Encoder(peripherals::Spi &spi, gpio_num_t spics_io_num);
  ~Encoder();

  bool update() override;

  float radian();
  float degree();
};
}  // namespace driver::hardware
