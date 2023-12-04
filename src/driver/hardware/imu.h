#pragma once

// C++
#include <cstdint>
#include <memory>

// Project
#include "../peripherals/spi.h"
#include "base.h"

namespace driver::hardware {
class Imu final : public DriverBase {
 private:
  class Lsm6dsrxImpl;
  std::unique_ptr<Lsm6dsrxImpl> impl_;

 public:
  static constexpr float ANGULAR_RATE_SENSITIVITY = 70.0f;          // 2000dps
  static constexpr float LINEAR_ACCELERATION_SENSITIVITY = 0.061f;  // fs2g

  template <typename T>
  struct Axis {
    T x;
    T y;
    T z;
  };
  using RawAxis = Axis<int16_t>;

  explicit Imu(peripherals::Spi &spi, gpio_num_t spics_io_num);
  ~Imu();

  bool update() override;

  const RawAxis &raw_angular_rate();
  const RawAxis &raw_linear_acceleration();

  void calibration();

  static constexpr float angular_rate_sensitivity() {
    return ANGULAR_RATE_SENSITIVITY;
  }
  static constexpr float linear_acceleration_sensitivity() {
    return LINEAR_ACCELERATION_SENSITIVITY;
  }
};
}  // namespace driver::hardware
