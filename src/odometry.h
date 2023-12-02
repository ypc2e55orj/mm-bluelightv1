#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "driver/driver.h"

namespace odometry {
struct WheelsPair {
  float left;
  float right;
};

class Odometry {
 private:
  class OdometryImpl;
  std::unique_ptr<OdometryImpl> impl_;

 public:
  explicit Odometry(driver::Driver &dri, config::Config &conf);
  ~Odometry();

  void reset();
  void update(uint32_t delta_us);

  float angle();
  float x();
  float y();

  const WheelsPair &wheels_angular_acceleration();
  const WheelsPair &wheels_angular_velocity();
  const WheelsPair &wheels_velocity();
};
}  // namespace odometry