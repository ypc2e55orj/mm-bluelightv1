#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "driver/driver.h"

namespace odometry {
class Odometry {
 private:
  class OdometryImpl;
  std::unique_ptr<OdometryImpl> impl_;

 public:
  struct Wheel {
    bool reset;
    float previous;          // [rad]
    float angular_velocity;  // [rad/s]
    float velocity;          // [mm/s]
    float length;            // [mm]
  };
  struct Wheels {
    Wheel left, right;
    float velocity;  // [mm/s]
    float length;    // [mm]
  };

  explicit Odometry(driver::Driver &dri, config::Config &conf);
  ~Odometry();

  void reset();
  void update(uint32_t delta_us);

  const Wheels &wheels();
};
}  // namespace odometry