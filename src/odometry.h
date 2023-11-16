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
  explicit Odometry(driver::Driver *dri, config::Config *conf);
  ~Odometry();

  void reset();
  void update(int32_t delta_us);

  float velocity();
  float length();
};

}  // namespace odometry