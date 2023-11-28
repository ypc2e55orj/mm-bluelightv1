#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "driver/driver.h"
#include "odometry.h"

namespace motion {
enum class Message { EmergencyStop, Running, Waiting };

class Motion {
 private:
  class MotionImpl;
  std::shared_ptr<MotionImpl> impl_;

 public:
  explicit Motion(driver::Driver &dri, config::Config &conf,
                  odometry::Odometry &odom);
  ~Motion();

  uint32_t delta_us();
  bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
  bool stop();
};
}  // namespace motion