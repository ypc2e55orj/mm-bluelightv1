#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "driver/driver.h"
#include "odometry.h"

namespace motion {
class Motion {
 private:
  class MotionImpl;
  std::shared_ptr<MotionImpl> impl_;

 public:
  enum class Mode {
    Stop,
    Straight,
    PivotTurn,
    SlalomTurn,
  };
  struct Target {
    Mode mode;
    float target_velocity;
    float target_angular_velocity;
    float target_degree;
  };

  explicit Motion(driver::Driver &dri, config::Config &conf,
                  odometry::Odometry &odom);
  ~Motion();

  bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
  bool stop();

  uint32_t delta_us();

  bool update_notify(uint32_t delta_us);
  bool enqueue(Target &target);
};
}  // namespace motion