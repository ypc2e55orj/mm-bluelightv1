#pragma once

// C++
#include <memory>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "driver/driver.h"
#include "odometry.h"

namespace sensor {
class Sensor {
 private:
  class SensorImpl;
  std::unique_ptr<SensorImpl> impl_;

 public:
  explicit Sensor(driver::Driver *dri, odometry::Odometry *odom);
  ~Sensor();

  bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
  bool stop();

  int32_t delta_us();
};
}  // namespace sensor