#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <tuple>

namespace driver::imu
{
  void init();

  void update();
  void wait();

  std::tuple<float, float, float> gyro(); // [rad/s]

  std::tuple<float, float, float> accel(); // [m/s^2]
}
