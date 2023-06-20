#pragma once

#include <tuple>

namespace driver::imu
{
  void init();
  std::tuple<float, float, float> gyro();
  std::tuple<float, float, float> accel();
}
