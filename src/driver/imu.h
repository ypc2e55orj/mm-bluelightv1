#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <tuple>

namespace driver::imu
{
  void init(EventGroupHandle_t xHandle, EventBits_t xBit);

  void update();

  std::tuple<float, float, float> gyro(); // [rad/s]
  std::tuple<int16_t, int16_t, int16_t> gyro_raw();

  std::tuple<float, float, float> accel(); // [m/s^2]
  std::tuple<int16_t, int16_t, int16_t> accel_raw();
}
