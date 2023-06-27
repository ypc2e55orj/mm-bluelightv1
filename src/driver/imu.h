#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <tuple>

namespace driver::imu
{
  void init(EventGroupHandle_t xHandle, EventBits_t xBit);
  void update();

  std::tuple<float, float, float> gyro();
  std::tuple<float, float, float> accel();
}
