#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <cstdint>
#include <utility>

namespace driver::encoder
{
  void init(EventGroupHandle_t xHandle, EventBits_t xBit);

  void update();

  std::pair<uint16_t, uint16_t> get();
}
