#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <cstdint>
#include <utility>

namespace driver::encoder
{
  uint16_t resolution();

  void init();

  void update();
  std::pair<uint16_t, uint16_t> get();
}
