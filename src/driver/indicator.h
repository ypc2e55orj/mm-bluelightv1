#pragma once

#include <cstdint>

namespace driver::indicator
{
  uint8_t nums();

  void init();

  void update();

  void set(uint8_t pos, uint8_t r, uint8_t g, uint8_t b);
  void set(uint8_t pos, uint32_t rgb);
  void clear();
}
