#pragma once

#include <cstdint>

namespace driver::indicator
{
  void init();

  void set(uint8_t pos, uint8_t r, uint8_t g, uint8_t b);
  void set(uint8_t pos, uint32_t rgb);
  void clear();

  void show();

  uint8_t num();
}
