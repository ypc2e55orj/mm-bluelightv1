#pragma once

#include <cstdint>

namespace driver::encoder
{
  void init();
  void angle(uint16_t &left, uint16_t &right);
}
