#pragma once

#include <cstdint>
#include <utility>

namespace driver::encoder
{
  void init();
  void angle(uint16_t &left, uint16_t &right);
  std::pair<uint16_t, uint16_t> angle();
}
