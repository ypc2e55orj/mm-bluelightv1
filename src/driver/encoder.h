#pragma once

#include <cstdint>
#include <utility>

namespace driver::encoder
{
  void init();
  std::pair<uint16_t, uint16_t> angle();
}
