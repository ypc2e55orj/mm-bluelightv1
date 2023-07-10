#pragma once

#include <cstdint>

namespace driver::photo
{
  enum
  {
    LEFT_90 = 0,
    LEFT_45 = 1,
    RIGHT_45 = 2,
    RIGHT_90 = 3,
    NUMS = 4
  };

  void init();

  void tx(uint8_t pos);
  void rx(uint8_t pos);

  void get(int *dest);
}
