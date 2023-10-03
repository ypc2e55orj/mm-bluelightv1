#pragma once

#include <cstdint>

namespace driver::photo
{
  enum
  {
    PHOTO_LEFT_90 = 0,
    PHOTO_LEFT_45 = 1,
    PHOTO_RIGHT_45 = 2,
    PHOTO_RIGHT_90 = 3,
    PHOTO_NUMS = 4
  };

  void init();

  void tx(uint8_t pos);
  void rx(uint8_t pos);

  void get(int *dest_ambient, int *dest_flush);
}
