#pragma once

#include <cstdint>

#define PHOTO_LEFT_90 0
#define PHOTO_LEFT_45 1
#define PHOTO_RIGHT_45 2
#define PHOTO_RIGHT_90 3
#define PHOTO_NUMS 4

namespace driver::photo
{
  void init();
  void sampling(uint32_t charge_us);
  void get(int *result);
}
