#pragma once

#include <cstdint>

#define PHOTO_LEFT_90 0
#define PHOTO_LEFT_45 1
#define PHOTO_RIGHT_45 2
#define PHOTO_RIGHT_90 3
#define PHOTO_NUMS 4

namespace driver::photo
{
  uint8_t nums();

  void init();

  void update();
  void get(int *result);
}
