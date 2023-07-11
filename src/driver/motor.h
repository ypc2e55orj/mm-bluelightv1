#pragma once

#include <cstdint>
#include <utility>

namespace driver::motor
{
  enum position
  {
    LEFT = 0,
    RIGHT = 1,
    NUMS = 2,
  };

  enum direction
  {
    FORWARD = 0,
    REVERSE = 1,
  };

  void init();

  void brake();
  void brake(position pos);

  void coast();
  void coast(position pos);

  void duty(position pos, float val);
  std::pair<float, float> duty(std::pair<float, float> val);
}
