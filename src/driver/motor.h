#pragma once

#include <cstdint>

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

  void speed(position pos, float duty);
}
