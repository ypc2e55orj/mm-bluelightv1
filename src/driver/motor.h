#pragma once

#include <utility>

namespace driver::motor
{
  void init();

  void enable();
  void disable();

  void brake();
  void coast();

  void duty(std::pair<float, float> val);
}
