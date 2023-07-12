#pragma once

#include <cstdint>
#include <utility>

namespace driver::motor
{
  void init();

  void brake();
  void coast();

  std::pair<float, float> duty(std::pair<float, float> val);
}
