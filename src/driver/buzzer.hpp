#pragma once

#include <cstdint>

namespace driver::buzzer
{
  void init();

  void tone(uint32_t freq, uint32_t ms);

  void beep();

  void start(uint32_t freq);
  void stop();
}
