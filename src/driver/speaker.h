#pragma once

#include <cstdint>

namespace driver::speaker
{
  struct note_t
  {
    uint32_t freq;
    int32_t ms;
  };

  void init();
  void play(note_t *notes, int size);
  void tone(uint32_t freq, int32_t ms);
}
