#pragma once

#include <cstdint>

#include "../third-party/musical_buzzer/musical_score_encoder.h"

namespace driver::buzzer
{
  struct note_t
  {
    uint32_t freq;
    uint32_t ms;
  };

  void init();
  void play(note_t *notes, int size);
  void tone(uint32_t freq, uint32_t ms);
}
