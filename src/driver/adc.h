#pragma once

#include <hal/adc_types.h>

namespace driver::adc
{
  void init();
  void chan(adc_channel_t channel);

  int raw(adc_channel_t channel);
  int calibrate(int raw);
}
