#pragma once

#include <memory>

#include <hal/adc_types.h>

namespace driver::peripherals
{
  class Adc
  {
  private:
    class AdcImpl;
    std::unique_ptr<AdcImpl> impl_;

  public:
    explicit Adc(adc_unit_t unit, adc_channel_t channel);
    ~Adc();

    int read();
    int to_voltage();
  };
}
