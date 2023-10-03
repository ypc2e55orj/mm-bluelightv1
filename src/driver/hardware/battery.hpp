#pragma once

#include <memory>

#include <hal/adc_hal.h>

namespace driver
{
  class Battery
  {
  private:
    class BatteryImpl;
    std::unique_ptr<BatteryImpl> impl_;

  public:
    explicit Battery(adc_unit_t unit, adc_channel_t channel);
    ~Battery();

    bool start();
    bool stop();

    int voltage();
    int average();
  };
}