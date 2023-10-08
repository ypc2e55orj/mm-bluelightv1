#pragma once

#include <memory>

#include <freertos/FreeRTOS.h>
#include <hal/adc_hal.h>

namespace driver::hardware
{
  class Battery
  {
  private:
    class BatteryImpl;
    std::unique_ptr<BatteryImpl> impl_;

  public:
    explicit Battery(adc_unit_t unit, adc_channel_t channel);
    ~Battery();

    bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
    bool stop(TickType_t xTicksToWait);

    int voltage();
    int average();
  };
}