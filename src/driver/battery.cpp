#include "battery.h"
#include "adc.h"

#define BATTERY_CHAN ADC_CHANNEL_4

namespace driver::battery
{
  void init()
  {
    driver::adc::init();
    driver::adc::chan(BATTERY_CHAN);
  }

  int get()
  {
    return (driver::adc::voltage(BATTERY_CHAN) * 2) + 100;
  }
}
