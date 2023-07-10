#include "battery.h"
#include "adc.h"

namespace driver::battery
{
  static const adc_channel_t BATTERY_CHAN = ADC_CHANNEL_4;

  void init()
  {
    driver::adc::init();
    driver::adc::chan(BATTERY_CHAN);
  }

  int IRAM_ATTR get()
  {
    return (driver::adc::voltage(BATTERY_CHAN) * 2) + 100;
  }
}
