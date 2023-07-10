#include "battery.h"
#include "adc.h"

namespace driver::battery
{
  static const adc_channel_t BATTERY_CHAN = ADC_CHANNEL_4;
  static int voltage = 0;

  void init()
  {
    driver::adc::init();
    driver::adc::chan(BATTERY_CHAN);
  }

  void IRAM_ATTR update()
  {
    voltage = (driver::adc::voltage(BATTERY_CHAN) * 2) + 100;
  }

  int get() { return voltage; }
}
