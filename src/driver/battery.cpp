#include "battery.h"
#include "adc.h"

namespace driver::battery
{
  static DRAM_ATTR const adc_channel_t BATTERY_CHAN = ADC_CHANNEL_4;
  static DRAM_ATTR int raw = 0;

  void init()
  {
    driver::adc::init();
    driver::adc::chan(BATTERY_CHAN);
  }

  void IRAM_ATTR update()
  {
    raw = driver::adc::raw(BATTERY_CHAN);
  }

  int get()
  {
    return driver::adc::calibrate(raw) * 2 + 100;
  }
}
