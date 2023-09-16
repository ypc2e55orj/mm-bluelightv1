#include "battery.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../data/average.h"
#include "adc.h"

namespace driver
{
  class Battery::BatteryImpl
  {
  private:
    // バッテリー分圧抵抗に接続されたADC
    Adc adc_;

    [[noreturn]] static void batteryMonitorTask(void *pvParameters)
    {
      auto* this_ptr = reinterpret_cast<BatteryImpl *>(pvParameters);
      auto xLastWakeTime = xTaskGetTickCount();
      while(true)
      {
        this_ptr->adc_.read();
        // 分圧されているため2倍、実測調整で+100
        int voltage = this_ptr->adc_.to_voltage() * 2 + 100;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
    }
  public:
    explicit BatteryImpl() : adc_(ADC_UNIT_1, ADC_CHANNEL_4)
    {
    }
    ~BatteryImpl() = default;

    bool start()
    {
      return true;
    }
    bool stop()
    {
      return true;
    }
  };
}
