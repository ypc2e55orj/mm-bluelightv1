#include "battery.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../data/average.h"
#include "adc.hpp"

namespace driver
{
  class Battery::BatteryImpl
  {
  private:
    // 電圧監視タスクのハンドラ

    // バッテリー分圧抵抗に接続されたADC
    Adc adc_;
    // バッテリー電圧を移動平均する
    data::MovingAverage<int, int, 512> average_;
    // 最終測定値
    int voltage_;
    // 移動平均した値
    int average_voltage_;

    [[noreturn]] void task(void *pvParameters)
    {
      auto this_ptr = reinterpret_cast<Battery::BatteryImpl *>(pvParameters);
      auto xLastWakeTime = xTaskGetTickCount();
      while (true)
      {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        this_ptr->adc_.read();
        // 分圧されているため2倍、実測調整で+100
        this_ptr->voltage_ = this_ptr->adc_.to_voltage() * 2 + 100;
        // 電圧の移動平均を取得
        this_ptr->average_voltage_ = this_ptr->average_.update(this_ptr->voltage_);
        // 3.5V以下になった場合、ユーザーに知らせる
        if (this_ptr->average_voltage_ < 3.5)
        {

        }
      }
    }

  public:
    explicit BatteryImpl() : adc_(ADC_UNIT_1, ADC_CHANNEL_4)
    {
    }
    ~BatteryImpl() = default;

    bool start()
    {
      xTaskCreatePinnedToCore();
      return true;
    }
    bool stop()
    {
      return true;
    }
  };
}
