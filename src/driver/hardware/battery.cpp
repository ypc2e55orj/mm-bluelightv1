#include "battery.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "data/average.hpp"
#include "driver/peripherals/adc.hpp"

namespace driver::hardware
{
  class Battery::BatteryImpl
  {
  private:
    // 親タスクのハンドラ
    TaskHandle_t parent_;
    // 電圧監視タスクのハンドラ
    TaskHandle_t task_;
    // 停止リクエスト
    bool req_stop_;
    // バッテリー分圧抵抗に接続されたADC
    peripherals::Adc adc_;
    // バッテリー電圧を移動平均する
    data::MovingAverage<int, int, 512> average_;
    // 最終測定値
    int voltage_;
    // 移動平均した値
    int average_voltage_;

    static void task(void *pvParameters)
    {
      auto this_ptr = reinterpret_cast<Battery::BatteryImpl *>(pvParameters);
      this_ptr->average_.reset();
      this_ptr->voltage_ = 0;
      this_ptr->average_voltage_ = 0;
      auto xLastWakeTime = xTaskGetTickCount();
      while (!this_ptr->req_stop_)
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
          // TODO: ブザー再生キューの先頭に挿入
        }
      }

      // 終了完了を停止要求タスクに通知
      xTaskNotifyGive(this_ptr->parent_);
      // タスクを削除
      vTaskDelete(nullptr);
    }

  public:
    explicit BatteryImpl(adc_unit_t unit, adc_channel_t channel)
      : parent_(nullptr), task_(nullptr), req_stop_(false), adc_(unit, channel), voltage_(0), average_voltage_(0)
    {
    }
    ~BatteryImpl() = default;

    bool start()
    {
      auto ret = xTaskCreatePinnedToCore(task, "driver::Battery::BatteryImpl::task", 8192, this, 25, &task_, 1);
      return ret == pdTRUE;
    }
    bool stop()
    {
      parent_ = xTaskGetCurrentTaskHandle();
      req_stop_ = true;
      return ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0;
    }
    [[nodiscard]] int voltage() const
    {
      return voltage_;
    }
    [[nodiscard]] int average() const
    {
      return average_voltage_;
    }
  };

  Battery::Battery(adc_unit_t unit, adc_channel_t channel) : impl_(new BatteryImpl(unit, channel))
  {
  }
  Battery::~Battery() = default;
  bool Battery::start()
  {
    return impl_->start();
  }
  bool Battery::stop()
  {
    return impl_->stop();
  }
  int Battery::voltage()
  {
    return impl_->voltage();
  }
  int Battery::average()
  {
    return impl_->average();
  }
}
