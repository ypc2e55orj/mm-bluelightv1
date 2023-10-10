#include "battery.hpp"

#include "data/average.hpp"
#include "driver/peripherals/adc.hpp"
#include "task.hpp"

namespace driver::hardware
{
  class Battery::BatteryImpl final : public task::Task
  {
  private:
    // バッテリー分圧抵抗に接続されたADC
    peripherals::Adc adc_;
    // バッテリー電圧を移動平均する
    data::MovingAverage<int, int, 512> average_;
    // 最終測定値
    int voltage_;
    // 移動平均した値
    int average_voltage_;

    void setup() override
    {
      average_.reset();
      voltage_ = 0;
      average_voltage_ = 0;
    }
    void loop() override
    {
      adc_.read();
      // 分圧されているため2倍、実測調整で+100
      voltage_ = adc_.to_voltage() * 2 + 100;
      // 電圧の移動平均を取得
      average_voltage_ = average_.update(voltage_);
      // 3.5V以下になった場合、ユーザーに知らせる
      if (average_voltage_ < 3.5)
      {
        // TODO: ブザー再生キューの先頭に挿入
      }
    }
    void end() override
    {
      // 何もしない
    }

  public:
    explicit BatteryImpl(adc_unit_t unit, adc_channel_t channel)
      : task::Task(__func__, pdMS_TO_TICKS(1)), adc_(unit, channel), voltage_(0), average_voltage_(0)
    {
    }
    ~BatteryImpl() override = default;

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
  bool Battery::start(const uint32_t usStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID)
  {
    return impl_->start(usStackDepth, uxPriority, xCoreID);
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
