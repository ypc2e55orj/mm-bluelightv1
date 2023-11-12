#pragma once

// C++
#include <memory>

// ESP-IDF
#include <hal/adc_hal.h>

// Project
#include "base.h"

namespace driver::hardware {
class Battery final : DriverBase {
 private:
  class BatteryImpl;
  std::unique_ptr<BatteryImpl> impl_;

 public:
  explicit Battery(adc_unit_t unit, adc_channel_t channel);
  ~Battery();

  bool update() override;

  int voltage();
  int average();
};
}  // namespace driver::hardware