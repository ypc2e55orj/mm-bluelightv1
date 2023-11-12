#pragma once

// C++
#include <cstdint>
#include <memory>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <hal/gpio_types.h>

// Project
#include "base.h"

namespace driver::hardware {
class Indicator final : DriverBase {
 private:
  class IndicatorImpl;
  std::unique_ptr<IndicatorImpl> impl_;

 public:
  explicit Indicator(gpio_num_t indicator_num, uint16_t led_counts);
  ~Indicator();

  bool update() override;

  void set(size_t pos, uint8_t r, uint8_t g, uint8_t b);
  void set(size_t pos, uint32_t rgb);
  void clear();

  void rainbow_yield(bool reset = false);
};
}  // namespace driver::hardware
