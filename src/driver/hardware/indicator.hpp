#pragma once

#include <cstdint>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <hal/gpio_types.h>

namespace driver::hardware
{
  class Indicator
  {
  private:
    class IndicatorImpl;
    std::unique_ptr<IndicatorImpl> impl_;

  public:
    explicit Indicator(gpio_num_t indicator_num, uint16_t led_counts);
    ~Indicator();

    bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
    bool stop();

    void set(size_t pos, uint8_t r, uint8_t g, uint8_t b);
    void set(size_t pos, uint32_t rgb);
    void clear();

    void rainbow_yield(bool reset = false);
  };
}
