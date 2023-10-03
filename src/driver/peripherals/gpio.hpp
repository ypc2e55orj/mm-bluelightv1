#pragma once

#include <memory>

#include <hal/gpio_types.h>

namespace driver
{
  class Gpio
  {
  private:
    class GpioImpl;
    std::unique_ptr<GpioImpl> impl_;

  public:
    explicit Gpio(gpio_num_t num, gpio_mode_t mode, bool enable_pullup, bool enable_pulldown);
    ~Gpio();

    bool set(bool level);
    bool get();
  };
}
