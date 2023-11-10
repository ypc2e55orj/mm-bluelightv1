#pragma once

// C++
#include <memory>

// ESP-IDF
#include <driver/gpio.h>

// Project
#include "base.hpp"

namespace driver::hardware
{
  class Motor
  {
  private:
    class MotorImpl;
    std::unique_ptr<MotorImpl> impl_;

  public:
    explicit Motor(int mcpwm_timer_group_id, gpio_num_t a_num, gpio_num_t b_num);
    ~Motor();

    bool enable();
    bool disable();

    void brake();
    void coast();
    void speed(int motor_voltage, int battery_voltage);
  };
}
