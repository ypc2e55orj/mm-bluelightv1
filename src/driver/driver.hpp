#pragma once

// C++

// ESP-IDF

// Project
#include "hardware/battery.hpp"
#include "hardware/buzzer.hpp"
#include "hardware/encoder.hpp"
#include "hardware/imu.hpp"
#include "hardware/indicator.hpp"
#include "hardware/motor.hpp"
#include "hardware/photo.hpp"
#include "peripherals/spi.hpp"

namespace driver
{
  class Context
  {
  private:
    peripherals::Spi *spi2_;
    peripherals::Spi *spi3_;

  public:
    hardware::Battery *battery;
    hardware::Buzzer *buzzer;
    hardware::Encoder *encoder_left;
    hardware::Encoder *encoder_right;
    hardware::Imu *imu;
    hardware::Indicator *indicator;
    hardware::Motor *motor_left;
    hardware::Motor *motor_right;
    hardware::Photo *photo;

    void initialize();
  };

  extern Context ctx;
}