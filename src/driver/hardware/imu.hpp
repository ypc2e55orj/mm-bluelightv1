#pragma once

// C++
#include <cstdint>
#include <memory>

// Projects
#include "../peripherals/spi.hpp"
#include "base.hpp"

namespace driver::hardware
{
  class Imu final : DriverBase
  {
  private:
    class Lsm6dsrxImpl;
    std::unique_ptr<Lsm6dsrxImpl> impl_;

  public:
    struct Axis
    {
      float x;
      float y;
      float z;
    };

    explicit Imu(peripherals::Spi &spi, gpio_num_t spics_io_num);
    ~Imu();

    bool update() override;

    Axis &gyro();
    Axis &accel();
  };
}

namespace driver::imu
{
  void init();

  void update();
  void wait();

  std::tuple<float, float, float> gyro(); // [rad/s]

  std::tuple<float, float, float> accel(); // [m/s^2]
}
