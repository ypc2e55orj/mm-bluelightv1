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

    const Axis &gyro();
    const Axis &accel();
  };
}
