#pragma once

#include "driver/hardware/battery.hpp"

namespace driver
{
  struct Context
  {
    hardware::Battery *battery;
  };
  extern Context ctx;
}