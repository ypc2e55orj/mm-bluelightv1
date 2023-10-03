#pragma once

#include "driver/hardware/battery.hpp"

namespace driver
{
  struct Context
  {
    Battery *battery;
  };
  extern Context ctx;
}