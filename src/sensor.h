#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace sensor
{
  void init();

  void start();
  void stop();

  bool wait();
}
