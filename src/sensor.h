#pragma once

#include <utility>

namespace sensor
{
  void init();

  void start();
  void stop();
  bool running();

  bool wait();

  std::pair<float, float> velocity();
}
