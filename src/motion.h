#pragma once

namespace motion
{
  enum class RunMode
  {
    Straight,
    PivotTurn,
    SlalomTurn
  };

  void init();

  void start();
  void stop();

  bool running();
}