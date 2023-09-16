#pragma once

#include <memory>

namespace driver
{
  class Battery
  {
  private:
    class BatteryImpl; std::unique_ptr<BatteryImpl> impl_;
  public:
    explicit Battery() = default;
    ~Battery() = default;

    bool start();
    bool stop();
  };
}