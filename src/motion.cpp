#include "motion.hpp"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/hardware/motor.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

#include "sensor.hpp"

namespace vehicle
{

}

namespace motion
{
  static bool req_running = false;
  static bool is_running = false;

  void init()
  {
    driver::motor::init();
    driver::motor::enable();
    driver::motor::brake();
  }

  static void motionTask(void *)
  {
    is_running = true;

    int64_t prev = esp_timer_get_time(), curr;
    while (req_running)
    {
      if (sensor::wait())
      {
        curr = esp_timer_get_time();
        auto velo = sensor::velocity();
        prev = curr;
      }
    }

    vTaskDelete(nullptr);
    is_running = false;
  }

  void start()
  {
    req_running = true;
    xTaskCreatePinnedToCore(motionTask, "motionTask", 8192, nullptr, 5, nullptr, 0);
  }

  void stop()
  {
    req_running = false;
  }

  bool running()
  {
    return is_running;
  }
}