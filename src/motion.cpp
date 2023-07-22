#include "motion.h"

#include <esp_timer.h>

#include "./driver/battery.h"
#include "./driver/encoder.h"
#include "./driver/imu.h"
#include "./driver/motor.h"
#include "./driver/photo.h"

#include "pid.h"
#include "sensor.h"

#include <iostream>

namespace motion
{
  void init()
  {
    driver::motor::init();
  }

  static void motionTask(void *)
  {
    const size_t sample_nums = 1000;
    int samples[1000] = {};
    size_t sample_index = 0;
    int64_t prev = esp_timer_get_time(), curr;
    while (true)
    {
      if (sensor::wait())
      {
        curr = esp_timer_get_time();
        samples[sample_index] = curr - prev;
        if (++sample_index == sample_nums)
        {
          break;
        }
        prev = curr;
      }
    }
    std::cout << "End measuring." << std::endl;

    float mean = 0.0f;
    for (size_t i = 0; i < sample_nums; i++)
    {
      mean += samples[i];
      std::cout << "samples[" << i << "]: " << samples[i] << std::endl;
    }
    mean /= sample_nums;
    std::cout << "mean: " << mean << std::endl;

    while (true)
      ;
  }

  void start()
  {
    xTaskCreatePinnedToCore(motionTask, "motionTask", 8192, nullptr, 5, nullptr, 0);
  }

  void stop()
  {
  }
}