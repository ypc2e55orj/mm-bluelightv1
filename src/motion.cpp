#include "motion.h"

#include <esp_timer.h>

#include "./driver/motor.h"

#include <cmath>
#include <cstdint>
#include <iostream>

#include "sensor.h"


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
  }

  void debug()
  {
    const int max_count = 1000;
    static int count = 0;
    static bool is_first = true;
    static float **sensor_log = nullptr;

    if (is_first)
    {
      sensor_log = new float *[max_count];
      for (int i = 0; i < max_count; i++)
      {
        sensor_log[i] = new float[13];
      }
      is_first = false;
    }

    if (count < max_count)
    {
      auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
      auto [accel_x, accel_y, accel_z] = driver::imu::accel();

      int ambient[4] = {}, flush[4] = {};
      driver::photo::get(ambient, flush);

      sensor_log[count][0] = driver::battery::get();
      sensor_log[count][1] = velo.first;
      sensor_log[count][2] = velo.second;
      sensor_log[count][3] = gyro_x;
      sensor_log[count][4] = gyro_y;
      sensor_log[count][5] = gyro_z;
      sensor_log[count][6] = accel_x;
      sensor_log[count][7] = accel_y;
      sensor_log[count][8] = accel_z;
      sensor_log[count][9] = flush[driver::photo::PHOTO_LEFT_90];
      sensor_log[count][10] = flush[driver::photo::PHOTO_LEFT_45];
      sensor_log[count][11] = flush[driver::photo::PHOTO_RIGHT_45];
      sensor_log[count][12] = flush[driver::photo::PHOTO_RIGHT_90];
      count++;
    }

    if (count == max_count)
    {
      std::cout << "Battery        , "
                << "Encoder Left   , "
                << "Encoder Right  , "
                << "Gyro  X [rad/s], "
                << "Gyro  Y [rad/s], "
                << "Gyro  Z [rad/s], "
                << "Accel X [m/s^2], "
                << "Accel Y [m/s^2], "
                << "Accel Z [m/s^2], "
                << "Photo Left  90 , "
                << "Photo Left  45 , "
                << "Photo Right 45 , "
                << "Photo Right 90 , " << std::endl;

      for (int i = 0; i < max_count; i++)
      {
        for (int j = 0; j < 13; j++)
        {
          std::cout << sensor_log[i][j] << ",";
        }
        std::cout << std::endl;
      }
      count++;
    }
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
        auto velo = wheel::velocity(driver::encoder::get());
        debug(velo);
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