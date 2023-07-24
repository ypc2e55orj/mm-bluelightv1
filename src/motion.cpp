#include "motion.h"

#include <esp_timer.h>

#include "./driver/battery.h"
#include "./driver/encoder.h"
#include "./driver/imu.h"
#include "./driver/motor.h"
#include "./driver/photo.h"

#include <cmath>
#include <cstdint>
#include <iostream>

#include "filter/pid.h"
#include "sensor.h"

namespace wheel
{
  static struct
  {
    uint16_t left;
    uint16_t right;
  } prev_raw = {};
  static struct
  {
    float left;
    float right;
  } speed = {};

  static int calculate_diff(const uint16_t &p, const uint16_t &c)
  {
    int d = c - p;
    if (std::abs(d) > driver::encoder::resolution() / 2 + 1)
    {
      return p > driver::encoder::resolution() / 2 + 1 ? driver::encoder::resolution() - p + c
                                                       : p + driver::encoder::resolution() - c;
    }
    return d;
  }

  void update(std::pair<uint16_t, uint16_t> curr)
  {
    int diff_left = calculate_diff(prev_raw.left, curr.first);
    int diff_right = calculate_diff(prev_raw.right, curr.second);
    prev_raw.left = curr.first;
    prev_raw.right = curr.second;

    speed.left = (static_cast<float>(diff_left) * (12.80f * static_cast<float>(M_PI) / static_cast<float>(driver::encoder::resolution() + 1))) * 0.1f + speed.left * 0.9f;
    speed.right = (static_cast<float>(diff_right) * (12.80f * static_cast<float>(M_PI) / static_cast<float>(driver::encoder::resolution() + 1))) * 0.1f + speed.right * 0.9f;
  }
}

namespace vehicle
{
  
}

namespace motion
{
  static bool stop_request = false;

  void init()
  {
    driver::motor::init();
  }

  void debug()
  {
    static int count = 0;

    if (++count == 100)
    {
      count = 0;

      auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
      auto [accel_x, accel_y, accel_z] = driver::imu::accel();

      int ambient[4] = {}, flush[4] = {};
      driver::photo::get(ambient, flush);

      std::cout << "\x1b[2J\x1b[0;0H"
                << "Battery        : " << driver::battery::get() << std::endl
                << "Encoder Left   : " << wheel::speed.left << std::endl
                << "Encoder Right  : " << wheel::speed.right << std::endl
                << "Gyro  X [rad/s]: " << gyro_x << std::endl
                << "Gyro  Y [rad/s]: " << gyro_y << std::endl
                << "Gyro  Z [rad/s]: " << gyro_z << std::endl
                << "Accel X [m/s^2]: " << accel_x << std::endl
                << "Accel Y [m/s^2]: " << accel_y << std::endl
                << "Accel Z [m/s^2]: " << accel_z << std::endl
                << "Photo Left  90 : " << flush[driver::photo::PHOTO_LEFT_90] << std::endl
                << "Photo Left  45 : " << flush[driver::photo::PHOTO_LEFT_45] << std::endl
                << "Photo Right 45 : " << flush[driver::photo::PHOTO_RIGHT_45] << std::endl
                << "Photo Right 90 : " << flush[driver::photo::PHOTO_RIGHT_90] << std::endl;
    }
  }

  static void motionTask(void *)
  {
    int64_t prev = esp_timer_get_time(), curr;
    while (!stop_request)
    {
      if (sensor::wait())
      {
        curr = esp_timer_get_time();
        wheel::update(driver::encoder::get());
        debug();
        prev = curr;
      }
    }

    vTaskDelete(nullptr);
  }

  void start()
  {
    stop_request = false;
    xTaskCreatePinnedToCore(motionTask, "motionTask", 8192, nullptr, 5, nullptr, 0);
  }

  void stop()
  {
    stop_request = true;
  }
}