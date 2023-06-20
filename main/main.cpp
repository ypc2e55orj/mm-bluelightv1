#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/indicator.h"
#include "../src/driver/speaker.h"

void mainTask(void *unused)
{
  driver::imu::init();
  driver::encoder::init();
  driver::indicator::init();

  driver::indicator::set(0, 255, 0, 0);
  driver::indicator::set(1, 0, 255, 0);
  driver::indicator::set(2, 0, 0, 255);
  driver::indicator::set(3, 255, 0, 0);
  driver::indicator::show();

  while (true)
  {
    auto [left, right] = driver::encoder::angle();
    auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
    auto [accel_x, accel_y, accel_z] = driver::imu::accel();
    std::cout << "encoder_left: " << left << ", encoder_right: " << right << std::endl;
    std::cout << "gyro_x:" << gyro_x << ", gyro_y: " << gyro_y << ", gyro_z: " << gyro_z << std::endl;
    std::cout << "accel_x:" << accel_x << ", accel_y: " << accel_y << ", accel_z: " << accel_z << std::endl;
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
