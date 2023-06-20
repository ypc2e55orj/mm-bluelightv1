#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/speaker.h"

void mainTask(void *unused)
{
  driver::imu::init();
  driver::encoder::init();
  driver::speaker::play();

  while (true)
  {
    auto [left, right] = driver::encoder::angle();
    auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
    auto [accel_x, accel_y, accel_z] = driver::imu::accel();
    std::cout << "gyro_x:" << gyro_x << ", gyro_y: " << gyro_y << ", gyro_z: " << gyro_z << std::endl;
    std::cout << "accel_x:" << accel_x << ", accel_y: " << accel_y << ", accel_z: " << accel_z << std::endl;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
