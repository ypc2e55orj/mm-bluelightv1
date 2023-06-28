#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <iostream>
#include <cassert>

#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/indicator.h"
#include "../src/driver/buzzer.h"

#define EVENT_GROUP_SENSOR_IMU (1 << 1)
#define EVENT_GROUP_SENSOR_ENCODER (1 << 2)
#define EVENT_GROUP_SENSOR_ALL (EVENT_GROUP_SENSOR_IMU | EVENT_GROUP_SENSOR_ENCODER)

EventGroupHandle_t xEventGroupSensor = nullptr;

void sensorTask(void *unused)
{
  while (true)
  {
    driver::imu::update();
    driver::encoder::update();
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void mainTask(void *unused)
{
  for (int i = 0; i < driver::indicator::nums(); i++)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::buzzer::tone(4000, 100);
    vTaskDelay(pdMS_TO_TICKS(100));
    driver::indicator::clear();
  }
  for (int i = driver::indicator::nums() - 1; i > -1; i--)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::buzzer::tone(4000, 100);
    vTaskDelay(pdMS_TO_TICKS(100));
    driver::indicator::clear();
  }

  while (true)
  {
    EventBits_t uBits = xEventGroupWaitBits(xEventGroupSensor, EVENT_GROUP_SENSOR_ALL, pdTRUE, pdTRUE, pdMS_TO_TICKS(100));
    if (uBits == EVENT_GROUP_SENSOR_ALL)
    {
      auto [angle_left, angle_right] = driver::encoder::get();
      auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
      auto [accel_x, accel_y, accel_z] = driver::imu::accel();

      std::cout << "\x1b[2J\x1b[0;0H"
                << "Left    : " << angle_left << std::endl
                << "Right   : " << angle_right << std::endl
                << "Gyro  X [mdps]: " << gyro_x << std::endl
                << "Gyro  Y [mdps]: " << gyro_y << std::endl
                << "Gyro  Z [mdps]: " << gyro_z << std::endl
                << "Accel X   [mg]: " << accel_x << std::endl
                << "Accel Y   [mg]: " << accel_y << std::endl
                << "Accel Z   [mg]: " << accel_z << std::endl
                << std::flush;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xEventGroupSensor = xEventGroupCreate();
  assert(xEventGroupSensor != NULL);

  driver::imu::init(xEventGroupSensor, EVENT_GROUP_SENSOR_ENCODER);
  driver::encoder::init(xEventGroupSensor, EVENT_GROUP_SENSOR_IMU);
  driver::buzzer::init();
  driver::indicator::init();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 0);
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
