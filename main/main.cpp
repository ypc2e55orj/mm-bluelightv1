#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <iostream>
#include <cassert>

#include "../src/driver/battery.h"
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/indicator.h"
#include "../src/driver/buzzer.h"
#include "../src/driver/photo.h"

#define EVENT_GROUP_SENSOR_IMU (1UL << 1)
#define EVENT_GROUP_SENSOR_ENCODER (1UL << 2)
#define EVENT_GROUP_SENSOR_ALL (EVENT_GROUP_SENSOR_IMU | EVENT_GROUP_SENSOR_ENCODER)

EventGroupHandle_t xEventGroupSensor = nullptr;
uint32_t sensorTaskDiff = 0;

void sensorTask(void *unused)
{
  while (true)
  {
    int64_t start = esp_timer_get_time();
    driver::imu::update();
    driver::encoder::update();
    driver::photo::update();
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(1));
    sensorTaskDiff = esp_timer_get_time() - start;
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

  bool blink = true;
  while (true)
  {
    EventBits_t uBits = xEventGroupWaitBits(xEventGroupSensor, EVENT_GROUP_SENSOR_ALL, pdTRUE, pdTRUE, pdMS_TO_TICKS(100));
    if (uBits == EVENT_GROUP_SENSOR_ALL)
    {
      for (int i = 0; i < driver::indicator::nums(); i++)
      {
        driver::indicator::set(i, blink ? 0x00000F : 0x000000);
      }

      blink = !blink;

      auto [angle_left, angle_right] = driver::encoder::get();
      auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
      auto [accel_x, accel_y, accel_z] = driver::imu::accel();

      int result[4] = {};
      driver::photo::get(result);

      std::cout << "\x1b[2J\x1b[0;0H"
                << "SensorTask Diff: " << sensorTaskDiff << std::endl
                << "Battery        : " << driver::battery::get() << std::endl
                << "Encoder Left   : " << angle_left << std::endl
                << "Encoder Right  : " << angle_right << std::endl
                << "Gyro  X [rad/s]: " << gyro_x << std::endl
                << "Gyro  Y [rad/s]: " << gyro_y << std::endl
                << "Gyro  Z [rad/s]: " << gyro_z << std::endl
                << "Accel X [m/s^2]: " << accel_x << std::endl
                << "Accel Y [m/s^2]: " << accel_y << std::endl
                << "Accel Z [m/s^2]: " << accel_z << std::endl
                << "Photo Left  90 : " << result[driver::photo::LEFT_90] << std::endl
                << "Photo Left  45 : " << result[driver::photo::LEFT_45] << std::endl
                << "Photo Right 45 : " << result[driver::photo::RIGHT_45] << std::endl
                << "Photo Right 90 : " << result[driver::photo::RIGHT_90] << std::endl
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

  driver::battery::init();
  driver::imu::init(xEventGroupSensor, EVENT_GROUP_SENSOR_ENCODER);
  driver::encoder::init(xEventGroupSensor, EVENT_GROUP_SENSOR_IMU);
  driver::buzzer::init();
  driver::indicator::init();
  driver::photo::init();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 0);
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
