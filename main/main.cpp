#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <iostream>

#include <cassert>

#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/indicator.h"
#include "../src/driver/speaker.h"

#include "../src/core.h"
#include "../src/priority.h"

#define EVENT_GROUP_SENSOR_IMU (1 << 1)
#define EVENT_GROUP_SENSOR_ENCODER (1 << 2)
#define EVENT_GROUP_SENSOR_ALL (EVENT_GROUP_SENSOR_IMU | EVENT_GROUP_SENSOR_ENCODER)

uint32_t diff = 0;
EventGroupHandle_t xEventGroupSensor = nullptr;

void sensorTask(void *pv)
{
  while (true)
  {
    int64_t start = esp_timer_get_time();
    driver::imu::update();
    driver::encoder::update();
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(10));
    diff = esp_timer_get_time() - start;
  }
}

void mainTask(void *pv)
{
  for (int i = 0; i < driver::indicator::nums(); i++)
  {
    driver::indicator::clear();
    driver::indicator::set(i, 0x0000FF);
    driver::speaker::tone(4000, 100);
  }
  for (int i = driver::indicator::nums() - 1; i > -1; i--)
  {
    driver::indicator::clear();
    driver::indicator::set(i, 0x0000FF);
    driver::speaker::tone(4000, 100);
  }
  driver::indicator::clear();

  while (true)
  {
    EventBits_t uBits = xEventGroupWaitBits(xEventGroupSensor, EVENT_GROUP_SENSOR_ALL, pdTRUE, pdTRUE, pdMS_TO_TICKS(100));
    if (uBits == EVENT_GROUP_SENSOR_ALL)
    {
      auto [angle_left, angle_right] = driver::encoder::get();
      auto [gyro_x, gyro_y, gyro_z] = driver::imu::gyro();
      auto [accel_x, accel_y, accel_z] = driver::imu::accel();

      std::cout << "\x1b[2J\x1b[0;0H"
                << "Diff             : " << diff << std::endl
                << "Encoder Left     : " << angle_left << std::endl
                << "Encoder Right    : " << angle_right << std::endl
                << "Gyro  X [dgree/s]: " << gyro_x << std::endl
                << "Gyro  Y [dgree/s]: " << gyro_y << std::endl
                << "Gyro  Z [dgree/s]: " << gyro_z << std::endl
                << "Accel X   [m/s^2]: " << accel_x << std::endl
                << "Accel Y   [m/s^2]: " << accel_y << std::endl
                << "Accel Z   [m/s^2]: " << accel_z << std::endl
                << std::endl;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xEventGroupSensor = xEventGroupCreate();
  assert(xEventGroupSensor != nullptr);

  driver::imu::init(xEventGroupSensor, EVENT_GROUP_SENSOR_ENCODER);
  driver::encoder::init(xEventGroupSensor, EVENT_GROUP_SENSOR_IMU);
  driver::indicator::init();
  driver::speaker::init();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, xTaskGetCurrentTaskHandle(), PRIORITY_HIGH, nullptr, CORE_SENSE);
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), PRIORITY_HIGH, nullptr, CORE_MAIN);
}
