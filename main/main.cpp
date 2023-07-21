#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <iostream>
#include <fstream>
#include <cassert>
#include <stdio.h>

#include "../src/driver/battery.h"
#include "../src/driver/encoder.h"
#include "../src/driver/fs.h"
#include "../src/driver/imu.h"
#include "../src/driver/indicator.h"
#include "../src/driver/buzzer.h"
#include "../src/driver/photo.h"
#include "../src/driver/motor.h"

#include "../src/sensor.h"
#include "../src/shell.h"

#define EVENT_GROUP_SENSOR_IMU (1UL << 1)
#define EVENT_GROUP_SENSOR_ENCODER (1UL << 2)
#define EVENT_GROUP_SENSOR_ALL (EVENT_GROUP_SENSOR_IMU | EVENT_GROUP_SENSOR_ENCODER)

EventGroupHandle_t xEventGroupSensor = nullptr;

void backgroundTask(void *)
{
  std::cout << "backgroundTask() start. Core ID: " << xPortGetCoreID() << std::endl;

  sensor::start();
  vTaskDelay(pdMS_TO_TICKS(100));

  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;

  for (int i = 0; i < driver::indicator::nums(); i++)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(50));
    driver::indicator::clear();
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  for (int i = driver::indicator::nums() - 1; i > -1; i--)
  {
    driver::indicator::set(i, 0x0000FF);
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(50));
    driver::indicator::clear();
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  sensor::stop();
  driver::fs::df();
  driver::fs::ls("/spiffs/");
  sensor::start();

  shell::shell();
  while (true)
  {
  }
}

// entrypoint
extern "C" void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;
  xEventGroupSensor = xEventGroupCreate();
  assert(xEventGroupSensor != nullptr);

  driver::battery::init();
  driver::buzzer::init();
  driver::encoder::init(xEventGroupSensor, EVENT_GROUP_SENSOR_IMU);
  driver::fs::init();
  driver::imu::init(xEventGroupSensor, EVENT_GROUP_SENSOR_ENCODER);
  driver::indicator::init();
  driver::motor::init();
  driver::photo::init();
  sensor::init();

  xTaskCreatePinnedToCore(backgroundTask, "backgroundTask", 8192, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 10, nullptr, 1);
}
