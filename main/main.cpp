#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <iostream>

#include "../src/driver/buzzer.h"
#include "../src/driver/fs.h"
#include "../src/driver/indicator.h"

#include "../src/motion.h"
#include "../src/sensor.h"

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
  driver::buzzer::tone(4000, 100);

  while(true)
  {
    driver::indicator::rainbow_yield();
    driver::indicator::update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// entrypoint
extern "C" void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  driver::buzzer::init();

  driver::fs::init();
  driver::indicator::init();
  sensor::init();
  motion::init();

  sensor::start();
  motion::start();

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 10, nullptr, 1);
}
