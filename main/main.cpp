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
    std::cout << "left(" << "): " << left << ", right: " << right << std::endl;
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
