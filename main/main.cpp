#include <sdkconfig.h>
#include <esp_rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"

void mainTask(void *unused)
{
  driver::imu::init();
  driver::encoder::init();

  while (true)
  {
    uint16_t left, right;

    driver::encoder::angle(left, right);
    std::cout << "left: " << left << ", right: " << right << std::endl;
  }
}

// entrypoint
extern "C" void app_main(void)
{
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
}
