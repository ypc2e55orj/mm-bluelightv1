#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"

void buzzer(uint16_t usleep)
{
  
}

// entrypoint
extern "C" void app_main(void)
{
  driver::imu::init();
  driver::encoder::init();

  uint16_t left, right;
  while (true)
  {
    driver::encoder::angle(left, right);
    std::cout << "left: " << left << ", right: " << right << std::endl;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
