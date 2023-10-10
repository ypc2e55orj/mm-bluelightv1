#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <iostream>

#include "driver/hardware/battery.hpp"
#include "driver/hardware/buzzer.hpp"
#include "driver/hardware/indicator.hpp"

void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;
  driver::hardware::Buzzer buzzer(GPIO_NUM_21);
  driver::hardware::Battery battery(ADC_UNIT_1, ADC_CHANNEL_4);
  driver::hardware::Indicator indicator(GPIO_NUM_45, 4);

  buzzer.start(8192, 10, 1);
  indicator.start(8192, 10, 1);
  battery.start(8192, 10, 0);

  buzzer.set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);

  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    std::cout << battery.average() << std::endl;
    indicator.rainbow_yield();
  }
}

// entrypoint
extern "C" void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 10, nullptr, 1);
}
