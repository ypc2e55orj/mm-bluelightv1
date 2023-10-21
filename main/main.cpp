#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <iostream>

#include "driver/hardware/battery.hpp"
#include "driver/hardware/buzzer.hpp"
#include "driver/peripherals/spi.hpp"
#include "driver/hardware/indicator.hpp"
#include "driver/hardware/encoder.hpp"

void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;
  driver::hardware::Buzzer buzzer(GPIO_NUM_21);
  driver::hardware::Battery battery(ADC_UNIT_1, ADC_CHANNEL_4);
  driver::hardware::Indicator indicator(GPIO_NUM_45, 4);
  driver::peripherals::Spi spi(SPI2_HOST, GPIO_NUM_37, GPIO_NUM_35, GPIO_NUM_36, 4);
  driver::hardware::Encoder left(spi, GPIO_NUM_26);
  driver::hardware::Encoder right(spi, GPIO_NUM_39);

  buzzer.start(8192, 10, 1);


  buzzer.set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);

  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    left.update();
    right.update();

    std::cout << left.radian() << std::endl;
    std::cout << right.radian() << std::endl;

    indicator.update();
    indicator.rainbow_yield();
  }
}

// entrypoint
extern "C" void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 10, nullptr, 1);
}
