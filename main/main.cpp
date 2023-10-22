#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <iostream>

#include "driver/hardware/battery.hpp"
#include "driver/hardware/buzzer.hpp"
#include "driver/hardware/encoder.hpp"
#include "driver/hardware/imu.hpp"
#include "driver/hardware/indicator.hpp"
#include "driver/peripherals/gpio.hpp"
#include "driver/peripherals/spi.hpp"

void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;

  driver::peripherals::Spi spi2(SPI2_HOST, GPIO_NUM_37, GPIO_NUM_35, GPIO_NUM_36, 4);
  driver::peripherals::Spi spi3(SPI3_HOST, GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_33, 16);

  driver::hardware::Buzzer buzzer(GPIO_NUM_21);
  driver::hardware::Battery battery(ADC_UNIT_1, ADC_CHANNEL_4);
  driver::hardware::Indicator indicator(GPIO_NUM_45, 4);

  driver::hardware::Imu imu(spi3, GPIO_NUM_34);
  driver::hardware::Encoder left(spi2, GPIO_NUM_26);
  driver::hardware::Encoder right(spi2, GPIO_NUM_39);

  buzzer.start(8192, 10, 1);
  buzzer.set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);

  auto xLastWakeTime = xTaskGetTickCount();
  printf("\x1b[2J");
  printf("\x1b[?25l");
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    imu.update();
    left.update();
    right.update();

    auto gyro = imu.gyro();
    auto accel = imu.accel();
    printf("\x1b[0;0H");
    printf("Gyro \nx: %f\ny: %f\nz: %f\n", gyro.x, gyro.y, gyro.z);
    printf("Accel\nx: %f\ny: %f\nz: %f\n", accel.x, accel.y, accel.z);
    printf("Encoder\nleft : %f\nright: %f\n", left.degree(), right.degree());

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
