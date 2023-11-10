// C++
#include <cstdio>
#include <iostream>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "driver/driver.hpp"

[[noreturn]] void mainTask(void *)
{
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;
  driver::ctx.initialize();

  printf("indicator & buzzer test\n");
  driver::ctx.buzzer->start(8192, 10, 1);
  for (int i = 0; i < 100; i++)
  {
    driver::ctx.buzzer->set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
    driver::ctx.indicator->rainbow_yield();
    driver::ctx.indicator->update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  driver::ctx.indicator->clear();
  driver::ctx.indicator->update();
  driver::ctx.buzzer->stop();

  printf("motor test\n");
  driver::ctx.motor_left->enable(), driver::ctx.motor_right->enable();
  driver::ctx.motor_left->speed(2000, 4000), driver::ctx.motor_right->speed(2000, 4000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  driver::ctx.motor_left->speed(0, 4000), driver::ctx.motor_right->speed(0, 4000);
  driver::ctx.motor_left->disable(), driver::ctx.motor_right->disable();

  printf("\x1b[2J");
  printf("\x1b[?25l");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    driver::ctx.photo->update();
    driver::ctx.battery->update();
    driver::ctx.imu->update();
    driver::ctx.encoder_left->update();
    driver::ctx.encoder_right->update();
    auto gyro = driver::ctx.imu->gyro();
    auto accel = driver::ctx.imu->accel();
    printf("\x1b[0;0H");
    printf("Battery\n"
           "  voltage: %4d\n"
           "  average: %4d\n\n",
           driver::ctx.battery->voltage(), driver::ctx.battery->average());

    printf("Photo\n"
           "  left90 : %4d, %4d\n"
           "  left45 : %4d, %4d\n"
           "  right45: %4d, %4d\n"
           "  right90: %4d, %4d\n\n",
           driver::ctx.photo->left90().ambient, driver::ctx.photo->left90().flash, driver::ctx.photo->left45().ambient,
           driver::ctx.photo->left45().flash, driver::ctx.photo->right45().ambient, driver::ctx.photo->right45().flash,
           driver::ctx.photo->right90().ambient, driver::ctx.photo->right90().flash);

    printf("Gyro \n"
           "  x: %f\n"
           "  y: %f\n"
           "  z: %f\n\n",
           gyro.x, gyro.y, gyro.z);

    printf("Accel\n"
           "  x: %f\n"
           "  y: %f\n"
           "  z: %f\n\n",
           accel.x, accel.y, accel.z);

    printf("Encoder\n"
           "  left : %f\n"
           "  right: %f\n\n",
           driver::ctx.encoder_left->degree(), driver::ctx.encoder_right->degree());
  }
}

// entrypoint
extern "C" [[maybe_unused]] void app_main(void)
{
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;

  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr, 1);
}
