// C++
#include <cstdio>
#include <iostream>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "config.h"
#include "driver/driver.h"

driver::Driver *dri = nullptr;
config::Config *conf = nullptr;

[[noreturn]] void mainTask(void *) {
  dri->init_app();
  vTaskDelay(pdMS_TO_TICKS(500));
  std::cout << "mainTask() start. Core ID: " << xPortGetCoreID() << std::endl;

  conf->read_stdin();
  conf->write_stdout();

  printf("indicator & buzzer test\n");
  dri->buzzer->start(8192, 10, 1);
  for (int i = 0; i < 100; i++) {
    dri->buzzer->set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
    dri->indicator->rainbow_yield();
    dri->indicator->update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  dri->indicator->clear();
  dri->indicator->update();
  dri->buzzer->stop();

  /*
  printf("motor test\n");
  dri->motor_left->enable(), dri->motor_right->enable();
  dri->motor_left->speed(2000, 4000), dri->motor_right->speed(2000, 4000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  dri->motor_left->speed(0, 4000), dri->motor_right->speed(0, 4000);
  dri->motor_left->disable(), dri->motor_right->disable();
  */

  printf("\x1b[2J");
  // printf("\x1b[?25l");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  // NOLINT
    dri->photo->update();
    dri->battery->update();
    dri->imu->update();
    dri->encoder_left->update();
    dri->encoder_right->update();
    auto gyro = dri->imu->gyro();
    auto accel = dri->imu->accel();
    printf("\x1b[0;0H");
    printf(
        "Battery\n"
        "  voltage: %4d\n"
        "  average: %4d\n\n",
        dri->battery->voltage(), dri->battery->average());

    printf(
        "Photo\n"
        "  left90 : %4d, %4d\n"
        "  left45 : %4d, %4d\n"
        "  right45: %4d, %4d\n"
        "  right90: %4d, %4d\n\n",
        dri->photo->left90().ambient, dri->photo->left90().flash,
        dri->photo->left45().ambient, dri->photo->left45().flash,
        dri->photo->right45().ambient, dri->photo->right45().flash,
        dri->photo->right90().ambient, dri->photo->right90().flash);

    printf(
        "Gyro \n"
        "  x: %f\n"
        "  y: %f\n"
        "  z: %f\n\n",
        static_cast<double>(gyro.x), static_cast<double>(gyro.y),
        static_cast<double>(gyro.z));

    printf(
        "Accel\n"
        "  x: %f\n"
        "  y: %f\n"
        "  z: %f\n\n",
        static_cast<double>(accel.x), static_cast<double>(accel.y),
        static_cast<double>(accel.z));

    printf(
        "Encoder\n"
        "  left : %f\n"
        "  right: %f\n\n",
        static_cast<double>(dri->encoder_left->degree()),
        static_cast<double>(dri->encoder_right->degree()));
  }
}

// entrypoint
extern "C" [[maybe_unused]] void app_main(void) {
  dri = new driver::Driver();
  conf = new config::Config();
  dri->init_pro();
  std::cout << "app_main() start. Core ID: " << xPortGetCoreID() << std::endl;
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr,
                          1);
}
