// C++
#include <cstdio>
#include <iostream>

// ESP-IDF
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "config.h"
#include "driver/driver.h"
#include "sensor.h"

static constexpr auto TAG = "mm-bluelight";

driver::Driver *dri = nullptr;
config::Config *conf = nullptr;
sensor::Sensor *sens = nullptr;

[[noreturn]] void mainTask(void *) {
  ESP_LOGI(TAG, "mainTask() is started. Core ID: %d", xPortGetCoreID());
  ESP_LOGI(TAG, "Initializing driver (for app cpu)");
  dri->init_app();
  dri->indicator->clear();
  dri->indicator->update();

  const auto config_path = std::string(dri->fs->base_path()) + "/config.json";
  ESP_LOGI(TAG, "Reading %s", config_path.c_str());
  if (!conf->read_file(config_path)) {
    ESP_LOGW(TAG, "%s is not found. creating...", config_path.c_str());
    conf->write_file(config_path);
  }
  conf->write_stdout();

  dri->buzzer->start(8192, 10, 1);
  dri->buzzer->set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
  sens->start(8192, 10, 0);

  /*
  printf("motor test\n");
  dri->motor_left->enable(), dri->motor_right->enable();
  dri->motor_left->speed(2000, 4000), dri->motor_right->speed(2000, 4000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  dri->motor_left->speed(0, 4000), dri->motor_right->speed(0, 4000);
  dri->motor_left->disable(), dri->motor_right->disable();
  */

  // dri->console->start();
  printf("\x1b[2J");
  // printf("\x1b[?25l");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  // NOLINT
    dri->indicator->rainbow_yield();
    dri->indicator->update();
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
  ESP_LOGI(TAG, "app_main() is started. Core ID: %d", xPortGetCoreID());
  dri = new driver::Driver();
  conf = new config::Config();
  sens = new sensor::Sensor(dri);
  ESP_LOGI(TAG, "Initializing driver (for pro cpu)");
  dri->init_pro();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr,
                          1);
}
