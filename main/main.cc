// C++
#include <cstdio>

// ESP-IDF
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "config.h"
#include "driver/driver.h"
#include "motion.h"
#include "odometry.h"
#include "sensor.h"

static constexpr auto TAG = "mm-bluelight";

motion::Motion *mot = nullptr;
odometry::Odometry *odom = nullptr;
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
  // conf->write_file(config_path);
  ESP_LOGI(TAG, "Reading %s", config_path.c_str());
  if (!conf->read_file(config_path)) {
    ESP_LOGW(TAG, "%s is not found. creating...", config_path.c_str());
    conf->write_file(config_path);
  }
  conf->write_stdout();

  dri->buzzer->set(driver::hardware::Buzzer::Mode::InitializeSuccess, false);
  dri->buzzer->update();
  mot->start(8192, 10, 0);
  sens->start(8192, 20, 0);

  uint64_t index = 0;
  printf(
      "Index"
      ",SensorDelta,MotionDelta"
      ",BatteryVoltage,BatteryVoltageAverage"
      ",AmbientLeft90,AmbientLeft45,AmbientRight45,AmbientRight90"
      ",FlashLeft90,FlashLeft45,FlashRight45,FlashRight90"
      ",GyroX,GyroY,GyroZ"
      ",AccelX,AccelY,AccelZ"
      ",EncoderLeft,EncoderRight"
      ",Radian,X,Y"
      "\n");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  // NOLINT
    dri->indicator->rainbow_yield();
    auto gyro = dri->imu->gyro();
    auto accel = dri->imu->accel();

    // clang-format off
    printf(
        "%lld"
        ",%ld,%ld"
        ",%d,%d"
        ",%d,%d,%d,%d"
        ",%d,%d,%d,%d"
        ",%f,%f,%f"
        ",%f,%f,%f"
        ",%f,%f"
        ",%f,%f,%f"
        "\n",
        index++,
        sens->delta_us(), mot->delta_us(),
        dri->battery->voltage(), dri->battery->average(),
        dri->photo->left90().ambient, dri->photo->left45().ambient, dri->photo->right45().ambient, dri->photo->right90().ambient,
        dri->photo->left90().flash, dri->photo->left45().flash, dri->photo->right45().flash, dri->photo->right90().flash,
        static_cast<double>(gyro.x), static_cast<double>(gyro.y), static_cast<double>(gyro.z),
        static_cast<double>(accel.x), static_cast<double>(accel.y), static_cast<double>(accel.z),
        static_cast<double>(dri->encoder_left->radian()), static_cast<double>(dri->encoder_right->radian()),
        static_cast<double>(odom->radian()), static_cast<double>(odom->x()), static_cast<double>(odom->y())
    );
    // clang-format on
  }
}

// entrypoint
extern "C" [[maybe_unused]] void app_main(void) {
  ESP_LOGI(TAG, "app_main() is started. Core ID: %d", xPortGetCoreID());
  dri = new driver::Driver();
  conf = new config::Config();
  odom = new odometry::Odometry(*dri, *conf);
  mot = new motion::Motion(*dri, *conf, *odom);
  sens = new sensor::Sensor(*dri, *mot);
  ESP_LOGI(TAG, "Initializing driver (for pro cpu)");
  dri->init_pro();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr,
                          1);
}
