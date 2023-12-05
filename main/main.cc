// C++
#include <cmath>
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

void calibrateImu() { dri->imu->calibration(); }

[[noreturn]] void printSummary() {
  uint64_t index = 0;

  sens->start(8192, 20, 0);
  printf(
      "Index"
      ",SensorDelta,MotionDelta"
      ",BatteryVoltage,BatteryVoltageAverage"
      ",MotorVoltageLeft,MotorVoltageRight"
      ",WheelVelocityLeft,WheelVelocityRight"
      ",AmbientLeft90,AmbientLeft45,AmbientRight45,AmbientRight90"
      ",FlashLeft90,FlashLeft45,FlashRight45,FlashRight90"
      ",RawGyroX,RawGyroY,RawGyroZ"
      ",RawAccelX,RawAccelY,RawAccelZ"
      ",WheelAngularVelocityLeft,WheelAngularVelocityRight"
      ",Radian,X,Y"
      "\n");
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  // NOLINT
    auto &gyro = dri->imu->raw_angular_rate();
    auto &accel = dri->imu->raw_linear_acceleration();

    float accel_x = static_cast<float>(accel.x) *
                    dri->imu->linear_acceleration_sensitivity();
    float accel_y = static_cast<float>(accel.y) *
                    dri->imu->linear_acceleration_sensitivity();
    float accel_z = static_cast<float>(accel.z) *
                    dri->imu->linear_acceleration_sensitivity();

    float gyro_x =
        static_cast<float>(gyro.x) * dri->imu->angular_rate_sensitivity();
    float gyro_y =
        static_cast<float>(gyro.y) * dri->imu->angular_rate_sensitivity();
    float gyro_z =
        static_cast<float>(gyro.z) * dri->imu->angular_rate_sensitivity();

    auto velocity = odom->wheels_velocity();
    auto angular_velocity = odom->wheels_angular_velocity();
    // clang-format off
    printf(
      "%lld"
      ",%ld,%ld"
      ",%d,%d"
      ",%d,%d"
      ",%f,%f"
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
      dri->motor_left->voltage(), dri->motor_right->voltage(),
      static_cast<double>(velocity.left), static_cast<double>(velocity.right),
      dri->photo->left90().ambient, dri->photo->left45().ambient, dri->photo->right45().ambient, dri->photo->right90().ambient,
      dri->photo->left90().flash, dri->photo->left45().flash, dri->photo->right45().flash, dri->photo->right90().flash,
      static_cast<double>(gyro_x), static_cast<double>(gyro_y), static_cast<double>(gyro_z),
      static_cast<double>(accel_x), static_cast<double>(accel_y), static_cast<double>(accel_z),
      static_cast<double>(angular_velocity.left), static_cast<double>(angular_velocity.right),
      static_cast<double>(odom->angle()), static_cast<double>(odom->x()), static_cast<double>(odom->y())
    );
    // clang-format on
  }
}

void mainTask(void *) {
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

  calibrateImu();
  printSummary();
}

// entrypoint
extern "C" [[maybe_unused]] void app_main(void) {
  ESP_LOGI(TAG, "app_main() is started. Core ID: %d", xPortGetCoreID());
  dri = new driver::Driver();
  conf = new config::Config();
  odom = new odometry::Odometry(*dri, *conf);
  mot = new motion::Motion(*dri, *conf, *odom);
  sens = new sensor::Sensor(*dri, *odom);
  ESP_LOGI(TAG, "Initializing driver (for pro cpu)");
  dri->init_pro();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192 * 2, nullptr, 10, nullptr,
                          1);
}
