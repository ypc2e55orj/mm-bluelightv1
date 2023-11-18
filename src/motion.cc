#include "motion.h"

// C++

// ESP-IDF
#include <esp_system.h>

// Project
#include "config.h"
#include "driver/driver.h"
#include "odometry.h"
#include "task.h"

namespace motion {
class Motion::MotionImpl final : public task::Task {
 private:
  driver::Driver &dri_;
  config::Config &conf_;
  odometry::Odometry &odom_;

  uint32_t delta_us_;

  void setup() override {}
  void loop() override {
    if (xTaskNotifyWait(0, 0, &delta_us_, portMAX_DELAY) == pdTRUE) {
      if (delta_us_ == 0) {
        // 開始
      } else {
        // センサ取得通知
        if (conf_.low_voltage > dri_.battery->average()) {
          esp_restart();
        }
        odom_.update(delta_us_);
        dri_.motor_left->enable();
        dri_.motor_right->enable();
        dri_.motor_left->disable();
        dri_.motor_right->disable();
      }
    }
  }
  void end() override {}

 public:
  explicit MotionImpl(driver::Driver &dri, config::Config &conf,
                      odometry::Odometry &odom)
      : task::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        conf_(conf),
        odom_(odom),
        delta_us_() {}

  bool update_notify(uint32_t delta_us) {
    auto pdRet = xTaskNotify(handle(), delta_us, eSetValueWithOverwrite);
    return pdRet == pdTRUE;
  }

  [[nodiscard]] uint32_t delta_us() const { return delta_us_; }
};

Motion::Motion(driver::Driver &dri, config::Config &conf,
               odometry::Odometry &odom)
    : impl_(new MotionImpl(dri, conf, odom)) {}
Motion::~Motion() = default;

bool Motion::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Motion::stop() { return impl_->stop(); }

bool Motion::update_notify(uint32_t delta_us) {
  return impl_->update_notify(delta_us);
}
uint32_t Motion::delta_us() { return impl_->delta_us(); }
}  // namespace motion
