#include "sensor.h"

// ESP-IDF
#include <esp_timer.h>

// Project
#include "driver/driver.h"
#include "motion.h"
#include "task.h"

namespace sensor {
class Sensor::SensorImpl final : public task::Task {
 private:
  static constexpr uint32_t WARM_UP_COUNTS = 10;

  driver::Driver &dri_;
  motion::Motion &mot_;

  int64_t prev_us_;
  uint32_t delta_us_;

  void update() {
    auto curr_us = esp_timer_get_time();
    delta_us_ = static_cast<uint32_t>(curr_us - prev_us_);
    prev_us_ = curr_us;
    dri_.photo->update();
    dri_.imu->update();
    dri_.encoder_left->update();
    dri_.encoder_right->update();
    dri_.battery->update();
    dri_.photo->wait();
  }

  void setup() override {
    for (uint32_t i = 0; i < WARM_UP_COUNTS; i++) {
      update();
    }
    // motionタスクに開始通知
    mot_.update_notify(0);
  }
  void loop() override {
    update();
    // motionタスクにセンサ値更新完了通知
    mot_.update_notify(delta_us_);
  }
  void end() override {
    prev_us_ = 0;
    delta_us_ = 0;
  }

 public:
  explicit SensorImpl(driver::Driver &dri, motion::Motion &mot)
      : task::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        mot_(mot),
        prev_us_(),
        delta_us_() {}
  ~SensorImpl() override = default;

  [[nodiscard]] uint32_t delta_us() const { return delta_us_; }
};

Sensor::Sensor(driver::Driver &dri, motion::Motion &mot)
    : impl_(new SensorImpl(dri, mot)) {}
Sensor::~Sensor() = default;

bool Sensor::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Sensor::stop() { return impl_->stop(); }
uint32_t Sensor::delta_us() { return impl_->delta_us(); }
}  // namespace sensor