#include "sensor.h"

// ESP-IDF
#include <esp_timer.h>

// Project
#include "driver/driver.h"
#include "odometry.h"
#include "task.h"

namespace sensor {
class Sensor::SensorImpl final : public task::Task {
 private:
  driver::Driver *dri_;
  odometry::Odometry *odom_;
  int64_t prev_us_;
  int32_t delta_us_;

  void setup() override { prev_us_ = esp_timer_get_time(); }
  void loop() override {
    auto curr_us = esp_timer_get_time();
    delta_us_ = static_cast<int32_t>(curr_us - prev_us_);
    prev_us_ = curr_us;
    dri_->photo->update();
    dri_->imu->update();
    dri_->encoder_left->update();
    dri_->encoder_right->update();
    dri_->battery->update();
    dri_->photo->wait();
    odom_->update(delta_us_);
  }
  void end() override {
    prev_us_ = 0;
    delta_us_ = 0;
  }

 public:
  explicit SensorImpl(driver::Driver *dri, odometry::Odometry *odom)
      : task::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        odom_(odom),
        prev_us_(),
        delta_us_() {}
  ~SensorImpl() override = default;

  [[nodiscard]] int32_t delta_us() const { return delta_us_; }
};

Sensor::Sensor(driver::Driver *dri, odometry::Odometry *odom)
    : impl_(new SensorImpl(dri, odom)) {}
Sensor::~Sensor() = default;

bool Sensor::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Sensor::stop() { return impl_->stop(); }
int32_t Sensor::delta_us() { return impl_->delta_us(); }
}  // namespace sensor