#include "sensor.h"

// ESP-IDF
#include <esp_timer.h>

// Project
#include "driver/driver.h"
#include "task.h"

namespace sensor {
class Sensor::SensorImpl final : public task::Task {
 private:
  driver::Driver *dri_;
  int64_t prev_us_, delta_us_;

  void setup() override { prev_us_ = esp_timer_get_time(); }
  void loop() override {
    auto curr_us = esp_timer_get_time();
    delta_us_ = curr_us - prev_us_;
    prev_us_ = curr_us;
    dri_->photo->update();
    dri_->imu->update();
    dri_->encoder_left->update();
    dri_->encoder_right->update();
    dri_->battery->update();
    dri_->photo->wait();
  }
  void end() override {
    prev_us_ = 0;
    delta_us_ = 0;
  }

 public:
  explicit SensorImpl(driver::Driver *dri)
      : task::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        prev_us_(),
        delta_us_() {}
  ~SensorImpl() override = default;

  [[nodiscard]] int64_t delta_us() const { return delta_us_; }
};

Sensor::Sensor(driver::Driver *dri) : impl_(new SensorImpl(dri)) {}
Sensor::~Sensor() = default;

bool Sensor::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Sensor::stop() { return impl_->stop(); }
int64_t Sensor::delta_us() { return impl_->delta_us(); }
}  // namespace sensor