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
  int64_t delta_us_;

  void setup() override {}
  void loop() override {
    auto start = esp_timer_get_time();
    dri_->photo->update();
    dri_->imu->update();
    dri_->encoder_left->update();
    dri_->encoder_right->update();
    dri_->battery->update();
    dri_->photo->wait();
    delta_us_ = esp_timer_get_time() - start;
  }
  void end() override {}

 public:
  explicit SensorImpl(driver::Driver *dri)
      : task::Task(__func__, pdMS_TO_TICKS(1)), dri_(dri), delta_us_() {}
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