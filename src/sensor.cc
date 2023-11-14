#include "sensor.h"

// Project
#include "driver/driver.h"
#include "task.h"

namespace sensor {
class Sensor::SensorImpl final : public task::Task {
 private:
  driver::Driver *dri_;

  void setup() override {}
  void loop() override {
    dri_->photo->update();
    dri_->battery->update();
    dri_->imu->update();
    dri_->encoder_left->update();
    dri_->encoder_right->update();
  }
  void end() override {}

 public:
  explicit SensorImpl(driver::Driver *dri)
      : task::Task(__func__, pdMS_TO_TICKS(1)), dri_(dri) {}
  ~SensorImpl() override = default;
};

Sensor::Sensor(driver::Driver *dri) : impl_(new SensorImpl(dri)) {}
Sensor::~Sensor() = default;

bool Sensor::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Sensor::stop() { return impl_->stop(); }
}  // namespace sensor