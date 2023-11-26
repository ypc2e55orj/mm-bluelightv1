#include "sensor.h"

// Project
#include "driver/driver.h"
#include "odometry.h"
#include "rtos/task.h"

namespace sensor {
class Sensor::SensorImpl final : public rtos::Task {
 private:
  static constexpr uint32_t WARM_UP_COUNTS = 10;

  driver::Driver &dri_;
  odometry::Odometry &odom_;

  void update() const {
    dri_.battery->update();
    dri_.photo->update();
    dri_.imu->update();
    dri_.encoder_left->update();
    dri_.encoder_right->update();
    dri_.photo->wait();
  }

  void setup() override {
    for (uint32_t i = 0; i < WARM_UP_COUNTS; i++) {
      update();
    }
    odom_.reset();
  }
  void loop() override {
    update();
    odom_.update(delta_us());
  }
  void end() override {}

 public:
  explicit SensorImpl(driver::Driver &dri, odometry::Odometry &odom)
      : rtos::Task(__func__, pdMS_TO_TICKS(1)), dri_(dri), odom_(odom) {}
  ~SensorImpl() override = default;
};

Sensor::Sensor(driver::Driver &dri, odometry::Odometry &odom)
    : impl_(new SensorImpl(dri, odom)) {}
Sensor::~Sensor() = default;

bool Sensor::start(uint32_t usStackDepth, UBaseType_t uxPriority,
                   BaseType_t xCoreID) {
  return impl_->start(usStackDepth, uxPriority, xCoreID);
}
bool Sensor::stop() { return impl_->stop(); }
uint32_t Sensor::delta_us() { return impl_->delta_us(); }
}  // namespace sensor