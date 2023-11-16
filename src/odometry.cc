#include "odometry.h"

// C++
#include <cmath>

// Project
#include "config.h"
#include "driver/driver.h"

namespace odometry {
class Odometry::OdometryImpl {
 private:
  driver::Driver *dri_;
  config::Config *conf_;
  bool reset_;

  struct Wheel {
    float previous;          // [rad]
    float angular_velocity;  // [rad/s]
    float velocity;          // [m/s]
    float length;            // [m]
  };
  struct Wheels {
    Wheel left, right;
    float velocity;  // [m/s]
    float length;    // [m]
  };
  Wheels wheels_;

  static float calculate_wheel_velocity(bool invert, float prev, float curr) {
    constexpr auto RADIAN_TWO = 2.0f * std::numbers::pi_v<float>;
    if (invert) {
      prev = RADIAN_TWO - prev;
      curr = RADIAN_TWO - curr;
    }

    const auto delta = curr - prev;
    if (std::abs(delta) < std::numbers::pi_v<float>) {
      return delta;
    } else {
      if (prev > std::numbers::pi_v<float>) {
        return RADIAN_TWO + delta;
      } else {
        return RADIAN_TWO - delta;
      }
    }
  }
  void reset_wheels() {
    wheels_.left.previous = dri_->encoder_left->radian();
    wheels_.left.velocity = 0.0f;
    wheels_.left.angular_velocity = 0.0f;
    wheels_.left.length = 0.0f;

    wheels_.right.previous = dri_->encoder_right->radian();
    wheels_.right.velocity = 0.0f;
    wheels_.right.angular_velocity = 0.0f;
    wheels_.right.length = 0.0f;

    wheels_.velocity = 0.0f;
    wheels_.length = 0.0f;
  }
  void update_wheels(int32_t delta_us) {
    auto left = dri_->encoder_left->radian();
    auto right = dri_->encoder_right->radian();

    wheels_.left.angular_velocity =
        calculate_wheel_velocity(false, wheels_.left.previous, left);
    wheels_.left.velocity =
        wheels_.left.angular_velocity * conf_->tire_diameter / 2.0f;
    wheels_.left.length += wheels_.left.velocity;

    wheels_.right.angular_velocity =
        calculate_wheel_velocity(true, wheels_.right.previous, right);
    wheels_.right.velocity =
        wheels_.right.angular_velocity * conf_->tire_diameter / 2.0f;
    wheels_.right.length += wheels_.right.velocity;

    wheels_.left.previous = left;
    wheels_.right.previous = right;

    wheels_.velocity = (wheels_.left.velocity + wheels_.right.velocity) / 2.0f;
    wheels_.length += wheels_.velocity;
  }

 public:
  explicit OdometryImpl(driver::Driver *dri, config::Config *conf)
      : dri_(dri), conf_(conf), reset_(true), wheels_() {}
  ~OdometryImpl() = default;

  void reset() { reset_ = true; }
  void update(int32_t delta_us) {
    if (reset_) [[unlikely]] {
      reset_wheels();
      reset_ = false;
    } else {
      update_wheels(delta_us);
    }
  }

  [[nodiscard]] float velocity() const { return wheels_.velocity; }
  [[nodiscard]] float length() const { return wheels_.length; }
};

Odometry::Odometry(driver::Driver *dri, config::Config *conf)
    : impl_(new OdometryImpl(dri, conf)) {}
Odometry::~Odometry() = default;

void Odometry::reset() { return impl_->reset(); }
void Odometry::update(int32_t delta_us) { return impl_->update(delta_us); }

float Odometry::velocity() { return impl_->velocity(); }
float Odometry::length() { return impl_->length(); }
}  // namespace odometry