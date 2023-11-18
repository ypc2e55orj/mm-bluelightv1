#include "odometry.h"

// C++
#include <cmath>

// Project
#include "config.h"
#include "driver/driver.h"

namespace odometry {
class Odometry::OdometryImpl {
 private:
  driver::Driver &dri_;
  config::Config &conf_;

  // 車輪の速度・移動距離などの情報
  Wheels wheels_;

  static float calculate_wheel_angular_velocity(bool invert, float prev,
                                                float curr, uint32_t delta_us) {
    constexpr auto RADIAN_TWO = 2.0f * std::numbers::pi_v<float>;
    if (invert) {
      prev = RADIAN_TWO - prev;
      curr = RADIAN_TWO - curr;
    }
    // センサ値更新間隔(delta_us)での観測値の変化量を計算
    auto delta = curr - prev;
    if (std::abs(delta) >= std::numbers::pi_v<float>) {
      if (prev > std::numbers::pi_v<float>) {
        delta = RADIAN_TWO + delta;
      } else {
        delta = RADIAN_TWO - delta;
      }
    }
    // 1ms での変化量に換算
    return delta / static_cast<float>(delta_us) * 1000.0f;
  }
  static void reset_wheel(Wheel &wheel) {
    wheel.reset = true;
    wheel.angular_velocity = 0.0f;
    wheel.velocity = 0.0f;
    wheel.length = 0.0f;
  }
  void update_wheel(Wheel &wheel, float curr, uint32_t delta_us) const {
    if (wheel.reset) [[unlikely]] {
      wheel.previous = curr;
      wheel.reset = false;
    }
    wheel.angular_velocity =
        calculate_wheel_angular_velocity(false, wheel.previous, curr, delta_us);
    wheel.velocity = wheel.angular_velocity * conf_.tire_diameter / 2.0f;
    wheel.length += wheel.velocity;
    wheel.previous = curr;
  }
  void reset_wheels() {
    // 左
    reset_wheel(wheels_.left);
    // 右
    reset_wheel(wheels_.right);
    // 並進方向
    wheels_.velocity = 0.0f;
    wheels_.length = 0.0f;
  }
  void update_wheels(uint32_t delta_us) {
    // 左
    update_wheel(wheels_.left, dri_.encoder_left->radian(), delta_us);
    // 右
    update_wheel(wheels_.left, dri_.encoder_right->radian(), delta_us);
    // 並進方向
    wheels_.velocity = (wheels_.left.velocity + wheels_.right.velocity) / 2.0f;
    wheels_.length += wheels_.velocity;
  }

 public:
  explicit OdometryImpl(driver::Driver &dri, config::Config &conf)
      : dri_(dri), conf_(conf), wheels_() {}
  ~OdometryImpl() = default;

  void reset() { reset_wheels(); }
  void update(uint32_t delta_us) { update_wheels(delta_us); }

  const Wheels &wheels() { return wheels_; }
};

Odometry::Odometry(driver::Driver &dri, config::Config &conf)
    : impl_(new OdometryImpl(dri, conf)) {}
Odometry::~Odometry() = default;

void Odometry::reset() { return impl_->reset(); }
void Odometry::update(uint32_t delta_us) { return impl_->update(delta_us); }
const Odometry::Wheels &Odometry::wheels() { return impl_->wheels(); }
}  // namespace odometry