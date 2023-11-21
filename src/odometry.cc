#include "odometry.h"

// C++
#include <cmath>
#include <limits>

// Project
#include "config.h"
#include "driver/driver.h"

/**
 * @brief マウスの自己位置を推定する。
 * @details
 * 以下の記事を参考にした。
 * https://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.htm
 * https://rikei-tawamure.com/entry/2020/05/22/232227
 */
namespace odometry {
/**
 * @brief 車輪から得られる車体情報を管理する
 */
class Wheel {
 private:
  //! 車輪の直径 [mm]
  const float tire_diameter_;

  //! エンコーダーの回転方向を反転するか
  const bool invert_;

  //! 初期化後かどうか
  bool reset_{true};

  //! 一つ前の観測角度 [rad]
  float previous_{0.0f};

  //! 車輪の角速度 [rad/s]
  float angular_velocity_{0.0f};

  //! 車輪位置の移動速度 [mm/s]
  float velocity_{0.0f};

  //! 車輪の移動距離 [mm]
  float length_{0.0f};

  /**
   * @brief 車輪エンコーダーの更新周期の差分を計算し角速度に変換する。
   * @param current 最新の観測角度 [rad]
   * @param delta_us 更新周期 [us]
   * @return 車輪の角速度 [rad/s]
   */
  float calculate_angular_velocity(float current, uint32_t delta_us) {
    constexpr auto RADIAN_TWO = 2.0f * std::numbers::pi_v<float>;
    // 回転方向を反転
    if (invert_) {
      previous_ = RADIAN_TWO - previous_;
      current = RADIAN_TWO - current;
    }
    // センサ値更新間隔(delta_us)での観測値の変化量を計算
    auto delta = current - previous_;
    if (std::abs(delta) >= std::numbers::pi_v<float>) {
      if (previous_ > std::numbers::pi_v<float>) {
        delta = RADIAN_TWO + delta;
      } else {
        delta = RADIAN_TWO - delta;
      }
    }
    // 1msでの変化量に換算する
    return (delta / static_cast<float>(delta_us)) * 1000'000.0f;
  }

 public:
  /**
   * @param tire_diameter 車輪の直径 [mm]
   * @param invert エンコーダーの回転方向を反転する
   */
  explicit Wheel(float tire_diameter, bool invert)
      : tire_diameter_(tire_diameter), invert_(invert) {}
  ~Wheel() = default;

  /**
   * @brief 車輪情報を更新する
   * @param current 最新の観測角度 [rad]
   * @param delta_us 更新周期 [us]
   */
  void update(float current, uint32_t delta_us) {
    if (reset_) [[unlikely]] {
      previous_ = current;
      reset_ = false;
    }

    angular_velocity_ = calculate_angular_velocity(current, delta_us);
    velocity_ = angular_velocity_ * (tire_diameter_ / 2.0f);
    length_ += velocity_ / 1000.0f;
    previous_ = current;
  }

  /**
   * @brief リセット
   */
  void reset() {
    reset_ = true;
    angular_velocity_ = 0.0f;
    velocity_ = 0.0f;
    length_ = 0.0f;
  }

  [[nodiscard]] float velocity() const { return velocity_; }
  [[nodiscard]] float length() const { return length_; }
};

/**
 * @brief 車輪エンコーダーを用いたオドメトリ
 */
class Wheels {
 private:
  //! 車輪間隔
  const float wheel_track_width_;

  //! 車輪
  Wheel left_, right_;

  //! 車体並進速度 [mm/s]
  float velocity_{0.0f};

  //! 車体並進移動距離 [mm]
  float length_{0.0f};

  //! 車体角速度 [rad/s]
  float angular_velocity_{0.0f};

  //! 車体角度 [rad]
  float radian_{0.0f};

  //! 車体位置 [mm]
  float x_{0.0f}, y_{0.0f};

 public:
  explicit Wheels(float tire_diameter, float wheel_track_width)
      : wheel_track_width_(wheel_track_width),
        left_(tire_diameter, false),
        right_(tire_diameter, true) {}

  /**
   * @brief リセット
   */
  void reset() {
    // 左
    left_.reset();
    // 右
    right_.reset();
    // 車体
    length_ = 0.0f;
    radian_ = 0.0f;
    x_ = 0.0f;
    y_ = 0.0f;
  }
  /**
   * @brief 車体情報を更新する
   * @param left 現在の左車輪角度
   * @param right 現在の右車輪角度
   * @param delta_us 更新周期
   */
  void update(float left, float right, uint32_t delta_us) {
    // 左
    left_.update(left, delta_us);
    // 右
    right_.update(right, delta_us);

    // 中心速度 [mm/s]
    const auto vel_left = left_.velocity();
    const auto vel_right = right_.velocity();
    velocity_ = (vel_left + vel_right) / 2.0f;
    // 中心並進移動距離 [mm]
    length_ += velocity_ / 1000.0f;

    // 中心角速度 [rad/s]
    angular_velocity_ = (vel_left - vel_right) / wheel_track_width_;
    // 中心角度 [rad]
    const auto radian_prev = radian_;
    radian_ += angular_velocity_ / 1000.0f;

    // x, yの位置を推定
    if (std::fabs(vel_left - vel_right) <=
        std::numeric_limits<float>::epsilon()) {
      // 直線運動
      auto a = velocity_ * static_cast<float>(delta_us) / 1000'000.0f;
      x_ += a * std::cos(radian_);
      y_ += a * std::sin(radian_);
    } else {
      // 曲線近似
      auto delta = (radian_ - radian_prev) / 2.0f;
      auto a = 2.0f * velocity_ / angular_velocity_ * std::sin(delta);
      auto b = radian_ + delta;
      x_ += a * std::cos(b);
      y_ += a * std::sin(b);
    }
  }

  [[nodiscard]] float radian() const { return radian_; }
  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
};

class Odometry::OdometryImpl {
 private:
  //! センサ値を取得するためのドライバクラス
  driver::Driver &dri_;

  //! エンコーダーを用いた推定
  Wheels wheels_;

 public:
  explicit OdometryImpl(driver::Driver &dri, config::Config &conf)
      : dri_(dri), wheels_(conf.tire_diameter, conf.wheel_track_width) {}
  ~OdometryImpl() = default;

  void reset() { wheels_.reset(); }
  void update(uint32_t delta_us) {
    wheels_.update(dri_.encoder_left->radian(), dri_.encoder_right->radian(),
                   delta_us);
  }

  float radian() { return wheels_.radian(); }
  float x() { return wheels_.x(); }
  float y() { return wheels_.y(); }
};

Odometry::Odometry(driver::Driver &dri, config::Config &conf)
    : impl_(new OdometryImpl(dri, conf)) {}
Odometry::~Odometry() = default;

void Odometry::reset() { return impl_->reset(); }
void Odometry::update(uint32_t delta_us) { return impl_->update(delta_us); }
float Odometry::radian() { return impl_->radian(); }
float Odometry::x() { return impl_->x(); }
float Odometry::y() { return impl_->y(); }
}  // namespace odometry