#include "motion.h"

// C++
#include <cmath>

// ESP-IDF
#include <esp_system.h>
#include <freertos/FreeRTOS.h>

// Project
#include "config.h"
#include "data/pid.h"
#include "driver/driver.h"
#include "odometry.h"
#include "rtos/queue.h"
#include "rtos/task.h"
#include "run.h"

namespace motion {
/**
 * 参考:
 * 車体モデル
 * https://rt-net.jp/mobility/archives/16525
 * https://rt-net.jp/mobility/archives/12621
 *
 * DCモータを使ったマイクロマウス入門シリーズ
 * https://www.rt-shop.jp/blog/archives/2387
 *
 * MK06-4.5特性
 * http://hidejrlab.blog104.fc2.com/blog-entry-1234.html
 * http://hidejrlab.blog104.fc2.com/blog-entry-1233.html
 */
class Model {
 private:
  /// 逆起電圧定数
  static constexpr float MOTOR_BACK_EMF = 0.062f / 1000.0f;  // [V/rpm]
  /// インダクタンス
  static constexpr float MOTOR_INDUCTANCE = 29.1f / 1000'000.0f;  // [H]
  /// 抵抗
  static constexpr float MOTOR_RESISTANCE = 5.0f;  // [ohm]
  /// トルク定数
  static constexpr float MOTOR_TORQUE = 0.59f / 1000.0f;  // [Nm/A]
  /// 機械的抵抗 (左右)
  static constexpr float WHEEL_MECHANICAL_RESISTANCE[2] = {0.0f, 0.0f};
  /// 車輪トルク定数(ギア比?)
  static constexpr float WHEEL_GEAR_RATIO = 38.0f / 9.0f;  // 1:n
  /// 車体質量
  static constexpr float WEIGHT = 10.0f / 1000.0f;  // [kg]

  /// 設定
  config::Config &conf_;
  /// オドメトリ
  odometry::Odometry &odom_;
  /// 速度PID制御
  data::Pid velocity_pid_{0.0f, 0.0f, 0.0f};
  /// 角速度PID制御
  data::Pid angular_velocity_pid_{0.0f, 0.0f, 0.0f};

 public:
  explicit Model(config::Config &conf, odometry::Odometry &odom)
      : conf_(conf), odom_(odom) {}
  ~Model() = default;

  /**
   * リセット
   */
  void reset() {
    velocity_pid_.reset();
    angular_velocity_pid_.reset();
  }

  /**
   * フィードフォワード・フィードバック制御
   * @param target 目標値
   * @return 左右のモーター電圧[V]
   */
  std::pair<int, int> update(run::Target &target) {
    auto &wheels = odom_.wheels_velocity();
    auto radian = odom_.radian();
    auto velocity = (wheels.left + wheels.right) / 2.0f;

    auto velocity_error = velocity_pid_.update(target.velocity, velocity, 1.0f);
    auto angular_velocity_error =
        angular_velocity_pid_.update(target.angular_velocity, radian, 1.0f);

    auto tire_radius = conf_.tire_diameter / 2.0f;
    auto torque_left = tire_radius;

    return {0, 0};
  }
};

class Motion::MotionImpl final : public rtos::Task {
 private:
  driver::Driver &dri_;
  config::Config &conf_;
  /// モデル
  Model model_;
  /// 走行モードを受け取るキュー
  rtos::Queue<run::Parameter> queue_;
  /// 目標値
  run::Parameter parameter{};
  /// 走行目標値生成クラス
  run::Run run_;

  // 緊急停止
  void emergency_stop() {
    for (uint16_t i = 0; i < dri_.indicator->counts(); i++) {
      dri_.indicator->set(i, 0xFF, 0, 0);
    }
    // ブレーキ
    dri_.motor_left->brake();
    dri_.motor_right->brake();
    // 停止を待つ
    vTaskDelay(pdMS_TO_TICKS(10));
    dri_.motor_left->disable();
    dri_.motor_right->disable();
    // 停止されるまで待つ
    while (!is_stopping()) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  void setup() override { queue_.reset(); }
  void loop() override {
    // センサ取得通知
    if (conf_.low_voltage > dri_.battery->average()) {
      emergency_stop();
    }
    // キューから最新の走行モードを取得
    if (queue_.receive(&parameter, 0)) {
      model_.reset();
    }
    // 走行パターンから目標値を生成
    auto target = run_.run(parameter);

    // 目標値から電圧値に変換
    auto [voltage_left, voltage_right] = model_.update(target);

    // 反映
    auto battery_voltage = dri_.battery->voltage();
    dri_.motor_left->speed(voltage_left, battery_voltage);
    dri_.motor_right->speed(voltage_right, battery_voltage);
  }
  void end() override {}

 public:
  explicit MotionImpl(driver::Driver &dri, config::Config &conf,
                      odometry::Odometry &odom)
      : rtos::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        conf_(conf),
        model_(conf, odom),
        queue_(1) {}
  ~MotionImpl() override = default;

  bool set(run::Parameter *param) { return queue_.overwrite(param); }
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
uint32_t Motion::delta_us() { return impl_->delta_us(); };
}  // namespace motion
