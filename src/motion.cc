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

namespace motion {
class Run {
 protected:
  Motion::RunConfig &config_;

 public:
  /**
   * @brief 走行設定を初期化
   */
  explicit Run(Motion::RunConfig &config) : config_(config) {}

  /**
   * @brief 子クラスの走行モードを定義
   * @return 走行モード
   */
  virtual Motion::RunMode mode() = 0;
  /**
   * @brief 1周期での動作を定義する
   * @return 左右のモーター電圧を返す
   */
  virtual std::pair<float, float> tick() = 0;
  /**
   * @brief 完了したかどうかを返す
   * @return trueのとき完了
   */
  virtual bool done() = 0;
};
// 停止
class Stop final : public Run {
 public:
  Motion::RunMode mode() override { return Motion::RunMode::Stop; }
  std::pair<float, float> tick() override {
    // 速度0にフィードバックする
    float v_l = 0.0f, v_r = 0.0f;

    return {v_l, v_r};
  }
  bool done() override { return true; }
};
// UI用触覚フィードバック
class HapticFeedback final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::HapticFeedback; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};
// 直線
class Straight final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::Straight; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};
// 超信地旋回
class PivotTurn final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::PivotTurn; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};
// スラローム旋回
class SlalomTurn final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::SlalomTurn; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};

class Motion::MotionImpl final : public rtos::Task {
 private:
  driver::Driver &dri_;
  config::Config &conf_;
  odometry::Odometry &odom_;

  data::Pid velocity_pid_;
  data::Pid angular_velocity_pid_;
  rtos::Queue<RunConfig> queue_;

  void setup() override {
    queue_.reset();
    velocity_pid_.reset();
    angular_velocity_pid_.reset();
  }
  void loop() override {
    // センサ取得通知
    if (conf_.low_voltage > dri_.battery->average()) {
      // 移動平均が停止電圧の場合
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
    // TODO: 走行モードに応じて制御
  }
  void end() override {}

 public:
  explicit MotionImpl(driver::Driver &dri, config::Config &conf,
                      odometry::Odometry &odom)
      : rtos::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        conf_(conf),
        odom_(odom),
        queue_(1) {}
  ~MotionImpl() override = default;

  bool run(RunConfig &target) { return queue_.send(&target, 0); }
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
bool Motion::run(motion::Motion::RunConfig &target) {
  return impl_->run(target);
}
}  // namespace motion
