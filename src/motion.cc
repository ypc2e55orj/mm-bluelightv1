#include "motion.h"

// C++
#include <cmath>

// ESP-IDF
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Project
#include "config.h"
#include "data/pid.h"
#include "driver/driver.h"
#include "odometry.h"
#include "task.h"

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

class Stop final : public Run {
 public:
  Motion::RunMode mode() override { return Motion::RunMode::Stop; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return true; }
};
class Straight final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::Straight; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};
class PivotTurn final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::PivotTurn; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};
class SlalomTurn final : public Run {
 private:
  bool done_{false};

 public:
  Motion::RunMode mode() override { return Motion::RunMode::SlalomTurn; }
  std::pair<float, float> tick() override { return {0.0f, 0.0f}; }
  bool done() override { return done_; }
};

class Motion::MotionImpl final : public task::Task {
 private:
  driver::Driver &dri_;
  config::Config &conf_;
  odometry::Odometry &odom_;

  QueueHandle_t queue_;

  void setup() override {}
  void loop() override {
    uint32_t delta_us;
    // センサタスクからの通知を待つ
    xTaskNotifyWait(0, 0, &delta_us, portMAX_DELAY);

    if (delta_us == 0) {
      // 開始通知
      // オドメトリをリセット
      odom_.reset();
    } else {
      // センサ取得通知

      // 移動平均が停止電圧の場合
      if (conf_.low_voltage > dri_.battery->average()) {
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
      odom_.update(delta_us);
      // TODO: 走行モードに応じて制御
      dri_.motor_left->enable();
      dri_.motor_right->enable();
      dri_.motor_left->disable();
      dri_.motor_right->disable();
    }
  }
  void end() override {}

 public:
  explicit MotionImpl(driver::Driver &dri, config::Config &conf,
                      odometry::Odometry &odom)
      : task::Task(__func__, pdMS_TO_TICKS(1)),
        dri_(dri),
        conf_(conf),
        odom_(odom) {
    queue_ = xQueueCreate(1, sizeof(RunConfig));
  }
  ~MotionImpl() override { vQueueDelete(queue_); }

  bool set_sensor_notify(uint32_t delta_us) {
    auto pdRet = xTaskNotify(handle(), delta_us, eSetValueWithOverwrite);
    return pdRet == pdTRUE;
  }
  bool enqueue(RunConfig &target) {
    return xQueueSend(queue_, &target, 0) == pdTRUE;
  }
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

bool Motion::set_sensor_notify(uint32_t delta_us) {
  return impl_->set_sensor_notify(delta_us);
}
bool Motion::enqueue(motion::Motion::RunConfig &target) {
  return impl_->enqueue(target);
}
}  // namespace motion
