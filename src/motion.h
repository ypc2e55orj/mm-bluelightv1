#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "driver/driver.h"
#include "odometry.h"

namespace motion {
class Motion {
 private:
  class MotionImpl;
  std::shared_ptr<MotionImpl> impl_;

 public:
  enum class RunMode {
    /// 停止
    Stop,
    /// 直進
    Straight,
    /// 超信地旋回
    PivotTurn,
    /// スラローム旋回
    SlalomTurn,
    /// UI用触覚フィードバック
    HapticFeedback,
  };
  struct RunConfig {
    /// 走行モード
    RunMode mode;
    /// 横壁補正有効
    bool enable_side_wall_adjust;
    /// 前壁補正有効
    bool enable_front_wall_adjust;
    /// 目標速度 [mm/s]
    float max_velocity;
    /// 目標加速度 [mm/s^2]
    float max_acceleration;
    /// 目標躍度 [mm/s^3]
    float max_jerk;
    /// 目標距離 [mm]
    float max_length;
    /// 目標角加速度 [rad/s]
    float max_angular_velocity;
    /// 目標角加速度 [rad/s^2]
    float max_angular_acceleration;
    /// 目標角躍度 [rad/s^3]
    float max_angular_jerk;
    /// 目標角度 [deg]
    float max_degree;
  };

  explicit Motion(driver::Driver &dri, config::Config &conf,
                  odometry::Odometry &odom);
  ~Motion();

  bool start(uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID);
  bool stop();

  uint32_t delta_us();

  bool run(motion::Motion::RunConfig &target);
};
}  // namespace motion