#pragma once

// C++
#include <memory>

// Project
#include "config.h"
#include "motion.h"

namespace run {
// パラメータレベル
enum class Level {
  Search,
  Fast0,
  Fast1,
  Fast2,
  Fast3,
  Fast4,
};

// 走行モード
enum class Mode {
  /// 自由回転
  Free,
  /// UI用触覚フィードバック
  HapticFeedback,
  /// 停止
  Stop,
  /// 前壁補正
  AdjustFront,
  /// 超信地旋回
  PivotTurn,
  /// 直進
  Straight,
  /// 斜め
  Diagonal,
  /// スラローム旋回
  SlalomTurn,
  SlalomTurnLeft45,
  SlalomTurnRight45,
  SlalomTurnLeft90,
  SlalomTurnRight90,
  SlalomTurnLeft135,
  SlalomTurnRight135,
  SlalomTurnLeft180,
  SlalomTurnRight180,
  /// 斜めスラローム旋回
  SlalomTurnVLeft90,
  SlalomTurnVRight90,
};

// Runクラスに与える拘束条件
struct Parameter {
  /// 走行モード
  Mode mode{Mode::Stop};
  /// 走行レベル
  Level level{Level::Search};
  /// 横壁補正有効
  bool enable_side_wall_adjust;
  /// 最大速度 [mm/s]
  float max_velocity;
  /// 最大加速度 [mm/s^2]
  float max_acceleration;
  /// 最大躍度 [mm/s^3]
  float max_jerk;
  /// 最大角速度 [rad/s]
  float max_angular_velocity;
  /// 最大角加速度 [rad/s^2]
  float max_angular_acceleration;
  /// 最大角躍度 [rad/s^3]
  float max_angular_jerk;
};

// Runクラスで生成する目標値
struct Target {
  /// 条件
  Parameter parameter;
  /// 目標速度 [mm/s]
  float velocity;
  /// 目標加速度 [mm/s^2]
  float acceleration;
  /// 目標躍度 [mm/s^3]
  float jerk;
  /// 目標距離 [mm]
  float length;
  /// 目標角速度 [rad/s]
  float angular_velocity;
  /// 目標角加速度 [rad/s^2]
  float angular_acceleration;
  /// 目標角躍度 [rad/s^3]
  float angular_jerk;
  /// 目標角度 [deg]
  float degree;
};

class Run {
 private:
  class RunImpl;
  std::unique_ptr<RunImpl> impl_;

 public:
  explicit Run();
  ~Run();

  const Target& run(const Parameter& param);
};
}  // namespace run
