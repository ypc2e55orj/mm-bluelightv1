#pragma once

// C++
#include <array>
#include <string>

namespace config {
struct Config {
  // 停止電圧 [mV]
  int low_voltage = 3500;
  // タイヤの直径 [mm]
  float tire_diameter = 12.80f;
  // ギア比
  float spur_gear_teeth = 38.0f;
  float pinion_gear_teeth = 9.0f;
  // 壁センサ 壁があるかないかのしきい値
  std::array<int, 4> photo_wall_threshold{0, 0, 0, 0};
  // 壁センサ 迷路中央にいるときの値
  std::array<int, 4> photo_wall_reference{0, 0, 0, 0};
  // 走行パラメータ
  std::array<float, 3> straight_pid{0.0f, 0.0f, 0.0f};
  float straight_velocity = 0.0f;
  float straight_accel = 0.0f;
  float straight_jerk = 0.0f;
  std::array<float, 3> turn_pid{0.0f, 0.0f, 0.0f};
  float turn_velocity = 0.0f;
  float turn_accel = 0.0f;
  float turn_jerk = 0.0f;
  float slalom_turn_velocity = 0.0f;
  float slalom_turn_accel = 0.0f;
  float slalom_turn_jerk = 0.0f;
  float slalom_turn_offset_pre = 0.0f;
  float slalom_turn_offset_post = 0.0f;
  // 迷路情報
  std::array<int, 2> maze_goal{0, 0};
  std::array<int, 2> maze_size{0, 0};

  [[maybe_unused]] bool read_file(std::string_view path);
  [[maybe_unused]] bool read_stdin();

  [[maybe_unused]] bool write_file(std::string_view path);
  [[maybe_unused]] bool write_stdout();

 private:
  bool to_struct(std::string_view str);
  std::string to_str();
};
}  // namespace config
