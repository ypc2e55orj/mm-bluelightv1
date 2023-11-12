#pragma once

// C++
#include <array>
#include <string>

namespace config {
struct Config {
  // タイヤの直径
  float tire_diameter;
  // ギア比
  float spur_gear_teeth;
  float pinion_gear_teeth;
  // 壁センサ 壁があるかないかのしきい値
  std::array<int, 4> photo_wall_threshold;
  // 壁センサ 迷路中央にいるときの値
  std::array<int, 4> photo_wall_reference;
  // 走行パラメータ
  std::array<float, 3> straight_pid;
  float straight_velocity;
  float straight_accel;
  float straight_jerk;
  std::array<float, 3> turn_pid;
  float turn_velocity;
  float turn_accel;
  float turn_jerk;
  float slalom_turn_velocity;
  float slalom_turn_accel;
  float slalom_turn_jerk;
  float slalom_turn_offset_pre;
  float slalom_turn_offset_post;
  // 迷路情報
  std::array<int, 2> maze_goal;
  std::array<int, 2> maze_size;

  [[maybe_unused]] bool read_file(const char *path);
  [[maybe_unused]] bool read_stdin();

  [[maybe_unused]] bool write_file(const char *path);
  [[maybe_unused]] bool write_stdout();

 private:
  bool to_struct(std::string_view str);
  std::string to_str();
};
}  // namespace config

extern config::Config conf;