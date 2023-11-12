#pragma once

// C++
#include <memory>

namespace config {
struct Config {
  // タイヤの直径
  float tire_diameter;
  // ギア比
  float spur_gear_teeth;
  float pinion_gear_teeth;
  // 壁センサ 壁があるかないかのしきい値
  int photo_wall_threshold[4];
  // 壁センサ 迷路中央にいるときの値
  int photo_wall_reference[4];
  // 走行パラメータ
  float straight_pid[3];
  float straight_velocity;
  float straight_accel;
  float straight_jerk;
  float turn_pid[3];
  float turn_velocity;
  float turn_accel;
  float turn_jerk;
  float slalom_turn_velocity;
  float slalom_turn_accel;
  float slalom_turn_jerk;
  float slalom_turn_offset_pre;
  float slalom_turn_offset_post;
  // 迷路情報
  int maze_goal[2];
  int size[2];

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