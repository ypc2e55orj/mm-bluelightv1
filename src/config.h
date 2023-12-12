#pragma once

// C++
#include <array>
#include <string>

namespace config {
struct Config {
  // 停止電圧 [mV]
  int low_voltage = 3500;
  // 車輪間距離 [mm]
  float wheel_track_width = 30.9f;
  // タイヤ幅 [mm]
  float tire_tread_width = 3.8f;
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
  std::array<float, 3> velocity_pid{0.0f, 0.0f, 0.0f};
  float velocity = 0.0f;
  float acceleration = 0.0f;
  float jerk = 0.0f;
  std::array<float, 3> angular_velocity_pid{0.0f, 0.0f, 0.0f};
  float angular_velocity = 0.0f;
  float angular_acceleration = 0.0f;
  float angular_jerk = 0.0f;

  // 迷路情報
  std::array<int, 2> maze_goal{7, 7};
  std::array<int, 2> maze_size{32, 32};

  [[maybe_unused]] bool read_file(std::string_view path);
  [[maybe_unused]] bool read_stdin();

  [[maybe_unused]] bool write_file(std::string_view path);
  [[maybe_unused]] bool write_stdout();

 private:
  bool to_struct(std::string_view str);
  std::string to_str();
};
}  // namespace config
