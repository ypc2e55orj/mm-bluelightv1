#pragma once

#include <cstdint>

#define CONFIG_MAX_PARAMS 10

namespace config
{
  struct photo_config
  {
    int left90;
    int left45;
    int right45;
    int right90;
  };
  struct pid_gain_config
  {
    float kp;
    float ki;
    float kd;
  };
  struct straight_config
  {
    float velocity;
    float accel;
    pid_gain_config speed;
    pid_gain_config omega;
  };
  struct turn_config
  {
    pid_gain_config speed;
    pid_gain_config omega;
    struct
    {
      float velocity;
      float accel;
    } pivot;
    struct
    {
      float velocity;
      float accel;
      float offset_pre;
      float offset_post;
    } slalom;
  };
  struct run_config
  {
    straight_config straight;
    turn_config turn;
  };

  struct config
  {
    float tire_diameter;
    struct
    {
      int x;
      int y;
    } goal;
    struct
    {
      int reference[4];
      int threshold[4];
    } photo;
    struct
    {
      run_config search;
      run_config fast[CONFIG_MAX_PARAMS];
      size_t fast_counts;
    } run;
  };

  void read(const char *path);
  void write(const char *path);
}