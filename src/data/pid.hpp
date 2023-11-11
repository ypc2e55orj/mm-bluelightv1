#pragma once

#include <cmath>

class Pid
{
private:
  float kp_;
  float ki_;
  float kd_;

  float prev_target_;
  float prev_;

  float sum_target_;
  float sum_;

public:
  explicit Pid()
  {
    reset();
  }

  void reset()
  {
    prev_target_ = 0.0f;
    prev_ = 0.0f;
    sum_target_ = 0.0f;
    sum_ = 0.0f;
  }
  void gain(float kp, float ki, float kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }
  float update(float target, float current, float dt)
  {
    float ret = kp_ * (target - current) + ki_ * (sum_target_ - sum_) + kd_ * (prev_target_ - prev_) / dt;

    if ((!std::signbit(target) && (sum_target_ + target) > sum_target_) ||
        (std::signbit(target) && (sum_target_ + target) < sum_target_))
    {
      sum_target_ += target;
    }

    if ((!std::signbit(current) && (sum_ + current) > sum_) || (std::signbit(current) && (sum_ + current) < sum_))
    {
      sum_ += current;
    }

    prev_target_ = target;
    prev_ = current;

    return ret;
  }
};
