#include "run.h"

// C++
#include <cmath>
#include <utility>

// Project
#include "motion.h"

namespace run {
class Run::RunImpl {
 private:
  Target target_;

  const Target& free(const Parameter& param) { return target_; }
  const Target& haptic_feedback(const Parameter& param) { return target_; }
  const Target& stop(const Parameter& param) {
    target_.velocity = 0;
    target_.length = 0;
    target_.angular_velocity = 0;
    target_.degree = 0;

    return target_;
  }
  const Target& adjust_front(const Parameter& param) { return target_; }
  const Target& pivot_turn(const Parameter& param) { return target_; }
  const Target& straight(const Parameter& param) { return target_; }
  const Target& diagonal(const Parameter& param) { return target_; }
  const Target& slalom_turn(const Parameter& param) { return target_; }

 public:
  const Target& run(const Parameter& param) {
    target_.parameter = param;

    switch (param.mode) {
      default:
      case Mode::Free:
        return free(param);

      case Mode::HapticFeedback:
        return haptic_feedback(param);

      case Mode::Stop:
        return stop(param);

      case Mode::AdjustFront:
        return adjust_front(param);

      case Mode::PivotTurn:
        return pivot_turn(param);

      case Mode::Straight:
        return straight(param);

      case Mode::Diagonal:
        return diagonal(param);

      case Mode::SlalomTurn:
      case Mode::SlalomTurnLeft90:
      case Mode::SlalomTurnRight90:
      case Mode::SlalomTurnLeft45:
      case Mode::SlalomTurnRight45:
      case Mode::SlalomTurnLeft135:
      case Mode::SlalomTurnRight135:
      case Mode::SlalomTurnLeft180:
      case Mode::SlalomTurnRight180:
      case Mode::SlalomTurnVLeft90:
      case Mode::SlalomTurnVRight90:
        return slalom_turn(param);
    }
  }
};

Run::Run() : impl_(new RunImpl()) {}
Run::~Run() = default;
const Target& Run::run(const Parameter& param) { return impl_->run(param); }
}  // namespace run