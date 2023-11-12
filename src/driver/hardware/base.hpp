#pragma once

namespace driver::hardware {
struct DriverBase {
  virtual bool update() = 0;
};
}  // namespace driver::hardware