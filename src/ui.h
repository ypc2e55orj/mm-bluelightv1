#pragma once

// C++
#include <memory>

// Project
#include "driver/driver.h"

namespace {
class Ui {
 private:
  class UiImpl;
  std::unique_ptr<UiImpl> impl_;

 public:
  explicit Ui(driver::Driver &dri);
  ~Ui();
};
}  // namespace