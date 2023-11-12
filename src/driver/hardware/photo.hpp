#pragma once

// C++
#include <cstdint>
#include <memory>

// ESP-IDF
#include <hal/adc_types.h>
#include <hal/gpio_types.h>

// Project
#include "base.hpp"

namespace driver::hardware {
static constexpr size_t PHOTO_COUNTS = 4;

class Photo final : DriverBase {
 private:
  class PhotoImpl;
  std::unique_ptr<PhotoImpl> impl_;

 public:
  struct Result {
    int ambient;
    int flash;
  };

  struct Config {
    adc_unit_t adc_unit;
    gpio_num_t gpio_num[4];
    adc_channel_t adc_channel[4];
  };

  explicit Photo(Config &config);
  ~Photo();

  bool update() override;

  const Result &left90();
  const Result &left45();
  const Result &right45();
  const Result &right90();
};
}  // namespace driver::hardware
