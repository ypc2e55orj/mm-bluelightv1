#pragma once

// C++
#include <cstdint>
#include <memory>

// ESP-IDF
#include <hal/adc_types.h>
#include <hal/gpio_types.h>

// Project
#include "base.hpp"

namespace driver::hardware
{
  class Photo final : DriverBase
  {
  private:
    class PhotoImpl;
    std::unique_ptr<PhotoImpl> impl_;

  public:
    struct Config
    {
      gpio_num_t gpio_num[4];
      adc_unit_t adc_unit;
      adc_channel_t adc_channel[4];
    };

    explicit Photo(Config &config);
    ~Photo();

    bool update() override;

    const adc_digi_output_data_t *buffer();
    size_t size();
  };
}
