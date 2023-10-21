#pragma once

// C++
#include <functional>
#include <memory>

// ESP-IDF
#include <driver/spi_master.h>
#include <hal/gpio_types.h>

namespace driver::peripherals
{
  class Spi
  {
  private:
    class SpiImpl;
    std::unique_ptr<SpiImpl> impl_;

  public:
    using SpiCallback = std::function<void(spi_transaction_t *)>;

    explicit Spi(spi_host_device_t host_id, gpio_num_t miso_io_num, gpio_num_t mosi_io_num, gpio_num_t sclk_io_num,
                 int max_transfer_sz);
    ~Spi();

    int add(uint8_t mode, int clock_speed_hz, gpio_num_t spics_io_num, int queue_size, SpiCallback &&pre_cb,
            SpiCallback &&post_cb);

    bool transaction(int index, SpiCallback &&pre_cb, SpiCallback &&post_cb, TickType_t ticks_to_wait);
    bool transaction(int index, TickType_t ticks_to_wait);
  };
}
