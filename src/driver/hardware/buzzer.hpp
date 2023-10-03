#pragma once

#include <memory>
#include <cstdint>

#include <hal/gpio_types.h>

namespace driver
{
  class Buzzer
  {
  private:
    class BuzzerImpl;
    std::unique_ptr<Buzzer> impl_;

  public:
    explicit Buzzer(gpio_num_t buzzer_num, uint32_t queue_size);
    ~Buzzer();

    bool start();
    bool stop();

    bool add(uint32_t frequency, uint32_t milliseconds);
    bool clear();
  };
}

namespace driver::buzzer
{


  void init();

  void tone(uint32_t freq, uint32_t ms);

  void beep();

  void start(uint32_t freq);
  void stop();
}
