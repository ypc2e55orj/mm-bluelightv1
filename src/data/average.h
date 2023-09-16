#pragma once

#include "ringbuffer.h"

namespace data
{
  template <typename T, std::size_t CAPACITY>
  class MovingAverage
  {
  private:
    bool is_init_;
    RingBuffer<T, CAPACITY> ringbuffer_;

  public:
    explicit MovingAverage(T &init_value): is_init_(false), ringbuffer_()
    {
      for (std::size_t i = 0; i < ringbuffer_.capacity(); i++)
      {
        ringbuffer_.pushBack(init_value);
      }
    }
    ~MovingAverage() = default;
  };
}