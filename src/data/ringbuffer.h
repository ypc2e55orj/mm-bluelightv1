#pragma once

namespace data
{
  template <typename T, std::size_t CAPACITY> class [[maybe_unused]] RingBuffer
  {
  private:
    // 先頭を指す添字
    std::size_t head_;
    // 末尾を指す添字
    std::size_t tail_;
    // 現在の要素数
    std::size_t size_;
    // 添字を最大要素数で切り捨てるマスク
    std::size_t mask_;
    // 要素を保持する配列
    T buffer_[CAPACITY];

  public:
    explicit RingBuffer() : head_(0), tail_(0), size_(0)
    {
      static_assert(CAPACITY && (CAPACITY & (CAPACITY - 1)) == 0, "CAPACITY must be a power of 2.");
      mask_ = CAPACITY - 1;
    }
    ~RingBuffer() = default;

    // 最大要素数を返す
    constexpr std::size_t capacity()
    {
      return CAPACITY;
    }
    // 現在の要素数を返す
    std::size_t size()
    {
      return size_;
    }

    // 添字アクセス (読み)
    const T &operator[](std::size_t index) const
    {
      return buffer_[(head_ + index) & mask_];
    }
    // 添字アクセス (書き)
    T &operator[](std::size_t index)
    {
      return buffer_[(head_ + index) & mask_];
    }

    // 先頭にデータを追加
    [[maybe_unused]] void pushFront(T &data)
    {
      size_++;
      head_ = (head_ - 1) & mask_;
      buffer_[head_] = data;
    }
    // 先頭のデータを削除
    [[maybe_unused]] void popFront()
    {
      size_--;
      head_ = (head_ + 1) & mask_;
    }
    // 末尾にデータを追加
    [[maybe_unused]] void pushBack(T &data)
    {
      buffer_[tail_] = data;
      tail_ = (tail_ + 1) & mask_;
      size_++;
    }
    // 末尾のデータを削除
    [[maybe_unused]] void popBack()
    {
      tail_ = (tail_ - 1) & mask_;
      size_--;
    }
  };
}