#pragma once

namespace data
{
  template <typename T, std::size_t CAPACITY> class RingBuffer
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
    // リングバッファの最大要素数
    std::size_t capacity_;
    // 要素を保持する配列
    T *buffer_;

    // 与えられた数以上の1のべき乗を求める
    constexpr std::size_t power2(std::size_t p)
    {
      --p;
      std::size_t n = 0;
      for (; p != 0; p >>= 1)
        n = (n << 1) + 1;
      return n;
    }

  public:
    explicit RingBuffer() : head_(0), tail_(0), size_(0)
    {
      mask_ = power2(CAPACITY);
      capacity_ = mask_ + 1;
      buffer_ = new T[capacity_];
    }
    ~RingBuffer()
    {
      delete[] buffer_;
    }

    // 最大要素数を返す
    constexpr std::size_t capacity()
    {
      return capacity_;
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
    void pushFront(T &data)
    {
      size_++;
      head_ = (head_ - 1) & mask_;
      buffer_[head_] = data;
    }
    // 先頭のデータを削除
    void popFront()
    {
      size_--;
      head_ = (head_ + 1) & mask_;
    }
    // 末尾にデータを追加
    void pushBack(T &data)
    {
      buffer_[tail_] = data;
      tail_ = (tail_ + 1) & mask_;
      size_++;
    }
    // 末尾のデータを削除
    void popBack()
    {
      tail_ = (tail_ - 1) & mask_;
      size_--;
    }
  };
}