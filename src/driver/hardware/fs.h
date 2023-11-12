#pragma once

// C++
#include <memory>

namespace driver::hardware {
class Fs {
 private:
  class FsImpl;
  std::unique_ptr<FsImpl> impl_;

 public:
  explicit Fs(size_t max_files);
  ~Fs();

  void info(size_t &total, size_t &used);
};
}  // namespace driver::hardware
