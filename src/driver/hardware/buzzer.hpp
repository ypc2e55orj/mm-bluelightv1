#pragma once

#include <cstdint>
#include <memory>

#include <hal/gpio_types.h>

namespace driver::hardware
{
  struct BuzzerNote
  {
    uint32_t frequency; // [Hz]
    uint32_t duration;  // [ms]
  };

  enum class BuzzerMode
  {
    None,             // 無音
    InitilizeFailed,  // 初期化失敗
    InitilizeSuccess, // 初期化成功
    Searching,        // 迷路探索中(ループ再生)
    SearchWarning,    // 迷路探索中警告
    SearchFailed,     // 迷路探索失敗
    SearchSuccess,    // 迷路探索成功
    FastFailed,       // 最短走行失敗
    FastSuccess,      // 最短走行成功
    StartRunning,     // 走行開始
    EndRunning,       // 走行終了
  };

  class Buzzer
  {
  private:
    class BuzzerImpl;
    std::unique_ptr<Buzzer> impl_;

  public:
    explicit Buzzer(gpio_num_t buzzer_num);
    ~Buzzer();

    bool start();
    bool stop();

    bool set(BuzzerNote &note);
    bool set(BuzzerMode mode);
  };
}
