#pragma once

// C++
#include <cstdint>

// ESP-IDF
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace rtos {
// タスク基底クラス
class Task {
 private:
  // 実行するタスクの名前
  const char *name_;
  // 実行しているタスクのハンドラ
  TaskHandle_t task_;
  // タスクの実行周期
  TickType_t tick_;
  // 呼び出し元のタスクのハンドラ
  TaskHandle_t notify_dest_;
  // 停止リクエスト
  bool req_stop_;
  // 前回の時刻
  int64_t prev_us_;
  // 前回との差分
  uint32_t delta_us_;

 protected:
  // 実行されるタスク
  static void task(void *pvParameters) {
    auto this_ptr = reinterpret_cast<Task *>(pvParameters);
    // 初期化
    this_ptr->setup();
    this_ptr->delta_us_ = 0;
    this_ptr->prev_us_ = esp_timer_get_time();
    auto xLastWakeTime = xTaskGetTickCount();
    while (!this_ptr->req_stop_) {
      xTaskDelayUntil(&xLastWakeTime, this_ptr->tick_);
      auto curr_us = esp_timer_get_time();
      this_ptr->delta_us_ = static_cast<uint32_t>(curr_us - this_ptr->prev_us_);
      this_ptr->prev_us_ = curr_us;
      this_ptr->loop();
    }
    // 終了
    this_ptr->end();
    // 終了完了を停止要求タスクに通知
    xTaskNotifyGive(this_ptr->notify_dest_);
    // タスクを削除
    vTaskDelete(nullptr);
  }
  virtual void setup() = 0;
  virtual void loop() = 0;
  virtual void end() = 0;

 public:
  explicit Task(const char *name, TickType_t tick)
      : name_(name),
        task_(nullptr),
        tick_(tick),
        notify_dest_(nullptr),
        req_stop_(false),
        prev_us_(),
        delta_us_() {}
  virtual ~Task() = default;

  // タスク開始
  bool start(uint32_t usStackDepth, UBaseType_t uxPriority,
             BaseType_t xCoreID) {
    req_stop_ = false;
    auto ret = xTaskCreatePinnedToCore(task, name_, usStackDepth, this,
                                       uxPriority, &task_, xCoreID);
    return ret == pdTRUE;
  }
  // タスク終了
  bool stop() {
    notify_dest_ = xTaskGetCurrentTaskHandle();
    req_stop_ = true;
    return ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0;
  }
  // タスクハンドル取得
  TaskHandle_t handle() { return task_; }
  // 計測実行周期の取得
  [[nodiscard]] uint32_t delta_us() const { return delta_us_; }
  // 停止中か
  [[nodiscard]] bool is_stopping() const { return req_stop_; }
};
}  // namespace task