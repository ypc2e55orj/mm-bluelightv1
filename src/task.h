#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

namespace task {
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

 protected:
  // 実行されるタスク
  static void task(void *pvParameters) {
    auto this_ptr = reinterpret_cast<Task *>(pvParameters);
    // 初期化
    this_ptr->setup();
    auto xLastWakeTime = xTaskGetTickCount();
    while (!this_ptr->req_stop_) {
      xTaskDelayUntil(&xLastWakeTime, this_ptr->tick_);
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
        req_stop_(false) {}
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
};
}  // namespace task