#include "buzzer.hpp"

#include <stdexcept>

#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace driver::hardware
{
  /**
   * 参考: https://github.com/espressif/esp-idf/tree/release/v5.1/examples/peripherals/rmt/musical_buzzer
   */
  class RmtBuzzer
  {
  private:
    // DMA有効時、DMAのバッファサイズ
    // DMA無効時、チャンネルが専有するメモリブロック(48以上)
    static constexpr size_t BUZZER_MEM_BLOCK_SYMBOLS = 64;
    // 内部カウンタの精度
    static constexpr uint32_t BUZZER_RESOLUTION_HZ = 1'000'000;
    // エンコーダーに渡す構造体
    struct BuzzerEncoder
    {
      rmt_encoder_t base;
      rmt_encoder_t *copy_encoder;
      uint32_t resolution;
    };
    using BuzzerEncoderHandle = BuzzerEncoder *;

    // copy_encoderをハンドルするコールバック
    static size_t rmt_buzzer_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data,
                                    size_t, rmt_encode_state_t *ret_state)
    {
      auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
      auto copy_encoder = buzzer_encoder->copy_encoder;
      auto note = reinterpret_cast<const BuzzerNote *>(primary_data);
      uint16_t duration = buzzer_encoder->resolution / note->frequency / 2;
      rmt_symbol_word_t symbol = {};
      symbol.level0 = 0;
      symbol.duration0 = duration;
      symbol.level1 = 1;
      symbol.duration1 = duration;

      return copy_encoder->encode(copy_encoder, channel, &symbol, sizeof(rmt_symbol_word_t), ret_state);
    }

    static esp_err_t rmt_buzzer_delete(rmt_encoder_t *encoder)
    {
      auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
      rmt_del_encoder(buzzer_encoder->copy_encoder);
      delete buzzer_encoder;

      return ESP_OK;
    }

    static esp_err_t rmt_buzzer_reset(rmt_encoder_t *encoder)
    {
      auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
      rmt_encoder_reset(buzzer_encoder->copy_encoder);

      return ESP_OK;
    }

  public:
    explicit RmtBuzzer(gpio_num_t buzzer_num, uint32_t queue_size, bool with_dma = false)
      : channel_(nullptr), encoder_(nullptr)
    {
      // RMT送信チャンネルを初期化
      rmt_tx_channel_config_t rmt_config = {.gpio_num = buzzer_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = BUZZER_RESOLUTION_HZ,
        .mem_block_symbols = BUZZER_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = queue_size,
        .flags = {.with_dma = with_dma}};
      ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_config, &channel_));

      // ブザー用エンコーダーを初期化
      encoder_ = new BuzzerEncoder;
      encoder_->base.encode = RmtBuzzer::rmt_buzzer_encode;
      encoder_->base.del = RmtBuzzer::rmt_buzzer_delete;
      encoder_->base.reset = RmtBuzzer::rmt_buzzer_reset;
      encoder_->resolution = BUZZER_RESOLUTION_HZ;
      // copy_encoderを初期化
      rmt_copy_encoder_config_t copy_encoder_config = {};
      if (rmt_new_copy_encoder(&copy_encoder_config, &encoder_->copy_encoder))
      {
        delete encoder_;
        encoder_ = nullptr;
        std::runtime_error("Buzzer::RmtBuzzer::RmtBuzzer(): Failed to rmt_new_copy_encoder()");
      }
    }

    ~RmtBuzzer()
    {
      rmt_del_channel(channel_);
    }

    bool enable()
    {
      return rmt_enable(channel_) == ESP_OK;
    }

    bool disable()
    {
      return rmt_disable(channel_) == ESP_OK;
    }

    bool tone(BuzzerNote &note)
    {
      rmt_transmit_config_t tx_config = {.loop_count = static_cast<int>(note.duration * note.frequency / 1000)};
      return rmt_transmit(channel_, &encoder_->base, &note, sizeof(BuzzerNote), &tx_config) == ESP_OK;
    }

  private:
    // 送信チャンネルのハンドラ
    rmt_channel_handle_t channel_;
    // エンコーダーのハンドラ
    BuzzerEncoderHandle encoder_;
  };

  class Buzzer::BuzzerImpl
  {
  private:
    // 親タスクのハンドラ
    TaskHandle_t parent_;
    // 電圧監視タスクのハンドラ
    TaskHandle_t task_;
    // 停止リクエスト
    bool req_stop_;
    // ブザー操作
    RmtBuzzer buzzer_;
    // BuzzerNoteを貯めるキュー
    QueueHandle_t queue_;

    static void task(void *pvParameters)
    {
      auto this_ptr = reinterpret_cast<BuzzerImpl *>(pvParameters);
      // ブザーを有効化
      this_ptr->buzzer_.enable();
      auto xLastWakeTime = xTaskGetTickCount();

      while (!this_ptr->req_stop_)
      {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
      // ブザーを無効化
      this_ptr->buzzer_.disable();

      // 終了完了を停止要求タスクに通知
      xTaskNotifyGive(this_ptr->parent_);
      // タスクを削除
      vTaskDelete(nullptr);
    }

  public:
    explicit BuzzerImpl(gpio_num_t buzzer_num)
      : parent_(nullptr), task_(nullptr), req_stop_(false), buzzer_(buzzer_num, 4),
        queue_(xQueueCreate(sizeof(BuzzerNote), 10))
    {
    }

    ~BuzzerImpl()
    {
      vQueueDelete(queue_);
    }

    bool start(const uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID)
    {
      auto ret = xTaskCreatePinnedToCore(task, "driver::Buzzer::BuzzerImpl::task", usStackDepth, this, uxPriority,
                                         &task_, xCoreID);
      return ret == pdTRUE;
    }

    bool stop(TickType_t xTicksToWait)
    {
      parent_ = xTaskGetCurrentTaskHandle();
      req_stop_ = true;
      return ulTaskNotifyTake(pdFALSE, xTicksToWait) != 0;
    }
  };

  Buzzer::Buzzer(gpio_num_t buzzer_num)
    : impl_(new BuzzerImpl(buzzer_num))
  {}

  Buzzer::~Buzzer() = default;

  bool Buzzer::start(const uint32_t usStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID)
  {
    return impl_->start(usStackDepth, uxPriority, xCoreID);
  }

  bool Buzzer::stop(TickType_t xTicksToWait)
  {
    return impl_->stop(xTicksToWait);
  }
}
