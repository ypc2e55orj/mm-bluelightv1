#include "photo.hpp"

// C++

// ESP-IDF
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_adc/adc_continuous.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>

// Project
#include "driver/peripherals/adc.hpp"

namespace driver::hardware
{
  static const auto TAG = "driver::hardware::Photo";
  static constexpr size_t PHOTO_COUNTS = 4;

  class PhotoFlashTimer
  {
  private:
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1'000'000;  // 1MHz
    static constexpr uint32_t INTERVAL_TIMER_FREQUENCY = 4'000; // 4kHz
    static constexpr uint32_t INTERVAL_TIMER_COUNTS = TIMER_RESOLUTION_HZ / INTERVAL_TIMER_FREQUENCY;
    static constexpr uint32_t FLASH_TIMER_FREQUENCY = 10'000; // 10kHz
    static constexpr uint32_t FLASH_TIMER_COUNTS = TIMER_RESOLUTION_HZ / FLASH_TIMER_FREQUENCY;

    struct UserData
    {
      gpio_num_t gpio_num[PHOTO_COUNTS];
      uint8_t index;
      gptimer_handle_t interval_timer;
      gptimer_handle_t flash_timer;
    } *user_data_;

    static bool IRAM_ATTR interval_callback(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *user_ctx)
    {
      auto user_data = reinterpret_cast<UserData *>(user_ctx);
      // 点灯
      gpio_set_level(user_data->gpio_num[user_data->index], 1);
      // 消灯タイマー開始
      ESP_ERROR_CHECK(gptimer_start(user_data->flash_timer));
      // portYIELD_FROM_ISRと同等
      return true;
    }

    static bool IRAM_ATTR flash_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *, void *user_ctx)
    {
      auto user_data = reinterpret_cast<UserData *>(user_ctx);
      // 消灯
      gpio_set_level(user_data->gpio_num[user_data->index], 0);
      // タイマー停止
      ESP_ERROR_CHECK(gptimer_stop(timer));
      // 更新
      if (user_data->index == 0x03)
      {
        // ESP_ERROR_CHECK(gptimer_stop(user_data->interval_timer));
      }
      user_data->index = (user_data->index + 1) & 0x03;
      return true;
    }

  public:
    explicit PhotoFlashTimer(gpio_num_t (&num)[PHOTO_COUNTS]) : user_data_(nullptr)
    {
      ESP_LOGI(TAG, "PhotoFlashTimer Initialized");

      // GPIOを初期化
      gpio_config_t config = {};
      config.mode = GPIO_MODE_OUTPUT;
      for (auto n : num)
      {
        config.pin_bit_mask |= 1ULL << n;
      }
      ESP_ERROR_CHECK(gpio_config(&config));

      // コールバックに渡すユーザー定義引数を設定
      user_data_ = reinterpret_cast<UserData *>(heap_caps_calloc(1, sizeof(UserData), MALLOC_CAP_DMA));
      for (int i = 0; i < PHOTO_COUNTS; i++)
      {
        user_data_->gpio_num[i] = num[i];
      }

      // 消灯タイマー
      gptimer_config_t flash_config = {};
      flash_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
      flash_config.direction = GPTIMER_COUNT_UP;
      flash_config.resolution_hz = TIMER_RESOLUTION_HZ;
      ESP_ERROR_CHECK(gptimer_new_timer(&flash_config, &user_data_->flash_timer));

      // 消灯タイマー コールバックを登録
      gptimer_event_callbacks_t flash_callback_config = {};
      flash_callback_config.on_alarm = flash_callback;
      ESP_ERROR_CHECK(gptimer_register_event_callbacks(user_data_->flash_timer, &flash_callback_config, user_data_));

      // 消灯タイマー コールバックが発火する条件を設定
      gptimer_alarm_config_t flash_alarm = {};
      flash_alarm.reload_count = 0;
      flash_alarm.alarm_count = FLASH_TIMER_COUNTS;
      flash_alarm.flags.auto_reload_on_alarm = false;
      ESP_ERROR_CHECK(gptimer_set_alarm_action(user_data_->flash_timer, &flash_alarm));

      // 発光タイマー
      gptimer_config_t interval_config = {};
      interval_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
      interval_config.direction = GPTIMER_COUNT_UP;
      interval_config.resolution_hz = TIMER_RESOLUTION_HZ;
      ESP_ERROR_CHECK(gptimer_new_timer(&interval_config, &user_data_->interval_timer));

      // 発光タイマー コールバックを登録
      gptimer_event_callbacks_t interval_callback_config = {};
      interval_callback_config.on_alarm = interval_callback;
      ESP_ERROR_CHECK(
        gptimer_register_event_callbacks(user_data_->interval_timer, &interval_callback_config, user_data_));

      // 発光タイマー コールバックが発火する条件を設定
      gptimer_alarm_config_t interval_alarm = {};
      interval_alarm.reload_count = 0;
      interval_alarm.alarm_count = INTERVAL_TIMER_COUNTS;
      interval_alarm.flags.auto_reload_on_alarm = true;
      ESP_ERROR_CHECK(gptimer_set_alarm_action(user_data_->interval_timer, &interval_alarm));
    }
    ~PhotoFlashTimer()
    {
      ESP_ERROR_CHECK(gptimer_del_timer(user_data_->interval_timer));
      ESP_ERROR_CHECK(gptimer_del_timer(user_data_->flash_timer));
      free(user_data_);
    }

    bool enable()
    {
      ESP_ERROR_CHECK(gptimer_enable(user_data_->flash_timer));
      ESP_ERROR_CHECK(gptimer_enable(user_data_->interval_timer));
      return true;
    }

    bool disable()
    {
      ESP_ERROR_CHECK(gptimer_disable(user_data_->flash_timer));
      ESP_ERROR_CHECK(gptimer_disable(user_data_->interval_timer));
      return true;
    }

    bool start()
    {
      ESP_ERROR_CHECK(gptimer_start(user_data_->interval_timer));
      return true;
    }
  };

  class PhotoFlashReceiver
  {
  private:
    static constexpr size_t SAMPLES_PER_CHANNEL = 20;
    static constexpr size_t BYTES_PER_CHANNEL = SAMPLES_PER_CHANNEL * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
    static constexpr size_t BUFFER_SIZE = BYTES_PER_CHANNEL * PHOTO_COUNTS;
    adc_continuous_handle_t handle_;
    uint8_t *buffer_;
    TaskHandle_t task_;

    static bool IRAM_ATTR conv_done_cb(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *user_data)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(reinterpret_cast<PhotoFlashReceiver *>(user_data)->task_, &xHigherPriorityTaskWoken);
      return xHigherPriorityTaskWoken == pdTRUE;
    }

  public:
    explicit PhotoFlashReceiver(adc_unit_t unit, adc_channel_t (&channel)[PHOTO_COUNTS])
      : handle_(nullptr), task_(nullptr)
    {
      buffer_ = reinterpret_cast<uint8_t *>(heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));

      adc_continuous_handle_cfg_t adc_handle_config = {};
      adc_handle_config.max_store_buf_size = BUFFER_SIZE;
      adc_handle_config.conv_frame_size = BUFFER_SIZE;
      ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_handle_config, &handle_));

      adc_continuous_config_t adc_config = {};
      adc_config.sample_freq_hz = 80'000;
      adc_config.conv_mode = unit == ADC_UNIT_1 ? ADC_CONV_SINGLE_UNIT_1 : ADC_CONV_SINGLE_UNIT_2;
      adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
      // adc_config.pattern_num = PHOTO_COUNTS;
      adc_config.pattern_num = 1;
      adc_digi_pattern_config_t adc_pattern[PHOTO_COUNTS] = {};
      for (int i = 0; i < adc_config.pattern_num; i++)
      {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
      }
      adc_config.adc_pattern = adc_pattern;
      ESP_ERROR_CHECK(adc_continuous_config(handle_, &adc_config));

      adc_continuous_evt_cbs_t evt_cbs = {};
      evt_cbs.on_conv_done = conv_done_cb;
      ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle_, &evt_cbs, this));
    }
    ~PhotoFlashReceiver()
    {
      ESP_ERROR_CHECK(adc_continuous_deinit(handle_));
      free(buffer_);
    }

    bool start()
    {
      task_ = xTaskGetCurrentTaskHandle();
      return adc_continuous_start(handle_) == ESP_OK;
    }

    bool stop()
    {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      return adc_continuous_stop(handle_) == ESP_OK;
    }

    bool read()
    {
      uint32_t length = 0;
      return adc_continuous_read(handle_, buffer_, BUFFER_SIZE, &length, 0);
    }

    size_t size()
    {
      return BUFFER_SIZE / SOC_ADC_DIGI_RESULT_BYTES;
    }

    const adc_digi_output_data_t *buffer()
    {
      return reinterpret_cast<adc_digi_output_data_t *>(buffer_);
    }
  };

  class Photo::PhotoImpl final : DriverBase
  {
  private:
    PhotoFlashTimer timer_;
    PhotoFlashReceiver receiver_;

  public:
    explicit PhotoImpl(Config &config) : timer_(config.gpio_num), receiver_(config.adc_unit, config.adc_channel)
    {
      timer_.enable();
      timer_.start();
    }
    ~PhotoImpl()
    {
      timer_.disable();
    }

    bool update() override
    {
      receiver_.start();
      receiver_.read();
      receiver_.stop();
      return true;
    }

    size_t size()
    {
      return receiver_.size();
    }

    const adc_digi_output_data_t *buffer()
    {
      return receiver_.buffer();
    }
  };

  Photo::Photo(Config &config) : impl_(new PhotoImpl(config))
  {
  }
  Photo::~Photo() = default;

  bool Photo::update()
  {
    return impl_->update();
  }

  size_t Photo::size()
  {
    return impl_->size();
  }

  const adc_digi_output_data_t *Photo::buffer()
  {
    return impl_->buffer();
  }
}
