#include "sensor.h"

#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "./driver/battery.h"
#include "./driver/encoder.h"
#include "./driver/imu.h"
#include "./driver/photo.h"

#include "./config.h"

#include <cassert>
#include <cmath>

namespace sensor
{
  static DRAM_ATTR bool is_running = false;

  static DRAM_ATTR gptimer_handle_t gptimer_interval = nullptr; // 4kHz
  static DRAM_ATTR gptimer_handle_t gptimer_flush = nullptr;    // 10us oneshot

  static DRAM_ATTR uint8_t photo_pos = driver::photo::PHOTO_LEFT_90;

  static EventGroupHandle_t xEventGroupSensor = nullptr;

  static const EventBits_t EVENT_GROUP_SENSOR_PHOTO = 1UL;

  static bool IRAM_ATTR update_interval(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *)
  {
    driver::battery::update();
    driver::photo::tx(photo_pos);

    ESP_ERROR_CHECK(gptimer_enable(gptimer_flush));
    ESP_ERROR_CHECK(gptimer_start(gptimer_flush));

    return false;
  }

  static bool IRAM_ATTR recieve_flush(gptimer_handle_t timer, const gptimer_alarm_event_data_t *, void *)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xResult = pdFAIL;

    driver::photo::rx(photo_pos);

    if (++photo_pos == driver::photo::PHOTO_NUMS)
    {
      photo_pos = driver::photo::PHOTO_LEFT_90;
      xResult = xEventGroupSetBitsFromISR(xEventGroupSensor, EVENT_GROUP_SENSOR_PHOTO, &xHigherPriorityTaskWoken);
    }

    ESP_ERROR_CHECK(gptimer_stop(timer));
    ESP_ERROR_CHECK(gptimer_disable(timer));

    return xResult == pdPASS && xHigherPriorityTaskWoken == pdTRUE;
  }

  static void init_interval()
  {
    // 10us oneshot timer
    gptimer_config_t flush_cfg = {};
    flush_cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    flush_cfg.direction = GPTIMER_COUNT_UP;
    flush_cfg.resolution_hz = 1'000'000; // 1MHz

    ESP_ERROR_CHECK(gptimer_new_timer(&flush_cfg, &gptimer_flush));

    gptimer_event_callbacks_t flush_cb = {};
    flush_cb.on_alarm = recieve_flush;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_flush, &flush_cb, nullptr));

    gptimer_alarm_config_t flush_alarm = {};
    flush_alarm.reload_count = 0;
    flush_alarm.alarm_count = 10;
    flush_alarm.flags.auto_reload_on_alarm = false;

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_flush, &flush_alarm));

    // 4kHz interval timer
    gptimer_config_t interval_cfg = {};
    interval_cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    interval_cfg.direction = GPTIMER_COUNT_UP;
    interval_cfg.resolution_hz = 1'000'000; // 1MHz

    ESP_ERROR_CHECK(gptimer_new_timer(&interval_cfg, &gptimer_interval));

    gptimer_event_callbacks_t interval_cb = {};
    interval_cb.on_alarm = update_interval;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_interval, &interval_cb, nullptr));

    gptimer_alarm_config_t interval_alarm = {};
    interval_alarm.reload_count = 0;
    interval_alarm.alarm_count = 250;
    interval_alarm.flags.auto_reload_on_alarm = true;

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_interval, &interval_alarm));
  }

  static void sensorStartTask(void *)
  {
    photo_pos = driver::photo::PHOTO_LEFT_90;
    ESP_ERROR_CHECK(gptimer_enable(gptimer_interval));
    ESP_ERROR_CHECK(gptimer_start(gptimer_interval));

    vTaskDelete(nullptr);
  }

  static void sensorStopTask(void *)
  {
    ESP_ERROR_CHECK(gptimer_stop(gptimer_interval));
    ESP_ERROR_CHECK(gptimer_disable(gptimer_interval));

    vTaskDelete(nullptr);
  }

  void init()
  {
    xEventGroupSensor = xEventGroupCreate();
    assert(xEventGroupSensor != nullptr);

    driver::photo::init();
    driver::battery::init();
    driver::encoder::init();
    driver::imu::init();

    init_interval();
  }

  void start()
  {
    if (!is_running)
    {
      xTaskCreatePinnedToCore(sensorStartTask, "sensorStartTask", 8192, nullptr, 0, nullptr, 0);
      is_running = true;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  void stop()
  {
    if (is_running)
    {
      xTaskCreatePinnedToCore(sensorStopTask, "sensorStopTask", 8192, nullptr, 0, nullptr, 0);
      is_running = false;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  bool running()
  {
    return is_running;
  }

  bool wait()
  {
    driver::encoder::update();
    driver::imu::update();
    driver::encoder::wait();
    driver::imu::wait();
    EventBits_t uBits =
      xEventGroupWaitBits(xEventGroupSensor, EVENT_GROUP_SENSOR_PHOTO, pdTRUE, pdTRUE, pdMS_TO_TICKS(1));
    if (uBits == EVENT_GROUP_SENSOR_PHOTO)
    {
      return true;
    }
    return false;
  }

  static float calculate_velocity(uint16_t prev, uint16_t curr)
  {
    const auto RESOLUTION = driver::encoder::resolution();

    int diff = curr - prev;
    if (std::abs(diff) > RESOLUTION / 2 + 1)
    {
      diff = prev > RESOLUTION / 2 + 1 ? RESOLUTION - prev + curr : prev + RESOLUTION - curr;
    }

    return static_cast<float>(diff) * (config::hardware.tireDiameter * static_cast<float>(M_PI) / static_cast<float>(RESOLUTION + 0));
  }

  std::pair<float, float> velocity()
  {
    static std::pair<uint16_t, uint16_t> prev = {0, 0};

    const auto curr = driver::encoder::get();
    const auto velo_left = calculate_velocity(prev.first, curr.first);
    const auto velo_right = calculate_velocity(prev.second, curr.second);

    prev = curr;

    return {velo_left, velo_right};
  }

  float angular_velocity(bool reset = false)
  {
    auto [x, y, z] = driver::imu::gyro();

    return z;
  }
}
