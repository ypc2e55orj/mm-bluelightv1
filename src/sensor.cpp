#include "sensor.h"

#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "./driver/battery.h"
#include "./driver/encoder.h"
#include "./driver/imu.h"
#include "./driver/photo.h"

#include <cassert>

namespace sensor
{
  static bool is_running = false;

  static gptimer_handle_t gptimer_interval = nullptr; // 4kHz
  static gptimer_handle_t gptimer_flush = nullptr;    // 10us oneshot

  static uint8_t photo_pos = driver::photo::PHOTO_LEFT_90;

  static EventGroupHandle_t xEventGroupSensor = nullptr;

  static const EventBits_t EVENT_GROUP_SENSOR_IMU = (1UL << 1);
  static const EventBits_t EVENT_GROUP_SENSOR_ENCODER = (1UL << 2);
  static const EventBits_t EVENT_GROUP_SENSOR_PHOTO = (1UL << 3);
  static const EventBits_t EVENT_GROUP_SENSOR_ALL =
    (EVENT_GROUP_SENSOR_IMU | EVENT_GROUP_SENSOR_ENCODER | EVENT_GROUP_SENSOR_PHOTO);

  static bool IRAM_ATTR update_interval(gptimer_handle_t timer, const gptimer_alarm_event_data_t *timer_ev, void *)
  {
    if (photo_pos == driver::photo::PHOTO_LEFT_90)
    {
      driver::imu::update();
      driver::encoder::update();
      driver::battery::update();
    }

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
    driver::encoder::init(xEventGroupSensor, EVENT_GROUP_SENSOR_IMU);
    driver::imu::init(xEventGroupSensor, EVENT_GROUP_SENSOR_ENCODER);

    init_interval();
  }

  void start()
  {
    if (is_running)
    {
      return;
    }
    is_running = true;

    xTaskCreatePinnedToCore(sensorStartTask, "sensorStartTask", 8192, nullptr, 0, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  void stop()
  {
    if (!is_running)
    {
      return;
    }
    is_running = false;

    xTaskCreatePinnedToCore(sensorStopTask, "sensorStopTask", 8192, nullptr, 0, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  bool wait()
  {
    EventBits_t uBits =
      xEventGroupWaitBits(xEventGroupSensor, EVENT_GROUP_SENSOR_ALL, pdTRUE, pdTRUE, pdMS_TO_TICKS(1));
    if (uBits == EVENT_GROUP_SENSOR_ALL)
    {
      return true;
    }
    return false;
  }
}
