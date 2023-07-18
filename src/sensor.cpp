#include "sensor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gptimer.h>

#include "../src/driver/battery.h"
#include "../src/driver/encoder.h"
#include "../src/driver/imu.h"
#include "../src/driver/photo.h"

namespace sensor
{
  static gptimer_handle_t gptimer_interval = nullptr; // 4kHz
  static gptimer_handle_t gptimer_flush = nullptr;    // 10us oneshot

  static uint8_t photo_pos = driver::photo::LEFT_90;

  static bool IRAM_ATTR update_interval(gptimer_handle_t timer, const gptimer_alarm_event_data_t *timer_ev, void *)
  {
    static int divisor = 0;

    if (++divisor == 4)
    {
      driver::imu::update();
      driver::encoder::update();
      driver::battery::update();
      divisor = 0;
    }

    driver::photo::tx(photo_pos);
    ESP_ERROR_CHECK(gptimer_enable(gptimer_flush));
    ESP_ERROR_CHECK(gptimer_start(gptimer_flush));

    return true;
  }

  static bool IRAM_ATTR recieve_flush(gptimer_handle_t timer, const gptimer_alarm_event_data_t *, void *)
  {
    driver::photo::rx(photo_pos);

    if (++photo_pos == driver::photo::NUMS)
    {
      photo_pos = driver::photo::LEFT_90;
    }
    ESP_ERROR_CHECK(gptimer_stop(timer));
    ESP_ERROR_CHECK(gptimer_disable(timer));

    return true;
  }

  void init()
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

  void start()
  {
    xTaskCreatePinnedToCore(sensorStartTask, "sensorStartTask", 8192, nullptr, 0, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  void stop()
  {
    xTaskCreatePinnedToCore(sensorStopTask, "sensorStopTask", 8192, nullptr, 0, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
