#include "speaker.h"

#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SPEAKER_PIN GPIO_NUM_21

namespace driver::speaker
{
  void init()
  {
    gpio_config_t gpio_cfg = {};
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pin_bit_mask = 1 << SPEAKER_PIN;
    gpio_config(&gpio_cfg);
    gpio_set_level(SPEAKER_PIN, 0);
  }

  void play(note_t *notes, int size)
  {
    for (int i = 0; i < size; i++)
    {
      tone(notes[i].freq, notes[i].ms);
    }
  }

  void tone(uint32_t freq, int32_t ms)
  {
    if (freq == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(ms));
      return;
    }

    uint32_t delay = 500000 / freq;
    int64_t until = (esp_timer_get_time() + ms * 1000);
    while (esp_timer_get_time() < until)
    {
      gpio_set_level(SPEAKER_PIN, 1);
      ets_delay_us(delay);
      gpio_set_level(SPEAKER_PIN, 0);
      ets_delay_us(delay);
    }

    gpio_set_level(SPEAKER_PIN, 0);
  }
}
