#include "photo.h"
#include "adc.h"

#include <driver/gpio.h>
#include <rom/ets_sys.h>

#include <algorithm>
#include <cstring>

namespace driver::photo
{
  static const struct
  {
    gpio_num_t ir;
    adc_channel_t photo;
  } photo_pins[] = {
    [PHOTO_LEFT_90] = {GPIO_NUM_13, ADC_CHANNEL_3},
    [PHOTO_LEFT_45] = {GPIO_NUM_12, ADC_CHANNEL_2},
    [PHOTO_RIGHT_45] = {GPIO_NUM_11, ADC_CHANNEL_1},
    [PHOTO_RIGHT_90] = {GPIO_NUM_10, ADC_CHANNEL_0},
  };

  static int ambient[4] = {};
  static int flush[4] = {};

  void init()
  {
    // GPIO config
    gpio_config_t gpio_cfg = {};
    gpio_cfg.mode = GPIO_MODE_OUTPUT;

    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      gpio_cfg.pin_bit_mask |= (1UL << photo_pins[i].ir);
    }

    gpio_config(&gpio_cfg);

    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      gpio_set_level(photo_pins[i].ir, 0);
    }

    driver::adc::init();
    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      driver::adc::chan(photo_pins[i].photo);
    }
  }

  void IRAM_ATTR tx(uint8_t pos)
  {
    ambient[pos] = driver::adc::raw(photo_pins[pos].photo);
    gpio_set_level(photo_pins[pos].ir, 1);
  }

  void IRAM_ATTR rx(uint8_t pos)
  {
    flush[pos] = driver::adc::raw(photo_pins[pos].photo);
    gpio_set_level(photo_pins[pos].ir, 0);
  }

  void get(int *dest)
  {
    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      dest[i] = std::max(driver::adc::calibrate(flush[i]) - driver::adc::calibrate(ambient[i]), 0);
    }
  }
}
