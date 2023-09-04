#include "photo.h"
#include "adc.h"

#include <driver/gpio.h>

namespace driver::photo
{
  static const struct
  {
    gpio_num_t ir;
    adc_channel_t photo;
  } photo_pins[] = {
    {GPIO_NUM_13, ADC_CHANNEL_3},
    {GPIO_NUM_12, ADC_CHANNEL_2},
    {GPIO_NUM_11, ADC_CHANNEL_1},
    {GPIO_NUM_10, ADC_CHANNEL_0},
  };

  static int ambient[4] = {};
  static int flush[4] = {};

  void init()
  {
    // GPIO config
    gpio_config_t gpio_cfg = {};
    gpio_cfg.mode = GPIO_MODE_OUTPUT;

    for (auto &photo_pin : photo_pins)
    {
      gpio_cfg.pin_bit_mask |= (1UL << photo_pin.ir);
    }

    gpio_config(&gpio_cfg);

    for (auto &photo_pin : photo_pins)
    {
      gpio_set_level(photo_pin.ir, 0);
    }

    driver::adc::init();
    for (auto &photo_pin : photo_pins)
    {
      driver::adc::chan(photo_pin.photo);
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

  void get(int *dest_ambient, int *dest_flush)
  {
    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      dest_ambient[i] = driver::adc::calibrate(ambient[i]);
      dest_flush[i] = driver::adc::calibrate(flush[i]);
    }
  }
}
