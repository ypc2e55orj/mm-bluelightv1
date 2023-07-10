#include "photo.h"
#include "adc.h"

#include <rom/ets_sys.h>
#include <driver/gpio.h>

#include <cstring>

namespace driver::photo
{
  static const struct
  {
    gpio_num_t ir;
    adc_channel_t photo;
  } photo_pins[] = {
      [LEFT_90] = {GPIO_NUM_13, ADC_CHANNEL_3},
      [LEFT_45] = {GPIO_NUM_12, ADC_CHANNEL_2},
      [RIGHT_45] = {GPIO_NUM_11, ADC_CHANNEL_1},
      [RIGHT_90] = {GPIO_NUM_10, ADC_CHANNEL_0},
  };

  static int result[4] = {};

  void init()
  {
    // GPIO config
    gpio_config_t gpio_cfg = {};
    gpio_cfg.mode = GPIO_MODE_OUTPUT;

    for (int i = 0; i < NUMS; i++)
    {
      gpio_cfg.pin_bit_mask |= (1UL << photo_pins[i].ir);
    }

    gpio_config(&gpio_cfg);

    for (int i = 0; i < NUMS; i++)
    {
      gpio_set_level(photo_pins[i].ir, 0);
    }

    driver::adc::init();
    for (int i = 0; i < NUMS; i++)
    {
      driver::adc::chan(photo_pins[i].photo);
    }
  }

  static void IRAM_ATTR sampling(uint8_t pos)
  {
    gpio_set_level(photo_pins[pos].ir, 1);
    ets_delay_us(10);
    result[pos] = driver::adc::voltage(photo_pins[pos].photo);
    gpio_set_level(photo_pins[pos].ir, 0);
  }

  void IRAM_ATTR update()
  {
    for (int i = 0; i < NUMS; i++)
    {
      sampling(i);
    }
  }

  void get(int *dest)
  {
    memcpy(dest, result, sizeof(int) * NUMS);
  }
}
