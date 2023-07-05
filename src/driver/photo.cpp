#include "photo.h"

#include <rom/ets_sys.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include <cstring>

#define ADC_UNIT_PHOTO ADC_UNIT_1
#define ADC_ATTEN_PHOTO ADC_ATTEN_DB_11
#define ADC_BITWITH_PHOTO ADC_BITWIDTH_12

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

  static adc_oneshot_unit_handle_t adc1 = nullptr;
  static adc_cali_handle_t adc1_cali = nullptr;

  static int result_raw[4] = {};
  static int result_vol[4] = {};

  uint8_t nums() {
    return PHOTO_NUMS;
  }

  void init()
  {
    // GPIO config
    gpio_config_t gpio_cfg = {};
    gpio_cfg.mode = GPIO_MODE_OUTPUT;

    for (int i = 0; i < PHOTO_NUMS; i++)
      gpio_cfg.pin_bit_mask |= (1UL << photo_pins[i].ir);

    gpio_config(&gpio_cfg);

    for (int i = 0; i < PHOTO_NUMS; i++)
    {
      gpio_set_level(photo_pins[i].ir, 1);
    }

    // adc1
    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = ADC_UNIT_PHOTO;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1));

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_PHOTO;
    chan_cfg.bitwidth = ADC_BITWITH_PHOTO;

    for (int i = 0; i < PHOTO_NUMS; i++)
      ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, photo_pins[i].photo, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = ADC_UNIT_PHOTO;
    cali_cfg.atten = ADC_ATTEN_PHOTO;
    cali_cfg.bitwidth = ADC_BITWITH_PHOTO;

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali));
  }

  void sampling1(uint32_t charge_us)
  {
    gpio_set_level(photo_pins[PHOTO_LEFT_90].ir, 0);
    gpio_set_level(photo_pins[PHOTO_RIGHT_45].ir, 0);

    ets_delay_us(charge_us); // TODO: タイマー割り込みに変更

    gpio_set_level(photo_pins[PHOTO_LEFT_90].ir, 1);
    gpio_set_level(photo_pins[PHOTO_RIGHT_45].ir, 1);

    ets_delay_us(10); // TODO: タイマー割り込みに変更

    ESP_ERROR_CHECK(adc_oneshot_read(adc1, photo_pins[PHOTO_LEFT_90].photo, &result_raw[PHOTO_LEFT_90]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, photo_pins[PHOTO_RIGHT_45].photo, &result_raw[PHOTO_RIGHT_45]));
  }

  void sampling2(uint32_t charge_us)
  {
    gpio_set_level(photo_pins[PHOTO_LEFT_45].ir, 0);
    gpio_set_level(photo_pins[PHOTO_RIGHT_90].ir, 0);

    ets_delay_us(charge_us); // TODO: タイマー割り込みに変更

    gpio_set_level(photo_pins[PHOTO_LEFT_45].ir, 1);
    gpio_set_level(photo_pins[PHOTO_RIGHT_90].ir, 1);

    ets_delay_us(10); // TODO: タイマー割り込みに変更

    ESP_ERROR_CHECK(adc_oneshot_read(adc1, photo_pins[PHOTO_LEFT_45].photo, &result_raw[PHOTO_LEFT_45]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, photo_pins[PHOTO_RIGHT_90].photo, &result_raw[PHOTO_RIGHT_90]));
  }

  void sampling(uint32_t charge_us)
  {
    sampling1(charge_us);
    sampling2(charge_us);

    for (int i = 0; i < PHOTO_NUMS; i++)
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali, result_raw[i], &result_vol[i]));
  }

  void get(int *result)
  {
    memcpy(result, result_vol, sizeof(int) * PHOTO_NUMS);
  }
}
