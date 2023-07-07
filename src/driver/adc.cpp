#include "adc.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

namespace driver::adc
{
  static bool initialized_ = false;

  static adc_oneshot_unit_handle_t adc1 = nullptr;
  static adc_cali_handle_t adc1_cali = nullptr;

  void init()
  {
    if (initialized_)
    {
      return;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = ADC_UNIT_1;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1));

    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = ADC_UNIT_1;
    cali_cfg.atten = ADC_ATTEN_DB_11;
    cali_cfg.bitwidth = ADC_BITWIDTH_12;

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali));

    initialized_ = true;
  }

  void chan(adc_channel_t channel)
  {
    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_11;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, channel, &chan_cfg));
  }

  int raw(adc_channel_t channel)
  {
    int raw_val = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, channel, &raw_val));

    return raw_val;
  }

  int voltage(adc_channel_t channel)
  {
    int voltage = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali, raw(channel), &voltage));

    return voltage;
  }
}
