#include "adc.h"

#include <stdexcept>

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

namespace driver
{
  class Adc::AdcImpl
  {
  private:
    // 各ユニットのハンドラ
    static adc_oneshot_unit_handle_t unit1_;
    static adc_oneshot_unit_handle_t unit2_;
    // 各ユニットの補正ハンドラ
    static adc_cali_handle_t unit1_cali_;
    static adc_cali_handle_t unit2_cali_;

    // 使用するチャンネル
    adc_channel_t channel_;
    // 使用するユニットのハンドラ
    adc_oneshot_unit_handle_t unit_;
    // 使用するユニットの補正ハンドラ
    adc_cali_handle_t unit_cali_;

    // ADC値
    int raw_;
    // 電圧補正値
    int voltage_;

  public:
    explicit AdcImpl(adc_unit_t unit, adc_channel_t channel)
      : channel_(channel), unit_(nullptr), unit_cali_(nullptr), raw_(0), voltage_(0)
    {
      adc_oneshot_unit_init_cfg_t init_cfg = {};
      adc_cali_curve_fitting_config_t cali_cfg = {};
      cali_cfg.atten = ADC_ATTEN_DB_11;
      cali_cfg.bitwidth = ADC_BITWIDTH_12;

      // ユニットを初期化 (すでに初期化されている場合は処理しない)
      switch (unit)
      {
      case ADC_UNIT_1:
        if (!unit1_) [[unlikely]]
        {
          init_cfg.unit_id = ADC_UNIT_1;
          ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit1_));
          cali_cfg.unit_id = ADC_UNIT_1;
          ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &unit1_cali_));
        }
        unit_ = unit1_;
        unit_cali_ = unit1_cali_;
        break;
      case ADC_UNIT_2:
        if (!unit2_) [[unlikely]]
        {
          init_cfg.unit_id = ADC_UNIT_2;
          ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit2_));
          cali_cfg.unit_id = ADC_UNIT_2;
          ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &unit2_cali_));
        }
        unit_ = unit2_;
        unit_cali_ = unit2_cali_;
        break;
      default:
        throw std::runtime_error("Error Adc::AdcImpl(): ADC unit must be specified");
      }

      // チャンネルを初期化
      adc_oneshot_chan_cfg_t chan_cfg = {};
      chan_cfg.atten = ADC_ATTEN_DB_11;
      chan_cfg.bitwidth = ADC_BITWIDTH_12;
      ESP_ERROR_CHECK(adc_oneshot_config_channel(unit_, channel, &chan_cfg));
    }
    ~AdcImpl() = default;

    // ADC値を取得する
    int read()
    {
      ESP_ERROR_CHECK(adc_oneshot_read(unit_, channel_, &raw_));
      return raw_;
    }
    // 取得済みのADC値から、電圧値に換算する
    int to_voltage()
    {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(unit_cali_, raw_, &voltage_));
      return voltage_;
    }
  };
  // 各ユニットのハンドラ
  adc_oneshot_unit_handle_t Adc::AdcImpl::unit1_ = nullptr;
  adc_oneshot_unit_handle_t Adc::AdcImpl::unit2_ = nullptr;
  // 各ユニットの補正ハンドラ
  adc_cali_handle_t Adc::AdcImpl::unit1_cali_ = nullptr;
  adc_cali_handle_t Adc::AdcImpl::unit2_cali_ = nullptr;

  Adc::Adc(adc_unit_t unit, adc_channel_t channel) : impl_(new AdcImpl(unit, channel))
  {
  }
  Adc::~Adc() = default;
  int Adc::read()
  {
    return impl_->read();
  }
  int Adc::to_voltage()
  {
    return impl_->to_voltage();
  }
}