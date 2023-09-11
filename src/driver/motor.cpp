#include "motor.h"

#include <bdc_motor.h>
#include <driver/gpio.h>

#include <cmath>

namespace driver::motor
{
  static const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 80'000'000;
  static const uint32_t BDC_MCPWM_FREQ_HZ = 800'000;
  static constexpr uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ;

  static const gpio_num_t AIN1 = GPIO_NUM_42;
  static const gpio_num_t AIN2 = GPIO_NUM_41;
  static const gpio_num_t BIN1 = GPIO_NUM_40;
  static const gpio_num_t BIN2 = GPIO_NUM_38;

  static bool enabled = false;

  static bdc_motor_handle_t bdc_handler[2] = {};
  static esp_err_t (*bdc_direction[])(bdc_motor_handle_t) = {
    bdc_motor_forward,
    bdc_motor_reverse,
  };

  void init()
  {
    bdc_motor_config_t motor_cfg = {};
    motor_cfg.pwm_freq_hz = BDC_MCPWM_FREQ_HZ;

    bdc_motor_mcpwm_config_t mcpwm_cfg = {};
    mcpwm_cfg.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

    // left
    motor_cfg.pwma_gpio_num = AIN1;
    motor_cfg.pwmb_gpio_num = AIN2;
    mcpwm_cfg.group_id = 0;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg, &mcpwm_cfg, &bdc_handler[0]));

    // right
    motor_cfg.pwma_gpio_num = BIN1;
    motor_cfg.pwmb_gpio_num = BIN2;
    mcpwm_cfg.group_id = 1;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg, &mcpwm_cfg, &bdc_handler[1]));
  }

  void enable()
  {
    enabled = true;
    ESP_ERROR_CHECK(bdc_motor_enable(bdc_handler[0]));
    ESP_ERROR_CHECK(bdc_motor_enable(bdc_handler[1]));
  }
  void disable()
  {
    ESP_ERROR_CHECK(bdc_motor_disable(bdc_handler[0]));
    ESP_ERROR_CHECK(bdc_motor_disable(bdc_handler[1]));
    enabled = false;
  }

  void brake()
  {
    ESP_ERROR_CHECK(bdc_motor_brake(bdc_handler[0]));
    ESP_ERROR_CHECK(bdc_motor_brake(bdc_handler[1]));
  }
  void coast()
  {
    ESP_ERROR_CHECK(bdc_motor_coast(bdc_handler[0]));
    ESP_ERROR_CHECK(bdc_motor_coast(bdc_handler[1]));
  }

  static void duty(uint8_t pos, float val)
  {
    auto duty_tick = static_cast<uint32_t>(static_cast<float>(BDC_MCPWM_DUTY_TICK_MAX) * std::abs(val));
    ESP_ERROR_CHECK(bdc_direction[val < 0.0f ? 1 : 0](bdc_handler[pos]));
    ESP_ERROR_CHECK(bdc_motor_set_speed(bdc_handler[pos], duty_tick));
  }
  void duty(std::pair<float, float> val)
  {
    if (!enabled)
    {
      return;
    }

    auto [left, right] = val;
    duty(0, left);
    duty(1, right);
  }
}
